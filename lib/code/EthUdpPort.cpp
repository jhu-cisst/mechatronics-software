/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "EthUdpPort.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"

#ifdef _MSC_VER
#define WIN32_LEAN_AND_MEAN
#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <Iphlpapi.h>
#include <mswsock.h>
#define WINSOCKVERSION MAKEWORD(2,2)
#undef min   // to be able to use std::min
#else // Linux, Mac
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>  // for memset
#include <math.h>    // for floor
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
// Following assertion checks that sizeof(struct in_addr) is equal to sizeof(unsigned long).
// In C++11, could instead use static_assert.
typedef char assertion_on_in_addr[(sizeof(struct in_addr)==sizeof(uint32_t))*2-1];

// to query MTU setting
#include <sys/ioctl.h>
#include <net/if.h>

#endif

#include <algorithm>   // for std::min

#ifdef _MSC_VER

typedef WSAMSG     MsgHeaderType;
typedef WSACMSGHDR CMsgHdrType;

#ifndef CMSG_FIRSTHDR
inline CMsgHdrType *CMSG_FIRSTHDR(MsgHeaderType *hdr)
{ return WSA_CMSG_FIRSTHDR(hdr); }
#endif
#ifndef CMSG_NXTHDR
inline CMsgHdrType *CMSG_NXTHDR(MsgHeaderType *hdr, CMsgHdrType *cmsg)
{ return WSA_CMSG_NXTHDR(hdr, cmsg); }
#endif
#ifndef CMSG_DATA
inline unsigned char *CMSG_DATA(CMsgHdrType *cmsg)
{ return WSA_CMSG_DATA(cmsg); }
#endif

#else
typedef struct msghdr  MsgHeaderType;
typedef struct cmsghdr CMsgHdrType;
#endif

struct SocketInternals {
    std::ostream &outStr;
#ifdef _MSC_VER
    SOCKET SocketFD;     // Actually a 64-bit value on 64-bit OS
#else
    int    SocketFD;
#endif
    int InterfaceIndex;
    std::string InterfaceName;
    int InterfaceMTU;

    struct sockaddr_in ServerAddr;
    struct sockaddr_in ServerAddrBroadcast;
    struct sockaddr_in ServerAddrMulticast;

    bool FirstRun;

    enum EthDestType { DEST_UNICAST, DEST_MULTICAST, DEST_BROADCAST };

    SocketInternals(std::ostream &ostr);
    ~SocketInternals();

    bool Open(const std::string &host, const std::string &multicast, unsigned short port);
    bool Close();

    // Set multicast address
    bool SetMulticastIP(const std::string &multicast);

    // Returns the number of bytes sent (-1 on error)
    int Send(const unsigned char *bufsend, size_t msglen, EthDestType destType = DEST_UNICAST,
             unsigned char ip_offset = 0);

    // Returns the number of bytes received (-1 on error)
    int Recv(unsigned char *bufrecv, size_t maxlen, const double timeoutSec);

    // Flush the receive buffer
    int FlushRecv(void);

    // Extract interface info (index, name and MTU) from message header
    bool ExtractInterfaceInfo(MsgHeaderType *hdr);
};

SocketInternals::SocketInternals(std::ostream &ostr) : outStr(ostr), SocketFD(INVALID_SOCKET),
                 InterfaceIndex(0), InterfaceName("undefined"), InterfaceMTU(ETH_MTU_DEFAULT), FirstRun(true)
{
    memset(&ServerAddr, 0, sizeof(ServerAddr));
    memset(&ServerAddrBroadcast, 0, sizeof(ServerAddrBroadcast));
    memset(&ServerAddrMulticast, 0, sizeof(ServerAddrMulticast));
}

SocketInternals::~SocketInternals()
{
    Close();
}

bool SocketInternals::Open(const std::string &host, const std::string &multicast, unsigned short port)
{
#ifdef _MSC_VER
    WSADATA wsaData;
    int retval = WSAStartup(WINSOCKVERSION, &wsaData);
    if (retval != 0) {
        outStr << "WSAStartup failed with error code " << retval << std::endl;
        return false;
    }
#endif
    ServerAddr.sin_family = AF_INET;
    ServerAddr.sin_port = htons(port);
    ServerAddr.sin_addr.s_addr = EthUdpPort::IP_ULong(host);

    // Open UDP socket
    SocketFD = socket(PF_INET, SOCK_DGRAM, 0);
    if (SocketFD == INVALID_SOCKET) {
        outStr << "Failed to open UDP socket" << std::endl;
        return false;
    }

    // Enable broadcasts
#ifdef _MSC_VER
    BOOL broadcastEnable = TRUE;
    DWORD packetInfo = 1;
    if (setsockopt(SocketFD, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char *>(&broadcastEnable),
                   sizeof(broadcastEnable)) != 0) {
        outStr << "Open: Failed to set SOCKET broadcast option" << std::endl;
        return false;
    }

    if (setsockopt(SocketFD, IPPROTO_IP, IP_PKTINFO, reinterpret_cast<const char *>(&packetInfo),
                   sizeof(packetInfo)) != 0) {
        outStr << "Open: Failed to set SOCKET packet info option" << std::endl;
    }
#else
    int broadcastEnable = 1;
    int packetInfo = 1;
    if (setsockopt(SocketFD, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) != 0) {
        outStr << "Open: Failed to set SOCKET broadcast option" << std::endl;
        return false;
    }

    if (setsockopt(SocketFD, IPPROTO_IP, IP_PKTINFO, &packetInfo, sizeof(packetInfo)) != 0) {
        outStr << "Open: Failed to set SOCKET packet info option" << std::endl;
    }
#endif

    // Determine the broadcast address. For simplicity, we assume that this is a Class A, B, or C network,
    // as defined by the InterNIC:
    //    Class A: first byte is 0-127, default subnet mask is 255.0.0.0
    //    Class B: first byte is 128-191, default subnet mask is 255.255.0.0
    //    Class C: first byte is 192-223, default subnet mask is 255.255.255.0
    // If the first byte is greater than 223, we use the global broadcast address, 255.255.255.255

    ServerAddrBroadcast.sin_family = AF_INET;
    ServerAddrBroadcast.sin_port = htons(port);
    unsigned char firstByte = static_cast<unsigned char>(ServerAddr.sin_addr.s_addr&0x000000ff);
    if (firstByte < 128)
        ServerAddrBroadcast.sin_addr.s_addr = ServerAddr.sin_addr.s_addr|0xffffff00;
    else if (firstByte < 192)
        ServerAddrBroadcast.sin_addr.s_addr = ServerAddr.sin_addr.s_addr|0xffff0000;
    else if (firstByte < 224)
        ServerAddrBroadcast.sin_addr.s_addr = ServerAddr.sin_addr.s_addr|0xff000000;
    else
        ServerAddrBroadcast.sin_addr.s_addr = 0xffffffff;

    // UDP Multicast address
    ServerAddrMulticast.sin_family = AF_INET;
    ServerAddrMulticast.sin_port = htons(port);
    ServerAddrMulticast.sin_addr.s_addr = EthUdpPort::IP_ULong(multicast);

    outStr << "Server IP: " << host << ", Port: " << std::dec << port << std::endl;
    outStr << "Broadcast IP: " << EthUdpPort::IP_String(ServerAddrBroadcast.sin_addr.s_addr) << std::endl;
    outStr << "Multicast IP: " << EthUdpPort::IP_String(ServerAddrMulticast.sin_addr.s_addr) << std::endl;
    return true;
}

bool SocketInternals::Close()
{
    if (SocketFD != INVALID_SOCKET) {
#ifdef _MSC_VER
        if (closesocket(SocketFD) != 0) {
            outStr << "Close: failed to close socket, Error: " << WSAGetLastError() <<std::endl;
            return false;
        }
        WSACleanup();
#else
        if (close(SocketFD) != 0) {
            outStr << "Close: failed to close socket, Error: " << errno << std::endl;
            return false;
        }
#endif
        SocketFD = INVALID_SOCKET;
    }
    return true;
}

// UDP multicast address must be between 224.0.0.0 and 239.255.255.255. Some addresses in this
// range are assigned for specific purposes. The default value for FPGA V3 is 224.0.0.100
// (ETH_UDP_MULTICAST_DEFAULT_IP), which is unassigned.

bool SocketInternals::SetMulticastIP(const std::string &multicast)
{
    unsigned long s_addr_new = EthUdpPort::IP_ULong(multicast);
    unsigned long first_byte = s_addr_new & 0x000000ff;
    if ((first_byte >= 224) && (first_byte <= 239)) {
        ServerAddrBroadcast.sin_addr.s_addr = s_addr_new;
        return true;
    }
    return false;
}

int SocketInternals::Send(const unsigned char *bufsend, size_t msglen, EthDestType destType, unsigned char ip_offset)
{
    int retval;
    if (destType == DEST_BROADCAST) {
        retval = sendto(SocketFD, reinterpret_cast<const char *>(bufsend), msglen, 0,
                        reinterpret_cast<struct sockaddr *>(&ServerAddrBroadcast), sizeof(ServerAddrBroadcast));
    }
    else if (destType == DEST_MULTICAST) {
        retval = sendto(SocketFD, reinterpret_cast<const char *>(bufsend), msglen, 0,
                        reinterpret_cast<struct sockaddr *>(&ServerAddrMulticast), sizeof(ServerAddrMulticast));
    }
    else {
        // Save base address (e.g., 169.254.0.100)
        uint32_t s_addr_saved = ServerAddr.sin_addr.s_addr;
        // Update IP address if needed (Firmware Rev 9+)
        if (ip_offset > 0)
            ServerAddr.sin_addr.s_addr += (ip_offset<<24);
        retval = sendto(SocketFD, reinterpret_cast<const char *>(bufsend), msglen, 0,
                        reinterpret_cast<struct sockaddr *>(&ServerAddr), sizeof(ServerAddr));
        // Restore base address
        ServerAddr.sin_addr.s_addr = s_addr_saved;
    }

    if (retval == SOCKET_ERROR) {
#ifdef _MSC_VER
        outStr << "Send: failed to send: " << WSAGetLastError() << std::endl;
#else
        outStr << "Send: failed to send: " << strerror(errno) << std::endl;
#endif
        return -1;
    }
    else if (retval != static_cast<int>(msglen)) {
        outStr << "Send: failed to send the whole message" << std::endl;
    }
    return retval;
}

int SocketInternals::Recv(unsigned char *bufrecv, size_t maxlen, const double timeoutSec)
{
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(SocketFD, &readfds);

#ifdef _MSC_VER
    long sec = static_cast<long>(floor(timeoutSec));
    long usec = static_cast<long>((timeoutSec - sec) * 1e6);
    int nfds = 1;   // On Windows, this parameter to select is ignored
#else
    time_t sec = static_cast<time_t>(floor(timeoutSec));
    suseconds_t usec = static_cast<suseconds_t>((timeoutSec - sec) * 1e6);
    int nfds = SocketFD+1;
#endif
    timeval timeout = { sec , usec };
    int retval = select(nfds, &readfds, NULL, NULL, &timeout);

    // It is o.k. to check against SOCKET_ERROR. On Windows, this is correct.
    // On Linux, we should check for -1 (which is how SOCKET_ERROR is defined above).
    if (retval == SOCKET_ERROR) {
#ifdef _MSC_VER
        outStr << "Recv: select failed: " << WSAGetLastError() << std::endl;
#else
        outStr << "Recv: select failed: " << strerror(errno) << std::endl;
#endif
    }
    else if (retval > 0) {

        if (FirstRun) {

            const size_t CONTROL_DATA_SIZE = 1000;  // THIS NEEDS TO BE BIG ENOUGH.
            char controldata[CONTROL_DATA_SIZE];
            MsgHeaderType hdr;

#ifdef _MSC_VER
            SOCKADDR addrRemote;
            GUID WSARecvMsg_GUID = WSAID_WSARECVMSG;
            LPFN_WSARECVMSG WSARecvMsg;
            DWORD nBytes;
            retval = WSAIoctl(SocketFD, SIO_GET_EXTENSION_FUNCTION_POINTER, &WSARecvMsg_GUID,
                              sizeof(WSARecvMsg_GUID), &WSARecvMsg, sizeof(WSARecvMsg), &nBytes,
                              NULL, NULL);
            if (retval == SOCKET_ERROR) {
                outStr << "Recv: could not get WSARecvMsg function pointer" << std::endl;
            }
            else {
                WSABUF WSAbuf;
                WSAbuf.buf = reinterpret_cast<char *>(bufrecv);
                WSAbuf.len = maxlen;

                hdr.name = &addrRemote;
                hdr.namelen = sizeof(addrRemote);
                hdr.lpBuffers = &WSAbuf;
                hdr.dwBufferCount = 1;
                hdr.Control.buf = controldata;
                hdr.Control.len = CONTROL_DATA_SIZE;
                hdr.dwFlags = MSG_PEEK;  // Do not remove message from queue

                retval = WSARecvMsg(SocketFD, &hdr, &nBytes, NULL, NULL);
                if (retval == SOCKET_ERROR)
                    outStr << "Recv: WSARecvMsg failed" << std::endl;
                else
                    ExtractInterfaceInfo(&hdr);
            }
            // For some reason, the call to WSARecvMsg (without MSG_PEEK flag) does
            // not work, so we instead specify MSG_PEEK above and call recv again.
            // This also handles the error case (unable to get WSARecvMsg pointer).
            retval = recv(SocketFD, reinterpret_cast<char *>(bufrecv), maxlen, 0);
#else
            sockaddr_storage addrRemote;

            struct iovec vec;
            vec.iov_base = bufrecv;
            vec.iov_len = maxlen;

            hdr.msg_name = &addrRemote;
            hdr.msg_namelen = sizeof(addrRemote);
            hdr.msg_iov = &vec;
            hdr.msg_iovlen = 1;
            hdr.msg_control = controldata;
            hdr.msg_controllen = CONTROL_DATA_SIZE;

            retval = recvmsg(SocketFD, &hdr, 0);
            ExtractInterfaceInfo(&hdr);
#endif

            outStr << "Using interface " << InterfaceName << " (" << InterfaceIndex << "), MTU: " << InterfaceMTU << std::endl;
            FirstRun = false;
        } else {
            //struct sockaddr_in fromAddr;
            //socklen_t length = sizeof(fromAddr);
            //retval = recvfrom(socketFD, bufrecv, maxlen, 0, reinterpret_cast<struct sockaddr *>(&fromAddr), &length);
            retval = recv(SocketFD, reinterpret_cast<char *>(bufrecv), maxlen, 0);
        }
        if (retval == SOCKET_ERROR) {
#ifdef _MSC_VER
            outStr << "Recv: failed to receive: " << WSAGetLastError() << std::endl;
#else
            outStr << "Recv: failed to receive: " << strerror(errno) << std::endl;
#endif
        }
    }
    return retval;
}

int SocketInternals::FlushRecv(void)
{
    unsigned char buffer[FW_QRESPONSE_SIZE];
    int numFlushed = 0;
    // If the packet is larger than FW_QRESPONSE_SIZE, the excess bytes will be discarded.
    while (Recv(buffer, FW_QRESPONSE_SIZE, 0.0) > 0)
        numFlushed++;
    return numFlushed;
}

bool SocketInternals::ExtractInterfaceInfo(MsgHeaderType *hdr)
{
    InterfaceIndex = -1;
    // iterate through all the control headers
    for (CMsgHdrType * cmsg = CMSG_FIRSTHDR(hdr);
         cmsg != NULL;
         cmsg = CMSG_NXTHDR(hdr, cmsg)) {
        // ignore the control headers that don't match what we want
        if (cmsg->cmsg_level != IPPROTO_IP ||
            cmsg->cmsg_type != IP_PKTINFO) {
            continue;
        }
        // struct in_pktinfo * pi = CMSG_DATA(cmsg)->ipi;
        // at this point, peeraddr is the source sockaddr
        // pi->ipi_spec_dst is the destination in_addr
        InterfaceIndex = ((struct in_pktinfo*)CMSG_DATA(cmsg))->ipi_ifindex;
    }

    // should check if index != -1

#ifdef _MSC_VER
    // Windows supports if_indextoname, but the name returned does not appear to be
    // meaningful. Also, Windows does not appear to support ioctl (or WSAIoctl) to
    // query the MTU value. Thus, we instead use GetAdaptersAddresses, which gives us
    // both the InterfaceName and InterfaceMTU for the specified InterfaceIndex.
    //
    // We first call GetAdaptersAddresses with a null buffer to query the required buffer size.
    PIP_ADAPTER_ADDRESSES addrList = 0;
    ULONG bufSize = 0;
    ULONG retval;
    retval = GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER,
                                  NULL, addrList, &bufSize);
    if (retval != ERROR_BUFFER_OVERFLOW) {
        outStr << "Recv: GetAdaptersAddresses failed to return buffer size" << std::endl;
        return false;
    }
    // Now, bufSize indicates the correct buffer size. Allocate memory and then call
    // GetAdaptersAddresses again to get the information.
    char *buffer = new char[bufSize];
    addrList = reinterpret_cast<PIP_ADAPTER_ADDRESSES>(buffer);
    retval = GetAdaptersAddresses(AF_INET, GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER,
                                  NULL, addrList, &bufSize);
    if (retval == NO_ERROR) {
        for (PIP_ADAPTER_ADDRESSES addr = addrList;  addr != 0; addr = addr->Next) {
            if (addr->IfIndex == InterfaceIndex) {
                char fname[256];  // Should be large enough
                if (WideCharToMultiByte(CP_ACP, 0, addr->FriendlyName, -1, fname, sizeof(fname), NULL, NULL) > 0) {
                        InterfaceName = fname;
                        InterfaceMTU = addr->Mtu;
                        break;
                }
            }
        }
    }
    delete [] buffer;
#else
    // get interface name and MTU
    char ifName[256];
    if (if_indextoname(InterfaceIndex, ifName) == NULL) {
        outStr << "Recv: failed if_indextoname: " << strerror(errno) << std::endl;
        return false;
    }
    InterfaceName = ifName;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, InterfaceName.c_str());
    if (!ioctl(SocketFD, SIOCGIFMTU, &ifr)) {
        InterfaceMTU = ifr.ifr_mtu;
        return true;
    }
#endif
    return false;
}

EthUdpPort::EthUdpPort(int portNum, const std::string &serverIP, bool forceFwBridge, std::ostream &debugStream, EthCallbackType cb):
    EthBasePort(portNum, forceFwBridge, debugStream, cb),
    ServerIP(serverIP),
    MulticastIP(ETH_UDP_MULTICAST_DEFAULT_IP),
    UDP_port(1394)
{
    sockPtr = new SocketInternals(debugStream);
    if (Init())
        outStr << "Initialization done" << std::endl;
    else
        outStr << "Initialization failed" << std::endl;
}

EthUdpPort::~EthUdpPort()
{
    Cleanup();
    delete sockPtr;
}

// Set UDP multicast address (must be between 224.0.0.0 and 239.255.255.255).
bool EthUdpPort::SetMulticastIP(const std::string &multicast)
{
    if (!sockPtr)
        return false;
    bool ret = sockPtr->SetMulticastIP(multicast);
    if (ret)
        MulticastIP = multicast;
    return ret;
}

bool EthUdpPort::Init(void)
{
    if (!sockPtr->Open(ServerIP, MulticastIP, UDP_port))
        return false;

    bool ret = ScanNodes();

    if (ret) {
        SetDefaultProtocol();
        if (useFwBridge)
            OptimizeFirewireGapCount();
    }

    //if (!ret)
    //    sockPtr->Close();
    return ret;
}

void EthUdpPort::Cleanup(void)
{
    sockPtr->Close();
}

nodeid_t EthUdpPort::InitNodes(void)
{
    // First, set IP address by Ethernet and FireWire broadcast
    // For Firmware Rev 9+, this also causes the FPGA to send a raw Ethernet multicast quadlet write packet
    // so that the port forwarding database in the FPGA V3 Ethernet Switch gets updated
    if (!WriteQuadletNode(FW_NODE_BROADCAST, BoardIO::IP_ADDR, sockPtr->ServerAddr.sin_addr.s_addr, FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to write IP address" << std::endl;
        return 0;
    }
    Amp1394_Sleep(0.2);

    quadlet_t data = 0;

    // Check hardware version of hub board
    if (!ReadQuadletNode(FW_NODE_BROADCAST, BoardIO::HARDWARE_VERSION, data,
                         FW_NODE_NOFORWARD_MASK | FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to read hardware version for hub/bridge board" << std::endl;
        return 0;
    }
    if (!HardwareVersionValid(data)) {
        outStr << "InitNodes: hub board is not a supported board, data = " << std::hex << data << std::endl;
        return 0;
    }

    // ReadQuadletNode should have updated bus generation
    FwBusGeneration = newFwBusGeneration;
    outStr << "InitNodes: Firewire bus generation = " << FwBusGeneration << std::endl;

    // Broadcast a command to initiate a read of Firewire PHY Register 0. In cases where there is no
    // Firewire bus master (i.e., only FPGA/QLA boards on the Firewire bus), this allows each board
    // to obtain its Firewire node id.
    data = 0;
    if (!WriteQuadletNode(FW_NODE_BROADCAST, BoardIO::FW_PHY_REQ, data, FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to broadcast PHY command" << std::endl;
        return 0;
    }
    Amp1394_Sleep(0.01);

    // Find board id for first board (i.e., one connected by Ethernet) by FireWire and Ethernet broadcast
    if (!ReadQuadletNode(FW_NODE_BROADCAST, BoardIO::BOARD_STATUS, data,
                         FW_NODE_NOFORWARD_MASK | FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to read board id for hub/bridge board" << std::endl;
        return 0;
    }
    // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
    HubBoard = (data & BOARD_ID_MASK) >> 24;
    outStr << "InitNodes: found hub board: " << static_cast<int>(HubBoard) << std::endl;

    // read firmware version
    if (!ReadQuadletNode(FW_NODE_BROADCAST, BoardIO::FIRMWARE_VERSION, data,
                         FW_NODE_NOFORWARD_MASK | FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: unable to read firmware version from hub/bridge board" << std::endl;
        return 0;
    }
    unsigned long fver = data;
    FirmwareVersion[HubBoard] = data;

    // Get FPGA version of Hub board
    if (!ReadQuadletNode(FW_NODE_BROADCAST, BoardIO::ETH_STATUS, data,
                         FW_NODE_NOFORWARD_MASK | FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to read Ethernet status from hub/bridge board" << std::endl;
        return 0;
    }
    unsigned long fpga_ver = BoardIO::GetFpgaVersionMajorFromStatus(data);
    FpgaVersion[HubBoard] = fpga_ver;

    if (!useFwBridge && ((fpga_ver != 3) || (fver < 9))) {
        if (fpga_ver != 3)
            outStr << "InitNodes: hub board is FPGA V" << fpga_ver;
        else if (fver < 9)
            outStr << "InitNodes: hub board has Firmware Rev " << fver;
        outStr << ", using Ethernet/Firewire bridge" << std::endl;
        useFwBridge = true;
    }
    else if (useFwBridge)
        outStr << "InitNodes: using Ethernet/Firewire bridge" << std::endl;
    else
        outStr << "InitNodes: using Ethernet network" << std::endl;

    // Scan for up to 16 nodes on bus
    return BoardIO::MAX_BOARDS;
}

bool EthUdpPort::IsOK(void)
{
    return (sockPtr && (sockPtr->SocketFD != INVALID_SOCKET));
}

unsigned int EthUdpPort::GetPrefixOffset(MsgType msg) const
{
    switch (msg) {
        case WR_CTRL:      return 0;
        case WR_FW_HEADER: return FW_CTRL_SIZE;
        case WR_FW_BDATA:  return FW_CTRL_SIZE+FW_BWRITE_HEADER_SIZE;
        case RD_FW_HEADER: return 0;
        case RD_FW_BDATA:  return FW_BRESPONSE_HEADER_SIZE;
    }
    outStr << "EthUdpPort::GetPrefixOffset: Invalid type: " << msg << std::endl;
    return 0;
}

// The largest data size supported by the firmware is:
//   1) Firewire limits maximum data size to 2048 bytes (MAX_POSSIBLE_DATA_SIZE)
//   2) The FPGA firmware reduces this limit by 24 (FW_BRESPONSE_HEADER_SIZE + FW_CRC_SIZE)
// But, there is also a limit imposed by the MTU (maximum Ethernet packet size); in addition
// to the Firewire packet, we need to consider ETH_UDP_HEADER and FW_EXTRA_SIZE.
unsigned int EthUdpPort::GetMaxReadDataSize(void) const
{
    return (std::min(MAX_POSSIBLE_DATA_SIZE, sockPtr->InterfaceMTU-ETH_UDP_HEADER-FW_EXTRA_SIZE)
            - FW_BRESPONSE_HEADER_SIZE - FW_CRC_SIZE);
}

// The largest data size supported by the firmware is:
//   1) Firewire limits maximum data size to 2048 bytes (MAX_POSSIBLE_DATA_SIZE)
//   2) The FPGA firmware reduces this limit by 24 (FW_BWRITE_HEADER_SIZE + FW_CRC_SIZE)
// But, there is also a limit imposed by the MTU (maximum Ethernet packet size); in addition
// to the Firewire packet, we need to consider ETH_UDP_HEADER and FW_CTRL_SIZE.
unsigned int EthUdpPort::GetMaxWriteDataSize(void) const
{
    return (std::min(MAX_POSSIBLE_DATA_SIZE, sockPtr->InterfaceMTU-ETH_UDP_HEADER-FW_CTRL_SIZE)
            - FW_BWRITE_HEADER_SIZE - FW_CRC_SIZE);
}

bool EthUdpPort::PacketSend(nodeid_t node, unsigned char *packet, size_t nbytes, bool useEthernetBroadcast)
{
    unsigned char ip_offset = 0;
    SocketInternals::EthDestType destType;
    if (useEthernetBroadcast) {
        // Checking the firmware version of HubBoard covers two cases:
        //   1) Supports firmware prior to Rev 9, which does not respond to UDP multicast.
        //   2) Ensures that BROADCAST is used for the first few packets (i.e., before this
        //      class reads the Firmware version), which is necessary for proper operation.
        unsigned long fver = GetFirmwareVersion(HubBoard);
        destType = (fver < 9) ? SocketInternals::DEST_BROADCAST
                              : SocketInternals::DEST_MULTICAST;
    }
    else {
        destType = SocketInternals::DEST_UNICAST;
        if (useFwBridge) {
            // If using the Ethernet/Firewire bridge, then we only need to talk to the Hub board,
            // which has ip_offset=0 for firmware prior to Rev 9
            if (GetFirmwareVersion(HubBoard) >= 9)
                ip_offset = HubBoard;
        }
        else {
            // If not using Ethernet/Firewire bridge, then node id is equal to board id;
            // Don't need to check firmware version because if we got here, it must be at
            // least Rev 9.
            if (node < BoardIO::MAX_BOARDS)
                ip_offset = static_cast<unsigned char>(node);
            else if (node == FW_NODE_BROADCAST)
                destType = SocketInternals::DEST_MULTICAST;
        }
    }

    int nSent = sockPtr->Send(packet, nbytes, destType, ip_offset);

    if (nSent != static_cast<int>(nbytes)) {
        outStr << "PacketSend: failed to send via UDP: return value = " << nSent
               << ", expected = " << nbytes << std::endl;
        return false;
    }
    return true;
}

int EthUdpPort::PacketReceive(unsigned char *packet, size_t nbytes)
{
    int nRecv = sockPtr->Recv(packet, nbytes, ReceiveTimeout);
    if (nRecv == static_cast<int>(FW_EXTRA_SIZE)) {
        outStr << "PacketReceive: only extra data" << std::endl;
        ProcessExtraData(packet);
        nRecv = 0;
    }
    return nRecv;
}

int EthUdpPort::PacketFlushAll(void)
{
    return sockPtr->FlushRecv();
}

// Convert IP address from uint32_t to string
std::string EthUdpPort::IP_String(uint32_t IPaddr)
{
    char IPstr[INET_ADDRSTRLEN];
#ifdef _MSC_VER
    // Windows does not provide inet_ntop prior to Vista, so we use inet_ntoa.
    strncpy(IPstr, inet_ntoa(*reinterpret_cast<const struct in_addr *>(&IPaddr)), INET_ADDRSTRLEN);
#else
    inet_ntop(AF_INET, reinterpret_cast<const struct in_addr *>(&IPaddr), IPstr, INET_ADDRSTRLEN);
#endif
    return std::string(IPstr);
}

// Convert IP address from string to uint32_t
uint32_t EthUdpPort::IP_ULong(const std::string &IPaddr)
{
    unsigned long ret;
#ifdef _MSC_VER
    // Windows does not provide inet_pton prior to Vista, so we use inet_addr.
    ret = inet_addr(IPaddr.c_str());
#else
    inet_pton(AF_INET, IPaddr.c_str(), reinterpret_cast<struct in_addr *>(&ret));
#endif
    return ret;
}
