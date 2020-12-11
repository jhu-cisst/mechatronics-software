/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
#define WINSOCKVERSION MAKEWORD(2,2)
#else
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
#endif

struct SocketInternals {
    std::ostream &outStr;
#ifdef _MSC_VER
    SOCKET SocketFD;     // Actually a 64-bit value on 64-bit OS
#else
    int    SocketFD;
#endif
    struct sockaddr_in ServerAddr;
    struct sockaddr_in ServerAddrBroadcast;

    SocketInternals(std::ostream &ostr);
    ~SocketInternals();

    bool Open(const std::string &host, unsigned short port);
    bool Close();

    // Returns the number of bytes sent (-1 on error)
    int Send(const unsigned char *bufsend, size_t msglen, bool useBroadcast = false);

    // Returns the number of bytes received (-1 on error)
    int Recv(unsigned char *bufrecv, size_t maxlen, const double timeoutSec);

    // Flush the receive buffer
    int FlushRecv(void);
};

SocketInternals::SocketInternals(std::ostream &ostr) : outStr(ostr), SocketFD(INVALID_SOCKET)
{
    memset(&ServerAddr, 0, sizeof(ServerAddr));
    memset(&ServerAddrBroadcast, 0, sizeof(ServerAddrBroadcast));
}

SocketInternals::~SocketInternals()
{
    Close();
}

bool SocketInternals::Open(const std::string &host, unsigned short port)
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
    char broadcastEnable = 1;
#else
    int broadcastEnable = 1;
#endif
    if (setsockopt(SocketFD, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) != 0) {
        outStr << "Open: Failed to set SOCKET broadcast option" << std::endl;
        return false;
    }

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
    outStr << "Server IP: " << host << ", Port: " << std::dec << port << std::endl;
    outStr << "Broadcast IP: " << EthUdpPort::IP_String(ServerAddrBroadcast.sin_addr.s_addr)
           << ", Port: " << std::dec << port << std::endl;
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

int SocketInternals::Send(const unsigned char *bufsend, size_t msglen, bool useBroadcast)
{
    int retval;
    if (useBroadcast)
        retval = sendto(SocketFD, reinterpret_cast<const char *>(bufsend), msglen, 0,
                        reinterpret_cast<struct sockaddr *>(&ServerAddrBroadcast), sizeof(ServerAddrBroadcast));
    else
        retval = sendto(SocketFD, reinterpret_cast<const char *>(bufsend), msglen, 0,
                        reinterpret_cast<struct sockaddr *>(&ServerAddr), sizeof(ServerAddr));

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
        //struct sockaddr_in fromAddr;
        //socklen_t length = sizeof(fromAddr);
        //retval = recvfrom(socketFD, bufrecv, maxlen, 0, reinterpret_cast<struct sockaddr *>(&fromAddr), &length);
        retval = recv(SocketFD, reinterpret_cast<char *>(bufrecv), maxlen, 0);
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

EthUdpPort::EthUdpPort(int portNum, const std::string &serverIP, std::ostream &debugStream, EthCallbackType cb):
    EthBasePort(portNum, debugStream, cb),
    ServerIP(serverIP),
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

bool EthUdpPort::Init(void)
{
    if (!sockPtr->Open(ServerIP, UDP_port))
        return false;

    bool ret = ScanNodes();

    if (ret)
        SetDefaultProtocol();

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
    if (!WriteQuadletNode(FW_NODE_BROADCAST, 11, sockPtr->ServerAddr.sin_addr.s_addr, FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to write IP address" << std::endl;
        return 0;
    }

    quadlet_t data = 0x0;   // initialize data to 0

    // Check hardware version of hub board
    if (!ReadQuadletNode(FW_NODE_BROADCAST, 4, data, FW_NODE_NOFORWARD_MASK)) {
        outStr << "InitNodes: failed to read hardware version for hub/bridge board" << std::endl;
        return 0;
    }
    if (data != QLA1_String) {
        outStr << "InitNodes: hub board is not a QLA board, data = " << std::hex << data << std::endl;
        return 0;
    }

    // ReadQuadletNode should have updated bus generation
    FwBusGeneration = newFwBusGeneration;
    outStr << "InitNodes: Firewire bus generation = " << FwBusGeneration << std::endl;

    // Broadcast a command to initiate a read of Firewire PHY Register 0. In cases where there is no
    // Firewire bus master (i.e., only FPGA/QLA boards on the Firewire bus), this allows each board
    // to obtain its Firewire node id.
    data = 0;
    if (!WriteQuadletNode(FW_NODE_BROADCAST, 1, data)) {
        outStr << "InitNodes: failed to broadcast PHY command" << std::endl;
        return 0;
    }

    // Find board id for first board (i.e., one connected by Ethernet) by FireWire broadcast
    if (!ReadQuadletNode(FW_NODE_BROADCAST, 0, data, FW_NODE_NOFORWARD_MASK)) {
        outStr << "InitNodes: failed to read board id for hub/bridge board" << std::endl;
        return 0;
    }
    // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
    HubBoard = (data & BOARD_ID_MASK) >> 24;
    outStr << "InitNodes: found hub board: " << static_cast<int>(HubBoard) << std::endl;

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

bool EthUdpPort::PacketSend(unsigned char *packet, size_t nbytes, bool useEthernetBroadcast)
{
    int nSent = sockPtr->Send(packet, nbytes, useEthernetBroadcast);

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
