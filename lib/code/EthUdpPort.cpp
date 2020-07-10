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

#ifdef _MSC_VER
#define WIN32_LEAN_AND_MEAN
#include <string>
#include <stdlib.h>   // for byteswap functions
#include <winsock2.h>
#include <ws2tcpip.h>
#define WINSOCKVERSION MAKEWORD(2,2)
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }
#else
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>  // for memset
#include <math.h>    // for floor
#include <byteswap.h>
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
    int Send(const char *bufsend, size_t msglen, bool useBroadcast = false);

    // Returns the number of bytes received (-1 on error)
    int Recv(char *bufrecv, size_t maxlen, const double timeoutSec);

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

int SocketInternals::Send(const char *bufsend, size_t msglen, bool useBroadcast)
{
    int retval;
    if (useBroadcast)
        retval = sendto(SocketFD, bufsend, msglen, 0, reinterpret_cast<struct sockaddr *>(&ServerAddrBroadcast), sizeof(ServerAddrBroadcast));
    else
        retval = sendto(SocketFD, bufsend, msglen, 0, reinterpret_cast<struct sockaddr *>(&ServerAddr), sizeof(ServerAddr));

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

int SocketInternals::Recv(char *bufrecv, size_t maxlen, const double timeoutSec)
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
        retval = recv(SocketFD, bufrecv, maxlen, 0);
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
    char buffer[FW_QRESPONSE_SIZE];
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
    sockPtr->Close();
    delete sockPtr;
}

bool EthUdpPort::Init(void)
{
    if (!sockPtr->Open(ServerIP, UDP_port))
        return false;

    bool ret = InitNodes();
    if (ret)
        ret = ScanNodes();
    //if (!ret)
    //    sockPtr->Close();
    return ret;
}

bool EthUdpPort::InitNodes(void)
{
    //  1. Set IP address of first connected board using UDP broadcast
    //  2. Read hub/bridge board id using FireWire broadcast (via unicast UDP)

    // First, set IP address by Ethernet and FireWire broadcast
    if (!WriteQuadletNode(FW_NODE_BROADCAST, 11, sockPtr->ServerAddr.sin_addr.s_addr, FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to write IP address" << std::endl;
        return false;
    }
    quadlet_t data = 0x0;   // initialize data to 0
        
    // Check hardware version of hub board
    if (!ReadQuadletNode(FW_NODE_BROADCAST, 4, data, FW_NODE_NOFORWARD_MASK)) {
        outStr << "InitNodes: failed to read hardware version for hub/bridge board" << std::endl;
        return false;
    }
    if (data != QLA1_String) {
        outStr << "InitNodes: hub board is not a QLA board, data = " << std::hex << data << std::endl;
        return false;
    }

    // Find board id for first board (i.e., one connected by Ethernet) by FireWire broadcast
    if (!ReadQuadletNode(FW_NODE_BROADCAST, 0, data, FW_NODE_NOFORWARD_MASK)) {
        outStr << "InitNodes: failed to read board id for hub/bridge board" << std::endl;
        return false;
    }
    // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
    HubBoard = (data & BOARD_ID_MASK) >> 24;
    outStr << "InitNodes: found hub board: " << static_cast<int>(HubBoard) << std::endl;

    return true;
}

bool EthUdpPort::IsOK(void)
{
    return (sockPtr && (sockPtr->SocketFD != INVALID_SOCKET));
}

void EthUdpPort::Reset(void)
{
    return;
}

bool EthUdpPort::AddBoard(BoardIO *board)
{
    bool ret = BasePort::AddBoard(board);
    if (ret) {
        // Allocate a buffer that is big enough for FireWire header as well
        // as the data to be sent.
        size_t block_write_len = (FW_BWRITE_HEADER_SIZE + board->GetWriteNumBytes() + FW_CRC_SIZE)/sizeof(quadlet_t);
        quadlet_t * buf = new quadlet_t[block_write_len];
        // Offset into the data part of the buffer
        size_t offset = FW_BWRITE_HEADER_SIZE/sizeof(quadlet_t);
        board->SetWriteBuffer(buf, offset);
    }
    return ret;
}

bool EthUdpPort::RemoveBoard(unsigned char boardId)
{
    bool ret = false;
    BoardIO *board = BoardList[boardId];
    if (board) {
        // Free up the memory that was allocated in AddBoard
        delete [] BoardList[boardId]->GetWriteBuffer();
        BoardList[boardId]->SetWriteBuffer(0, 0);
        ret = BasePort::RemoveBoard(boardId);
    }
    else
        outStr << "RemoveBoard: board " << static_cast<unsigned int>(boardId) << " not in use" << std::endl;
    return ret;
}

bool EthUdpPort::ReadAllBoardsBroadcast(void)
{
    if (!IsOK()) {
        outStr << "ReadAllBoardsBroadcast: not initialized" << std::endl;
        return false;
    }

    bool allOK = true;

#if 0
    // sequence number from 16 bits 0 to 65535
    ReadSequence_++;
    if (ReadSequence_ == 65536) {
        ReadSequence_ = 1;
    }
    quadlet_t bcReqData = (ReadSequence_ << 16) | BoardInUseMask_;

    // PK TODO: Fix the following to call make_bread_packet
    //--- send out broadcast read request -----
    const size_t length_fw = 5;
    const size_t numBytes = length_fw*4;
    quadlet_t packet_FW[length_fw];
    // length field (big endian 16-bit integer)
    frame_hdr[12] = 0;
    frame_hdr[13] = numBytes;

    packet_FW[0] = bswap_32(0xFFC00000);
    packet_FW[1] = bswap_32(0xFFCFFFFF);
    packet_FW[2] = bswap_32(0xFFFF000F);
    packet_FW[3] = bswap_32(bcReqData);
    packet_FW[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, 16)));

    int nSent = sockPtr->Send(reinterpret_cast<const char *>(packet_FW), numBytes);
    if (nSent != numBytes) {
        outStr << "ReadAllBoardsBroadcast: UDP send returned " << nSent << ", expected " << numBytes << std::endl;
        return false;
    }

    bool ret = true;

    // record the number of cycle
    int loopSeq = 0;
    for (int bid = 0; bid < BoardIO::MAX_BOARDS; bid++) {
        if (BoardList[bid]) {
            const int readSize = 17;  // 1 seq + 16 data, unit quadlet
            quadlet_t readBuffer[readSize];

            memcpy(readBuffer, packet + ETH_HEADER_LEN + loopSeq * readSize * 4, readSize * 4);

            unsigned int seq = (bswap_32(readBuffer[0]) >> 16);

            //! check boardID

            static int errorcounter = 0;
            if (ReadSequence_ != seq) {
                errorcounter++;
                outStr << "errorcounter = " << errorcounter << std::endl;
                outStr << std::hex << seq << "  " << ReadSequence_ << "  " << (int)bid << std::endl;
            }

            memcpy(BoardList[bid]->GetReadBuffer(), &(readBuffer[1]), (readSize-1) * 4);

            if (!ret) allOK = false;
            BoardList[bid]->SetReadValid(ret);
            loopSeq++;
        }
    }
#else
    outStr << "ReadAllBoardsBroadcast: not yet implemented" << std::endl;
#endif

    return allOK;
}

bool EthUdpPort::ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags)
{
    // Flush before reading
    int numFlushed = sockPtr->FlushRecv();
    if (numFlushed > 0)
        outStr << "ReadQuadlet: flushed " << numFlushed << " packets" << std::endl;

    // Create buffer that is large enough for Firewire packet
    quadlet_t sendPacket[FW_QREAD_SIZE/sizeof(quadlet_t)];

    // Increment transaction label
    fw_tl = (fw_tl+1)&FW_TL_MASK;

    // Build FireWire packet
    make_qread_packet(sendPacket, node, addr, fw_tl, flags&FW_NODE_NOFORWARD_MASK);
    int nSent = sockPtr->Send(reinterpret_cast<const char *>(sendPacket), FW_QREAD_SIZE, flags&FW_NODE_ETH_BROADCAST_MASK);
    if (nSent != static_cast<int>(FW_QREAD_SIZE)) {
        outStr << "ReadQuadlet: failed to send read request via UDP: return value = " << nSent << ", expected = " << FW_QREAD_SIZE << std::endl;
        return false;
    }

    // Invoke callback (if defined) between sending read request
    // and checking for read response. If callback returns false, we
    // skip checking for a received packet.
    if (eth_read_callback && !(*eth_read_callback)(*this, node, outStr)) {
        outStr << "ReadQuadlet: callback aborting (not reading packet)" << std::endl;
        return false;
    }

    quadlet_t recvPacket[FW_QRESPONSE_SIZE/sizeof(quadlet_t)];
    int nRecv = sockPtr->Recv(reinterpret_cast<char *>(recvPacket), FW_QRESPONSE_SIZE, ReceiveTimeout);
    if (nRecv != static_cast<int>(FW_QRESPONSE_SIZE)) {
        unsigned int boardId = Node2Board[node];
        if (boardId < BoardIO::MAX_BOARDS) {
            // Only print message if Node2Board contains valid board number, to avoid unnecessary error messages during ScanNodes.
            outStr << "ReadQuadlet: failed to receive read response from board " << boardId << " via UDP: return value = "
                   << nRecv << ", expected = " << FW_QRESPONSE_SIZE << std::endl;
        }
        return false;
    }
    if (!CheckFirewirePacket(reinterpret_cast<const unsigned char *>(recvPacket), 0, node, EthBasePort::QRESPONSE, fw_tl))
        return false;
    data = bswap_32(recvPacket[3]);
    return true;
}


bool EthUdpPort::WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags)
{
    // Create buffer that is large enough for Firewire packet
    quadlet_t buffer[FW_QWRITE_SIZE/sizeof(quadlet_t)];

    // Increment transaction label
    fw_tl = (fw_tl+1)&FW_TL_MASK;

    // Build FireWire packet
    make_qwrite_packet(buffer, node, addr, data, fw_tl, flags&FW_NODE_NOFORWARD_MASK);


    int nSent = sockPtr->Send(reinterpret_cast<const char *>(buffer), FW_QWRITE_SIZE, flags&FW_NODE_ETH_BROADCAST_MASK);
    return (nSent == static_cast<int>(FW_QWRITE_SIZE));
}

bool EthUdpPort::ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata, unsigned int nbytes)
{
    if (nbytes == 4)
        return ReadQuadlet(boardId, addr, *rdata);
    else if ((nbytes == 0) || ((nbytes%4) != 0)) {
        outStr << "ReadBlock: illegal size (" << nbytes << "), must be multiple of 4" << std::endl;
        return false;
    }

    nodeid_t node = ConvertBoardToNode(boardId);
    if (node == MAX_NODES) {
        outStr << "ReadBlock: board " << static_cast<unsigned int>(boardId&FW_NODE_MASK) << " does not exist" << std::endl;
        return false;
    }

    // Flush before reading
    int numFlushed = sockPtr->FlushRecv();
    if (numFlushed > 0)
        outStr << "ReadBlock: flushed " << numFlushed << " packets" << std::endl;

    // Create buffer that is large enough for Firewire packet
    quadlet_t sendPacket[FW_BREAD_SIZE/sizeof(quadlet_t)];

    // Increment transaction label
    fw_tl = (fw_tl+1)&FW_TL_MASK;

    // Build FireWire packet
    make_bread_packet(sendPacket, node, addr, nbytes, fw_tl, boardId&FW_NODE_NOFORWARD_MASK);
    int nSent = sockPtr->Send(reinterpret_cast<const char *>(sendPacket), FW_BREAD_SIZE, boardId&FW_NODE_ETH_BROADCAST_MASK);
    if (nSent != static_cast<int>(FW_BREAD_SIZE)) {
        outStr << "ReadBlock: failed to send read request via UDP: return value = " << nSent << ", expected = " << FW_BREAD_SIZE << std::endl;
        return false;
    }

    // Invoke callback (if defined) between sending read request
    // and checking for read response. If callback returns false, we
    // skip checking for a received packet.
    if (eth_read_callback && !(*eth_read_callback)(*this, node, outStr)) {
        outStr << "ReadBlock: callback aborting (not reading packet)" << std::endl;
        return false;
    }

    size_t packetSize = FW_BRESPONSE_HEADER_SIZE + nbytes + FW_CRC_SIZE;   // PK TEMP
    quadlet_t *recvPacket = new quadlet_t[packetSize/sizeof(quadlet_t)];
    int nRecv = sockPtr->Recv(reinterpret_cast<char *>(recvPacket), packetSize, ReceiveTimeout);
    if (nRecv != static_cast<int>(packetSize)) {
        outStr << "ReadBlock: failed to receive read response via UDP: return value = " << nRecv << ", expected = " << packetSize << std::endl;
        return false;
    }
    if (!CheckFirewirePacket(reinterpret_cast<const unsigned char *>(recvPacket), nbytes, node, EthBasePort::BRESPONSE, fw_tl))
        return false;
    memcpy(rdata, &recvPacket[5], nbytes);   // PK TEMP
    return true;
}

bool EthUdpPort::WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes)
{
    if (nbytes == 4) {
        return WriteQuadlet(boardId, addr, *wdata);
    }
    else if ((nbytes == 0) || ((nbytes%4) != 0)) {
        outStr << "WriteBlock: illegal size (" << nbytes << "), must be multiple of 4" << std::endl;
        return false;
    }

    nodeid_t node = ConvertBoardToNode(boardId);
    if (node == MAX_NODES) {
        outStr << "WriteBlock: board " << static_cast<unsigned int>(boardId&FW_NODE_MASK) << " does not exist" << std::endl;
        return false;
    }

    // Packet to send
    quadlet_t *packet = 0;
    size_t packetSize = FW_BWRITE_HEADER_SIZE + nbytes + FW_CRC_SIZE;  // includes CRC on data

    // Check for real-time write
    bool realTimeWrite = false;
    boardId = boardId&FW_NODE_MASK;
    if (boardId < BoardIO::MAX_BOARDS) {
        if ((BoardList[boardId]->GetWriteBufferData() == wdata) &&
            (nbytes <= BoardList[boardId]->GetWriteNumBytes())) {
             realTimeWrite = true;
             packet = BoardList[boardId]->GetWriteBuffer();
        }
    }
    if (!packet) {
        packet = new quadlet_t[packetSize];
    }

    // PK: temporarily save the last quadlet, which is the power control on a real-time write,
    //     so that it does not get overwritten by the CRC.
    quadlet_t temp_saved = wdata[nbytes/sizeof(quadlet_t)];

    // Build FireWire packet
    make_bwrite_packet(packet, node, addr, wdata, nbytes, fw_tl, boardId&FW_NODE_NOFORWARD_MASK);
    int nSent = sockPtr->Send(reinterpret_cast<const char *>(packet), packetSize, boardId&FW_NODE_ETH_BROADCAST_MASK);
    if (nSent != static_cast<int>(packetSize)) {
        outStr << "WriteBlock: failed to send write via UDP: return value = " << nSent << ", expected = " << packetSize << std::endl;
        return false;
    }

    wdata[nbytes/sizeof(quadlet_t)] = temp_saved;   // PK TEMP for real-time writes
    if (!realTimeWrite)
        delete [] packet;
    return true;
}

bool EthUdpPort::WriteBlockBroadcast(
        nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes)
{
    outStr << "WriteBlockBroadcast not yet implemented" << std::endl;
    return false;
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

