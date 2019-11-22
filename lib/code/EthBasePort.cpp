/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "EthBasePort.h"
#include "Amp1394Time.h"

#ifdef _MSC_VER
#include <stdlib.h>   // for byteswap functions
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }
#else
#include <string.h>  // for memset
#include <byteswap.h>
#endif

// crc related
uint32_t BitReverse32(uint32_t input);
uint32_t crc32(uint32_t crc, const void *buf, size_t size);


EthBasePort::EthBasePort(int portNum, std::ostream &debugStream, EthCallbackType cb):
    BasePort(portNum, debugStream),
    fw_tl(0),
    eth_read_callback(cb),
    ReceiveTimeout(0.01)
{
}

EthBasePort::~EthBasePort()
{
}

void EthBasePort::GetDestMacAddr(unsigned char *macAddr)
{
    // CID,0x1394,boardid(0)
    macAddr[0] = 0xFA;
    macAddr[1] = 0x61;
    macAddr[2] = 0x0E;
    macAddr[3] = 0x13;
    macAddr[4] = 0x94;
    macAddr[5] = 0x00;
}

void EthBasePort::GetDestMulticastMacAddr(unsigned char *macAddr)
{
    // Same as GetDestMacAddr, except with multicast bit set and
    // last byte set to 0xFF.
    GetDestMacAddr(macAddr);
    macAddr[0] |= 0x01;
    macAddr[5] = 0xFF;
}

void EthBasePort::PrintMAC(std::ostream &outStr, const char* name, const uint8_t *addr)
{
    outStr << name << ": " << std::hex
           << (int)addr[0] << ":" << (int)addr[1] << ":" << (int)addr[2] << ":"
           << (int)addr[3] << ":" << (int)addr[4] << ":" << (int)addr[5] << std::endl;
}

void EthBasePort::PrintIP(std::ostream &outStr, const char* name, const uint8_t *addr)
{
    outStr << name << ": " << std::dec
           << (int)addr[0] << "." << (int)addr[1] << "." << (int)addr[2] << "." << (int)addr[3]
           << std::endl;
}

//TODO: fix for byteswapping
bool EthBasePort::CheckFirewirePacket(const unsigned char *packet, size_t length, unsigned int tcode, unsigned int tl)
{
    if (!checkCRC(packet)) {
        outStr << "CheckFirewirePacket: CRC error" << std::endl;
        return false;
    }
    int tcode_recv = packet[3] >> 4;
    if (tcode_recv != tcode) {
        outStr << "Unexpected tcode: received = " << tcode_recv << ", expected = " << tcode << std::endl;
        return false;
    }
    // TODO: could also check QRESPONSE length
    if (tcode == BRESPONSE) {
        size_t length_recv = static_cast<size_t>((packet[12] << 8) | packet[13]);
        if (length_recv != length) {
            outStr << "Inconsistent length: received = " << length_recv << ", expected = " << length << std::endl;
            return false;
        }
    }
    int tl_recv = packet[2] >> 2;
    if (tl_recv != tl) {
        outStr << "WARNING: received tl = " << tl_recv
               << ", expected tl = " << tl << std::endl;
    }
    return true;
}

void EthBasePort::PrintFirewirePacket(std::ostream &out, const quadlet_t *packet, unsigned int max_quads)
{
    static const char *tcode_name[16] = { "qwrite", "bwrite", "wresponse", "", "qread", "bread",
                                          "qresponse", "bresponse", "cycstart", "lockreq",
                                          "stream", "lockresp", "", "", "", "" };
    unsigned char tcode = (packet[0]&0x000000F0)>>4;
    unsigned int data_length = 0;
    // No point in printing anything if less than 4
    if (max_quads < 4) {
        out << "PrintPacket: should print more than 4 quadlets (max_quads = "
            << max_quads << ")" << std::endl;
        return;
    }
    out << "dest: " << std::hex << ((packet[0]&0xffc00000)>>20)
        << ", node: " << std::dec << ((packet[0]&0x003f0000)>>16)
        << ", tl: " << std::hex << ((packet[0]&0x0000fc00)>>10)
        << ", rt: " << ((packet[0]&0x00000300)>>8)
        << ", tcode: " << static_cast<unsigned int>(tcode) << " (" << tcode_name[tcode] << ")"
        << ", pri: " << (packet[0]&0x0000000F) << std::endl;
    out << "src: " << std::hex << ((packet[1]&0xffc00000)>>20)
        << ", node: " << std::dec << ((packet[1]&0x003f0000)>>16);

    if ((tcode == QRESPONSE) || (tcode == BRESPONSE)) {
        out << ", rcode: " << std::dec << ((packet[1]&0x0000f0000)>>12);
    }
    else if ((tcode == QWRITE) || (tcode == QREAD) || (tcode == BWRITE) || (tcode == BREAD)) {
        out << ", dest_off: " << std::hex << (packet[1]&0x0000ffff) << std::endl;
        out << "dest_off: " << std::hex << packet[2];
    }
    out << std::endl;

    if ((tcode == BWRITE) || (tcode == BRESPONSE)) {
        data_length = (packet[3]&0xffff0000) >> 16;
        out << "data_length: " << std::dec << data_length
            << ", ext_tcode: " << std::hex << (packet[3]&0x0000ffff) << std::endl;
    }
    else if ((tcode == QWRITE) || (tcode == QRESPONSE)) {
        out << "data: " << std::hex << packet[3] << std::endl;
    }

    if (tcode == QREAD)
        out << "header_crc: " << std::hex << packet[3] << std::endl;
    else if (max_quads < 5)  // Nothing else to do for short packets
        return;
    else
        out << "header_crc: " << std::hex << packet[4] << std::endl;

    unsigned int lim = (data_length <= max_quads-5) ? data_length : max_quads-5;
    for (unsigned int i = 0; i < lim; i++)
        out << "data[" << std::dec << i << "]: " << std::hex << packet[5+i] << std::endl;
    if ((data_length > 0) && (data_length < max_quads-5))
        out << "data_crc: " << std::hex << packet[5+data_length] << std::endl;
}

void EthBasePort::PrintDebug(std::ostream &debugStream, unsigned short status)
{
    debugStream << "Status: ";
    if (status&0x4000) debugStream << "error ";
    if (status&0x2000) debugStream << "initOK ";
    if (status&0x1000) debugStream << "initReq ";
    if (status&0x0800) debugStream << "ethIoErr ";
    if (status&0x0400) debugStream << "PacketErr ";
    if (status&0x0200) debugStream << "DestErr ";
    //if (status&0x0400) debugStream << "cmdReq ";
    //if (status&0x0200) debugStream << "cmdAck ";
    //if (status&0x0400) debugStream << "local ";
    //if (status&0x0200) debugStream << "remote ";
    if (status&0x0100) debugStream << "qRead ";
    if (status&0x0080) debugStream << "qWrite ";
    if (status&0x0040) debugStream << "bRead ";
    if (status&0x0020) debugStream << "bWrite ";
    //if ((status&0x0010)) debugStream << "multicast ";
    if ((status&0x0010)) debugStream << "UDP ";
    if (status&0x0008) debugStream << "KSZ-idle ";
    if (status&0x0004) debugStream << "ETH-idle ";
    int waitInfo = status&0x0003;
    if (waitInfo == 0) debugStream << "wait-none";
    else if (waitInfo == 1) debugStream << "wait-ack";
    else if (waitInfo == 2) debugStream << "wait-ack-clear";
    else debugStream << "wait-flush";
    debugStream << std::endl;
}

void EthBasePort::PrintDebugData(std::ostream &debugStream, const quadlet_t *data)
{
    // Following structure must match DebugData in EthernetIO.v
    struct DebugData {
        char     header[4];        // Quad 1
        uint32_t timestampBegin;   // Quad 2
        uint16_t eth_status;       // Quad 3
        uint16_t node_id;
        uint8_t  isFlags;          // Quad 4
        uint8_t  moreFlags;
        uint8_t  nextState;
        uint8_t  state;
        uint16_t RegISROther;      // Quad 5
        uint16_t RegISR;
        uint8_t  count;            // Quad 6
        uint8_t  FrameCount;
        uint16_t unused2233;
        uint8_t  destMac[6];       // Quad 7,8
        uint8_t  srcMac[6];        // Quad 8,9
        uint16_t LengthFW;         // Quad 10
        uint8_t  maxCount;
        uint8_t  unused11;
        uint8_t  hostIP[4];        // Quad 11
        uint8_t  fpgaIP[4];        // Quad 12
        uint16_t rxPktWords;       // Quad 13
        uint16_t ipv4_length;
        uint16_t txPktWords;       // Quad 14
        uint16_t unused4455;
        uint16_t numPacketValid;   // Quad 15
        uint16_t numPacketInvalid;
        uint16_t numIPv4;          // Quad 16
        uint16_t numUDP;
        uint16_t numARP;           // Quad 17
        uint16_t numICMP;
        uint16_t numPacketError;   // Quad 18
        uint16_t numIPv4Mismatch;
        uint16_t numStateInvalid;  // Quad 19
        uint16_t unused0000;
        uint32_t timestampEnd;     // Quad 20
    };
    if (sizeof(DebugData) != 20*sizeof(quadlet_t)) {
        debugStream << "PrintDebugData: structure packing problem" << std::endl;
        return;
    }
    const DebugData *p = reinterpret_cast<const DebugData *>(data);
    if (strncmp(p->header, "DBG0", 4) != 0) {
        debugStream << "Unexpected header string: " << p->header[0] << p->header[1]
                    << p->header[2] << p->header[3] << " (should be DBG0)" << std::endl;
        return;
    }
    debugStream << "TimestampBegin: " << std::hex << p->timestampBegin << std::endl;
    debugStream << "FireWire node_id: " << std::dec << p->node_id << std::endl;
    unsigned short status = static_cast<unsigned short>(p->eth_status&0x0000ffff);
    EthBasePort::PrintDebug(debugStream, status);
    debugStream << "State: " << std::dec << static_cast<uint16_t>(p->state)
                << ", nextState: " << static_cast<uint16_t> (p->nextState) << std::endl;
    debugStream << "Flags: ";
    if (p->moreFlags&0x08) debugStream << "fwPacketFresh ";
    if (p->moreFlags&0x04) debugStream << "isBroadcast ";
    if (p->moreFlags&0x02) debugStream << "isMulticast ";
    if (p->moreFlags&0x01) debugStream << "IRQ ";
    if (p->isFlags&0x80) debugStream << "isForward ";
    if (p->isFlags&0x40) debugStream << "isInIRQ ";
    if (p->isFlags&0x20) debugStream << "sendARP ";
    if (p->isFlags&0x10) debugStream << "isUDP ";
    if (p->isFlags&0x08) debugStream << "isICMP  ";
    if (p->isFlags&0x04) debugStream << "isEcho ";
    if (p->isFlags&0x02) debugStream << "ipv4_long ";
    if (p->isFlags&0x01) debugStream << "ipv4_short ";
    debugStream << std::endl;
    debugStream << "RegISR: " << std::hex << p->RegISR << std::endl;
    debugStream << "RegISROther: " << std::hex << p->RegISROther << std::endl;
    debugStream << "FrameCount: " << std::dec << static_cast<uint16_t>(p->FrameCount) << std::endl;
    debugStream << "Count: " << std::dec << static_cast<uint16_t>(p->count) << std::endl;
    debugStream << "Unused2233: " << std::hex << p->unused2233 << std::endl;
    EthBasePort::PrintMAC(debugStream, "DestMac", p->destMac);
    EthBasePort::PrintMAC(debugStream, "SrcMac", p->srcMac);
    debugStream << "LengthFW: " << std::dec << p->LengthFW << std::endl;
    debugStream << "Unused11: " << std::hex << static_cast<uint16_t>(p->unused11) << std::endl;
    debugStream << "MaxCount: " << std::dec << static_cast<uint16_t>(p->maxCount) << std::endl;
    EthBasePort::PrintIP(debugStream, "HostIP", p->hostIP);
    EthBasePort::PrintIP(debugStream, "FpgaIP", p->fpgaIP);
    debugStream << "rxPktWords: " << std::dec << p->rxPktWords << std::endl;
    debugStream << "ipv4_length: " << std::dec << p->ipv4_length << std::endl;
    debugStream << "Unused4455: " << std::hex << p->unused4455 << std::endl;
    debugStream << "txPktWords: " << std::dec << p->txPktWords << std::endl;
    debugStream << "numPacketValid: " << std::dec << p->numPacketValid << std::endl;
    debugStream << "numPacketInvalid: " << std::dec << p->numPacketInvalid << std::endl;
    debugStream << "numIPv4: " << std::dec << p->numIPv4 << std::endl;
    debugStream << "numUDP: " << std::dec << p->numUDP << std::endl;
    debugStream << "numARP: " << std::dec << p->numARP << std::endl;
    debugStream << "numICMP: " << std::dec << p->numICMP << std::endl;
    debugStream << "numPacketError: " << std::dec << p->numPacketError << std::endl;
    debugStream << "numIPv4Mismatch: " << std::dec << p->numIPv4Mismatch << std::endl;
    debugStream << "numStateInvalid: " << std::dec << p->numStateInvalid << std::endl;
    debugStream << "TimestampEnd: " << std::hex << p->timestampEnd << std::endl;
}

bool EthBasePort::ScanNodes(void)
{
    int node, board;  // loop counters

    // Clear any existing Node2Board
    memset(Node2Board, BoardIO::MAX_BOARDS, sizeof(Node2Board));

    IsAllBoardsBroadcastCapable_ = true;
    IsAllBoardsBroadcastShorterWait_ = false;    // Not yet supported for Ethernet interface
    IsNoBoardsBroadcastShorterWait_ = true;      // Not yet supported for Ethernet interface
    NumOfNodes_ = 0;

    quadlet_t data;
    outStr << "ScanNodes: building node map" << std::endl;
    for (int node = 0; node < BoardIO::MAX_BOARDS; node++)
    {
        // check hardware version
        if (!ReadQuadlet(node, 4, data))
            continue;

        if (data != QLA1_String) {
            outStr << "Node " << node << " is not a QLA board    "
                   << "data = " << std::hex << data << std::endl;
            continue;
        }

        // read firmware version
        unsigned long fver = 0;
        if (!ReadQuadlet(node, 7, data)) {
            outStr << "ScanNodes: unable to read firmware version from node "
                   << node << std::endl;
            continue;
        }
        fver = data;

        // read board id
        if (!ReadQuadlet(node, 0, data)) {
            outStr << "ScanNodes: unable to read status from node " << node << std::endl;
            continue;
        }
        // board_id is bits 27-24, BOARD_ID_MASK = 0x0F000000
        int board = (data & BOARD_ID_MASK) >> 24;
        outStr << "  Node " << node << ", BoardId = " << board
               << ", Firmware Version = " << fver << std::endl;

        if (Node2Board[node] < BoardIO::MAX_BOARDS) {
            outStr << "    Duplicate entry, previous value = "
                   << static_cast<int>(Node2Board[node]) << std::endl;
        }

        Node2Board[node] = static_cast<unsigned char>(board);
        FirmwareVersion[board] = fver;
        BoardExists[board] = true;

        // check firmware version
        // FirmwareVersion >= 4, broadcast capable
        if (fver < 4) IsAllBoardsBroadcastCapable_ = false;
        NumOfNodes_++;
    }
    outStr << "ScanNodes: found " << NumOfNodes_ << " boards" << std::endl;

    // Use broadcast by default if all firmware are bc capable
    if (IsAllBoardsBroadcastCapable_) {
        Protocol_ = BasePort::PROTOCOL_SEQ_R_BC_W;
        outStr << "ScanNodes: all nodes broadcast capable" << std::endl;
    }

    // update Board2Node
    for (board = 0; board < BoardIO::MAX_BOARDS; board++) {
        Board2Node[board] = BoardIO::MAX_BOARDS;
        for (node = 0; node < NumOfNodes_; node++) {
            if (Node2Board[node] == board) {
                if (Board2Node[board] < BoardIO::MAX_BOARDS)
                    outStr << "Warning: GetNodeId detected duplicate board id for " << board << std::endl;
                Board2Node[board] = node;
            }
        }
    }
    return (NumOfNodes_ > 0);
}

// CAN BE MERGED WITH FIREWIREPORT
bool EthBasePort::ReadAllBoards(void)
{
    if (!IsOK()) {
        outStr << "ReadAllBoards: port is not initialized" << std::endl;
        return false;
    }

    if (Protocol_ == BasePort::PROTOCOL_BC_QRW) {
        return ReadAllBoardsBroadcast();
    }

    bool allOK = true;
    bool noneRead = true;
    for (unsigned char bid = 0; bid < BoardIO::MAX_BOARDS; bid++)
    {
        if (BoardList[bid]) {
            bool ret = ReadBlock(bid, 0, BoardList[bid]->GetReadBuffer(),
                                 BoardList[bid]->GetReadNumBytes());
            if (ret) noneRead = false;
            else allOK = false;
            BoardList[bid]->SetReadValid(ret);
        }
    }
    if (noneRead) {
        outStr << "Failed to read any board, check Ethernet physical connection" << std::endl;
    }
    return allOK;
}

// CAN BE MERGED WITH FIREWIREPORT
bool EthBasePort::WriteAllBoards()
{
    if (!IsOK()) {
        outStr << "WriteAllBoards: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }

    if ((Protocol_ == BasePort::PROTOCOL_SEQ_R_BC_W) || (Protocol_ == BasePort::PROTOCOL_BC_QRW)) {
        return WriteAllBoardsBroadcast();
    }

    bool allOK = true;
    bool noneWritten = true;
    for (unsigned char bid = 0; bid < BoardIO::MAX_BOARDS; bid++) {
        if (BoardList[bid]) {
            bool noneWrittenThisBoard = true;
            quadlet_t *buf = BoardList[bid]->GetWriteBufferData();
            unsigned int numBytes = BoardList[bid]->GetWriteNumBytes();
            unsigned int numQuads = numBytes/4;
            // Currently (Rev 1-6 firmware), the last quadlet (Status/Control register)
            // is done as a separate quadlet write.
            bool ret = WriteBlock(bid, 0, buf, numBytes-4);
            if (ret) { noneWritten = false; noneWrittenThisBoard = false; }
            else allOK = false;
            quadlet_t ctrl = buf[numQuads-1];  // Get last quadlet
            bool ret2 = true;
            if (ctrl) {    // if anything non-zero, write it
                ret2 = WriteQuadlet(bid, 0x00, ctrl);
                if (ret2) { noneWritten = false; noneWrittenThisBoard = false; }
                else allOK = false;
            }
            if (noneWrittenThisBoard
                || !(BoardList[bid]->WriteBufferResetsWatchdog())) {
                // send no-op to reset watchdog
                bool ret3 = WriteNoOp(bid);
                if (ret3) noneWritten = false;
            }
            // SetWriteValid clears the buffer if the write was valid
            BoardList[bid]->SetWriteValid(ret&&ret2);
        }
    }
    if (noneWritten)
        outStr << "Failed to write any board, check Ethernet physical connection" << std::endl;
    return allOK;
}

bool EthBasePort::WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data)
{
    // special case of WriteBlockBroadcast
    // nbytes = 4
    data = bswap_32(data);
    return WriteBlockBroadcast(addr, &data, 4);
}

void EthBasePort::PromDelay(void) const
{
    // Wait 1 msec
    Amp1394_Sleep(0.001);
}

// ---------------------------------------------------------
// Protected
// ---------------------------------------------------------

int EthBasePort::ConvertBoardToNode(unsigned char boardId, bool &useEthernetBroadcast, bool &doNotForward) const
{
    int node = MAX_NODES;                  // Invalid value
    useEthernetBroadcast = (boardId&FW_NODE_ETH_BROADCAST_MASK) ? true : false;
    doNotForward = (boardId&FW_NODE_NOFORWARD_MASK) ? true : false;
    boardId = boardId&FW_NODE_MASK;
    if (boardId < BoardIO::MAX_BOARDS)
        node = GetNodeId(boardId);
    else if (boardId == FW_NODE_BROADCAST)
        node = FW_NODE_BROADCAST;
    return node;
}

void EthBasePort::make_1394_header(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tcode,
                                   unsigned int tl, bool doNotForward)
{
    // FFC0 replicates the base node ID when using FireWire on PC. This is followed by a transaction
    // label (arbitrary value that is returned by any resulting FireWire packets) and the transaction code.
    // Finally, we use the PRI field to indicate when a packet should not be forwarded (PRI=1).
    unsigned char fw_pri = doNotForward ? 1 : 0;
    packet[0] = bswap_32((0xFFC0 | (node&FW_NODE_MASK)) << 16 | (tl & 0x003F) << 10 | (tcode & 0x000F) << 4 | (fw_pri & 0x000F));
    // FFFF is used as source ID (most significant 16 bits); not sure if this is needed.
    // This is followed by the destination address, which is 48-bits long
    packet[1] = bswap_32(0xFFFF << 16 | ((addr & 0x0000FFFF00000000) >> 32));
    packet[2] = bswap_32(addr&0xFFFFFFFF);
}

void EthBasePort::make_qread_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tl, bool doNotForward)
{
    make_1394_header(packet, node, addr, EthBasePort::QREAD, tl, doNotForward);
    // CRC
    packet[3] = bswap_32(BitReverse32(crc32(0U, (void*)packet, FW_QREAD_SIZE-FW_CRC_SIZE)));
}

void EthBasePort::make_qwrite_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned int tl, bool doNotForward)
{
    make_1394_header(packet, node, addr, EthBasePort::QWRITE, tl, doNotForward);
    // quadlet data
    packet[3] = bswap_32(data);
    // CRC
    packet[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet, FW_QWRITE_SIZE-FW_CRC_SIZE)));
}

void EthBasePort::make_bread_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int nBytes, unsigned int tl, bool doNotForward)
{
    make_1394_header(packet, node, addr, EthBasePort::BREAD, tl, doNotForward);
    packet[3] = bswap_32((nBytes & 0xffff) << 16);
    // CRC
    packet[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet, FW_BREAD_SIZE-FW_CRC_SIZE)));
}

void EthBasePort::make_bwrite_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, quadlet_t *data, unsigned int nBytes, unsigned int tl, bool doNotForward)
{
    make_1394_header(packet, node, addr, EthBasePort::BWRITE, tl, doNotForward);
    packet[3] = bswap_32((nBytes & 0xffff) << 16);
    // header CRC
    packet[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet, FW_BWRITE_HEADER_SIZE-FW_CRC_SIZE)));
    // Now, copy the data. We first check if the copy is needed.
    size_t data_offset = FW_BWRITE_HEADER_SIZE/sizeof(quadlet_t);
    if (data != &packet[data_offset])
        memcpy(&packet[data_offset], data, nBytes);
    // Now, compute the data CRC
    size_t data_crc_offset = data_offset + nBytes/sizeof(quadlet_t);
    packet[data_crc_offset] = bswap_32(BitReverse32(crc32(0U, static_cast<void *>(packet+data_offset), nBytes)));
}

bool EthBasePort::checkCRC(const unsigned char *packet)
{
    // Eliminate CRC checking of FireWire packets received via Ethernet
    // because Ethernet already includes CRC.
#if 0
    uint32_t crc_check = BitReverse32(crc32(0U,(void*)(packet+14),16));
    uint32_t crc_original = bswap_32(*((uint32_t*)(packet+30)));
    return (crc_check == crc_original);
#else
    return true;
#endif
}

//  -----------  CRC ----------------
//source: http://www.opensource.apple.com/source/xnu/xnu-1456.1.26/bsd/libkern/crc32.c
//online check: http://www.lammertbies.nl/comm/info/crc-calculation.html
static uint32_t crc32_tab[] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3,	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de,	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,	0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5,	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,	0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940,	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,	0x76dc4190, 0x01db7106,
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};
static const unsigned char BitReverseTable[] =
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};


uint32_t BitReverse32(uint32_t input)
{
    unsigned char inputs[4];
    inputs[0] = BitReverseTable[input & 0x000000ff];
    inputs[1] = BitReverseTable[(input & 0x0000ff00)>>8];
    inputs[2] = BitReverseTable[(input & 0x00ff0000)>>16];
    inputs[3] = BitReverseTable[(input & 0xff000000)>>24];
    uint32_t output = 0x00000000;
    output |= (uint32_t)inputs[0] << 24;
    output |= (uint32_t)inputs[1] << 16;
    output |= (uint32_t)inputs[2] << 8;
    output |= (uint32_t)inputs[3];
    return output;
}


// The sample use of CRC
// crc = BitReverse32(crc32(0U,(void*)array_char,len_in_byte));
// It is also needed to be byteSwaped before putting into stream
uint32_t crc32(uint32_t crc, const void *buf, size_t size)
{
    const uint8_t *p;

    p = (uint8_t*)buf;
    crc = crc ^ ~0U;

    while (size--)
        crc = crc32_tab[(crc ^ BitReverseTable[*p++]) & 0xFF] ^ (crc >> 8);

  return crc ^ ~0U;
}
