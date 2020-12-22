/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen

  (C) Copyright 2014-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <stdio.h>
#include <string>

#include <Amp1394/AmpIORevision.h>
#include "BasePort.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"

BasePort::BasePort(int portNum, std::ostream &ostr):
        outStr(ostr),
        Protocol_(BasePort::PROTOCOL_SEQ_RW),
        IsAllBoardsBroadcastCapable_(false),
        IsAllBoardsBroadcastShorterWait_(false),
        IsNoBoardsBroadcastShorterWait_(true),
        IsAllBoardsRev7_(false),
        IsNoBoardsRev7_(false),
        ReadSequence_(0),
        ReadErrorCounter_(0),
        PortNum(portNum),
        FwBusGeneration(0),
        newFwBusGeneration(0),
        autoReScan(true),
        NumOfNodes_(0),
        NumOfBoards_(0),
        BoardInUseMask_(0),
        max_board(0),
        HubBoard(BoardIO::MAX_BOARDS)
{
    size_t i;
    for (i = 0; i < BoardIO::MAX_BOARDS; i++) {
        BoardList[i] = 0;
        ReadBuffer[i] = 0;
        WriteBuffer[i] = 0;
        FirmwareVersion[i] = 0;
        Board2Node[i] = MAX_NODES;
    }
    ReadBufferBroadcast = 0;
    WriteBufferBroadcast = 0;
    GenericBuffer = 0;
    for (i = 0; i < MAX_NODES; i++)
        Node2Board[i] = BoardIO::MAX_BOARDS;
}

BasePort::~BasePort()
{
    for (size_t i = 0; i < BoardIO::MAX_BOARDS; i++) {
        delete [] ReadBuffer[i];
        delete [] WriteBuffer[i];
    }
    delete [] ReadBufferBroadcast;
    delete [] WriteBufferBroadcast;
    delete [] GenericBuffer;
}

bool BasePort::SetProtocol(ProtocolType prot) {
    if (prot != BasePort::PROTOCOL_SEQ_RW) {
        if (!IsAllBoardsBroadcastCapable_) {
            outStr << "***Error: not all boards support broadcasting, " << std::endl
                   << "          please upgrade your firmware"  << std::endl;
            return false;
        }
        if (!(IsAllBoardsRev7_ || IsNoBoardsRev7_)) {
            outStr << "***Error: cannot use broadcast mode with mix of Rev 7 and older boards, " << std::endl
                   << "          please upgrade your firmware to Rev 7" << std::endl;
            return false;
        }
    }
    switch (prot) {
        case BasePort::PROTOCOL_SEQ_RW:
            outStr << "System running in NON broadcast mode" << std::endl;
            if (Protocol_ != prot) {
                SetReadBuffer();
                SetWriteBuffer();
            }
            Protocol_ = prot;
            break;
        case BasePort::PROTOCOL_SEQ_R_BC_W:
            outStr << "System running with broadcast write" << std::endl;
            if (Protocol_ != prot) {
                SetReadBuffer();
                SetWriteBufferBroadcast();
                if (IsNoBoardsRev7_)
                    SetWriteBuffer();
            }
            Protocol_ = prot;
            break;
        case BasePort::PROTOCOL_BC_QRW:
            outStr << "System running with broadcast query, read, and write" << std::endl;
            if (Protocol_ != prot) {
                SetReadBufferBroadcast();
                SetWriteBufferBroadcast();
                if (IsNoBoardsRev7_)
                    SetWriteBuffer();
            }
            Protocol_ = prot;
            break;
        default:
            outStr << "Unknown protocol (ignored): " << prot << std::endl;
            break;
    }
    return (Protocol_ == prot);
}

void BasePort::SetGenericBuffer(void)
{
    if (!GenericBuffer) {
        size_t maxWritePacket = GetWriteQuadAlign()+GetPrefixOffset(WR_FW_BDATA)+GetMaxWriteDataSize()+GetWritePostfixSize();
        size_t maxReadPacket = GetReadQuadAlign()+GetPrefixOffset(RD_FW_BDATA)+GetMaxReadDataSize()+GetReadPostfixSize();
        GenericBuffer = reinterpret_cast<unsigned char *>(new quadlet_t[std::max(maxWritePacket,maxReadPacket)/sizeof(quadlet_t)]);
    }
}

void BasePort::SetReadBuffer(void)
{
    for (size_t i = 0; i < BoardIO::MAX_BOARDS; i++) {
        BoardIO *board = BoardList[i];
        if (board)
            board->SetReadBuffer(reinterpret_cast<quadlet_t *>(ReadBuffer[i]+GetReadQuadAlign()+GetPrefixOffset(RD_FW_BDATA)));
    }
}

void BasePort::SetWriteBuffer(void)
{
    for (size_t i = 0; i < BoardIO::MAX_BOARDS; i++) {
        BoardIO *board = BoardList[i];
        if (board)
            board->SetWriteBuffer(reinterpret_cast<quadlet_t *>(WriteBuffer[i]+GetWriteQuadAlign()+GetPrefixOffset(WR_FW_BDATA)));
    }
}

void BasePort::SetReadBufferBroadcast(void)
{
    if (!ReadBufferBroadcast) {
        size_t numReadBytes = GetReadQuadAlign()+GetPrefixOffset(RD_FW_BDATA)+GetMaxReadDataSize()+GetReadPostfixSize();
        quadlet_t *buf = new quadlet_t[numReadBytes/sizeof(quadlet_t)];
        ReadBufferBroadcast = reinterpret_cast<unsigned char *>(buf);
    }
}

void BasePort::SetWriteBufferBroadcast(void)
{
    if (!WriteBufferBroadcast) {
        size_t numWriteBytes = GetWriteQuadAlign()+GetPrefixOffset(WR_FW_BDATA)+GetMaxWriteDataSize()+GetWritePostfixSize();
        quadlet_t *buf = new quadlet_t[numWriteBytes/sizeof(quadlet_t)];
        WriteBufferBroadcast = reinterpret_cast<unsigned char *>(buf);
    }
    if (IsAllBoardsRev7_) {
        unsigned char *curPtr = WriteBufferBroadcast+GetWriteQuadAlign()+GetPrefixOffset(WR_FW_BDATA);
        for (size_t i = 0; i < BoardIO::MAX_BOARDS; i++) {
            BoardIO *board = BoardList[i];
            if (board) {
                board->SetWriteBuffer(reinterpret_cast<quadlet_t *>(curPtr));
                curPtr += board->GetWriteNumBytes();
            }
        }
    }
}

void BasePort::SetReadInvalid(void)
{
    for (unsigned int boardNum = 0; boardNum < max_board; boardNum++) {
        BoardIO *board = BoardList[boardNum];
        if (board)
            board->SetReadValid(false);
    }
}

void BasePort::Reset(void)
{
    Cleanup();
    Init();
}

bool BasePort::ScanNodes(void)
{
    unsigned int board;
    nodeid_t node;

    // Clear any existing Node2Board
    memset(Node2Board, BoardIO::MAX_BOARDS, sizeof(Node2Board));

    IsAllBoardsBroadcastCapable_ = true;
    IsAllBoardsBroadcastShorterWait_ = true;
    IsNoBoardsBroadcastShorterWait_ = true;
    IsAllBoardsRev7_ = true;
    IsNoBoardsRev7_ = true;
    NumOfNodes_ = 0;

    nodeid_t max_nodes = InitNodes();

    outStr << "ScanNodes: building node map for " << max_nodes << " nodes:" << std::endl;
    // Iterate through all possible nodes
    for (node = 0; node < max_nodes; node++) {
        quadlet_t data;
        // check hardware version
        if (!ReadQuadletNode(node, 4, data)) {
            if (GetPortType() == PORT_FIREWIRE)
                outStr << "ScanNodes: unable to read from node " << node << std::endl;
            continue;
        }
        if (data != QLA1_String) {
            outStr << "ScanNodes: node " << node << " is not a QLA board (data = "
                   << std::hex << data << ")" << std::endl;
            continue;
        }

        // read firmware version
        unsigned long fver = 0;
        if (!ReadQuadletNode(node, 7, data)) {
            outStr << "ScanNodes: unable to read firmware version from node "
                   << node << std::endl;
            continue;
        }
        fver = data;

        // read board id
        if (!ReadQuadletNode(node, 0, data)) {
            outStr << "ScanNodes: unable to read status from node " << node << std::endl;
            continue;
        }
        // board_id is bits 27-24, BOARD_ID_MASK = 0x0F000000
        board = (data & BOARD_ID_MASK) >> 24;
        outStr << "  Node " << node << ", BoardId = " << board
               << ", Firmware Version = " << fver << std::endl;

        if (Node2Board[node] < BoardIO::MAX_BOARDS) {
            outStr << "    Duplicate entry, previous value = "
                   << static_cast<int>(Node2Board[node]) << std::endl;
        }

        Node2Board[node] = static_cast<unsigned char>(board);
        FirmwareVersion[board] = fver;

        // check firmware version
        // FirmwareVersion >= 4, broadcast capable
        if (fver < 4) IsAllBoardsBroadcastCapable_ = false;
        // FirmwareVersion >= 6, broadcast with possibly shorter wait (i.e., skipping nodes
        // on the bus that are not part of the current configuration).
        if (fver < 6) IsAllBoardsBroadcastShorterWait_ = false;
        else          IsNoBoardsBroadcastShorterWait_ = false;
        if (fver < 7) IsAllBoardsRev7_ = false;
        else          IsNoBoardsRev7_ = false;
        NumOfNodes_++;
    }
    outStr << "ScanNodes: found " << NumOfNodes_ << " boards" << std::endl;

    // update Board2Node
    for (board = 0; board < BoardIO::MAX_BOARDS; board++) {
        Board2Node[board] = MAX_NODES;
        // search up to max_nodes (not NumOfNodes_) in case nodes are not sequential (i.e., if some nodes
        // are not associated with valid boards).
        for (node = 0; node < max_nodes; node++) {
            if (Node2Board[node] == board) {
                if (Board2Node[board] < MAX_NODES)
                    outStr << "ScanNodes: warning: duplicate node id for board " << board
                           << "(" << Board2Node[board] << ", " << node << ")" << std::endl;
                Board2Node[board] = node;
            }
        }
    }

    return (NumOfNodes_ > 0);
}

void BasePort::SetDefaultProtocol(void)
{
    Protocol_ = BasePort::PROTOCOL_SEQ_RW;
    // Use broadcast by default if all firmware are bc capable
    if ((NumOfNodes_ > 0) && IsAllBoardsBroadcastCapable_) {
        if (IsAllBoardsRev7_ || IsNoBoardsRev7_) {
            Protocol_ = BasePort::PROTOCOL_SEQ_R_BC_W;
            if (IsAllBoardsBroadcastShorterWait_)
                outStr << "All nodes broadcast capable and support shorter wait" << std::endl;
            else if (IsNoBoardsBroadcastShorterWait_)
                outStr << "All nodes broadcast capable and do not support shorter wait" << std::endl;
            else
                outStr << "All nodes broadcast capable and some support shorter wait" << std::endl;
        }
        else
            outStr << "All nodes broadcast capable, but disabled due to mix of Rev 7 and older firmware" << std::endl;
    }
}

bool BasePort::AddBoard(BoardIO *board)
{
    unsigned int id = board->BoardId;
    if (id >= BoardIO::MAX_BOARDS) {
        outStr << "AddBoard: board number out of range: " << id << std::endl;
        return false;
    }
    BoardList[id] = board;
    board->port = this;
    // We allocate the individual buffers, even if the current protocol would not use them
    // (e.g., a broadcast protocol), because it simplifies the logic, especially if the protocol
    // is changed later.
    delete [] ReadBuffer[id];
    size_t numReadBytes = GetReadQuadAlign()+GetPrefixOffset(RD_FW_BDATA)+board->GetReadNumBytes()+GetReadPostfixSize();
    quadlet_t *rdBuf = new quadlet_t[numReadBytes/sizeof(quadlet_t)];
    ReadBuffer[id] = reinterpret_cast<unsigned char *>(rdBuf);
    delete [] WriteBuffer[id];
    size_t numWriteBytes = GetWriteQuadAlign()+GetPrefixOffset(WR_FW_BDATA)+board->GetWriteNumBytes()+GetWritePostfixSize();
    quadlet_t *wrBuf = new quadlet_t[numWriteBytes/sizeof(quadlet_t)];
    WriteBuffer[id] = reinterpret_cast<unsigned char *>(wrBuf);

    if (id >= max_board)
        max_board = id+1;
    // update BoardInUseMask_
    BoardInUseMask_ = (BoardInUseMask_ | (1 << id));

    NumOfBoards_++;   // increment board counts

    // Now, set the BoardIO buffers based on the current protocol. Note that we have to do this here
    // and in SetProtocol because we do not make any assumptions about which is called last.
    if (Protocol_ == BasePort::PROTOCOL_SEQ_RW) {
        board->SetReadBuffer(reinterpret_cast<quadlet_t *>(ReadBuffer[id]+GetReadQuadAlign()+GetPrefixOffset(RD_FW_BDATA)));
        board->SetWriteBuffer(reinterpret_cast<quadlet_t *>(WriteBuffer[id]+GetWriteQuadAlign()+GetPrefixOffset(WR_FW_BDATA)));
    }
    else if (Protocol_ == BasePort::PROTOCOL_SEQ_R_BC_W) {
        board->SetReadBuffer(reinterpret_cast<quadlet_t *>(ReadBuffer[id]+GetReadQuadAlign()+GetPrefixOffset(RD_FW_BDATA)));
        SetWriteBufferBroadcast();
        if (IsNoBoardsRev7_)
            board->SetWriteBuffer(reinterpret_cast<quadlet_t *>(WriteBuffer[id]+GetWriteQuadAlign()+GetPrefixOffset(WR_FW_BDATA)));
    }
    else if (Protocol_ == BasePort::PROTOCOL_BC_QRW) {
        SetReadBufferBroadcast();
        SetWriteBufferBroadcast();
        if (IsNoBoardsRev7_)
            board->SetWriteBuffer(reinterpret_cast<quadlet_t *>(WriteBuffer[id]+GetWriteQuadAlign()+GetPrefixOffset(WR_FW_BDATA)));
    }

    return true;
}

bool BasePort::RemoveBoard(unsigned char boardId)
{
    if (boardId >= BoardIO::MAX_BOARDS) {
        outStr << "RemoveBoard: board number out of range: " << static_cast<unsigned int>(boardId) << std::endl;
        return false;
    }
    BoardIO *board = BoardList[boardId];
    if (!board) {
        outStr << "RemoveBoard: board not found: " << static_cast<unsigned int>(boardId) << std::endl;
        return false;
    }

    // update BoardInuseMask_
    BoardInUseMask_ = (BoardInUseMask_ & (~(1 << boardId)));

    // set board to use internal buffers
    board->SetReadBuffer(0);
    board->SetWriteBuffer(0);

    BoardList[boardId] = 0;
    board->port = 0;
    delete [] ReadBuffer[boardId];
    ReadBuffer[boardId] = 0;
    delete [] WriteBuffer[boardId];
    WriteBuffer[boardId] = 0;
    NumOfBoards_--;

    if (boardId >= max_board-1) {
        // If max_board was just removed, find the new max_board
        max_board = 0;
        for (int bd = 0; bd < boardId; bd++)
            if (BoardList[bd]) max_board = bd+1;
    }
    return true;
}

std::string BasePort::PortTypeString(PortType portType)
{
    if (portType == PORT_FIREWIRE)
        return std::string("Firewire");
    else if (portType == PORT_ETH_RAW)
        return std::string("Ethernet-Raw");
    else if (portType == PORT_ETH_UDP)
        return std::string("Ethernet-UDP");
    else
        return std::string("Unknown");
}

// Helper function for parsing command line options.
// In particular, this is typically called after a certain option, such as -p, is
// recognized and it parses the rest of that option string:
// N               for FireWire, where N is the port number (backward compatibility)
// fwN             for FireWire, where N is the port number
// ethN            for raw Ethernet (PCAP), where N is the port number
// udpxx.xx.xx.xx  for UDP, where xx.xx.xx.xx is the (optional) server IP address
bool BasePort::ParseOptions(const char *arg, PortType &portType, int &portNum, std::string &IPaddr,
                            std::ostream &ostr)
{
    // no option, using default
    if ((arg == 0) || (strlen(arg) == 0)) {
        ostr << "ParseOptions: no option provided, using default "
             << DefaultPort() << std::endl;
        return ParseOptions(DefaultPort().c_str(), portType, portNum, IPaddr);
    }
    // expecting proper options
    if (strncmp(arg, "fw", 2) == 0) {
        portType = PORT_FIREWIRE;
        // no port specified
        if (strlen(arg) == 2) {
            portNum = 0;
            return true;
        }
        // make sure separator is here
        if (arg[2] != ':') {
            ostr << "ParseOptions: missing \":\" after \"fw\"" << std::endl;
            return false;
        }
        // scan port number
        if (sscanf(arg+3, "%d", &portNum) == 1) {
            return true;
        }
        ostr << "ParseOptions: failed to find a port number after \"fw:\" in " << arg+3 << std::endl;
        return false;
    }
    else if (strncmp(arg, "eth", 3) == 0) {
        portType = PORT_ETH_RAW;
        return (sscanf(arg+4, "%d", &portNum) == 1);
    }
    else if (strncmp(arg, "udp", 3) == 0) {
        portType = PORT_ETH_UDP;
        // no option specified
        if (strlen(arg) == 3) {
            IPaddr = "169.254.0.100";
            return true;
        }
        // make sure separator is here
        if (arg[3] != ':') {
            ostr << "ParseOptions: missing \":\" after \"udp\"" << std::endl;
            return false;
        }
        // For now, if at least 8 characters, assume a valid IP address
        if (strlen(arg+4) >= 8)
            IPaddr.assign(arg+4);
        else if (strlen(arg+4) > 0)
            sscanf(arg+4, "%d", &portNum);  // TEMP: portNum==1 for UDP means set eth1394 mode
        return true;
    }
    // older default, fw and looking for port number
    portType = PORT_FIREWIRE;
    // scan port number
    if (sscanf(arg, "%d", &portNum) == 1) {
        return true;
    }
    ostr << "ParseOptions: failed to find a FireWire port number in " << arg << std::endl;
    return false;
}

std::string BasePort::DefaultPort(void)
{
#if Amp1394_HAS_RAW1394
    return "fw:0";
#else
    return "udp:169.254.0.100";
#endif

}

nodeid_t BasePort::ConvertBoardToNode(unsigned char boardId) const
{
    nodeid_t node = MAX_NODES;                  // Invalid value
    boardId = boardId&FW_NODE_MASK;
    if (boardId < BoardIO::MAX_BOARDS)
        node = GetNodeId(boardId);
    else if (boardId == FW_NODE_BROADCAST)
        node = FW_NODE_BROADCAST;
    return node;
}

bool BasePort::CheckFwBusGeneration(const std::string &caller, bool doScan)
{
    bool ret = (FwBusGeneration == newFwBusGeneration);
    if (!ret) {
        outStr << caller << ": Firewire bus reset, old generation = " << FwBusGeneration
               << ", new generation = " << newFwBusGeneration << std::endl;
        if (doScan)
            ret = ReScanNodes(caller);
    }
    return ret;
}

bool BasePort::ReScanNodes(const std::string &caller)
{
    unsigned int oldFwBusGeneration = FwBusGeneration;
    UpdateBusGeneration(newFwBusGeneration);
    bool ret = ScanNodes();
    if (!ret) {
        outStr << caller << ": failed to rescan nodes" << std::endl;
        UpdateBusGeneration(oldFwBusGeneration);
    }
    return ret;
}

bool BasePort::ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data)
{
    nodeid_t node = ConvertBoardToNode(boardId);
    return (node < MAX_NODES) ? ReadQuadletNode(node, addr, data, boardId&FW_NODE_MASK) : false;
}

bool BasePort::WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data)
{
    nodeid_t node = ConvertBoardToNode(boardId);
    return (node < MAX_NODES) ? WriteQuadletNode(node, addr, data, boardId&FW_NODE_FLAGS_MASK) : false;
}

bool BasePort::ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata,
                             unsigned int nbytes)
{
    if (nbytes == 4)
        return ReadQuadlet(boardId, addr, *rdata);
    else if ((nbytes == 0) || ((nbytes%4) != 0)) {
        outStr << "ReadBlock: illegal size (" << nbytes << "), must be multiple of 4" << std::endl;
        return false;
    }
    else if (nbytes > GetMaxReadDataSize()) {
        outStr << "ReadBlock: packet size " << std::dec << nbytes << " too large (max = "
               << GetMaxReadDataSize() << " bytes)" << std::endl;
        return false;
    }

    nodeid_t node = ConvertBoardToNode(boardId);
    return (node < MAX_NODES) ? ReadBlockNode(node, addr, rdata, nbytes, boardId&FW_NODE_FLAGS_MASK) : false;
}

bool BasePort::WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata,
                              unsigned int nbytes)
{
    if (nbytes == 4) {
        return WriteQuadlet(boardId, addr, *wdata);
    }
    else if ((nbytes == 0) || ((nbytes%4) != 0)) {
        outStr << "WriteBlock: illegal size (" << nbytes << "), must be multiple of 4" << std::endl;
        return false;
    }
    else if (nbytes > GetMaxWriteDataSize()) {
        outStr << "WriteBlock: packet size " << std::dec << nbytes << " too large (max = "
               << GetMaxWriteDataSize() << " bytes)" << std::endl;
        return false;
    }

    nodeid_t node = ConvertBoardToNode(boardId);
    return (node < MAX_NODES) ? WriteBlockNode(node, addr, wdata, nbytes, boardId&FW_NODE_FLAGS_MASK) : false;
}

bool BasePort::ReadAllBoards(void)
{
    if (!IsOK()) {
        outStr << "ReadAllBoards: port not initialized" << std::endl;
        OnNoneRead();
        return false;
    }

    if (Protocol_ == BasePort::PROTOCOL_BC_QRW) {
        return ReadAllBoardsBroadcast();
    }

    if (!CheckFwBusGeneration("ReadAllBoards", autoReScan)) {
        SetReadInvalid();
        OnNoneRead();
        return false;
    }

    bool allOK = true;
    bool noneRead = true;

    bool rtRead = true;
    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            bool ret = ReadBlock(board, 0, BoardList[board]->GetReadBuffer(), BoardList[board]->GetReadNumBytes());
            if (ret) {
                noneRead = false;
            } else {
                allOK = false;
            }
            BoardList[board]->SetReadValid(ret);

            if (!ret) {
                if (ReadErrorCounter_ == 0) {
                    outStr << "ReadAllBoards: read failed on port "
                           << PortNum << ", board " << board << std::endl;
                }
                ReadErrorCounter_++;
                if (ReadErrorCounter_ == 10000) {
                    outStr << "ReadAllBoards: read failed on port "
                           << PortNum << ", board " << board << " occurred 10,000 times" << std::endl;
                    ReadErrorCounter_ = 0;
                }
            } else {
                ReadErrorCounter_ = 0;
            }
        }
    }
    if (!rtRead)
        outStr << "ReadAllBoards: rtRead is false" << std::endl;

    if (noneRead) {
        OnNoneRead();
    }
    return allOK;
}

bool BasePort::ReadAllBoardsBroadcast(void)
{
    if (!IsOK()) {
        outStr << "ReadAllBoardsBroadcast: port not initialized" << std::endl;
        OnNoneRead();
        return false;
    }

    if (!(IsAllBoardsRev7_||IsNoBoardsRev7_)) {
        outStr << "ReadAllBoardsBroadcast: invalid mix of firmware" << std::endl;
        OnNoneRead();
        return false;
    }

    if (!CheckFwBusGeneration("ReadAllBoardsBroadcast", autoReScan)) {
        SetReadInvalid();
        OnNoneRead();
        return false;
    }

    bool allOK = true;
    bool noneRead = true;

    //--- send out broadcast read request -----

    bool rtRead = true;
    // sequence number from 16 bits 0 to 65535
    ReadSequence_++;
    if (ReadSequence_ == 65536) {
        ReadSequence_ = 1;
    }

    if (!WriteBroadcastReadRequest(ReadSequence_)) {
        outStr << "ReadAllBoardsBroadcast: failed to send broadcast read request, seq = " << ReadSequence_ << std::endl;
        OnNoneRead();
        return false;
    }

    // Wait for broadcast read data
    WaitBroadcastRead();

    int readSize;        // Block size per board (depends on firmware version)
    if (IsNoBoardsRev7_)
        readSize = 17;   // Rev 1-6: 1 seq + 16 data, unit quadlet (should actually be 1 seq + 20 data)
    else
        readSize = 29;   // Rev 7: 1 seq + 28 data, unit quadlet (Rev 7)

    int hubReadSize;        // Actual read size (depends on firmware version)
    if (IsNoBoardsRev7_)
        hubReadSize = BoardIO::MAX_BOARDS*readSize;  // Rev 1-6: 16 * 17 = 272 max (though really should have been 16*21)
    else
        hubReadSize = readSize*NumOfBoards_;         // Rev 7: NumOfBoards * 29

    quadlet_t *hubReadBuffer = reinterpret_cast<quadlet_t *>(ReadBufferBroadcast + GetReadQuadAlign() + GetPrefixOffset(RD_FW_BDATA));
    //memset(hubReadBuffer, 0, hubReadSize*sizeof(quadlet_t));

    bool ret = ReadBlock(HubBoard, 0x1000, hubReadBuffer, hubReadSize*sizeof(quadlet_t));

    if (!ret) {
        SetReadInvalid();
        OnNoneRead();
        return false;
    }

    quadlet_t *curPtr = hubReadBuffer;
    unsigned boardMax = IsAllBoardsRev7_ ? NumOfBoards_ : max_board;
    for (unsigned int boardNum = 0; boardNum < boardMax; boardNum++) {
        unsigned int seq = (bswap_32(curPtr[0]) >> 16);
        quadlet_t statusQuad = bswap_32(curPtr[2]);
        unsigned int numAxes = (statusQuad&0xf0000000)>>28;
        unsigned int thisBoard = (statusQuad&0x0f000000)>>24;
        BoardIO *board = 0;
        if (numAxes == 4) {
            if (IsAllBoardsRev7_) {
                // Should check against BoardInUseMask_
                board = BoardList[thisBoard];
            }
            else {
                board = BoardList[boardNum];
                if (board) {
                    if (board->GetBoardId() != thisBoard) {
                        outStr << "Board mismatch: expecting " << static_cast<unsigned int>(board->GetBoardId())
                               << ", found " << thisBoard << std::endl;
                        board = 0;
                    }
                }
            }
        }
        else {
            outStr << "Invalid status (not a 4 axis board): " << std::hex << statusQuad << std::dec << std::endl;
        }
        if (board) {
            if (seq == ReadSequence_) {
               board->SetReadBuffer(curPtr+1);
               board->SetReadValid(true);
               noneRead = false;
               allOK = true; // PK TEMP
            }
            else {
               outStr << "Board " << thisBoard << ", seq = " << seq << ", expected = " << ReadSequence_ << std::endl;
               board->SetReadValid(false);
               allOK = false; // PK TEMP
            }
        }
        curPtr += readSize;
    }

#if 0
    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            unsigned int boardOffset = IsNoBoardsRev7_ ? (readSize*board) : (readSize*curIndex);
            curIndex++;
            unsigned int seq = (bswap_32(hubReadBuffer[boardOffset+0]) >> 16);
            if (IsAllBoardsRev7_) {
                // Sanity check on board number (from status register)
                unsigned int thisBoard = (bswap_32(hubReadBuffer[boardOffset+2])&0x0f000000)>>24;
                if (board != thisBoard)
                    outStr << "board mismatch: expecting " << board << ", found " << thisBoard << std::endl;
            }

            static int errorcounter = 0;
            if (ReadSequence_ != seq) {
                errorcounter++;
                outStr << "block read error: counter = " << std::dec << errorcounter << ", read = "
                       << seq << ", expected = " << ReadSequence_ << ", board =  " << (int)board << std::endl;
            }
            BoardList[board]->SetReadBuffer(&(hubReadBuffer[boardOffset+1]));
            BoardList[board]->SetReadValid(ret);
            if (ret) noneRead = false;
            else allOK = false;
        }
    }
#endif

    if (noneRead) {
        OnNoneRead();
    }
    if (!rtRead)
        outStr << "ReadAllBoardsBroadcast: rtRead is false" << std::endl;

    return allOK;
}

bool BasePort::WriteAllBoards(void)
{
    if (!IsOK()) {
        outStr << "WriteAllBoards: port not initialized" << std::endl;
        OnNoneWritten();
        return false;
    }

    if ((Protocol_ == BasePort::PROTOCOL_SEQ_R_BC_W) || (Protocol_ == BasePort::PROTOCOL_BC_QRW)) {
        return WriteAllBoardsBroadcast();
    }

    if (!CheckFwBusGeneration("WriteAllBoards", autoReScan)) {
        OnNoneWritten();
        return false;
    }

    rtWrite = true;   // for debugging
    bool allOK = true;
    bool noneWritten = true;
    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            quadlet_t *buf = BoardList[board]->GetWriteBuffer();
            unsigned int numBytes = BoardList[board]->GetWriteNumBytes();
            if (FirmwareVersion[board] < 7) {
                // Rev 1-6 firmware: the last quadlet (Status/Control register)
                // is done as a separate quadlet write.
                unsigned int numQuads = numBytes/4;
                bool noneWrittenThisBoard = true;
                bool ret = WriteBlock(board, 0, buf, numBytes-4);
                if (ret) { noneWritten = false; noneWrittenThisBoard = false; }
                else allOK = false;
                quadlet_t ctrl = buf[numQuads-1];  // Get last quadlet
                bool ret2 = true;
                if (ctrl) {    // if anything non-zero, write it
                    ret2 = WriteQuadlet(board, 0, ctrl);
                    if (ret2) { noneWritten = false; noneWrittenThisBoard = false; }
                    else allOK = false;
                }
                if (noneWrittenThisBoard
                    || !(BoardList[board]->WriteBufferResetsWatchdog())) {
                    // send no-op to reset watchdog
                    bool ret3 = WriteNoOp(board);
                    if (ret3) noneWritten = false;
                }
                BoardList[board]->SetWriteValid(ret&&ret2);
                // Initialize (clear) the write buffer
                BoardList[board]->InitWriteBuffer();
            }
            else {
                // Rev 7 firmware: write DAC (x4) and Status/Control register
                bool ret = WriteBlock(board, 0, buf, numBytes);
                if (ret) noneWritten = false;
                else allOK = false;
                BoardList[board]->SetWriteValid(ret);
                // Initialize (clear) the write buffer
                BoardList[board]->InitWriteBuffer();
            }
            // Check for data collection callback
            BoardList[board]->CheckCollectCallback();
        }
    }
    if (noneWritten) {
        OnNoneWritten();
    }
    if (!rtWrite)
        outStr << "WriteAllBoards: rtWrite is false" << std::endl;
    return allOK;
}

bool BasePort::WriteAllBoardsBroadcast(void)
{
    if (!IsOK()) {
        outStr << "WriteAllBoardsBroadcast: port not initialized" << std::endl;
        OnNoneWritten();
        return false;
    }

    if (!(IsAllBoardsRev7_||IsNoBoardsRev7_)) {
        outStr << "WriteAllBoardsBroadcast: invalid mix of firmware" << std::endl;
        OnNoneWritten();
        return false;
    }

    if (!CheckFwBusGeneration("WriteAllBoardsBroadcast", autoReScan)) {
        OnNoneWritten();
        return false;
    }

    bool rtWrite = true;   // for debugging

    // sanity check vars
    bool allOK = true;
    bool noneWritten = true;

    // construct broadcast write buffer
    quadlet_t *bcBuffer = reinterpret_cast<quadlet_t *>(WriteBufferBroadcast + GetWriteQuadAlign() + GetPrefixOffset(WR_FW_BDATA));

    int bcBufferOffset = 0; // the offset for new data to be stored in bcBuffer (bytes)
    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            unsigned int numBytes = BoardList[board]->GetWriteNumBytes();
            quadlet_t *buf = BoardList[board]->GetWriteBuffer();
            quadlet_t *bcPtr = bcBuffer+bcBufferOffset/sizeof(quadlet_t);
            if (IsNoBoardsRev7_) {
                numBytes -= 4;   // -4 for ctrl offset
                memcpy(bcPtr, buf, numBytes);
            }
            else {
                // For firmware Rev 7, the data should already be in WriteBufferBroadcast,
                // but we check to be sure.
                if (buf != bcPtr) {
                    rtWrite = false;
                    memcpy(bcPtr, buf, numBytes);
                }
            }
            // bcBufferOffset equals total numBytes to write, when the loop ends
            bcBufferOffset = bcBufferOffset + numBytes;
        }
    }

    // now broadcast out the huge packet
    bool ret;

    ret = WriteBroadcastOutput(bcBuffer, bcBufferOffset);

    // Send out control quadlet if necessary (firmware prior to Rev 7);
    //    also check for data collection
    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            if (FirmwareVersion[board] < 7) {
                bool noneWrittenThisBoard = true;
                quadlet_t *buf = BoardList[board]->GetWriteBuffer();
                unsigned int numBytes = BoardList[board]->GetWriteNumBytes();
                unsigned int numQuads = numBytes/4;
                quadlet_t ctrl = buf[numQuads-1];  // Get last quadlet
                bool ret2 = true;
                if (ctrl) {  // if anything non-zero, write it
                    ret2 = WriteQuadlet(board, 0x00, ctrl);
                    if (ret2) { noneWritten = false; noneWrittenThisBoard = false; }
                    else allOK = false;
                }
                if (noneWrittenThisBoard
                    && !(BoardList[board]->WriteBufferResetsWatchdog())) {
                    // send no-op to reset watchdog
                    bool ret3 = WriteNoOp(board);
                    if (ret3) noneWritten = false;
                }
                BoardList[board]->SetWriteValid(ret&&ret2);
                // Initialize (clear) the write buffer
                BoardList[board]->InitWriteBuffer();
            }
            else {
                if (ret) noneWritten = false;
                BoardList[board]->SetWriteValid(ret);
                // Initialize (clear) the write buffer
                BoardList[board]->InitWriteBuffer();
            }

            // Check for data collection callback
            BoardList[board]->CheckCollectCallback();
        }
    }

    if (noneWritten) {
        OnNoneWritten();
    }
    if (!rtWrite)
        outStr << "WriteAllBoardsBroadcast: rtWrite is false" << std::endl;

    // return
    return allOK;
}
