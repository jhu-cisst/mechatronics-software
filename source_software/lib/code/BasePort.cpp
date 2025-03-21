/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <string>
#include <algorithm>   // for std::max
#include <cstdlib>

#include <Amp1394/AmpIORevision.h>
#include "BasePort.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"

// Starting with C++11, can initialize using an initializer list.
// Currently, the supported hardware (e.g., QLA1) is added in the BasePort constructor.
std::vector<unsigned long> BasePort::SupportedHardware;

unsigned int BasePort::BroadcastReadInfo::IncrementSequence()
{
    readSequence++;
    if (readSequence == 65536) {
        readSequence = 1;
    }
    return readSequence;
}

// Prepare for broadcast read
void BasePort::BroadcastReadInfo::PrepareForRead()
{
    // Initialize update Start/Finish times (i.e., start/end of data update on FPGA Hub)
    // Units are seconds.
    updateStartTime = 1.0;
    updateFinishTime = 0.0;
    updateOverflow = false;
    readOverflow = false;
    for (unsigned int bnum = 0; bnum < BoardIO::MAX_BOARDS; bnum++)
        boardInfo[bnum].updated = false;
}

void BasePort::BroadcastReadInfo::PrintTiming(std::ostream &outStr) const
{
    unsigned int num_bds = 0;
    outStr << "Updates (usec): ";
    for (unsigned int bnum = 0; bnum < BoardIO::MAX_BOARDS; bnum++) {
        if (boardInfo[bnum].inUse) {
            double bdTime_us = boardInfo[bnum].updateTime*1e6;
            outStr << bnum << ": " << std::fixed << std::setprecision(2) << bdTime_us
                   << "   ";
            num_bds++;
        }
    }
    double bdTimeDiff_us = (updateFinishTime-updateStartTime)*1e6;
    outStr << "Range: " << std::fixed << std::setprecision(2) << bdTimeDiff_us;
    if (num_bds > 1)
        outStr << " (" << (bdTimeDiff_us/(num_bds-1)) << ") ";
    outStr << (updateOverflow ? "OVF" : "   ");
    outStr << std::endl;
    outStr << "Read start: " << std::fixed << std::setprecision(2) << std::setw(6) << (readStartTime*1e6)
           << "  finish: " << std::setw(6) << (readFinishTime*1e6) << "  delta: " << ((readFinishTime-readStartTime)*1e6)
           << "  gap: " << std::setw(6) << (gapTime*1e6);
    if ((gapTimeMin != 1.0) && (gapTimeMax != 0.0)) {
        outStr << " (min " << std::setw(6) << (gapTimeMin*1e6) << ", max " << (gapTimeMax*1e6) << ") ";
    }
    outStr << (readOverflow ? "OVF" : "   ");
    outStr << std::endl;
}

BasePort::BasePort(int portNum, std::ostream &ostr):
        outStr(ostr),
        Protocol_(BasePort::PROTOCOL_SEQ_RW),
        IsAllBoardsBroadcastCapable_(false),
        IsAllBoardsRev4_5_(false),
        IsAllBoardsRev4_6_(false),
        IsAllBoardsRev6_(false),
        IsAllBoardsRev7_(false),
        IsAllBoardsRev8_9_(false),
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
        FirmwareVersion[i] = 0;
        FpgaVersion[i] = 0;
        HardwareVersion[i] = 0;
        Board2Node[i] = MAX_NODES;
    }
    ReadBufferBroadcast = 0;
    WriteBufferBroadcast = 0;
    GenericBuffer = 0;
    for (i = 0; i < MAX_NODES; i++)
        Node2Board[i] = BoardIO::MAX_BOARDS;
    // Note that AddHardwareVersion will not add duplicates
    BasePort::AddHardwareVersion(QLA1_String);
    BasePort::AddHardwareVersion(dRA1_String);
    BasePort::AddHardwareVersion(DQLA_String);
    BasePort::AddHardwareVersion(BCFG_String);
    BasePort::AddHardwareVersion(0x54455354); // "TEST"
}

BasePort::~BasePort()
{
    delete [] ReadBufferBroadcast;
    delete [] WriteBufferBroadcast;
    delete [] GenericBuffer;
}

std::string BasePort::ProtocolString(ProtocolType protocol)
{
    if (protocol == PROTOCOL_SEQ_RW) {
        return std::string("sequential-read-write");
    } else if (protocol == PROTOCOL_SEQ_R_BC_W) {
        return std::string("sequential-read-broadcast-write");
    } else if (protocol == PROTOCOL_BC_QRW) {
        return std::string("broadcast-query-read-write");
    }
    return std::string("Unknown");
}

bool BasePort::ParseProtocol(const char * arg, ProtocolType & protocol,
                             std::ostream & ostr)
{
    // no option, using default
    if ((arg == 0) || (strlen(arg) == 0)) {
        ostr << "ParseProtocol: no protocol provided" << std::endl;
        return false;
    }
    // convert arg to string
    std::string name(arg);

    // this could likely be implemented using a static map
    if ((name == "sequential-read-write") ||
        (name == "srw")) {
        protocol = PROTOCOL_SEQ_RW;
        return true;
    } else if ((name == "sequential-read-broadcast-write") ||
               (name == "srbw")) {
        protocol = PROTOCOL_SEQ_R_BC_W;
        return true;
    } else if ((name == "broadcast-read-write") ||
               (name == "brw") ||
               (name == "broadcast-query-read-write") ||
               (name == "bqrw")) {
        protocol = PROTOCOL_BC_QRW;
        return true;
    }
    ostr << "ParseProtocol: no protocol \"" << name << "\" is not supported, use sequential-read-write, srw, sequential-read-broadcast-write, srbw, broadcast-query-read-write or bqrw" << std::endl;
    return false;
}


bool BasePort::SetProtocol(ProtocolType prot) {
    if (prot != BasePort::PROTOCOL_SEQ_RW) {
        if (!IsAllBoardsBroadcastCapable_) {
            outStr << "BasePort::SetProtocol" << std::endl
                   << "***Error: not all boards support broadcasting, " << std::endl
                   << "          please upgrade your firmware"  << std::endl;
            return false;
        }
        if (!IsBroadcastFirmwareMixValid()) {
            outStr << "BasePort::SetProtocol" << std::endl
                   << "***Error: cannot use broadcast mode with this mix of firmware, " << std::endl
                   << "          please upgrade your firmware" << std::endl;
            return false;
        }
    }
    switch (prot) {
        case BasePort::PROTOCOL_SEQ_RW:
            outStr << "BasePort::SetProtocol: system running in NON broadcast mode" << std::endl;
            Protocol_ = prot;
            break;
        case BasePort::PROTOCOL_SEQ_R_BC_W:
            outStr << "BasePort::SetProtocol: system running with broadcast write" << std::endl;
            Protocol_ = prot;
            break;
        case BasePort::PROTOCOL_BC_QRW:
            outStr << "BasePort::SetProtocol: system running with broadcast query, read, and write" << std::endl;
            Protocol_ = prot;
            break;
        default:
            outStr << "BasePort::SetProtocol: warning: unknown protocol (ignored): " << prot << std::endl;
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
}

// Return expected size for broadcast read, in quadlets
unsigned int BasePort::GetBroadcastReadSizeQuads(void) const
{
    unsigned int bcReadSize;                    // Broadcast read size (depends on firmware version)
    if (IsAllBoardsRev4_6_)
        bcReadSize = 17*BoardIO::MAX_BOARDS;    // Rev 1-6: 16 * 17 = 272 max (though really should have been 16*21)
    else if (IsAllBoardsRev7_)
        bcReadSize = 29*NumOfBoards_+1;         // Rev 7: NumOfBoards * (1 seq + 28 data) + 1
    else {                                      // Rev 8+
        bcReadSize = 0;
        for (unsigned int boardNum = 0; boardNum < max_board; boardNum++) {
            BoardIO *board = BoardList[boardNum];
            if (board)
                bcReadSize += board->GetReadNumBytes()/sizeof(quadlet_t) + 1;
        }
        bcReadSize += 1;                       // Add one quadlet for timing info at end
    }
    return bcReadSize;
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
    IsAllBoardsRev4_5_ = true;
    IsAllBoardsRev4_6_ = true;
    IsAllBoardsRev6_ = true;
    IsAllBoardsRev7_ = true;
    IsAllBoardsRev8_9_ = true;
    NumOfNodes_ = 0;

    nodeid_t max_nodes = InitNodes();

    outStr << "BasePort::ScanNodes: building node map for " << max_nodes << " nodes:" << std::endl;
    // Iterate through all possible nodes
    for (node = 0; node < max_nodes; node++) {
        quadlet_t data;
        unsigned long hver = 0;
        // check hardware version
        if (!ReadQuadletNode(node, BoardIO::HARDWARE_VERSION, data)) {
            if (GetPortType() == PORT_FIREWIRE)
                outStr << "BasePort::ScanNodes: unable to read from node " << node << std::endl;
            continue;
        }
        hver = data;

        if (!HardwareVersionValid(hver)) {
            outStr << "BasePort::ScanNodes: node " << node << " is not a supported board (data = "
                   << std::hex << data << std::dec << ")" << std::endl;
            continue;
        }

        // read firmware version
        unsigned long fver = 0;
        if (!ReadQuadletNode(node, BoardIO::FIRMWARE_VERSION, data)) {
            outStr << "BasePort::ScanNodes: unable to read firmware version from node "
                   << node << std::endl;
            continue;
        }
        fver = data;

        // read FPGA version (for Firmware Rev 5+)
        unsigned long fpga_ver = 1;
        if (fver >= 5) {
            if (!ReadQuadletNode(node, BoardIO::ETH_STATUS, data)) {
                outStr << "BasePort::ScanNodes: unable to read FPGA version (ETH_STATUS) from node "
                       << node << std::endl;
                continue;
            }
            fpga_ver = BoardIO::GetFpgaVersionMajorFromStatus(data);
        }

        // read board id
        if (!ReadQuadletNode(node, BoardIO::BOARD_STATUS, data)) {
            outStr << "BasePort::ScanNodes: unable to read status from node " << node << std::endl;
            continue;
        }
        // board_id is bits 27-24, BOARD_ID_MASK = 0x0F000000
        board = (data & BOARD_ID_MASK) >> 24;
        FpgaVersion[board] = fpga_ver;
        HardwareVersion[board] = hver;
        FirmwareVersion[board] = fver;

        // read git description (introduced after Rev 8 release)
        uint32_t git_desc = 0;
        if (!ReadQuadletNode(node, BoardIO::GIT_DESC, git_desc)) {
            outStr << "BasePort::ScanNodes: unable to read git desc from node " << node << std::endl;
        }
        outStr << "  Node " << node << ", BoardId = " << board
               << ", " << GetFpgaVersionMajorString(board)
               << ", Hardware = " << GetHardwareVersionString(board)
               << ", Firmware Version = " << GetFirmwareVersion(board);

        if (git_desc != 0) {
            // git_dirty bit indicates that firmware was built with uncommitted changes;
            // shown by appending * to the git SHA.
            bool git_dirty = git_desc&0x00000008;
            // git_commits bit indicates whether there were any commits since the last tag
            bool git_commits = git_desc&0x00000004;
            // git_flag is set by the firmware, based on comparing the firmware version (F) to
            // the most recent git tag ("RevG", where G is the git tag version)
            //    0:  F == G     actual release ("F") or update to last release ("F+"),
            //                   depending on git_dirty and git_commits
            //    1:  F == G+1   new firmware preview, shown as "F-"
            //    2:  F > G+1    should not happen (missing git tag?), shown as "F?"
            //    3:  F < G      should not happen, shown as "F?"
            uint8_t git_flag = git_desc&0x00000003;
            bool showSHA = true;
            if (git_flag == 0) {
                if (git_dirty|git_commits)
                    outStr << "+";
                else
                    showSHA = false;
            }
            else if (git_flag == 1)
                outStr << "-";
            else
                outStr << "?";
            if (showSHA) {
                outStr << " (git-" << std::hex << std::setw(7) << std::setfill('0')
                       << (git_desc>>4) << std::dec;
                if (git_dirty)
                    outStr << "*";
                outStr << ")";
            }
        }
        outStr << std::endl;

        if (Node2Board[node] < BoardIO::MAX_BOARDS) {
            outStr << "    Duplicate entry, previous value = "
                   << static_cast<int>(Node2Board[node]) << std::endl;
        }
        Node2Board[node] = static_cast<unsigned char>(board);

        // check firmware version
        // Firmware Version >= 4, broadcast capable
        if (fver < 4) {
            IsAllBoardsBroadcastCapable_ = false;
            IsAllBoardsRev4_5_ = false;
            IsAllBoardsRev4_6_ = false;
        }
        if (fver > 5) IsAllBoardsRev4_5_ = false;
        if (fver > 6) IsAllBoardsRev4_6_ = false;
        // Firmware Version 6, broadcast with possibly shorter wait (i.e., skipping nodes
        // on the bus that are not part of the current configuration).
        if (fver != 6) IsAllBoardsRev6_ = false;
        // Firmware Version 7, added power control to block write; added velocity estimation
        // fields to block read
        if (fver != 7) IsAllBoardsRev7_ = false;
        // Firmware Version 8, added header quadlet to block write; support larger entries in
        // block read (both changes to support dRAC).
        if ((fver != 8) && (fver != 9)) IsAllBoardsRev8_9_ = false;
        NumOfNodes_++;
    }
    outStr << "BasePort::ScanNodes: found " << NumOfNodes_ << " boards" << std::endl;

    // update Board2Node
    for (board = 0; board < BoardIO::MAX_BOARDS; board++) {
        Board2Node[board] = MAX_NODES;
        // search up to max_nodes (not NumOfNodes_) in case nodes are not sequential (i.e., if some nodes
        // are not associated with valid boards).
        for (node = 0; node < max_nodes; node++) {
            if (Node2Board[node] == board) {
                if (Board2Node[board] < MAX_NODES)
                    outStr << "BasePort::ScanNodes: warning: duplicate node id for board " << board
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
        if (IsBroadcastFirmwareMixValid()) {
            Protocol_ = BasePort::PROTOCOL_SEQ_R_BC_W;
            if (IsBroadcastShorterWait())
                outStr << "BasePort::SetDefaultProtocol: all nodes broadcast capable and support shorter wait" << std::endl;
            else if (IsAllBoardsRev4_5_)
                outStr << "BasePort::SetDefaultProtocol: all nodes broadcast capable and do not support shorter wait" << std::endl;
            else if (IsAllBoardsRev4_6_)
                outStr << "BasePort::SetDefaultProtocol: all nodes broadcast capable and some support shorter wait" << std::endl;
        }
        else
            outStr << "BasePort::SetDefaultProtocol: all nodes broadcast capable, but disabled due to mix of firmware" << std::endl;
    }
}

bool BasePort::AddBoard(BoardIO *board)
{
    unsigned int id = board->BoardId;
    if (id >= BoardIO::MAX_BOARDS) {
        outStr << "BasePort::AddBoard: board number out of range: " << id << std::endl;
        return false;
    }
    BoardList[id] = board;
    board->port = this;
    board->InitBoard();

    // Make sure read/write buffers are allocated
    SetReadBufferBroadcast();
    SetWriteBufferBroadcast();

    if (id >= max_board)
        max_board = id+1;
    // update BoardInUseMask_
    BoardInUseMask_ = (BoardInUseMask_ | (1 << id));
    bcReadInfo.boardInfo[id].inUse = true;
    NumOfBoards_++;   // increment board counts

    bcReadInfo.readSizeQuads = GetBroadcastReadSizeQuads();

    return true;
}

bool BasePort::RemoveBoard(unsigned char boardId)
{
    if (boardId >= BoardIO::MAX_BOARDS) {
        outStr << "BasePort::RemoveBoard: board number out of range: " << static_cast<unsigned int>(boardId) << std::endl;
        return false;
    }
    BoardIO *board = BoardList[boardId];
    if (!board) {
        outStr << "BasePort::RemoveBoard: board not found: " << static_cast<unsigned int>(boardId) << std::endl;
        return false;
    }

    // update BoardInuseMask_
    BoardInUseMask_ = (BoardInUseMask_ & (~(1 << boardId)));
    bcReadInfo.boardInfo[boardId].inUse = false;

    BoardList[boardId] = 0;
    board->port = 0;
    NumOfBoards_--;

    if (boardId >= max_board-1) {
        // If max_board was just removed, find the new max_board
        max_board = 0;
        for (int bd = 0; bd < boardId; bd++)
            if (BoardList[bd]) max_board = bd+1;
    }

    bcReadInfo.readSizeQuads = GetBroadcastReadSizeQuads();

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
    else if (portType == PORT_ZYNQ_EMIO)
        return std::string("Zynq-EMIO");
    else
        return std::string("Unknown");
}

// Helper function for parsing command line options.
// In particular, this is typically called after a certain option, such as -p, is
// recognized and it parses the rest of that option string:
// N                  for FireWire, where N is the port number (backward compatibility)
// fw:N               for FireWire, where N is the port number
// eth:N              for raw Ethernet (PCAP), where N is the port number
// ethfw:N            as above, forcing bridge to FireWire if possible
// udp:xx.xx.xx.xx    for UDP, where xx.xx.xx.xx is the (optional) server IP address
// udpfw:xx.xx.xx.xx  as above, forcing bridge to FireWire if possible
// emio:N             for Zynq EMIO (N=1 for gpiod interface; otherwise use mmap interface)
bool BasePort::ParseOptions(const char *arg, PortType &portType, int &portNum, std::string &IPaddr,
                            bool &fwBridge, std::ostream &ostr)
{
    // no option, using default
    if ((arg == 0) || (strlen(arg) == 0)) {
        ostr << "ParseOptions: no option provided, using default "
             << DefaultPort() << std::endl;
        return ParseOptions(DefaultPort().c_str(), portType, portNum, IPaddr, fwBridge);
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
        fwBridge = false;
        unsigned int numOffset = 4;
        if (strncmp(arg+3, "fw", 2) == 0) {
            fwBridge = true;
            numOffset += 2;
        }
        return (sscanf(arg+numOffset, "%d", &portNum) == 1);
    }
    else if (strncmp(arg, "udp", 3) == 0) {
        portType = PORT_ETH_UDP;
        fwBridge = false;
        unsigned int colonPos = 3;
        if (strncmp(arg+3, "fw", 2) == 0) {
            fwBridge = true;
            colonPos += 2;
        }
        // no option specified
        if (strlen(arg) == colonPos) {
            IPaddr = ETH_UDP_DEFAULT_IP;
            return true;
        }
        // make sure separator is here
        if (arg[colonPos] != ':') {
            if (colonPos == 3)
                ostr << "ParseOptions: missing \":\" after \"udp\"" << std::endl;
            else
                ostr << "ParseOptions: missing \":\" after \"udpfw\"" << std::endl;
            return false;
        }
        // For now, if at least 8 characters, assume a valid IP address
        if (strlen(arg+colonPos+1) >= 8)
            IPaddr.assign(arg+colonPos+1);
        return true;
    }
    else if (strncmp(arg, "emio", 4) == 0) {
        portType = PORT_ZYNQ_EMIO;
        // no port specified
        if (strlen(arg) == 4) {
            portNum = 0;
            return true;
        }
        // make sure separator is here
        if (arg[4] != ':') {
            ostr << "ParseOptions: missing \":\" after \"emio\"" << std::endl;
            return false;
        }
        // scan port number
        if (sscanf(arg+5, "%d", &portNum) == 1) {
            return true;
        }
        ostr << "ParseOptions: failed to find a port number after \"emio:\" in " << arg+3 << std::endl;
        return false;
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

BasePort::PortType BasePort::DefaultPortType(void)
{
#if Amp1394_HAS_RAW1394
    return PORT_FIREWIRE;
#elif Amp1394_HAS_EMIO
    return PORT_ZYNQ_EMIO;
#else
    return PORT_ETH_UDP;
#endif

}

std::string BasePort::DefaultPort(void)
{
#if Amp1394_HAS_RAW1394
    return "fw:0";
#elif Amp1394_HAS_EMIO
    return "emio";
#else
    return "udp:" ETH_UDP_DEFAULT_IP;
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

std::string BasePort::GetFpgaVersionMajorString(unsigned char boardId) const
{
    unsigned int fpga_ver = GetFpgaVersionMajor(boardId);
    std::string fStr("FPGA_V");
    if (fpga_ver == 0)
        fStr.push_back('?');
    else
        fStr.push_back('0'+fpga_ver);
    return fStr;
}

std::string BasePort::GetHardwareVersionString(unsigned char boardId) const
{
    std::string hStr;
    if ((boardId < BoardIO::MAX_BOARDS) && HardwareVersion[boardId]) {
        unsigned long hver = bswap_32(HardwareVersion[boardId]);
        hStr.assign(reinterpret_cast<const char *>(&hver), sizeof(unsigned long));
        hStr.resize(4);
    } else {
        hStr = "Invalid";
    }
    return hStr;
}

void BasePort::AddHardwareVersion(unsigned long hver)
{
    // Add if non-zero and not already on list
    if (hver && !HardwareVersionValid(hver))
        SupportedHardware.push_back(hver);
}

void BasePort::AddHardwareVersionString(const std::string &hStr)
{
    if (hStr.empty()) return;
    unsigned long hver = 0;
    if ((hStr.compare(0,2,"0x") == 0) || (hStr.compare(0,2,"0X") == 0)) {
        // if hex
        hver = std::strtoul(hStr.c_str()+2, 0, 16);
    }
    else {
        std::string(hStr.rbegin(), hStr.rend()).copy(reinterpret_cast<char *>(&hver), sizeof(unsigned long));
    }
    AddHardwareVersion(hver);
}

void BasePort::AddHardwareVersionStringList(const std::string &hStr)
{
    if (hStr.empty()) return;
    std::string::size_type n = 0;
    while (n != std::string::npos) {
        std::string::size_type p = hStr.find(',', n);
        if (p == std::string::npos) {
            AddHardwareVersionString(hStr.substr(n, p));
            n = std::string::npos;
        }
        else {
            AddHardwareVersionString(hStr.substr(n, (p-n)));
            n = p+1;
        }
    }
}

bool BasePort::HardwareVersionValid(unsigned long hver)
{
    bool valid = false;
    for (size_t i = 0; i < SupportedHardware.size(); i++) {
        if (SupportedHardware[i] == hver) {
            valid = true;
            break;
        }
    }
    return valid;
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
        outStr << "BasePort::ReadBlock: illegal size (" << nbytes << "), must be multiple of 4" << std::endl;
        return false;
    }
    else if (nbytes > GetMaxReadDataSize()) {
        outStr << "BasePort::ReadBlock: packet size " << std::dec << nbytes << " too large (max = "
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
        outStr << "BasePort::WriteBlock: illegal size (" << nbytes << "), must be multiple of 4" << std::endl;
        return false;
    }
    else if (nbytes > GetMaxWriteDataSize()) {
        outStr << "BasePort::WriteBlock: packet size " << std::dec << nbytes << " too large (max = "
               << GetMaxWriteDataSize() << " bytes)" << std::endl;
        return false;
    }

    nodeid_t node = ConvertBoardToNode(boardId);
    return (node < MAX_NODES) ? WriteBlockNode(node, addr, wdata, nbytes, boardId&FW_NODE_FLAGS_MASK) : false;
}

bool BasePort::ReadAllBoards(void)
{
    if (!IsOK()) {
        outStr << "BasePort::ReadAllBoards: port not initialized" << std::endl;
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
            quadlet_t *readBuffer = reinterpret_cast<quadlet_t *>(ReadBufferBroadcast + GetReadQuadAlign() + GetPrefixOffset(RD_FW_BDATA));
            bool ret = ReadBlock(board, 0, readBuffer, BoardList[board]->GetReadNumBytes());
            if (ret) {
                BoardList[board]->SetReadData(readBuffer);
                noneRead = false;
            } else {
                allOK = false;
            }
            BoardList[board]->SetReadValid(ret);

            if (ret) {
                ReadErrorCounter_ = 0;
            }
            else {
                if (ReadErrorCounter_ == 0) {
                    outStr << "BasePort::ReadAllBoards: read failed on port "
                           << PortNum << ", board " << board << std::endl;
                }
                ReadErrorCounter_++;
                if (ReadErrorCounter_ == 10000) {
                    outStr << "BasePort::ReadAllBoards: read failed on port "
                           << PortNum << ", board " << board << " occurred 10,000 times" << std::endl;
                    ReadErrorCounter_ = 0;
                }
            }
        }
    }
    if (!rtRead)
        outStr << "BasePort::ReadAllBoards: rtRead is false" << std::endl;

    if (noneRead) {
        OnNoneRead();
    }
    return allOK;
}

bool BasePort::ReadAllBoardsBroadcast(void)
{
    if (!IsOK()) {
        outStr << "BasePort::ReadAllBoardsBroadcast: port not initialized" << std::endl;
        OnNoneRead();
        return false;
    }

    if (!IsBroadcastFirmwareMixValid()) {
        outStr << "BasePort::ReadAllBoardsBroadcast: invalid mix of firmware" << std::endl;
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
    bcReadInfo.IncrementSequence();

    if (!WriteBroadcastReadRequest(bcReadInfo.readSequence)) {
        outStr << "BasePort::ReadAllBoardsBroadcast: failed to send broadcast read request, seq = "
               << bcReadInfo.readSequence << std::endl;
        OnNoneRead();
        return false;
    }

    // Wait for broadcast read data
    WaitBroadcastRead();

    bcReadInfo.PrepareForRead();

    quadlet_t *hubReadBuffer = reinterpret_cast<quadlet_t *>(ReadBufferBroadcast + GetReadQuadAlign() + GetPrefixOffset(RD_FW_BDATA));
    memset(hubReadBuffer, 0, bcReadInfo.readSizeQuads*sizeof(quadlet_t));
    bool ret = ReceiveBroadcastReadResponse(hubReadBuffer, bcReadInfo.readSizeQuads*sizeof(quadlet_t));
    if (!ret) {
        SetReadInvalid();
        OnNoneRead();
        return false;
    }

    double clkPeriod = GetBroadcastReadClockPeriod();
    quadlet_t *curPtr = hubReadBuffer;
    unsigned int bdCnt = 0;   // Number of board processed
    // Loop through all boards, processing the boards in use.
    // Note that prior to Firmware Rev 7, we always read data for all 16 boards.
    // Starting with Firmware Rev 9, the board data will generally not be in sequential order
    // (i.e., isBroadcastReadOrdered will return false) for the Ethernet-only interface.
    for (unsigned int index = 0; (index < BoardIO::MAX_BOARDS) && (bdCnt < NumOfBoards_); index++) {
        if ((curPtr[0] == 0) || (curPtr[2] == 0)) {
            outStr << "BasePort::ReadAllBoardsBroadcast: invalid block after reading "
                   << bdCnt << " boards" << std::endl;
            outStr << "Failed to read from board(s) ";
            for (size_t i = 0; i < BoardIO::MAX_BOARDS; i++) {
                if (bcReadInfo.boardInfo[i].inUse && !bcReadInfo.boardInfo[i].updated)
                    outStr << i << " ";
            }
            outStr << std::endl;
            allOK = false;
            break;
        }
        quadlet_t statusQuad = bswap_32(curPtr[2]);
        unsigned int numAxes = (statusQuad&0xf0000000)>>28;
        if (!IsAllBoardsRev8_9_ && (numAxes != 4)) {
            outStr << "BasePort::ReadAllBoardsBroadcast: invalid status (not a 4 axis board): " << std::hex << statusQuad
                   << std::dec << std::endl;
            allOK = false;
            break;
        }
        unsigned int thisBoard = (statusQuad&0x0f000000)>>24;
        unsigned int boardNum = isBroadcastReadOrdered() ? index : thisBoard;
        BoardIO *board = BoardList[boardNum];
        if (board && (boardNum != thisBoard)) {
            outStr << "BasePort::ReadAllBoardsBroadcast: board mismatch, expecting "
                   << boardNum << ", found " << thisBoard << std::endl;
            allOK = false;
            break;
        }
        if (bcReadInfo.boardInfo[boardNum].inUse && board) {
            bool thisOK = false;
            bcReadInfo.boardInfo[boardNum].updated = true;
            bcReadInfo.boardInfo[boardNum].blockNum = bdCnt++;
            bcReadInfo.boardInfo[boardNum].sequence = bswap_32(curPtr[0]) >> 16;
            unsigned int seq_expected = bcReadInfo.readSequence;
            if (IsAllBoardsRev8_9_) {
                // For Rev 8+, only the LSB of the sequence is returned, but bit 15 also indicates
                // whether the 16-bit sequence number did not match on the FPGA side.
                seq_expected &= 0x00ff;
                bcReadInfo.boardInfo[boardNum].sequence &= 0x00ff;  // lowest byte only
                bcReadInfo.boardInfo[boardNum].seq_error = bswap_32(curPtr[0]) & 0x00008000;  // bit 15
                if (bcReadInfo.boardInfo[boardNum].sequence != seq_expected)
                    bcReadInfo.boardInfo[boardNum].seq_error = true;
                bcReadInfo.boardInfo[boardNum].blockSize = (bswap_32(curPtr[0]) & 0xff000000) >> 24;
                unsigned int bdReadSize = board->GetReadNumBytes()/sizeof(quadlet_t) + 1;
                if (bcReadInfo.boardInfo[boardNum].blockSize != bdReadSize) {
                    outStr << "BasePort::ReadAllBoardsBroadcast: board " << boardNum
                           << ", blockSize = " << bcReadInfo.boardInfo[boardNum].blockSize
                           << ", expected = " << bdReadSize << std::endl;
                }
            }
            else {
                bcReadInfo.boardInfo[boardNum].seq_error = (bcReadInfo.boardInfo[boardNum].sequence != seq_expected);
                // Rev7 block size is 29 (1 + 28), Rev4_6 block size is 17 (should have been 21)
                bcReadInfo.boardInfo[boardNum].blockSize = IsAllBoardsRev7_ ? 29 : 17;
            }
            if (IsAllBoardsRev7_ || IsAllBoardsRev8_9_) {
                unsigned int quad0_lsb = bswap_32(curPtr[0])&0x0000ffff;
                bcReadInfo.boardInfo[boardNum].updateTime = (quad0_lsb&0x7fff)*clkPeriod;
                if (!bcReadInfo.boardInfo[boardNum].seq_error) {
                    // Set the update Start/Finish times based on the minimum and maximum update times
                    // Note that update times should be increasing, so if our current updateTime is
                    // less than the previous max (updateFinishTime), then there must have been overflow
                    if (bcReadInfo.boardInfo[boardNum].updateTime < bcReadInfo.updateFinishTime) {
                        // Firmware Rev 9 uses 15 bits; prior versions use 14 bits
                        uint16_t overflow_val = (FirmwareVersion[HubBoard] < 9) ? 0x4000 : 0x8000;
                        bcReadInfo.boardInfo[boardNum].updateTime += overflow_val*clkPeriod;
                        bcReadInfo.updateOverflow = true;
                    }
                    if (bcReadInfo.boardInfo[boardNum].updateTime < bcReadInfo.updateStartTime)
                        bcReadInfo.updateStartTime = bcReadInfo.boardInfo[boardNum].updateTime;
                    if (bcReadInfo.boardInfo[boardNum].updateTime > bcReadInfo.updateFinishTime)
                        bcReadInfo.updateFinishTime = bcReadInfo.boardInfo[boardNum].updateTime;
                }
            }
            if (!bcReadInfo.boardInfo[boardNum].seq_error) {
                thisOK = true;
            }
            else {
                outStr << "BasePort::ReadAllBoardsBroadcast: board " << boardNum
                       << ", seq = " << bcReadInfo.boardInfo[boardNum].sequence
                       << ", expected = " << seq_expected
                       << ", diff = " << static_cast<int>(seq_expected-bcReadInfo.boardInfo[boardNum].sequence)
                       << std::endl;
            }
            board->SetReadValid(thisOK);
            if (thisOK) {
                board->SetReadData(curPtr+1);
                noneRead = false;
            }
            else {
                allOK = false;
            }
            curPtr += bcReadInfo.boardInfo[boardNum].blockSize;
        }
        else if (IsAllBoardsRev4_6_) {
            // Skip unused boards for firmware < 7
            // Rev4_6 block size is 17 (should have been 21)
            curPtr += 17;
        }
    }

    if (!allOK) {
        for (unsigned int bnum = 0; bnum < BoardIO::MAX_BOARDS; bnum++) {
            BoardIO *board = BoardList[bnum];
            if (board && bcReadInfo.boardInfo[bnum].inUse && !bcReadInfo.boardInfo[bnum].updated) {
               board->SetReadValid(false);
            }
        }
        bcReadInfo.readStartTime = 0.0;
        bcReadInfo.readFinishTime = 0.0;
        bcReadInfo.gapTime = 0.0;
    }
    else if (IsAllBoardsRev7_ || IsAllBoardsRev8_9_) {
        quadlet_t timingInfo = bswap_32(curPtr[0]);
        bcReadInfo.readStartTime = ((timingInfo&0xffff0000) >> 16)*clkPeriod;
        bcReadInfo.readFinishTime = (timingInfo&0x0000ffff)*clkPeriod;
        if (timingInfo != 0) {
            // timingInfo == 0 indicates invalid data
            if (bcReadInfo.readStartTime < bcReadInfo.updateFinishTime) {
                // Firmware Rev 9 uses 16 bits; prior versions use 14 bits
                uint32_t overflow_val = (FirmwareVersion[HubBoard] < 9) ? 0x00004000 : 0x00010000;
                bcReadInfo.readStartTime += overflow_val*clkPeriod;
                bcReadInfo.readOverflow = true;
            }
            if (bcReadInfo.readFinishTime < bcReadInfo.readStartTime) {
                // Firmware Rev 9 uses 16 bits; prior versions use 14 bits
                uint32_t overflow_val = (FirmwareVersion[HubBoard] < 9) ? 0x00004000 : 0x00010000;
                bcReadInfo.readFinishTime += overflow_val*clkPeriod;
                bcReadInfo.readOverflow = true;
            }
        }
        // Gap between end of update and start of read; this can be tuned to be as
        // close to 0 as possible.
        if (timingInfo == 0) {
            bcReadInfo.gapTime = 0.0;
        }
        else {
            bcReadInfo.gapTime = bcReadInfo.readStartTime - bcReadInfo.updateFinishTime;
            if (bcReadInfo.gapTime < bcReadInfo.gapTimeMin)
                bcReadInfo.gapTimeMin = bcReadInfo.gapTime;
            if (bcReadInfo.gapTime > bcReadInfo.gapTimeMax)
                bcReadInfo.gapTimeMax = bcReadInfo.gapTime;
        }
    }

    if (noneRead) {
        OnNoneRead();
    }
    if (!rtRead)
        outStr << "BasePort::ReadAllBoardsBroadcast: rtRead is false" << std::endl;

#if 0
    if (IsAllBoardsRev7_ || IsAllBoardsRev8_9_) {
        bcReadInfo.PrintTiming(outStr);
    }
#endif

    return allOK;
}

bool BasePort::ReceiveBroadcastReadResponse(quadlet_t *rdata, unsigned int nbytes)
{
    return ReadBlock(HubBoard, 0x1000, rdata, nbytes);
}

double BasePort::GetBroadcastReadClockPeriod(void) const
{
    const double FPGA_sysclk_MHz = 49.152;      /* FPGA sysclk in MHz (from AmpIO.cpp) */
    return (1.0e-6/FPGA_sysclk_MHz);
}

bool BasePort::WriteAllBoards(void)
{
    if (!IsOK()) {
        outStr << "BasePort::WriteAllBoards: port not initialized" << std::endl;
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
            quadlet_t *buf = reinterpret_cast<quadlet_t *>(WriteBufferBroadcast + GetWriteQuadAlign() + GetPrefixOffset(WR_FW_BDATA));
            unsigned int numBytes = BoardList[board]->GetWriteNumBytes();
            unsigned int numQuads = numBytes/sizeof(quadlet_t);
            if (FirmwareVersion[board] < 7) {
                // Rev 1-6 firmware: the last quadlet (Status/Control register)
                // is done as a separate quadlet write.
                BoardList[board]->GetWriteData(buf, 0, numQuads-1);
                bool noneWrittenThisBoard = true;
                bool ret = WriteBlock(board, 0, buf, numBytes-sizeof(quadlet_t));
                if (ret) { noneWritten = false; noneWrittenThisBoard = false; }
                else allOK = false;
                // Get last quadlet (false -> no byteswapping)
                quadlet_t ctrl;
                BoardList[board]->GetWriteData(&ctrl, numQuads-1, 1, false);
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
                BoardList[board]->GetWriteData(buf, 0, numQuads);
                bool ret = WriteBlock(board, 0, buf, numBytes);
                BoardList[board]->SetWriteValid(ret);
                // Initialize (clear) the write buffer
                BoardList[board]->InitWriteBuffer();
                if (ret) {
                    noneWritten = false;
                    // Check for data collection callback
                    BoardList[board]->CheckCollectCallback();
                }
                else {
                    allOK = false;
                }
            }
        }
    }
    if (noneWritten) {
        OnNoneWritten();
    }
    if (!rtWrite)
        outStr << "BasePort::WriteAllBoards: rtWrite is false" << std::endl;
    return allOK;
}

bool BasePort::WriteAllBoardsBroadcast(void)
{
    if (!IsOK()) {
        outStr << "BasePort::WriteAllBoardsBroadcast: port not initialized" << std::endl;
        OnNoneWritten();
        return false;
    }

    if (!IsBroadcastFirmwareMixValid()) {
        outStr << "BasePort::WriteAllBoardsBroadcast: invalid mix of firmware" << std::endl;
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
            quadlet_t *bcPtr = bcBuffer+bcBufferOffset/sizeof(quadlet_t);
            if (IsAllBoardsRev4_6_) {
                numBytes -= sizeof(quadlet_t);   // for ctrl offset
                unsigned int numQuads = numBytes/4;
                BoardList[board]->GetWriteData(bcPtr, 0, numQuads);
            }
            else {
                unsigned int numQuads = numBytes/4;
                BoardList[board]->GetWriteData(bcPtr, 0, numQuads);
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
                unsigned int numBytes = BoardList[board]->GetWriteNumBytes();
                // Get last quadlet (false -> no byteswapping)
                quadlet_t ctrl;
                BoardList[board]->GetWriteData(&ctrl, (numBytes/sizeof(quadlet_t))-1, 1, false);
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
                BoardList[board]->SetWriteValid(ret);
                // Initialize (clear) the write buffer
                BoardList[board]->InitWriteBuffer();
                if (ret) {
                    noneWritten = false;
                    // Check for data collection callback
                    BoardList[board]->CheckCollectCallback();
                }
            }
        }
    }

    if (noneWritten) {
        OnNoneWritten();
    }
    if (!rtWrite)
        outStr << "BasePort::WriteAllBoardsBroadcast: rtWrite is false" << std::endl;

    // return
    return allOK;
}
