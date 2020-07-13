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
#include "BasePort.h"


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
        NumOfNodes_(0),
        NumOfBoards_(0),
        BoardInUseMask_(0),
        max_board(0),
        HubBoard(BoardIO::MAX_BOARDS)
{
    size_t i;
    for (i = 0; i < BoardIO::MAX_BOARDS; i++) {
        BoardExists[i] = false;
        BoardList[i] = 0;
        FirmwareVersion[i] = 0;
        Board2Node[i] = MAX_NODES;
    }
    for (i = 0; i < MAX_NODES; i++)
        Node2Board[i] = BoardIO::MAX_BOARDS;
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
            Protocol_ = prot;
            break;
        case BasePort::PROTOCOL_SEQ_R_BC_W:
            outStr << "System running with broadcast write" << std::endl;
            Protocol_ = prot;
            break;
        case BasePort::PROTOCOL_BC_QRW:
            outStr << "System running with broadcast query, read, and write" << std::endl;
            Protocol_ = prot;
            break;
        default:
            outStr << "Unknown protocol (ignored): " << prot << std::endl;
            break;
    }
    return (Protocol_ == prot);
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

    if (id >= max_board)
        max_board = id+1;
    // update BoardInUseMask_
    BoardInUseMask_ = (BoardInUseMask_ | (1 << id));

    NumOfBoards_++;   // increment board counts
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

    BoardList[boardId] = 0;
    board->port = 0;
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
bool BasePort::ParseOptions(const char *arg, PortType &portType, int &portNum, std::string &IPaddr)
{
    if (strncmp(arg, "fw", 2) == 0) {
        portType = PORT_FIREWIRE;
        return (sscanf(arg+2, "%d", &portNum) == 1);
    }
    else if (strncmp(arg, "eth", 3) == 0) {
        portType = BasePort::PORT_ETH_RAW;
        return (sscanf(arg+3, "%d", &portNum) == 1);
    }
    else if (strncmp(arg, "udp", 3) == 0) {
        portType = BasePort::PORT_ETH_UDP;
        // For now, if at least 8 characters, assume a valid IP address
        if (strlen(arg+3) >= 8)
            IPaddr.assign(arg+5);
        return true;
    }
    portType = PORT_FIREWIRE;
    return (sscanf(arg, "%d", &portNum) == 1);
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

bool BasePort::ReadAllBoards(void)
{
    if (!IsOK()) {
        outStr << "BasePort::ReadAllBoards: port not initialized" << std::endl;
        return false;
    }

    if (Protocol_ == BasePort::PROTOCOL_BC_QRW) {
        return ReadAllBoardsBroadcast();
    }

    bool allOK = true;
    bool noneRead = true;
    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            bool ret = ReadBlock(board, 0, BoardList[board]->GetReadBuffer(),
                                 BoardList[board]->GetReadNumBytes());
            if (ret) {
                noneRead = false;
            } else {
                allOK = false;
            }
            BoardList[board]->SetReadValid(ret);

            if (!ret) {
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
            } else {
                ReadErrorCounter_ = 0;
            }
        }
    }
    if (noneRead) {
        OnNoneRead();
    }
    return allOK;
}


bool BasePort::WriteAllBoards(void)
{
    if (!IsOK()) {
        outStr << "BasePort::WriteAllBoards: port not initialized" << std::endl;
        return false;
    }

    if ((Protocol_ == BasePort::PROTOCOL_SEQ_R_BC_W) || (Protocol_ == BasePort::PROTOCOL_BC_QRW)) {
        return WriteAllBoardsBroadcast();
    }

    bool allOK = true;
    bool noneWritten = true;
    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            quadlet_t *buf = BoardList[board]->GetWriteBufferData();
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
                // SetWriteValid clears the buffer if the write was valid
                BoardList[board]->SetWriteValid(ret&&ret2);
            }
            else {
                // Rev 7 firmware: write DAC (x4) and Status/Control register
                bool ret = WriteBlock(board, 0, buf, numBytes);
                if (ret) noneWritten = false;
                else allOK = false;
                // SetWriteValid clears the buffer if the write was valid
                BoardList[board]->SetWriteValid(ret);
            }
            // Check for data collection callback
            BoardList[board]->CheckCollectCallback();
        }
    }
    if (noneWritten) {
        OnNoneWritten();
    }
    return allOK;
}

bool BasePort::WriteAllBoardsBroadcast(void)
{
    if (!IsOK()) {
        outStr << "BasePort::WriteAllBoardsBroadcast: port not initialized" << std::endl;
        return false;
    }

    // sanity check vars
    bool allOK = true;
    bool noneWritten = true;

    // loop 1: broadcast write block

    // construct broadcast write buffer
    const int numOfChannel = 4;
    quadlet_t bcBuffer[(numOfChannel+1) * BoardIO::MAX_BOARDS];  // +1 for Rev 7
    memset(bcBuffer, 0, sizeof(bcBuffer));
    int bcBufferOffset = 0; // the offset for new data to be stored in bcBuffer (bytes)
    int numOfBoards = 0;

    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            numOfBoards++;
            quadlet_t *buf = BoardList[board]->GetWriteBufferData();
            unsigned int numBytes = BoardList[board]->GetWriteNumBytes();
            if (FirmwareVersion[board] < 7)
                numBytes -= 4;   // -4 for ctrl offset
            memcpy(bcBuffer + bcBufferOffset/4, buf, numBytes);
            // bcBufferOffset equals total numBytes to write, when the loop ends
            bcBufferOffset = bcBufferOffset + numBytes;
        }
    }

    // now broadcast out the huge packet
    bool ret;

#if 1
    ret = WriteBlockNode(0, 0xffffffff0000, bcBuffer, bcBufferOffset);
#else
    ret = WriteBlockBroadcast(0xffffff000000,  // now the address is hardcoded
                              bcBuffer,
                              bcBufferOffset);
#endif

    // loop 2: send out control quadlet if necessary (firmware prior to Rev 7);
    //         also check for data collection
    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            if (FirmwareVersion[board] < 7) {
                bool noneWrittenThisBoard = true;
                quadlet_t *buf = BoardList[board]->GetWriteBufferData();
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
                // SetWriteValid clears the buffer if the write was valid
                BoardList[board]->SetWriteValid(ret&&ret2);
            }
            else {
                // SetWriteValid clears the buffer if the write was valid
                BoardList[board]->SetWriteValid(ret);
            }

            // Check for data collection callback
            BoardList[board]->CheckCollectCallback();
        }
    }

    if (noneWritten) {
        OnNoneWritten();
    }

    // return
    return allOK;
}
