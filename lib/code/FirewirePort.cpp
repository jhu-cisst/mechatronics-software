/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <algorithm>
#include <string.h>
#include <byteswap.h>
#include <sys/select.h>
#include <errno.h>   // errno
#include <string.h>  // strerror(errno)

// firewire
#include "FirewirePort.h"

const unsigned long QLA1_String = 0x514C4131;

const unsigned long BOARD_ID_MASK    = 0x0f000000;  /*!< Mask for board_id */

FirewirePort::PortListType FirewirePort::PortList;

FirewirePort::FirewirePort(int portNum, std::ostream &ostr) :
    PortNum(portNum),
    outStr(ostr),
    max_board(0),
    WriteAllBoardsBroadcastSequence_(0),
    BoardExistMask_(0)
{
    memset(BoardList, 0, sizeof(BoardList));
    Init();
}

bool FirewirePort::Init(void)
{
    // create firewire port handle 
    handle = raw1394_new_handle();
    handle_bc = raw1394_new_handle();
    if (!handle || !handle_bc) {
        outStr << "FirewirePort: could not create handle" << std::endl;
        return false;
    }

    PortListType::const_iterator it;
    for (it = PortList.begin(); it != PortList.end(); it++) {
        if ((*it)->PortNum == PortNum)
            outStr << "WARNING: Firewire port " << PortNum
                   << " is already used (be careful about thread safety)" << std::endl;
        if ((*it)->handle == handle) // should never happen
            outStr << "WARNING: Firewire handle is already used" << std::endl;
    }
    PortList.push_back(this);

    memset(Node2Board, BoardIO::MAX_BOARDS, sizeof(Node2Board));

    // set the bus reset handler
    old_reset_handler = raw1394_set_bus_reset_handler(handle, reset_handler);
    old_reset_handler = raw1394_set_bus_reset_handler(handle_bc, reset_handler);

    // get number of ports
    int nports = raw1394_get_port_info(handle, NULL, 0);
    outStr << "FirewirePort: number of ports = " << nports << std::endl;
    if (nports < PortNum) {
        outStr << "FirewirePort: port " << PortNum << " does not exist" << std::endl;
        raw1394_destroy_handle(handle);
        handle = NULL;
        return false;
    }

    if (raw1394_set_port(handle, PortNum) || raw1394_set_port(handle_bc, PortNum)) {
        outStr << "FirewirePort: error setting port to " << PortNum << std::endl;
        raw1394_destroy_handle(handle);
        raw1394_destroy_handle(handle_bc);
        handle = NULL;
        handle_bc = NULL;
        return false;
    }
    outStr << "FirewirePort: successfully initialized port " << PortNum << std::endl;

#if 1
    // IMPORTANT: Disable Cycle Start Packet, no isochronous
    int rc = 0;  // return code
    quadlet_t data_stop_cmc = bswap_32(0x100);
    rc = raw1394_write(handle,
                       raw1394_get_local_id(handle),
                       CSR_REGISTER_BASE + CSR_STATE_CLEAR,
                       4,
                       &data_stop_cmc);
    if (rc) {
        outStr << "*****Error: can NOT disable cycle start packet" << std::endl;
    } else {
        outStr << "FirewirePort: successfully disabled cycle start packet" << std::endl;
    }
#endif

    return ScanNodes();
}

FirewirePort::~FirewirePort()
{
    Cleanup();
}

int FirewirePort::NumberOfUsers(void)
{
    // try to count of many users on the handle
    char command[256];
    sprintf(command, "lsof /dev/fw%d 2> /dev/null", PortNum);
    FILE * fp;
    int status;
    char path[512];
    fp = popen(command, "r");
    if (fp == NULL) {
        outStr << "FirewirePort: unable to start lsof to count number of users on port " << PortNum << std::endl;
    }
    std::string result;
    int numberOfLines = 0;
    while (fgets(path, 512, fp) != NULL) {
        result.append(path);
        numberOfLines++;
    }
    status = pclose(fp);
    if (status == -1) {
        outStr << "FirewirePort: error in pclose after lsof call" << std::endl;
    }
    int numberOfUsers = numberOfLines - 2; // info line from lsof and lsof itself
    if (numberOfUsers > 1) {
        outStr << "FirewirePort::NumberOfUsers: found " << numberOfUsers << " users" << std::endl
               << "Full lsof result:" << std::endl
               << result << std::endl;
    }
    return numberOfUsers;
}

void FirewirePort::Cleanup(void)
{
    PortListType::iterator it = std::find(PortList.begin(), PortList.end(), this);
    if (it == PortList.end())
        outStr << "FirewirePort cleanup could not find entry for port " << PortNum << std::endl;
    else
        PortList.erase(it);
    raw1394_destroy_handle(handle);
    raw1394_destroy_handle(handle_bc);
    handle = NULL;
    handle_bc = NULL;
}

void FirewirePort::Reset(void)
{
    Cleanup();
    Init();
}

int FirewirePort::reset_handler(raw1394handle_t hdl, uint gen)
{
    int ret = 0;
    PortListType::iterator it;
    for (it = PortList.begin(); it != PortList.end(); it++) {
        if ((*it)->handle == hdl) {
            (*it)->outStr << "Firewire bus reset: generation = " << gen << std::endl;
            ret = (*it)->old_reset_handler(hdl, gen);
            (*it)->outStr << "Firewire bus reset: scanning port " << (*it)->PortNum << std::endl;
            (*it)->ScanNodes();
            break;
      }
    }
    if (it == PortList.end())
        std::cerr << "Firewire bus reset: could not find port" << std::endl;
    return ret;
}

// This function checks if there is a pending read on the IEEE-1394 bus,
// which could include a bus reset
void FirewirePort::PollEvents(void)
{
    fd_set fds;
    struct timeval timeout = {0, 0};

    int fd = raw1394_get_fd(handle);
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    int ret = select(fd+1, &fds, 0, 0, &timeout);
    if (ret > 0)
        raw1394_loop_iterate(handle);
}

bool FirewirePort::ScanNodes(void)
{
    int node, board;  // loop counters

    // Clear any existing Node2Board
    memset(Node2Board, BoardIO::MAX_BOARDS, sizeof(Node2Board));

    // Get base node id (zero out 6 lsb)
    baseNodeId = raw1394_get_local_id(handle) & 0xFFC0;
    outStr << "ScanNodes: base node id = " << std::hex << baseNodeId << std::endl;

    // iterate through all the nodes and find out their boardId
    int numNodes = raw1394_get_nodecount(handle);

    outStr << "ScanNodes: building node map for " << numNodes << " nodes:" << std::endl;
    // Iterate through all connected nodes (except for last one, which is the PC).
    for (node = 0; node < numNodes-1; node++){
        quadlet_t data;
        if (raw1394_read(handle, baseNodeId+node, 4, 4, &data)) {
            outStr << "ScanNodes: unable to read from node " << node << std::endl;
            return false;
        }
        data = bswap_32(data);
        if ((data != 0xC0FFEE) && (data != QLA1_String)) {
            outStr << "Node " << node << " is not a QLA board" << std::endl;
            continue;
        }
        // Now, read firmware version
        unsigned long fver = 0;
        if (data == QLA1_String) {
            if (raw1394_read(handle, baseNodeId+node, 7, 4, &data)) {
                outStr << "ScanNodes: unable to read firmware version from node "
                       << node << std::endl;
                return false;
            }
            data = bswap_32(data);
            fver = data;
        }
        if (raw1394_read(handle, baseNodeId+node, 0, 4, &data)) {
            outStr << "ScanNodes: unable to read status from node " << node << std::endl;
            return false;
        }
        data = bswap_32(data);
        // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
        board = (data & BOARD_ID_MASK) >> 24;
        outStr << "  Node " << node << ", BoardId = " << board 
               << ", Firmware Version = " << fver << std::endl;
        if (Node2Board[node] < BoardIO::MAX_BOARDS)
            outStr << "    Duplicate entry, previous value = "
                   << static_cast<int>(Node2Board[node]) << std::endl;
        Node2Board[node] = board;
        FirmwareVersion[board] = fver;
    }

    for (board = 0; board < BoardIO::MAX_BOARDS; board++) {
        Board2Node[board] = MAX_NODES;
        for (node = 0; node < numNodes-1; node++) {
            if (Node2Board[node] == board) {
                if (Board2Node[board] < MAX_NODES)
                    outStr << "Warning: GetNodeId detected duplicate board id for " << board << std::endl;
                Board2Node[board] = node;
            }
        }
    }

    return true;
}

bool FirewirePort::AddBoard(BoardIO *board)
{
    int id = board->BoardId;
    if ((id < 0) || (id >= BoardIO::MAX_BOARDS)) {
        outStr << "AddBoard: board number out of range: " << id << std::endl;
        return false;
    }
    BoardList[id] = board;
    if (id >= max_board)
        max_board = id+1;
    board->port = this;

    // update BoardExistMask_
    BoardExistMask_ = (BoardExistMask_ | (1 << id));

    // start addr for the board
    const nodeaddr_t arm_start_addr = 0xffffff000000 + (id << 20);
    board->arm_addr = arm_start_addr;
    const size_t arm_length = 30 * 4;  // max 30 quadlets data

    std::cout << "arm_addr = " << std::hex << arm_start_addr << std::endl;

    // arm initial buffer
    byte_t arm_init_buffer[arm_length];
    memset(arm_init_buffer, 0, arm_length);

    // arm_handle
    raw1394_arm_reqhandle arm_reqhandle;
    char arm_callback_context[] = "FPGA QLA board";
    arm_reqhandle.pcontext = arm_callback_context;
    arm_reqhandle.arm_callback = NULL;
    int access_mode = RAW1394_ARM_WRITE | RAW1394_ARM_READ;

    // register
    int rc = 0;
    rc = raw1394_arm_register(handle,
                              arm_start_addr,
                              arm_length,
                              arm_init_buffer,
                              (octlet_t) &arm_reqhandle,
                              access_mode,
                              0,
                              0);
    if (rc) {
        std::cerr << "**** Error: failed to setup arm register, error "
                  << strerror(errno) << std::endl;
    } else {
        std::cout << "Board " << id << "  addr = " << arm_start_addr << " arm successful" << std::endl;
    }

    return true;
}

bool FirewirePort::RemoveBoard(unsigned char boardId)
{
    if (boardId >= BoardIO::MAX_BOARDS) {
        outStr << "RemoveBoard: board number out of range: " << boardId << std::endl;
        return false;
    }
    BoardIO *board = BoardList[boardId];
    if (!board) {
        outStr << "RemoveBoard: board not found: " << boardId << std::endl;
        return false;
    }

    // update BoardExistMask_
    BoardExistMask_ = (BoardExistMask_ & (~(1 << boardId)));

    BoardList[boardId] = 0;
    if (boardId >= max_board-1) {
        // If max_board was just removed, find the new max_board
        max_board = 0;
        for (int bd = 0; bd < boardId; bd++)
            if (BoardList[bd]) max_board = bd+1;
    }
    board->port = 0;
    return true;
}

BoardIO *FirewirePort::GetBoard(unsigned char boardId) const
{
    return BoardList[boardId];
}

int FirewirePort::GetNodeId(unsigned char boardId) const
{
    if (boardId < BoardIO::MAX_BOARDS)
        return Board2Node[boardId];
    else
        return MAX_NODES;
}

unsigned long FirewirePort::GetFirmwareVersion(unsigned char boardId) const
{
    if (boardId < BoardIO::MAX_BOARDS)
        return FirmwareVersion[boardId];
    else
        return 0;
}

bool FirewirePort::ReadAllBoards(void)
{
    if (!handle) {
        outStr << "ReadAllBoards: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }
    bool allOK = true;
    bool noneRead = true;
    for (int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            bool ret = ReadBlock(board, 0, BoardList[board]->GetReadBuffer(),
                                 BoardList[board]->GetReadNumBytes());
            if (ret) noneRead = false;
            else allOK = false;
            BoardList[board]->SetReadValid(ret);
        }
    }
    if (noneRead) {
        PollEvents();
    }
    return allOK;
}

bool FirewirePort::ReadAllBoardsBroadcast(void)
{
    if (!handle) {
        outStr << "ReadAllBoardsBroadcast: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }

    bool allOK = true;
    bool noneRead = true;
    for (int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            int rc;
//            rc = raw1394_arm_get_buf(handle,
//                                     BoardList[board]->GetArmAddress(),
//                                     BoardList[board]->GetReadNumBytes(),
//                                     BoardList[board]->GetReadBuffer());

            // pull events
            PollEvents();

            const int readSize = 13;  // 1 seq + 12 data, unit quadlet
            quadlet_t readBuff[readSize];
            raw1394_arm_get_buf(handle,
                                BoardList[board]->GetArmAddress(), readSize * 4, readBuff);

            outStr << "Arm_addr = " << std::hex << BoardList[board]->GetArmAddress() << std::endl;

            for (int i = 0; i < readSize; i++) {
                outStr.fill('0');
                outStr << " " << std::hex << std::setw(8) << bswap_32(readBuff[i]) << std::endl;
            }
            outStr << std::endl;

            if (!rc) noneRead = false;
            else allOK = false;
            BoardList[board]->SetReadValid(!rc);
        }
    }

    outStr << "===================  END ============================\n" << std::endl;


    if (noneRead) {
        PollEvents();
    }
    return allOK;
}


bool FirewirePort::WriteAllBoards(void)
{
    if (!handle) {
        outStr << "WriteAllBoards: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }
    bool allOK = true;
    bool noneWritten = true;
    for (int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            quadlet_t *buf = BoardList[board]->GetWriteBuffer();
            unsigned int numBytes = BoardList[board]->GetWriteNumBytes();
            unsigned int numQuads = numBytes/4;
            // Currently (Rev 1 firmware), the last quadlet (Status/Control register)
            // is done as a separate quadlet write.
            bool ret = WriteBlock(board, 0, buf, numBytes-4);
            if (ret) noneWritten = false;
            else allOK = false;
            quadlet_t ctrl = buf[numQuads-1];  // Get last quadlet
            bool ret2 = true;
            if (ctrl) {    // if anything non-zero, write it
                ret2 = WriteQuadlet(board, 0, ctrl);
                if (ret2) noneWritten = false;
                else allOK = false;
            }
            // SetWriteValid clears the buffer if the write was valid
            BoardList[board]->SetWriteValid(ret&&ret2);
        }
    }
    if (noneWritten)
        PollEvents();
    return allOK;
}

bool FirewirePort::WriteAllBoardsBroadcast(void)
{
    // check hanle
    if (!handle) {
        outStr << "WriteAllBoardsBroadcast: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }

    // sanity check vars
    bool allOK = true;
    bool noneWritten = true;

    // loop 1: broadcast write block

    // construct broadcast write buffer
    const int numOfChannel = 4;
    quadlet_t bcBuffer[numOfChannel * MAX_NODES + 1];  // 1 for sequence + fpga board status
    memset(bcBuffer, 0, sizeof(bcBuffer));
    int bcBufferOffset = 0; // the offset for new data to be stored in bcBuffer
    int numOfBoards = 0;

    // set the first quadlet
//    bcBuffer[0] = bswap_32((WriteAllBoardsBroadcastSequence_ << 16) + BoardExistMask_);
//    bcBufferOffset = bcBufferOffset + 4;   // 1 quadlet data

    for (int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            numOfBoards++;
            quadlet_t *buf = BoardList[board]->GetWriteBuffer();
            unsigned int numBytes = BoardList[board]->GetWriteNumBytes();
            memcpy(bcBuffer + bcBufferOffset/4, buf, numBytes-4);
            // bcBufferOffset equals total numBytes to write, when the loop ends
            bcBufferOffset = bcBufferOffset + numBytes - 4;
        }
    }

    // now broadcast out the huge packet
    bool ret = true;
    ret = WriteBlockBroadcast(0xffffff000000,  // now the address is hardcoded
                              bcBuffer,
                              bcBufferOffset);


    // loop 2: send out control quadlet if necessary
    for (int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            quadlet_t *buf = BoardList[board]->GetWriteBuffer();
            unsigned int numBytes = BoardList[board]->GetWriteNumBytes();
            unsigned int numQuads = numBytes/4;
            quadlet_t ctrl = buf[numQuads-1];  // Get last quedlet
            bool ret2 = true;
            if (ctrl) {  // if anything non-zero, write it
                ret2 = WriteQuadlet(board, 0, ctrl);
                if (ret2) noneWritten = false;
                else allOK = false;
            }
            // SetWriteValid clears the buffer if the write was valid
            BoardList[board]->SetWriteValid(ret&&ret2);
        }
    }

    // pullEvents
    if (noneWritten) {
        PollEvents();
    }

    // sequence number from 16 bits 0 to 65535
    WriteAllBoardsBroadcastSequence_++;
    if (WriteAllBoardsBroadcastSequence_ == 65536) {
        WriteAllBoardsBroadcastSequence_ = 0;
    }

    // return
    return allOK;
}

bool FirewirePort::ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data)
{
    int node = GetNodeId(boardId);
    if (node < MAX_NODES)
        return !raw1394_read(handle, baseNodeId+node, addr, 4, &data);
    else
        return false;
}

bool FirewirePort::WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data)
{
    int node = GetNodeId(boardId);
    if (node < MAX_NODES)
        return !raw1394_write(handle, baseNodeId+node, addr, 4, &data);
    else
        return false;
}

bool FirewirePort::WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data)
{
    // special case of WriteBlockBroadcast
    // nbytes = 4
    return WriteBlockBroadcast(addr, &data, 4);
}


bool FirewirePort::ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data,
                             unsigned int nbytes)
{
    int node = GetNodeId(boardId);
    if (node < MAX_NODES)
        return !raw1394_read(handle, baseNodeId+node, addr, nbytes, data);
    else
        return false;
}

bool FirewirePort::WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data,
                              unsigned int nbytes)
{
    int node = GetNodeId(boardId);
    if (node < MAX_NODES)
        return !raw1394_write(handle, baseNodeId+node, addr, nbytes, data);
    else
        return false;
}


bool FirewirePort::WriteBlockBroadcast(
        nodeaddr_t addr, quadlet_t *data, unsigned int nbytes)
{
    // check handle
    if (!handle_bc) {
        outStr << "WriteQuadletBroadcast: invald firewire handle" << std::endl;
        return false;
    }

    // check address
    // ZC: maybe limit address to 8 bits reg_addr[7:0]
    //     and cheat firewire driver
    if (addr < CSR_REGISTER_BASE + CSR_CONFIG_ROM_END) {
        outStr << "WriteQuadletBroadcast: address not allowed, \n"
               << "addr should > CSR_REG_BASE + CSR_CONFIG_ROM_END" << std::endl;
        return false;
    }

    // broadcast
    int rc;    // return code
    const nodeid_t broadcast_node_id = 0xffff;  // use node_id 0xffff to broadcast
    const unsigned long tag = 11;  // tag is random picked, not used
    // send broadcast request
    rc = raw1394_start_write(handle_bc, broadcast_node_id, addr, nbytes, data, tag);
    if (rc) {
        outStr << "WriteQuadletBroadcast: errno = " << strerror(errno) << std::endl;
        return false;
    } else {
        return true;
    }
}
