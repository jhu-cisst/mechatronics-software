/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2011-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>

// firewire
#include "FirewirePort.h"
#include <libraw1394/raw1394.h>
#include <libraw1394/csr.h>

#define FAKEBC 1    // fake broadcast mode

FirewirePort::PortListType FirewirePort::PortList;

FirewirePort::FirewirePort(int portNum, std::ostream &debugStream):
    BasePort(portNum, debugStream)
{
    Init();
}

bool FirewirePort::Init(void)
{
    // create firewire port handle
    handle = raw1394_new_handle();
    handle_bc = raw1394_new_handle();
    if (!handle || !handle_bc) {
        outStr << "FirewirePort::Init: error, could not create handle" << std::endl;
        return false;
    }

    PortListType::const_iterator it;
    for (it = PortList.begin(); it != PortList.end(); it++) {
        if ((*it)->PortNum == PortNum)
            outStr << "FirewirePort::Init: warning, port " << PortNum
                   << " is already used (be careful about thread safety)" << std::endl;
        if ((*it)->handle == handle) // should never happen
            outStr << "FirewirePort::Init: warning, handle is already used" << std::endl;
    }
    PortList.push_back(this);

    memset(Node2Board, BoardIO::MAX_BOARDS, sizeof(Node2Board));

    // set the bus reset handlers
    old_reset_handler = raw1394_set_bus_reset_handler(handle, reset_handler);
    old_reset_handler_bc = raw1394_set_bus_reset_handler(handle_bc, reset_handler);

    // get number of ports
    int nports = raw1394_get_port_info(handle, NULL, 0);
    outStr << "FirewirePort::Init: number of ports = " << nports << std::endl;
    if (nports < PortNum) {
        outStr << "FirewirePort::Init: port " << PortNum << " does not exist" << std::endl;
        raw1394_destroy_handle(handle);
        handle = NULL;
        return false;
    }

    if (raw1394_set_port(handle, PortNum) || raw1394_set_port(handle_bc, PortNum)) {
        outStr << "FirewirePort::Init: error setting port to " << PortNum << std::endl;
        raw1394_destroy_handle(handle);
        raw1394_destroy_handle(handle_bc);
        handle = NULL;
        handle_bc = NULL;
        return false;
    }
    outStr << "FirewirePort::Init: successfully initialized port " << PortNum << std::endl;

    // stop cycle start packet
    StopCycleStartPacket();

    return ScanNodes();
}

void FirewirePort::StopCycleStartPacket(void)
{
    // IMPORTANT: Disable Cycle Start Packet, no isochronous
    int rc = 0;  // return code
    quadlet_t data_stop_cmc = bswap_32(0x100);
    rc = raw1394_write(handle,
                       raw1394_get_local_id(handle),
                       CSR_REGISTER_BASE + CSR_STATE_CLEAR,
                       4,
                       &data_stop_cmc);
    if (rc) {
        outStr << "FirewirePort::Init: error, can NOT disable cycle start packet" << std::endl;
    } else {
        outStr << "FirewirePort::Init: successfully disabled cycle start packet" << std::endl;
    }
}


FirewirePort::~FirewirePort()
{
    Cleanup();
}

int FirewirePort::NumberOfUsers(void)
{
    // try to count of many users on the handle
    char command[256];
    sprintf(command, "lsof -t /dev/fw%d 2> /dev/null", PortNum);
    FILE * fp;
    int status;
    char path[512];
    fp = popen(command, "r");
    if (fp == NULL) {
        outStr << "FirewirePort::NumberOfUsers: unable to start lsof to count number of users on port " << PortNum << std::endl;
    }
    std::string result;
    int numberOfLines = 0;
    while (fgets(path, 512, fp) != NULL) {
        result.append(path);
        numberOfLines++;
    }
    status = pclose(fp);

    if (status == -1) {
        outStr << "FirewirePort::NumberOfUsers: error in pclose after lsof call" << std::endl;
    }
    int numberOfUsers = numberOfLines - 1; // lsof itself
    if (numberOfUsers > 1) {
        sprintf(command, "lsof -R /dev/fw%d 2> /dev/null", PortNum);
        fp = popen(command, "r");
        if (fp == NULL) {
            outStr << "FirewirePort::NumberOfUsers: unable to start lsof to count number of users on port " << PortNum << std::endl;
        }
        result = "";   // clear result
        while (fgets(path, 512, fp) != NULL) {
            result.append(path);
        }
        status = pclose(fp);

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
        outStr << "FirewirePort::Cleanup: could not find entry for port " << PortNum << std::endl;
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
            (*it)->outStr << "FirewirePort::reset_handler: generation = " << gen << std::endl;
            ret = (*it)->old_reset_handler(hdl, gen);
            (*it)->outStr << "FirewirePort::reset_handler: scanning port " << (*it)->PortNum << std::endl;
            (*it)->ScanNodes();
            break;
        }
        else if ((*it)->handle_bc == hdl) {
            (*it)->outStr << "FirewirePort::reset_handler: [bc] generation = " << gen << std::endl;
            ret = (*it)->old_reset_handler_bc(hdl, gen);
            (*it)->outStr << "FirewirePort::reset_handler: [bc] scanning port " << (*it)->PortNum << std::endl;
            (*it)->ScanNodes();
            break;
        }
    }
    if (it == PortList.end())
        std::cerr << "FirewirePort::reset_handler: could not find port" << std::endl;
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

    unsigned int board;
    nodeid_t node;

    // Clear any existing Node2Board
    memset(Node2Board, BoardIO::MAX_BOARDS, sizeof(Node2Board));

    // Get base node id (zero out 6 lsb)
    baseNodeId = raw1394_get_local_id(handle) & 0xFFC0;
    outStr << "FirewirePort::ScanNodes: base node id = " << std::hex << baseNodeId << std::endl;

    // iterate through all the nodes and find out their boardId
    int numNodes = raw1394_get_nodecount(handle);
    NumOfNodes_ = numNodes - 1;

    outStr << "FirewirePort::ScanNodes: building node map for " << numNodes << " nodes:" << std::endl;
    IsAllBoardsBroadcastCapable_ = true;
    IsAllBoardsBroadcastShorterWait_ = true;
    IsNoBoardsBroadcastShorterWait_ = true;
    IsAllBoardsRev7_ = true;
    IsNoBoardsRev7_ = true;

    outStr << "FirewirePort::ScanNodes: building node map for " << numNodes << " nodes:" << std::endl;
    // Iterate through all connected nodes (except for last one, which is the PC).
    for (node = 0; node < numNodes-1; node++) {
        quadlet_t data;
        // check hardware version
        if (!ReadQuadletNode(node, 4, data)) {
            outStr << "ScanNodes: unable to read from node " << node << std::endl;
            return false;
        }
        if ((data != 0xC0FFEE) && (data != QLA1_String)) {
            outStr << "ScanNodes: node " << node << " is not a QLA board" << std::endl;
            continue;
        }

        // read firmware version
        unsigned long fver = 0;
        if (data == QLA1_String) {
            if (!ReadQuadletNode(node, 7, data)) {
                outStr << "ScanNodes: unable to read firmware version from node "
                       << node << std::endl;
                return false;
            }
            fver = data;
        }

        // read board id
        if (!ReadQuadletNode(node, 0, data)) {
            outStr << "ScanNodes: unable to read status from node " << node << std::endl;
            return false;
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
    }

    // Use broadcast by default if all firmware are bc capable
    if (IsAllBoardsBroadcastCapable_) {
        if (IsAllBoardsRev7_ || IsNoBoardsRev7_) {
            Protocol_ = BasePort::PROTOCOL_SEQ_R_BC_W;
            if (IsAllBoardsBroadcastShorterWait_)
                outStr << "FirewirePort::ScanNodes: all nodes broadcast capable and support shorter wait" << std::endl;
            else if (IsNoBoardsBroadcastShorterWait_)
                outStr << "FirewirePort::ScanNodes: all nodes broadcast capable and do not support shorter wait" << std::endl;
            else
                outStr << "FirewirePort::ScanNodes: all nodes broadcast capable and some support shorter wait" << std::endl;
        }
        else
            outStr << "FirewirePort::ScanNodes: all nodes broadcast capable, but disabled due to mix of Rev 7 and older firmware" << std::endl;
    }

    // update Board2Node
    for (board = 0; board < BoardIO::MAX_BOARDS; board++) {
        Board2Node[board] = MAX_NODES;
        for (node = 0; node < numNodes-1; node++) {
            if (Node2Board[node] == board) {
                if (Board2Node[board] < MAX_NODES)
                    outStr << "FirewirePort::ScanNodes: warning, GetNodeId detected duplicate board id for " << board << std::endl;
                Board2Node[board] = node;
            }
        }
    }

    return true;
}


bool FirewirePort::AddBoard(BoardIO *board)
{
    bool ret = BasePort::AddBoard(board);
    if (ret) {
        unsigned int id = board->BoardId;
        HubBoard = id;  // last added board would be hub board
    }
    return ret;
}

bool FirewirePort::RemoveBoard(unsigned char boardId)
{
    return BasePort::RemoveBoard(boardId);
}

// CAN BE MERGED WITH ETHBASEPORT
bool FirewirePort::ReadAllBoardsBroadcast(void)
{
    if (!handle || !handle_bc) {
        outStr << "FirewirePort::ReadAllBoardsBroadcast: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }

    bool ret;
    bool allOK = true;
    bool noneRead = true;
    int hub_node_id = GetNodeId(HubBoard);   //  ZC: NOT USE PLACEHOLDER

    //--- send out broadcast read request -----

#if 0
    quadlet_t debugData;
    nodeaddr_t debugAddr = 0x03;

    bool retdebug = !raw1394_read(handle,
                                  baseNodeId + hub_node_id,    // boardid 7
                                  debugAddr,           // read from hub addr
                                  4,                   // read all 16 boards
                                  &debugData);
    if (!retdebug) {
        raw1394_errcode_t ecode = raw1394_get_errcode(handle);
        outStr << "debug read ecode = " << ecode << " to_errno = " << raw1394_errcode_to_errno(ecode) << "  "
               << strerror(raw1394_errcode_to_errno(ecode)) << std::endl;
    }
#endif

    // sequence number from 16 bits 0 to 65535
    ReadSequence_++;
    if (ReadSequence_ == 65536) {
        ReadSequence_ = 1;
    }

    quadlet_t bcReqData = (ReadSequence_ << 16);
    if (IsAllBoardsBroadcastShorterWait_ || IsNoBoardsBroadcastShorterWait_)
        bcReqData += BoardInUseMask_;
    else
        // For a mixed set of boards, disable the shorter wait capability by "tricking" the system into thinking
        // that all nodes are used in this configuration. Otherwise, boards with the shorter wait capability may
        // attempt to transmit at the same time as boards that do not have this capability. The FireWire bus
        // arbitration protocol should prevent disaster, but this could lead to lower efficiency.
        bcReqData += ((1 << NumOfNodes_)-1);

    nodeaddr_t bcReqAddr = 0xffffffff000f;    // special address to trigger broadcast read


//#if FAKEBC
#if 1
    bcReqData = bswap_32(bcReqData);
    ret = !raw1394_write(handle,
                         baseNodeId,
                         bcReqAddr,
                         4,
                         &bcReqData);
    if (!ret) {
        raw1394_errcode_t ecode = raw1394_get_errcode(handle);
        outStr << "bbbbbbb fake ecode = " << ecode << " to_errno = " << raw1394_errcode_to_errno(ecode) << "  "
               << strerror(raw1394_errcode_to_errno(ecode)) << std::endl;
    }
#else
    WriteQuadletBroadcast(bcReqAddr, bcReqData);
#endif

    // Wait for all boards to respond with data
    timeval start, check;
    gettimeofday(&start, NULL);
    while(true) {
        gettimeofday(&check, NULL);
        double timeDiff = (check.tv_sec-start.tv_sec)*1000000 + (check.tv_usec-start.tv_usec);
        if (IsAllBoardsBroadcastShorterWait_ && (timeDiff > (10 + 5.0*NumOfBoards_)))
            break;  // Shorter wait: 10 + 5 * Nb us, where Nb is number of boards used in this configuration
        else if (timeDiff > (5.0*NumOfNodes_+5.0))
            break;  // Standard wait: 5 + 5 * Nn us, where Nn is the total number of nodes on the FireWire bus
    }

    // initialize max buffer
    const int hubReadSizeMax = 464;  // 16 * 29 = 464 max
    quadlet_t hubReadBuffer[hubReadSizeMax];
    memset(hubReadBuffer, 0, sizeof(hubReadBuffer));
    int hubReadSize;        // Actual read size (depends on firmware version)
    if (IsNoBoardsRev7_)
        hubReadSize = 272;             // Rev 1-6: 16 * 17 = 272 max (though really should have been 16*21)
    else
        hubReadSize = hubReadSizeMax;  // Rev 7

#if 1
    // raw1394_read 0 = SUCCESS, -1 = FAIL, flip return value
    ret = !raw1394_read(handle,
                        baseNodeId + hub_node_id,
                        0x1000,           // read from hub addr
                        hubReadSize * sizeof(quadlet_t),          // read all 16 boards
                        hubReadBuffer);
#endif

#if 1
    // ----- DEBUG -----------
    static int raw1394readCounter = 0;
    if (!ret) {
        raw1394readCounter++;
        raw1394_errcode_t ecode = raw1394_get_errcode(handle);
        outStr << "ecode = " << ecode << " to_errno = " << raw1394_errcode_to_errno(ecode) << "  "
                  << strerror(raw1394_errcode_to_errno(ecode)) << std::endl;
        outStr << "raw1394_read failed " << raw1394readCounter << ": " << strerror(errno) << std::endl;
    }
    // -----------------------

    for (unsigned int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            const int readSizeMax = 29;  // 1 seq + 28 data, unit quadlet (Rev 7)
            quadlet_t readBuffer[readSizeMax];

            int readSize;    // Actual size per board (depends on firmware version)
            if (IsNoBoardsRev7_)
                readSize = 17;  // Rev 1-6: 1 seq + 16 data, unit quadlet (should actually be 1 seq + 20 data)
            else
                readSize = readSizeMax;   // Rev 7

            memcpy(readBuffer, &(hubReadBuffer[readSize * board + 0]), readSize * 4);

            unsigned int seq = (bswap_32(readBuffer[0]) >> 16);

            static int errorcounter = 0;
            if (ReadSequence_ != seq) {
                errorcounter++;
                outStr << "errorcounter = " << errorcounter << std::endl;
                outStr << std::hex << seq << "  " << ReadSequence_ << "  " << (int)board << std::endl;
            }
//            std::cerr << std::hex << seq << "  " << ReadSequence_ << "  " << (int)board << std::endl;

            memcpy(BoardList[board]->GetReadBuffer(), &(readBuffer[1]), (readSize-1) * sizeof(quadlet_t));

            if (ret) noneRead = false;
            else allOK = false;
            BoardList[board]->SetReadValid(ret);
        }
    }
#endif

    if (noneRead) {
        OnNoneRead();
    }

    return allOK;
}

void FirewirePort::OnNoneRead(void)
{
    PollEvents();
}

void FirewirePort::OnNoneWritten(void)
{
    PollEvents();
}

bool FirewirePort::ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char)
{
    bool ret = !raw1394_read(handle, baseNodeId+node, addr, 4, &data);
    if (ret)
        data = bswap_32(data);
    return ret;
}

// PK TODO: Why is byteswapping done in ReadQuadletNode, but not WriteQuadletNode
bool FirewirePort::WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char)
{
    return !raw1394_write(handle, baseNodeId+node, addr, 4, &data);
}

bool FirewirePort::ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data)
{
    nodeid_t node = GetNodeId(boardId);
    return (node < MAX_NODES) ? ReadQuadletNode(node, addr, data) : false;
}

bool FirewirePort::WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data)
{
    nodeid_t node = GetNodeId(boardId);
    bool ret = false;
    if (node < MAX_NODES) {
        data = bswap_32(data);
        ret = ReadQuadletNode(node, addr, data);
    }
    return ret;
}

bool FirewirePort::WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data)
{
    // special case of WriteBlockBroadcast
    // nbytes = 4
    data = bswap_32(data);
    return WriteBlockBroadcast(addr, &data, 4);
}


bool FirewirePort::ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata,
                             unsigned int nbytes)
{
    nodeid_t node = GetNodeId(boardId);
    if (node < MAX_NODES)
        return !raw1394_read(handle, baseNodeId+node, addr, nbytes, rdata);
    else
        return false;
}

bool FirewirePort::WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata,
                                  unsigned int nbytes)
{
    return !raw1394_write(handle, baseNodeId+node, addr, nbytes, wdata);
}

bool FirewirePort::WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata,
                              unsigned int nbytes)
{
    nodeid_t node = GetNodeId(boardId);
    if (node < MAX_NODES)
        return WriteBlockNode(baseNodeId+node, addr, wdata, nbytes);
    else
        return false;
}

bool FirewirePort::WriteBlockBroadcast(
        nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes)
{
    // check handle
    if (!handle_bc) {
        outStr << "FirewirePort::WriteBlockBroadcast: invalid firewire handle" << std::endl;
        return false;
    }

    // check address
    // ZC: maybe limit address to 8 bits reg_addr[7:0]
    //     and cheat firewire driverg
    if (addr < CSR_REGISTER_BASE + CSR_CONFIG_ROM_END) {
        outStr << "FirewirePort::WriteBlockBroadcast: address not allowed, \n"
               << "addr should > CSR_REG_BASE + CSR_CONFIG_ROM_END" << std::endl;
        return false;
    }

    // broadcast
    int rc;    // return code
    const nodeid_t broadcast_node_id = 0xffff;  // use node_id 0xffff to broadcast
    const unsigned long tag = 11;  // tag is random picked, not used
    // send broadcast request
    rc = raw1394_start_write(handle_bc, broadcast_node_id, addr, nbytes, wdata, tag);
    if (rc) {
        outStr << "FirewirePort::WriteBlockBroadcast: errno = " << strerror(errno) << std::endl;
        return false;
    } else {
        return true;
    }
}
