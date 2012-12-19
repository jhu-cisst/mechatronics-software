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

#include <iostream>
#include <algorithm>
#include <string.h>
#include <byteswap.h>
#include "FirewirePort.h"

const unsigned long BOARD_ID_MASK    = 0x0f000000;  /*!< Mask for board_id */

FirewirePort::PortListType FirewirePort::PortList;

FirewirePort::FirewirePort(int portNum) : PortNum(portNum), max_board(0)
{
    memset(BoardList, 0, sizeof(BoardList));
    Init();
}

bool FirewirePort::Init(void)
{
    // create firewire port handle 
    handle = raw1394_new_handle();
    if (!handle) {
        std::cerr << "FirewirePort: could not create handle" << std::endl;
        return false;
    }

    PortListType::const_iterator it;
    for (it = PortList.begin(); it != PortList.end(); it++) {
        if ((*it)->PortNum == PortNum)
            std::cerr << "WARNING: Firewire port " << PortNum
                      << " is already used (be careful about thread safety)" << std::endl;
        if ((*it)->handle == handle) // should never happen
            std::cerr << "WARNING: Firewire handle is already used" << std::endl;
    }
    PortList.push_back(this);

    memset(Node2Board, MAX_BOARDS, sizeof(Node2Board));

    // set the bus reset handler
    old_reset_handler = raw1394_set_bus_reset_handler(handle, reset_handler);

    // get number of ports
    int nports = raw1394_get_port_info(handle, NULL, 0);
    std::cerr << "FirewirePort: number of ports = " << nports << std::endl;
    if (nports < PortNum) {
        std::cerr << "FirewirePort: port " << PortNum << " does not exist" << std::endl;
        raw1394_destroy_handle(handle);
        handle = NULL;
        return false;
    }

    if (raw1394_set_port(handle, PortNum)) {
        std::cerr << "FirewirePort: error setting port to " << PortNum << std::endl;
        raw1394_destroy_handle(handle);
        handle = NULL;
        return false;
    }
    std::cerr << "FirewirePort: successfully initialized port " << PortNum << std::endl;

    return ScanNodes();
}

FirewirePort::~FirewirePort()
{
    Cleanup();
}

void FirewirePort::Cleanup(void)
{
    PortListType::iterator it = std::find(PortList.begin(), PortList.end(), this);
    if (it == PortList.end())
        std::cerr << "FirewirePort cleanup could not find entry for port " << PortNum << std::endl;
    else
        PortList.erase(it);
    raw1394_destroy_handle(handle);
    handle = NULL;
}

void FirewirePort::Reset(void)
{
    Cleanup();
    Init();
}

int FirewirePort::reset_handler(raw1394handle_t hdl, uint gen)
{
    int ret = 0;
    std::cerr << "Firewire bus reset: generation = " << gen << std::endl;
    PortListType::iterator it;
    for (it = PortList.begin(); it != PortList.end(); it++) {
        if ((*it)->handle == hdl) {
            ret = (*it)->old_reset_handler(hdl, gen);
            std::cerr << "Firewire bus reset: scanning port " << (*it)->PortNum << std::endl;
            (*it)->ScanNodes();
            break;
      }
    }
    if (it == PortList.end())
        std::cerr << "Firewire bus reset: could not find port" << std::endl;
    return ret;
}

bool FirewirePort::ScanNodes(void)
{
    int node, board;  // loop counters

    // Clear any existing Node2Board
    memset(Node2Board, MAX_BOARDS, sizeof(Node2Board));

    // Get base node id (zero out 6 lsb)
    baseNodeId = raw1394_get_local_id(handle) & 0xFFC0;
    std::cerr << "ScanNodes: base node id = " << std::hex << baseNodeId << std::endl;

    // iterate through all the nodes and find out their boardId
    int numNodes = raw1394_get_nodecount(handle);

    std::cerr << "ScanNodes: building node map for " << numNodes << " nodes:" << std::endl;
    // Iterate through all connected nodes (except for last one, which is the PC).
    // Need a better way to make sure we only choose compatible nodes
    for (node = 0; node < numNodes-1; node++){
        quadlet_t data;
        if (raw1394_read(handle, baseNodeId+node, 0, 4, &data)) {
            std::cerr << "ScanNodes: unable to read from node " << node << std::endl;
            return false;
        }
        data = bswap_32(data);
        // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
        board = (data & BOARD_ID_MASK) >> 24;
        if ((board >= 0) && (board < MAX_BOARDS)) {
            std::cerr << "  Node " << node << ", BoardId = " << board << std::endl;
            if (Node2Board[node] < MAX_BOARDS)
                std::cerr << "    Duplicate entry, previous value = "
                          << static_cast<int>(Node2Board[node]) << std::endl;
            Node2Board[node] = board;
        }
        else  // can't happen unless BOARD_ID_MASK changes
            std::cerr << "  Node " << node << " does not appear to be a QLA" << std::endl;
    }

    for (board = 0; board < MAX_BOARDS; board++) {
        Board2Node[board] = MAX_NODES;
        for (node = 0; node < numNodes-1; node++) {
            if (Node2Board[node] == board) {
                if (Board2Node[board] < MAX_NODES)
                    std::cerr << "Warning: GetNodeId detected duplicate board id for " << board << std::endl;
                Board2Node[board] = node;
            }
        }
    }

    return true;
}

bool FirewirePort::AddBoard(BoardIO *board)
{
    if ((board->BoardId < 0) || (board->BoardId >= MAX_BOARDS)) {
        std::cerr << "AddBoard: board number out of range: " << board->BoardId << std::endl;
        return false;
    }
    BoardList[board->BoardId] = board;
    if (board->BoardId > max_board)
        max_board = board->BoardId;
    board->port = this;
    return true;
}

bool FirewirePort::RemoveBoard(unsigned char boardId)
{
    if ((boardId < 0) || (boardId >= MAX_BOARDS)) {
        std::cerr << "RemoveBoard: board number out of range: " << boardId << std::endl;
        return false;
    }
    BoardIO *board = BoardList[boardId];
    if (!board) {
        std::cerr << "RemoveBoard: board not found: " << boardId << std::endl;
        return false;
    }    
    BoardList[boardId] = 0;
    if (boardId >= max_board) {
        // If max_board was just removed, find the new max_board
        max_board = 0;
        for (int bd = 0; bd < boardId; bd++)
            if (BoardList[bd]) max_board = bd;
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
    if (boardId < MAX_BOARDS)
        return Board2Node[boardId];
    else
        return MAX_NODES;
}

bool FirewirePort::ReadAllBoards(void)
{
    if (!handle) {
        std::cerr << "ReadAllBoards: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }
    bool allOK = true;
    for (int board = 0; board <= max_board; board++) {
        if (BoardList[board]) {
            bool ret = false;
            int node = Board2Node[board];
            if (node < MAX_NODES)
                ret = !raw1394_read(handle, baseNodeId+node, 0, BoardList[board]->GetReadNumBytes(),
                                    BoardList[board]->GetReadBuffer());
            if (!ret) allOK = false;
            BoardList[board]->SetReadValid(ret);
        }
    }
    return allOK;
}

bool FirewirePort::WriteAllBoards(void)
{
    if (!handle) {
        std::cerr << "ReadAllBoards: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }
    bool allOK = true;
    for (int board = 0; board < max_board; board++) {
        if (BoardList[board]) {
            bool ret = false;
            int node = Board2Node[board];
            if (node < MAX_NODES)
                ret = !raw1394_write(handle, baseNodeId+node, 0, BoardList[board]->GetWriteNumBytes(),
                                     BoardList[board]->GetWriteBuffer());
            if (!ret) allOK = false;
            BoardList[board]->SetWriteValid(ret);
        }
    }
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
