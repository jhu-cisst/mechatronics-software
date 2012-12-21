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

#ifndef __FirewirePort_H__
#define __FirewirePort_H__

#include <vector>
#include <ostream>
#include <libraw1394/raw1394.h>
#include "BoardIO.h"

class FirewirePort {
public:
    enum { MAX_NODES = 64 };     // maximum number of nodes (IEEE-1394 limit)

protected:
    int PortNum;

    raw1394handle_t handle;
    nodeid_t baseNodeId;

    unsigned char Node2Board[MAX_NODES];
    unsigned char Board2Node[BoardIO::MAX_BOARDS];

    // Stream for debugging output (default is std::cerr)
    std::ostream &outStr;

    int max_board;  // highest index of used (non-zero) entry in BoardList
    BoardIO *BoardList[BoardIO::MAX_BOARDS];

    // List of all ports instantiated (for use by reset_handler)
    typedef std::vector<FirewirePort *> PortListType;
    static PortListType PortList;
    bus_reset_handler_t old_reset_handler;
    // callback for 1394 bus reset event
    static int reset_handler(raw1394handle_t hdl, uint gen);

    // Poll for IEEE 1394 events, such as bus reset.
    void PollEvents(void);

    // Initialize Firewire port
    bool Init(void);

    // Cleanup Firewire port
    void Cleanup(void);

    // Look for nodes on the bus
    bool ScanNodes(void);

public:
    // Initialize IEEE-1394 (Firewire) port.
    FirewirePort(int portNum, std::ostream &debugStream = std::cerr);
    ~FirewirePort();

    void Reset(void);

    bool IsOK(void) { return (handle != NULL); }

    // Adds board(s)
    bool AddBoard(BoardIO *board);
    //bool AddBoardPair(BoardIO *board1, BoardIO *board2);

    // Removes board
    bool RemoveBoard(unsigned char boardId);
    inline bool RemoveBoard(BoardIO *board) { return RemoveBoard(board->BoardId); }

    BoardIO *GetBoard(unsigned char boardId) const;

    int GetNodeId(unsigned char boardId) const;

    // Read all boards
    bool ReadAllBoards(void);

    // Write to all boards
    bool WriteAllBoards(void);

    // Read a quadlet to the specified board
    bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data);

    // Write a quadlet to the specified board
    bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data);
};

#endif // __FirewirePort_H__
