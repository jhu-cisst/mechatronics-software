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
#include <libraw1394/raw1394.h>
#include "BoardIO.h"

class FirewirePort {
public:
    enum { MAX_NODES = 64,     // maximum number of nodes (IEEE-1394 limit)
           MAX_BOARDS = 16,    // maximum number of Amp boards (16-pos switch)
    };
protected:
    int PortNum;

    raw1394handle_t handle;
    nodeid_t baseNodeId;

    unsigned char Node2Board[MAX_NODES];
    unsigned char Board2Node[MAX_BOARDS];

    int max_board;  // highest index of used (non-zero) entry in BoardList
    BoardIO *BoardList[MAX_BOARDS];

    // List of all ports instantiated (for use by reset_handler)
    typedef std::vector<FirewirePort *> PortListType;
    static PortListType PortList;
    // callback for 1394 bus reset event
    static int reset_handler(raw1394handle_t hdl, uint gen);

    // Look for nodes on the bus
    bool ScanNodes(void);

public:
    FirewirePort(int portNum);
    ~FirewirePort();

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
