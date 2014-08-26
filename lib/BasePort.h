/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Long Qian, Zihan Chen

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __BasePort_H__
#define __BasePort_H__

#include "BoardIO.h"

class BasePort
{
protected:
    virtual bool ScanNodes(void) = 0;

public:
    virtual bool IsOK(void) = 0;

    virtual void Reset(void) = 0;

    virtual int GetNodeId(unsigned char boardId) const = 0;
    virtual unsigned long GetFirmwareVersion(unsigned char boardId) const = 0;

    // Adds board(s)
    virtual bool AddBoard(BoardIO *board) = 0;

    // Removes board
    virtual bool RemoveBoard(unsigned char boardId) = 0;
    inline bool RemoveBoard(BoardIO *board) { return RemoveBoard(board->BoardId); }
    // Read all boards
    virtual bool ReadAllBoards(void) = 0;
    // Write to all boards
    virtual bool WriteAllBoards(void) = 0;
    // Read a quadlet from the specified board
    virtual bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data) = 0;

    // Write a quadlet to the specified board
    virtual bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data) = 0;

    // Read a block from the specified board
    virtual bool ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data,
                   unsigned int nbytes) = 0;

    // Write a block to the specified board
    virtual bool WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data,
                    unsigned int nbytes) = 0;


};






#endif
