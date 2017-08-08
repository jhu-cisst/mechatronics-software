/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Long Qian, Zihan Chen

  (C) Copyright 2014-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __BasePort_H__
#define __BasePort_H__

#include <iostream>
#include "BoardIO.h"

class BasePort
{
public:

    // Protocol types:
    //   PROTOCOL_SEQ_RW      sequential (individual) read and write to each board
    //   PROTOCOL_SEQ_R_BC_W  sequential read from each board, broadcast write to all boards
    //   PROTOCOL_BC_QRW      broadcast query, read, and write to/from all boards
    enum ProtocolType { PROTOCOL_SEQ_RW, PROTOCOL_SEQ_R_BC_W, PROTOCOL_BC_QRW };

protected:
    // Stream for debugging output (default is std::cerr)
    std::ostream &outStr;
    ProtocolType Protocol_;         // protocol type in use
    bool IsAllBoardsBroadcastCapable_;   // TRUE if all nodes bc capable
    bool IsAllBoardsBroadcastShorterWait_;   // TRUE if all nodes bc capable and support shorter wait
    bool IsNoBoardsBroadcastShorterWait_;   // TRUE if no nodes support the shorter wait
    unsigned int ReadSequence_;   // sequence number for WABB

    // Port Index, e.g. eth0 -> PortNum = 0
    int PortNum;
    BoardIO *BoardList[BoardIO::MAX_BOARDS];

    unsigned long FirmwareVersion[BoardIO::MAX_BOARDS];

    virtual bool ScanNodes(void) = 0;

public:

    // Constructor
    BasePort(int portNum, std::ostream &ostr = std::cerr):
        outStr(ostr),
        Protocol_(BasePort::PROTOCOL_SEQ_RW),
        IsAllBoardsBroadcastCapable_(false),
        IsAllBoardsBroadcastShorterWait_(false),
        IsNoBoardsBroadcastShorterWait_(true),
        ReadSequence_(0),
        PortNum(portNum)
    {
        memset(BoardList, 0, sizeof(BoardList));
    }

    virtual ~BasePort() {}

    virtual int NumberOfUsers(void) = 0;

    virtual bool IsOK(void) = 0;

    virtual void Reset(void) = 0;

    virtual int GetNodeId(unsigned char boardId) const = 0;
    virtual unsigned long GetFirmwareVersion(unsigned char boardId) const = 0;

    // Adds board(s)
    virtual bool AddBoard(BoardIO *board) = 0;

    // Removes board
    virtual bool RemoveBoard(unsigned char boardId) = 0;
    inline bool RemoveBoard(BoardIO *board) { return RemoveBoard(board->BoardId); }

    // Set protocol type
    void SetProtocol(ProtocolType prot) {
        if (!IsAllBoardsBroadcastCapable_ && (prot != BasePort::PROTOCOL_SEQ_RW)) {
            outStr << "***Error: not all boards support broadcasting, " << std::endl
                   << "          please upgrade your firmware"  << std::endl;
        } else {
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
        }
    }

    // Read all boards
    virtual bool ReadAllBoards(void) = 0;

    // Read all boards broadcasting
    virtual bool ReadAllBoardsBroadcast(void) = 0;

    // Write to all boards
    virtual bool WriteAllBoards(void) = 0;

    // Write to all boards using broadcasting
    virtual bool WriteAllBoardsBroadcast(void) = 0;

    // Read a quadlet from the specified board
    virtual bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data) = 0;

    // Write a quadlet to the specified board
    virtual bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data) = 0;

    //
    /*!
     \brief Write a quadlet to all boards using broadcasting

     \param addr  The register address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param data  The quadlet data to be broadcasted
     \return bool  True on success or False on failure
    */
    virtual bool WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data) = 0;

    // Read a block from the specified board
    virtual bool ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata,
                           unsigned int nbytes) = 0;

    // Write a block to the specified board
    virtual bool WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata,
                            unsigned int nbytes) = 0;

    /*!
     \brief Write a block of data using asynchronous broadcast

     \param addr  The starting target address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param data  The pointer to write buffer data
     \param nbytes  Number of bytes to be broadcasted
     \return bool  True on success or False on failure
    */
    virtual bool WriteBlockBroadcast(nodeaddr_t addr, quadlet_t *data, unsigned int nbytes) = 0;

    /*!
     \brief Add delay (if needed) for PROM I/O operations
     The delay is 0 for FireWire, and non-zero for Ethernet.
    */
    virtual void PromDelay(void) const = 0;
};

#endif
