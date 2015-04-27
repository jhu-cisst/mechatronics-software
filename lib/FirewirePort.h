/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <libraw1394/csr.h>
#include "BoardIO.h"
#include <iostream>

class FirewirePort {
public:
    enum { MAX_NODES = 64 };     // maximum number of nodes (IEEE-1394 limit)

    // Protocol types:
    //   PROTOCOL_SEQ_RW      sequential (individual) read and write to each board
    //   PROTOCOL_SEQ_R_BC_W  sequential read from each board, broadcast write to all boards
    //   PROTOCOL_BC_QRW      broadcast query, read, and write to/from all boards
    enum ProtocolType { PROTOCOL_SEQ_RW, PROTOCOL_SEQ_R_BC_W, PROTOCOL_BC_QRW };

protected:
    int PortNum;

    raw1394handle_t handle;   // normal read/write handle
    raw1394handle_t handle_bc;  // broadcasting handle
    nodeid_t baseNodeId;

    unsigned char Node2Board[MAX_NODES];
    unsigned char Board2Node[BoardIO::MAX_BOARDS];
    unsigned long FirmwareVersion[BoardIO::MAX_BOARDS];

    // Stream for debugging output (default is std::cerr)
    std::ostream &outStr;

    int max_board;  // highest index of used (non-zero) entry in BoardList
    BoardIO *BoardList[BoardIO::MAX_BOARDS];
    BoardIO *HubBoard_;
    int NumOfBoards_;    // number of boards
    int NumOfNodes_;     // number of nodes on the bus (exclude PC node)

    // Broadcast Related
    unsigned int ReadSequence_;   // sequence number for WABB
    unsigned int BoardExistMask_;   // mask showing indicating whether board exists
    ProtocolType Protocol_;         // protocol type in use
    bool IsAllBoardsBroadcastCapable_;   // TRUE if all nodes bc capable

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

    // Stop Cycle Start Packets
    void StopCycleStartPacket(void);

    // Cleanup Firewire port
    void Cleanup(void);

    // Look for nodes on the bus
    bool ScanNodes(void);

public:
    // Initialize IEEE-1394 (Firewire) port.
    FirewirePort(int portNum, std::ostream &debugStream = std::cerr);
    ~FirewirePort();

    // Call lsof to count the number of users, assumes /dev/fw<port-number>
    int NumberOfUsers(void);

    // Getter
    int GetNumOfNodes(void){return NumOfNodes_;}
    unsigned char* GetNode2Board(void){return Node2Board;}

    void Reset(void);

    bool IsOK(void) { return (handle != NULL); }

    // Set hub board
    bool SetHubBoard(BoardIO *hubboard);

    // Adds board(s)
    bool AddBoard(BoardIO *board);

    // Removes board
    bool RemoveBoard(unsigned char boardId);
    inline bool RemoveBoard(BoardIO *board) { return RemoveBoard(board->BoardId); }

    BoardIO *GetBoard(unsigned char boardId) const;

    int GetNodeId(unsigned char boardId) const;
    unsigned long GetFirmwareVersion(unsigned char boardId) const;

    // Set protocol type
    void SetProtocol(ProtocolType prot);

    // Read all boards
    bool ReadAllBoards(void);

    // Read all boards broadcasting
    bool ReadAllBoardsBroadcast(void);

    // Write to all boards
    bool WriteAllBoards(void);

    // Write to all boards using broadcasting
    bool WriteAllBoardsBroadcast(void);

    // Read a quadlet from the specified board
    bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data);

    // Write a quadlet to the specified board
    bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data);

    //
    /*!
     \brief Write a quadlet to all boards using broadcasting

     \param addr  The register address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param data  The quadlet data to be broadcasted
     \return bool  True on success or False on failure
    */
    bool WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data);

    // Read a block from the specified board
    bool ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data,
                   unsigned int nbytes);

    // Write a block to the specified board
    bool WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data,
                    unsigned int nbytes);

    /*!
     \brief Write a block of data using asynchronous broadcast

     \param addr  The starting target address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param data  The pointer to write buffer data
     \param nbytes  Number of bytes to be broadcasted
     \return bool  True on success or False on failure
    */
    bool WriteBlockBroadcast(nodeaddr_t addr, quadlet_t *data, unsigned int nbytes);
};

#endif // __FirewirePort_H__
