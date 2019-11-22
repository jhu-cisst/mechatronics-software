/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2011-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
#include "BoardIO.h"
#include "BasePort.h"

// Forward declarations
struct raw1394_handle;
typedef struct raw1394_handle *raw1394handle_t;
typedef int (*bus_reset_handler_t)(raw1394handle_t, unsigned int generation);

class FirewirePort : public BasePort {
public:
protected:

    raw1394handle_t handle;   // normal read/write handle
    raw1394handle_t handle_bc;  // broadcasting handle
    nodeid_t baseNodeId;

    size_t ReadErrorCounter_;

    int max_board;  // highest index of used (non-zero) entry in BoardList

    // List of all ports instantiated (for use by reset_handler)
    typedef std::vector<FirewirePort *> PortListType;
    static PortListType PortList;
    bus_reset_handler_t old_reset_handler;
    bus_reset_handler_t old_reset_handler_bc;
    // callback for 1394 bus reset event
    static int reset_handler(raw1394handle_t hdl, unsigned int gen);

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

    //****************** BasePort pure virtual methods ***********************

    PortType GetPortType(void) const { return PORT_FIREWIRE; }

    // Call lsof to count the number of users, assumes /dev/fw<port-number>
    int NumberOfUsers(void);

    void Reset(void);

    bool IsOK(void) { return (handle != NULL); }

    // Adds board(s)
    bool AddBoard(BoardIO *board);

    // Removes board
    bool RemoveBoard(unsigned char boardId);

    // Read all boards
    bool ReadAllBoards(void);

    // Read all boards broadcasting
    bool ReadAllBoardsBroadcast(void);

    // Write to all boards
    bool WriteAllBoards(void);

    // Write to all boards using broadcasting
    bool WriteAllBoardsBroadcast(void);

    // Read a quadlet from the specified board, true on success
    bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data);

    // Write a quadlet to the specified board
    bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data);

    /*!
     \brief Write a quadlet to all boards using broadcasting

     \param addr  The register address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param data  The quadlet data to be broadcasted
     \return bool  True on success or False on failure
    */
    bool WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data);

    // Read a block from the specified board
    bool ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata,
                   unsigned int nbytes);

    // Write a block to the specified board
    bool WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata,
                    unsigned int nbytes);

    /*!
     \brief Write a block of data using asynchronous broadcast

     \param addr  The starting target address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param data  The pointer to write buffer data
     \param nbytes  Number of bytes to be broadcasted
     \return bool  True on success or False on failure
    */
    bool WriteBlockBroadcast(nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes);

    /*!
     \brief Add delay (if needed) for PROM I/O operations
     The delay is 0 for FireWire.
    */
    void PromDelay(void) const {}

    //****************** FireWire specific methods ***********************

    // Getter (OBSOLETE?)
    //int GetNumOfNodes(void) const {return NumOfNodes_;}
    //unsigned char* GetNode2Board(void) {return Node2Board;}

    // Stop Cycle Start Packets
    void StopCycleStartPacket(void);

};

#endif // __FirewirePort_H__
