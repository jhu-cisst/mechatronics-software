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

// Maximum Firewire packet size in bytes (at 400 Mbits/sec)
const unsigned int FW_MAX_PACKET_SIZE = 2048;

// We subtract 24 bytes for the Block Write or Block Read Response header and CRC.
// The Firewire specification will actually allow the full 2048 bytes for the data,
// but this would require a firmware update.
const unsigned int FW_MAX_DATA_SIZE = FW_MAX_PACKET_SIZE-24;

class FirewirePort : public BasePort {
public:
protected:

    raw1394handle_t handle;   // normal read/write handle
    nodeid_t baseNodeId;

    // List of all ports instantiated (for use by reset_handler)
    typedef std::vector<FirewirePort *> PortListType;
    static PortListType PortList;
    bus_reset_handler_t old_reset_handler;
    // callback for 1394 bus reset event
    static int reset_handler(raw1394handle_t hdl, unsigned int gen);

    //! Read quadlet from node (internal method called by ReadQuadlet).
    //  Parameter "flags" is not used for Firewire.
    bool ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags = 0);

    //! Write quadlet to node (internal method called by WriteQuadlet)
    //  Parameter "flags" is not used for Firewire.
    bool WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags = 0);

    // Method called by ReadAllBoards/ReadAllBoardsBroadcast if no data read
    void OnNoneRead(void);

    // Method called by WriteAllBoards/WriteAllBoardsBroadcast if no data written
    void OnNoneWritten(void);

    // Poll for IEEE 1394 events, such as bus reset.
    void PollEvents(void);

    // Initialize Firewire port
    bool Init(void);

    // Cleanup Firewire port
    void Cleanup(void);

    // Initialize nodes on the bus; called by ScanNodes
    // \return Maximum number of nodes on bus (0 if error)
    nodeid_t InitNodes(void);

    // Write the broadcast packet containing the DAC values and power control
    bool WriteBroadcastOutput(quadlet_t *buffer, unsigned int size);

    // Write a block to the specified node. Internal method called by WriteBlock and
    // WriteAllBoardsBroadcast.
    // Parameter "flags" is not used for Firewire.
    bool WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata,
                        unsigned int nbytes, unsigned char flags = 0);

    // Read a block from the specified node. Internal method called by ReadBlock.
    // Parameter "flags" is not used for Firewire.
    bool ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata,
                       unsigned int nbytes, unsigned char flags = 0);

public:
    // Initialize IEEE-1394 (Firewire) port.
    FirewirePort(int portNum, std::ostream &debugStream = std::cerr);
    ~FirewirePort();

    //****************** BasePort pure virtual methods ***********************

    PortType GetPortType(void) const { return PORT_FIREWIRE; }

    // Call lsof to count the number of users, assumes /dev/fw<port-number>
    int NumberOfUsers(void);

    bool IsOK(void) { return (handle != NULL); }

    unsigned int GetBusGeneration(void) const;

    void UpdateBusGeneration(unsigned int gen);

    unsigned int GetPrefixOffset(MsgType) const   { return 0; }
    unsigned int GetWritePostfixSize(void) const  { return 0; }
    unsigned int GetReadPrefixSize(void) const    { return 0; }
    unsigned int GetReadPostfixSize(void) const   { return 0; }

    unsigned int GetWriteQuadAlign(void) const    { return 0; }
    unsigned int GetReadQuadAlign(void) const     { return 0; }

    // Get the maximum number of data bytes that can be read
    // (via ReadBlock) or written (via WriteBlock).
    unsigned int GetMaxReadDataSize(void) const  { return FW_MAX_DATA_SIZE; }
    unsigned int GetMaxWriteDataSize(void) const { return FW_MAX_DATA_SIZE; }

    // Adds board(s)
    bool AddBoard(BoardIO *board);

    // Removes board
    bool RemoveBoard(unsigned char boardId);

    /*!
     \brief Write the broadcast read request
    */
    bool WriteBroadcastReadRequest(unsigned int seq);

    /*!
     \brief Wait for broadcast read data to be available
    */
    void WaitBroadcastRead(void);

    /*!
     \brief Add delay (if needed) for PROM I/O operations
     The delay is 0 for FireWire.
    */
    void PromDelay(void) const {}

    //****************** FireWire specific methods ***********************

    // Stop Cycle Start Packets
    void StopCycleStartPacket(void);

};

#endif // __FirewirePort_H__
