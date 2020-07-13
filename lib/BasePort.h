/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Long Qian, Zihan Chen

  (C) Copyright 2014-2020 Johns Hopkins University (JHU), All Rights Reserved.

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

/*
 * BasePort
 *
 * The original intent of BasePort was to abstract the port so that it would not be
 * dependent on a particular bus, such as FireWire (IEEE-1394). However, the protocol
 * is based on FireWire; for example, reading/writing of quadlets and blocks.
 * Thus, this class still maintains an abstraction about the physical port, but now
 * assumes that FireWire packets will be sent via that port.
 *
 * BasePort assumes that some number of boards will be connected, and that each board
 * may have a node number assigned by the underlying bus (this is true for FireWire).
 * The maximum number of boards is given by BoardIO::MAX_BOARDS (16) and we set the
 * maximum number of nodes (MAX_NODES) to 64, which corresponds to the FireWire limit.
 * Note, however, that FireWire assigns nodes sequentially, starting with 0, so there
 * should not be more than MAX_BOARDS+1 nodes on the bus (+1 for PC) unless other
 * FireWire device are connected.
 *
 * There are three concrete derived classes:
 *     FirewirePort:  sends FireWire packets via FireWire
 *     EthUdpPort:    sends FireWire packets via Ethernet UDP
 *     EthRawPort:    sends FireWire packets via raw Ethernet frames (using PCAP)
 */

// Some useful constants
const unsigned long BOARD_ID_MASK    = 0x0f000000;  /* Mask for board_id */
const unsigned long QLA1_String = 0x514C4131;

// The FireWire node number is represented by an unsigned char (8 bits), but only 6
// bits are used for the node number (0-63). The Ethernet/FireWire bridge protocol
// uses the upper two bits as flags to indicate whether the packet should be sent
// via Ethernet broadcast (0x80) and whether the Ethernet/FireWire bridge should
// not forward (0x40) the received packet to other boards via FireWire.
const unsigned char FW_NODE_ETH_BROADCAST_MASK = 0x80;   // Mask for Ethernet broadcast
const unsigned char FW_NODE_NOFORWARD_MASK     = 0x40;   // Mask to prevent forwarding by Ethernet/FireWire bridge
const unsigned char FW_NODE_FLAGS_MASK         = 0xc0;   // Mask for above flags
const unsigned char FW_NODE_MASK               = 0x3f;   // Mask for valid FireWire node numbers (0-63)
const unsigned char FW_NODE_BROADCAST          = 0x3f;   // Value for FireWire broadcast

const unsigned char FW_TL_MASK                 = 0x3f;

class BasePort
{
public:

    enum { MAX_NODES = 64 };     // maximum number of nodes (IEEE-1394 limit)

    enum PortType { PORT_FIREWIRE, PORT_ETH_UDP, PORT_ETH_RAW };

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
    bool IsNoBoardsBroadcastShorterWait_;    // TRUE if no nodes support the shorter wait
    bool IsAllBoardsRev7_;                   // TRUE if all boards are Firmware Rev 7
    bool IsNoBoardsRev7_;                    // TRUE if no boards are Firmware Rev 7
    unsigned int ReadSequence_;   // sequence number for WABB

    size_t ReadErrorCounter_;

    // Port Index, e.g. eth0 -> PortNum = 0
    int PortNum;

    // Indicates whether board exists in network (might not be used in current configuration)
    bool BoardExists[BoardIO::MAX_BOARDS];
    unsigned int NumOfNodes_;       // number of nodes (boards) on bus

    // Indicates which boards are used in the current configuration
    BoardIO *BoardList[BoardIO::MAX_BOARDS];
    unsigned int NumOfBoards_;      // number of boards in use
    unsigned int BoardInUseMask_;   // mask indicating whether board in use
    unsigned int max_board;         // highest index of used (non-zero) entry in BoardList
    unsigned char HubBoard;         // board number of hub/bridge

    // Firmware versions
    unsigned long FirmwareVersion[BoardIO::MAX_BOARDS];

    // Mappings between board numbers and node numbers
    unsigned char Node2Board[MAX_NODES];
    nodeid_t Board2Node[BoardIO::MAX_BOARDS];

    // Write a block to the specified node. Internal method called by WriteBlock and
    // WriteAllBoardsBroadcast.
    virtual bool WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes) = 0;

    // Method called by ReadAllBoards/ReadAllBoardsBroadcast if no data read
    virtual void OnNoneRead(void) {}

    // Method called by WriteAllBoards/WriteAllBoardsBroadcast if no data written
    virtual void OnNoneWritten(void) {}

public:

    // Constructor
    BasePort(int portNum, std::ostream &ostr = std::cerr);

    virtual ~BasePort() {}

    // Set protocol type
    bool SetProtocol(ProtocolType prot);

    // Add board to list of boards in use
    virtual bool AddBoard(BoardIO *board);

    // Remove board from list of boards in use
    virtual bool RemoveBoard(unsigned char boardId);
    inline bool RemoveBoard(BoardIO *board) { return RemoveBoard(board->BoardId); }

    // Get number of boards that were added to BoardList
    unsigned int GetNumOfBoards(void) const { return NumOfBoards_; }

    // Get highest board number present in BoardList
    unsigned int GetMaxBoardNum(void) const { return max_board; }

    BoardIO *GetBoard(unsigned char boardId) const
    { return (boardId < BoardIO::MAX_BOARDS) ? BoardList[boardId] : 0; }

    // Get total number of nodes on bus
    unsigned int GetNumOfNodes(void) const
    { return NumOfNodes_; }

    // Return node id given board id (number). See also ConvertBoardToNode.
    nodeid_t GetNodeId(unsigned char boardId) const
    { return (boardId < BoardIO::MAX_BOARDS) ? Board2Node[boardId] : static_cast<nodeid_t>(MAX_NODES); }

    // Return board number given node id
    unsigned int GetBoardId(unsigned char nodeId) const
    { return (nodeId < MAX_NODES) ? Node2Board[nodeId] : static_cast<unsigned int>(BoardIO::MAX_BOARDS); }

    // Returns board id of hub board
    unsigned int GetHubBoardId(void) const { return static_cast<unsigned int>(HubBoard); }

    // Return node number given board id. This function first masks boardId with FW_NODE_MASK and also
    // checks for the FireWire broadcast (0x3f).
    nodeid_t ConvertBoardToNode(unsigned char boardId) const;

    /*!
     \brief Get board firmware version

     \param boardId: board ID (rotary switch value)
     \return unsigned long: firmware version number
    */
    unsigned long GetFirmwareVersion(unsigned char boardId) const
    { return (boardId < BoardIO::MAX_BOARDS) ? FirmwareVersion[boardId] : 0; }

    // Return string version of PortType
    static std::string PortTypeString(PortType portType);

    // Helper function for parsing command line options.
    // In particular, this is typically called after a certain option, such as -p, is
    // recognized and it parses the rest of that option string:
    // N               for FireWire, where N is the port number (backward compatibility)
    // fwN             for FireWire, where N is the port number
    // ethN            for raw Ethernet (PCAP), where N is the port number
    // udpxx.xx.xx.xx  for UDP, where xx.xx.xx.xx is the (optional) server IP address
    static bool ParseOptions(const char *arg, PortType &portType, int &portNum, std::string &IPaddr);

    //*********************** Pure virtual methods **********************************

    virtual PortType GetPortType(void) const = 0;

    virtual int NumberOfUsers(void) = 0;

    virtual bool IsOK(void) = 0;

    virtual void Reset(void) = 0;

    // Read all boards
    virtual bool ReadAllBoards(void);

    // Read all boards broadcasting
    virtual bool ReadAllBoardsBroadcast(void) = 0;

    // Write to all boards
    virtual bool WriteAllBoards(void);

    // Write to all boards using broadcasting
    virtual bool WriteAllBoardsBroadcast(void);

    // Read a quadlet from the specified board
    virtual bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data) = 0;

    // Write a quadlet to the specified board
    virtual bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data) = 0;

    // Write a No-op quadlet to reset watchdog counters on boards.
    // This is used by WriteAllBoards if no other valid command is written
    virtual bool WriteNoOp(unsigned char boardId)
    { return WriteQuadlet(boardId, 0x00, 0); }

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
