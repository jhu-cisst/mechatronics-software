/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Long Qian, Zihan Chen

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __BasePort_H__
#define __BasePort_H__

#include <iostream>
#include <vector>
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
 * There are four concrete derived classes:
 *     FirewirePort:  sends FireWire packets via FireWire
 *     EthUdpPort:    sends FireWire packets via Ethernet UDP
 *     EthRawPort:    sends FireWire packets via raw Ethernet frames (using PCAP)
 *     ZynqEmioPort:  sends FireWire packets via Zynq EMIO interface
 */

// Defined here for static methods ParseOptions and DefaultPort
#define ETH_UDP_DEFAULT_IP           "169.254.0.100"
#define ETH_UDP_MULTICAST_DEFAULT_IP "224.0.0.100"

// Some useful constants
const unsigned long BOARD_ID_MASK = 0x0f000000;  /* Mask for board_id */
const unsigned long QLA1_String = 0x514C4131;
const unsigned long dRA1_String = 0x64524131;
const unsigned long DQLA_String = 0x44514C41;
const unsigned long BCFG_String = 0x42434647;

// Maximum possible data size in bytes; this should only be used for sizing static buffers;
// the actual limit is port-specific and generally less than this.
const unsigned int MAX_POSSIBLE_DATA_SIZE = 2048;

// The FireWire node number is represented by an unsigned char (8 bits), but only 6
// bits are used for the node number (0-63). The Ethernet/FireWire bridge protocol
// uses the upper two bits as flags to indicate whether the packet should be sent
// via Ethernet broadcast/multicast (0x80) and whether the Ethernet/FireWire bridge should
// not forward (0x40) the received packet to other boards via FireWire.
const unsigned char FW_NODE_ETH_BROADCAST_MASK = 0x80;   // Mask for Ethernet broadcast or multicast
const unsigned char FW_NODE_NOFORWARD_MASK     = 0x40;   // Mask to prevent forwarding by Ethernet/FireWire bridge
const unsigned char FW_NODE_FLAGS_MASK         = 0xc0;   // Mask for above flags
const unsigned char FW_NODE_MASK               = 0x3f;   // Mask for valid FireWire node numbers (0-63)
const unsigned char FW_NODE_BROADCAST          = 0x3f;   // Value for FireWire broadcast

const unsigned char FW_TL_MASK                 = 0x3f;

class BasePort
{
public:

    enum { MAX_NODES = 64 };     // maximum number of nodes (IEEE-1394 limit)

    enum PortType { PORT_FIREWIRE, PORT_ETH_UDP, PORT_ETH_RAW, PORT_ZYNQ_EMIO };

    // Protocol types:
    //   PROTOCOL_SEQ_RW      sequential (individual) read and write to each board
    //   PROTOCOL_SEQ_R_BC_W  sequential read from each board, broadcast write to all boards
    //   PROTOCOL_BC_QRW      broadcast query, read, and write to/from all boards
    enum ProtocolType { PROTOCOL_SEQ_RW, PROTOCOL_SEQ_R_BC_W, PROTOCOL_BC_QRW };

    // Information about broadcast read.
    // With Firmware V7+, each FPGA starts a timer when it receives the broadcast query command
    // sent by the host PC. The following times are relative to this timer.
    struct BroadcastReadInfo {
        unsigned int readSizeQuads;   // Read size in quadlets
        unsigned int readSequence;    // The sequence number sent with the broadcast query command
        double updateStartTime;       // When the FPGA started updating the hub data
        double updateFinishTime;      // When the FPGA finished updating the hub data
        bool   updateOverflow;        // Whether timer overflow was detected during data update
        double readStartTime;         // When the PC started reading the hub feedback data
        double readFinishTime;        // When the PC finished reading the hub feedback data
        bool   readOverflow;          // Whether timer overflow was detected during gap or read
        double gapTime;               // Time between update finish and read start
        double gapTimeMin;            // Minimum gap time
        double gapTimeMax;            // Maximum gap time
        struct BroadcastBoardInfo {   // For each board:
            bool inUse;               //   Whether board is participating in broadcast read
            bool updated;             //   Whether successfully updated
            unsigned int blockNum;    //   Block number that contained board data
            unsigned int blockSize;   //   Number of quadlets from this board
            unsigned int sequence;    //   The sequence number received (should be same as readSequence)
            bool seq_error;           //   Sequence error mismatch on FPGA
            double updateTime;        //   When the hub feedback data was updated

            BroadcastBoardInfo() : inUse(false), updated(false), blockNum(0), blockSize(0), sequence(0),
                                   seq_error(false), updateTime(0.0) {}
            ~BroadcastBoardInfo() {}
        };
        BroadcastBoardInfo boardInfo[BoardIO::MAX_BOARDS];

        BroadcastReadInfo() : readSizeQuads(0), readSequence(0), updateStartTime(0.0), updateFinishTime(0.0),
                              updateOverflow(false), readStartTime(0.0), readFinishTime(0.0), readOverflow(false),
                              gapTime(0.0) { Clear(); }
        ~BroadcastReadInfo() {}
        unsigned int IncrementSequence();
        void PrepareForRead();
        void PrintTiming(std::ostream &outStr) const;
        void Clear(void)
        { gapTimeMin = 1.0; gapTimeMax = 0.0; }
    };

protected:
    // Stream for debugging output (default is std::cerr)
    std::ostream &outStr;
    ProtocolType Protocol_;               // protocol type in use
    bool IsAllBoardsBroadcastCapable_;    // TRUE if all nodes bc capable (Firmware Rev 4+)
    bool IsAllBoardsRev4_5_;              // TRUE if all boards are Firmware Rev 4 or 5
    bool IsAllBoardsRev4_6_;              // TRUE if all boards are Firmware Rev 4-6
    bool IsAllBoardsRev6_;                // TRUE if all boards are Firmware Rev 6 (shorter wait)
    bool IsAllBoardsRev7_;                // TRUE if all boards are Firmware Rev 7
    bool IsAllBoardsRev8_9_;              // TRUE if all boards are Firmware Rev 8 or 9

    size_t ReadErrorCounter_;

    // Port Index, e.g. eth0 -> PortNum = 0
    int PortNum;

    unsigned int FwBusGeneration;     // Current Firewire bus generation
    unsigned int newFwBusGeneration;  // New Firewire bus generation

    bool autoReScan;                // Whether to automatically re-scan after bus reset

    unsigned int NumOfNodes_;       // number of nodes (boards) on bus

    // Indicates which boards are used in the current configuration
    BoardIO *BoardList[BoardIO::MAX_BOARDS];
    unsigned int NumOfBoards_;      // number of boards in use
    unsigned int BoardInUseMask_;   // mask indicating whether board in use
    unsigned int max_board;         // highest index of used (non-zero) entry in BoardList
    unsigned char HubBoard;         // board number of hub/bridge

    // Memory for ReadAllBoards/WriteAllBoards and ReadAllBoardsBroadcast/WriteAllBoardsBroadcast
    // (used for both sequential and broadcast protocols)
    unsigned char *WriteBufferBroadcast;
    unsigned char *ReadBufferBroadcast;
    // Memory for generic use
    unsigned char *GenericBuffer;

    // For debugging
    bool rtWrite;
    bool rtRead;

    // Information about broadcast read
    BroadcastReadInfo bcReadInfo;

    // Firmware versions
    unsigned long FirmwareVersion[BoardIO::MAX_BOARDS];

    // FPGA versions
    unsigned int FpgaVersion[BoardIO::MAX_BOARDS];

    // Hardware versions (e.g., QLA1)
    unsigned long HardwareVersion[BoardIO::MAX_BOARDS];

    // List of supported hardware versions.
    // Static so that it can be initialized before calling constructor.
    static std::vector<unsigned long> SupportedHardware;

    // Mappings between board numbers and node numbers
    unsigned char Node2Board[MAX_NODES];
    nodeid_t Board2Node[BoardIO::MAX_BOARDS];

    // Returns true if Node2Board has a valid entry, which indicates that
    // the node has been initialized.
    bool isNodeValid(nodeid_t node) const
    { return (Node2Board[node] < BoardIO::MAX_BOARDS); }

    // Initialize port (called by constructor and Reset)
    virtual bool Init(void) = 0;

    // Cleanup port (called by destructor and Reset)
    virtual void Cleanup(void) = 0;

    // Whether all boards support broadcast with shorter wait
    bool IsBroadcastShorterWait(void) const
    { return (IsAllBoardsRev6_ || IsAllBoardsRev7_ || IsAllBoardsRev8_9_); }

    // Whether a valid mix of firmware for broadcast
    bool IsBroadcastFirmwareMixValid(void) const
    { return (IsAllBoardsRev4_6_ || IsAllBoardsRev7_ || IsAllBoardsRev8_9_); }

    // Sets default protocol based on firmware
    void SetDefaultProtocol(void);

    // Following initializes the generic buffer, if needed
    void SetGenericBuffer(void);

    // Following methods initialize the buffers, if needed, for
    // the real-time read and write.
    void SetReadBufferBroadcast(void);
    void SetWriteBufferBroadcast(void);

    // Return expected size for broadcast read, in quadlets
    unsigned int GetBroadcastReadSizeQuads(void) const;

    // Convenience function
    void SetReadInvalid(void);

    // Initialize nodes on the bus; called by ScanNodes
    // \return Maximum number of nodes on bus (0 if error)
    virtual nodeid_t InitNodes(void) = 0;

    // Look for nodes on the bus
    virtual bool ScanNodes(void);

    //! Read quadlet from node (internal method called by ReadQuadlet).
    //  Flags are defined above (FW_NODE_xxx) and are only used for Ethernet interface.
    virtual bool ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags = 0) = 0;

    //! Write quadlet to node (internal method called by WriteQuadlet)
    //  Flags are defined above (FW_NODE_xxx) and are only used for Ethernet interface.
    virtual bool WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags = 0) = 0;

    // Write a block to the specified node. Internal method called by WriteBlock and
    // WriteAllBoardsBroadcast.
    virtual bool WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata,
                                unsigned int nbytes, unsigned char flags = 0) = 0;

    // Read a block from the specified node. Internal method called by ReadBlock.
    virtual bool ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata,
                               unsigned int nbytes, unsigned char flags = 0) = 0;

    // Method called by ReadAllBoards/ReadAllBoardsBroadcast if no data read
    virtual void OnNoneRead(void) {}

    // Method called by WriteAllBoards/WriteAllBoardsBroadcast if no data written
    virtual void OnNoneWritten(void) {}

public:

    // Constructor
    BasePort(int portNum, std::ostream &ostr = std::cerr);

    virtual ~BasePort();

    // Get protocol
    ProtocolType GetProtocol(void) const { return Protocol_; }

    inline std::string GetProtocolString(void) const {
        return BasePort::ProtocolString(GetProtocol());
    }

    // Return string version of protocol
    static std::string ProtocolString(ProtocolType protocol);

    // Helper function for parsing protocol as string
    // Return true is protocol is one of:
    // - "sequential-read-write" or "srw" and sets type to PROTOCOL_SEQ_RW
    // - "sequential-read-broadcast-write" or "srbw" and sets type to PROTOCOL_SEQ_R_BC_W
    // - "broadcast-read-write", "brw", "broadcast-query-read-write" or "bqrw" and sets type to PROTOCOL_BC_QRW
    static bool ParseProtocol(const char * arg, ProtocolType & protocol,
                              std::ostream & ostr = std::cerr);

    // Set protocol type
    bool SetProtocol(ProtocolType prot);

    // Reset the port (call Cleanup, then Init)
    virtual void Reset(void);

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
    inline unsigned long GetFirmwareVersion(unsigned char boardId) const {
        return (boardId < BoardIO::MAX_BOARDS) ? FirmwareVersion[boardId] : 0;
    }

    /*!
     \brief Get FPGA major version

     \param boardId: board ID (rotary switch value)
     \return unsigned int: FPGA major version (1, 2 or 3)
    */
    inline unsigned int GetFpgaVersionMajor(unsigned char boardId) const {
        return (boardId < BoardIO::MAX_BOARDS) ? FpgaVersion[boardId] : 0;
    }

    /*!
     \brief Get FPGA major version as a string
     \param boardId: board ID
     \return A string that identifies the FPGA major version, "FPGA_V1", "FPGA_V2", or "FPGA_V3".
             If version could not be identified, returns "FPGA_V?".
    */
    std::string GetFpgaVersionMajorString(unsigned char boardId) const;

    /*!
     \brief Get hardware version
     \param boardId: board ID
     \return unsigned long: hardware version (a 32-bit value that identifies the type of board
             connected to the FPGA).
    */
    inline unsigned long GetHardwareVersion(unsigned char boardId) const {
        return (boardId < BoardIO::MAX_BOARDS) ? HardwareVersion[boardId] : 0;
    }

    /*!
     \brief Get hardware version as a string
     \param boardId: board ID
     \return A string that identifies the hardware version of the board connected to the FPGA (e.g., "QLA1")
     \note Assumes that all valid (non-zero) hardware versions contain printable characters.
    */
    std::string GetHardwareVersionString(unsigned char boardId) const;

    /*!
     \brief Add a hardware version to the list of supported (valid) hardware
     This method is static so that it can be called before the constructor.
    */
    static void AddHardwareVersion(unsigned long hver);

    /*!
     \brief Add a hardware version (specified as a 4-character string)
            to the list of supported (valid) hardware.
     This method is static so that it can be called before the constructor.
    */
    static void AddHardwareVersionString(const std::string &hStr);

    /*!
     \brief Add comma-separated hardware versions (specified as 4-character strings)
            to the list of supported (valid) hardware.
     This method is static so that it can be called before the constructor.
    */
    static void AddHardwareVersionStringList(const std::string &hStr);

    /*!
     \brief Whether hardware version is valid (i.e., supported hardware)
    */
    static bool HardwareVersionValid(unsigned long hver);

    // Get BroadcastReadInfo
    BroadcastReadInfo GetBroadcastReadInfo(void) const
    { return bcReadInfo; }

    // Clear gapTime min/max
    void ClearBroadcastReadInfo(void)
    {  bcReadInfo.Clear(); }

    // Return string version of PortType
    static std::string PortTypeString(PortType portType);

    // Helper function for parsing command line options.
    // In particular, this is typically called after a certain option, such as -p, is
    // recognized and it parses the rest of that option string:
    // N                  for FireWire, where N is the port number (backward compatibility)
    // fw:N               for FireWire, where N is the port number
    // eth:N              for raw Ethernet (PCAP), where N is the port number
    // ethfw:N            as above, forcing bridge to FireWire if possible
    // udp:xx.xx.xx.xx    for UDP, where xx.xx.xx.xx is the (optional) server IP address
    // udpfw:xx.xx.xx.xx  as above, forcing bridge to FireWire if possible
    static bool ParseOptions(const char *arg, PortType &portType, int &portNum, std::string &IPaddr,
                             bool &fwBridge, std::ostream &ostr = std::cerr);

    static PortType DefaultPortType(void);
    static std::string DefaultPort(void);

    //*********************** Pure virtual methods **********************************

    virtual PortType GetPortType(void) const = 0;

    inline std::string GetPortTypeString(void) const {
        return BasePort::PortTypeString(GetPortType());
    }

    virtual int NumberOfUsers(void) = 0;

    virtual bool IsOK(void) = 0;

    // Get the bus generation number
    virtual unsigned int GetBusGeneration(void) const = 0;

    // Update the bus generation number
    virtual void UpdateBusGeneration(unsigned int gen) = 0;

    enum MsgType {
        WR_CTRL,          // Offset to Control word (used in Ethernet implementations)
        WR_FW_HEADER,     // Offset to Firewire packet header (when writing)
        WR_FW_BDATA,      // Offset to Firewire block write data
        RD_FW_HEADER,     // Offset to Firewire packet header (when reading)
        RD_FW_BDATA       // Offset to Firewire block read data
    };

    // Port-specific prefix/postfix sizes (in bytes).
    // These are non-zero for the Ethernet ports.
    virtual unsigned int GetPrefixOffset(MsgType msg) const = 0;
    virtual unsigned int GetWritePostfixSize(void) const = 0;
    virtual unsigned int GetReadPostfixSize(void) const = 0;

    // Quadlet alignment offset, used to align buffers on 32-bit boundaries.
    // Probably not necessary for modern processors/compilers.
    virtual unsigned int GetWriteQuadAlign(void) const = 0;
    virtual unsigned int GetReadQuadAlign(void) const = 0;

    // Get the maximum number of data bytes that can be read
    // (via ReadBlock) or written (via WriteBlock).
    virtual unsigned int GetMaxReadDataSize(void) const = 0;
    virtual unsigned int GetMaxWriteDataSize(void) const = 0;

    //*********************** Virtual methods **********************************

    // Check whether bus generation has changed. If doScan is true, then
    // call ReScanNodes if bus generation mismatch.
    virtual bool CheckFwBusGeneration(const std::string &caller, bool doScan = false);

    // Rescan the Firewire bus (e.g., after bus reset)
    virtual bool ReScanNodes(const std::string &caller);

    // Get/Set autoReScan
    // If true, system will automatically rescan the bus when the bus generation changes
    // (e.g., due to a bus reset). If false, user application will need to rescan the bus,
    // for example, by calling ReScanNodes or CheckFwBusGeneration (with doScan = true).
    bool GetAutoReScan(void) const { return autoReScan; }
    void SetAutoReScan(bool newValue) { autoReScan = newValue; }

    // Read all boards
    virtual bool ReadAllBoards(void);

    // Read all boards broadcasting
    virtual bool ReadAllBoardsBroadcast(void);

    // Returns true if broadcast read packet contains boards in sequential order,
    // starting with lowest numbered board
    virtual bool isBroadcastReadOrdered(void) const { return true; }

    // Return clock period used for broadcast read timing measurements
    // 49.152 MHz, except 125 MHz when using FPGA V3 Ethernet-only
    virtual double GetBroadcastReadClockPeriod(void) const;

    // Write to all boards
    virtual bool WriteAllBoards(void);

    // Write to all boards using broadcasting
    virtual bool WriteAllBoardsBroadcast(void);

    // Read a quadlet from the specified board
    virtual bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data);

    // Write a quadlet to the specified board
    virtual bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data);

    // Write a No-op quadlet to reset watchdog counters on boards.
    // This is used by WriteAllBoards if no other valid command is written
    virtual bool WriteNoOp(unsigned char boardId)
    { return WriteQuadlet(boardId, 0x00, 0); }

    // Read a block from the specified board
    virtual bool ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata,
                           unsigned int nbytes);

    // Write a block to the specified board
    virtual bool WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata,
                            unsigned int nbytes);

    /*!
     \brief Write the broadcast packet containing the DAC values and power control
    */
    virtual bool WriteBroadcastOutput(quadlet_t *buffer, unsigned int size) = 0;

    /*!
     \brief Write the broadcast read request
    */
    virtual bool WriteBroadcastReadRequest(unsigned int seq) = 0;

    /*!
     \brief Wait for broadcast read data to be available
    */
    virtual void WaitBroadcastRead(void) = 0;

    /*!
     \brief Receive the broadcast read response. This is usually a block read from
            address 0x1000 (Hub memory); in the case of Ethernet-only, it receives
            a response from an FPGA board some time after WriteBroadcastReadRequest
            is issued.
    */
    virtual bool ReceiveBroadcastReadResponse(quadlet_t *rdata, unsigned int nbytes);

    /*!
     \brief Add delay (if needed) for PROM I/O operations
     The delay is 0 for FireWire, and non-zero for Ethernet.
    */
    virtual void PromDelay(void) const = 0;
};

#endif
