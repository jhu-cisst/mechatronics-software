/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef __EthBasePort_H__
#define __EthBasePort_H__

#include <iostream>
#include "BasePort.h"

// Some useful constants related to the FireWire protocol
const unsigned int FW_QREAD_SIZE      = 16;        // Number of bytes in Firewire quadlet read request packet
const unsigned int FW_QWRITE_SIZE     = 20;        // Number of bytes in Firewire quadlet write packet
const unsigned int FW_QRESPONSE_SIZE  = 20;        // Number of bytes in Firewire quadlet read response packet
const unsigned int FW_BREAD_SIZE      = 20;        // Number of bytes in Firewire block read request
const unsigned int FW_BRESPONSE_HEADER_SIZE = 20;  // Number of bytes in Firewire block read response header (including header CRC)
const unsigned int FW_BWRITE_HEADER_SIZE = 20;     // Number of bytes in Firewire block write header (including header CRC)
const unsigned int FW_CRC_SIZE = 4;                // Number of bytes in Firewire CRC

// Firewire control word -- specific to our implementation.
// First byte is FW_CTRL_FLAGS, second byte is Firewire bus generation
const unsigned int FW_CTRL_SIZE = 2;               // Number of bytes in Firewire control word

// Firewire extra data, received after the quadlet and block responses. This is specific to our implementation.
const unsigned int FW_EXTRA_SIZE = 8;              // Number of extra bytes (4 words)

// FW_CTRL_FLAGS
const unsigned char FW_CTRL_NOFORWARD = 0x01;      // Prevent forwarding by Ethernet/Firewire bridge

class EthBasePort : public BasePort
{
public:
    //! FireWire tcode
    enum TCODE {
        QWRITE = 0,
        BWRITE = 1,
        QREAD = 4,
        BREAD = 5,
        QRESPONSE = 6,
        BRESPONSE = 7
    };

    struct FPGA_Status {
        bool FwBusReset;
        bool FwPacketDropped;
        bool EthInternalError;
        bool EthSummaryError;
        bool noForwardFlag;
        unsigned int srcPort;
        unsigned int numStateInvalid;   // Not used, except in debug builds of firmware
        unsigned int numPacketError;
        FPGA_Status() : FwBusReset(false), FwPacketDropped(false), EthInternalError(false), EthSummaryError(false),
                        noForwardFlag(false), srcPort(0), numStateInvalid(0), numPacketError(0) {}
        ~FPGA_Status() {}
    };

    typedef bool (*EthCallbackType)(EthBasePort &port, unsigned char boardId, std::ostream &debugStream);

protected:

    // Whether to use Ethernet/Firewire bridge (if false, use Ethernet only)
    bool useFwBridge;

    uint8_t fw_tl;          // FireWire transaction label (6 bits)

    EthCallbackType eth_read_callback;
    double ReceiveTimeout;      // Ethernet receive timeout (seconds)

    enum FPGA_FLAGS {
        FwBusReset = 0x01,          // Firewire bus reset is active
        FwPacketDropped = 0x02,     // Firewire packet dropped
        EthInternalError = 0x04,    // Internal FPGA error (bus access or invalid state)
        EthSummaryError = 0x08,     // Summary of Ethernet protocol errors (see Status)
        noForwardFlag = 0x10,       // 1 -> Ethernet only (no forward to Firewire)
        srcPortMask   = 0x60,       // Mask for srcPort bits
        srcPortShift  = 5           // Shift for srcPort bits
    };

    FPGA_Status FpgaStatus;     // FPGA status from extra data returned

    double FPGA_RecvTime;       // Time for FPGA to receive Ethernet packet (seconds)
    double FPGA_TotalTime;      // Total time for FPGA to receive packet and respond (seconds)

    //! Read quadlet from node (internal method called by ReadQuadlet)
    bool ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags = 0);

    //! Write quadlet to node (internal method called by WriteQuadlet)
    bool WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags = 0);

    // Read a block from the specified node (also calls ReceiveResponseNode). Internal method called by ReadBlock.
    bool ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata, unsigned int nbytes, unsigned char flags = 0);

    // Receive the block data. Internal method called by ReadBlockNode.
    bool ReceiveResponseNode(nodeid_t node, quadlet_t *rdata, unsigned int nbytes, uint8_t fw_tl, nodeid_t *src_node = 0);

    // Write a block to the specified node. Internal method called by WriteBlock and
    // WriteAllBoardsBroadcast.
    bool WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes, unsigned char flags = 0);

    // Send packet
    virtual bool PacketSend(nodeid_t node, unsigned char *packet, size_t nbytes, bool useEthernetBroadcast) = 0;

    // Receive packet
    virtual int PacketReceive(unsigned char *packet, size_t nbytes) = 0;

    // Flush all packets in receive buffer
    virtual int PacketFlushAll(void) = 0;

    // Method called by ReadAllBoards/ReadAllBoardsBroadcast if no data read
    void OnNoneRead(void);

    // Method called by WriteAllBoards/WriteAllBoardsBroadcast if no data written
    void OnNoneWritten(void);

    // Method called when Firewire bus reset has caused the Firewire generation number on the FPGA
    // to be different than the one on the PC.
    virtual void OnFwBusReset(unsigned int FwBusGeneration_FPGA);

    // Make Ethernet header (only needed for raw Ethernet)
    virtual void make_ethernet_header(unsigned char *packet, unsigned int numBytes, nodeid_t node, unsigned char flags);

    // Make control word
    void make_write_header(unsigned char *packet, unsigned int nBytes, unsigned char flags);

    void make_1394_header(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tcode, unsigned int tl);

    void make_qread_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tl);
    void make_qwrite_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned int tl);
    void make_bread_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int nBytes, unsigned int tl);
    void make_bwrite_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, quadlet_t *data, unsigned int nBytes, unsigned int tl);

    // Optimize Firewire gap count if needed
    bool OptimizeFirewireGapCount();

public:

    EthBasePort(int portNum, bool forceFwBridge, std::ostream &debugStream = std::cerr, EthCallbackType cb = 0);

    ~EthBasePort();

    void SetEthCallback(EthCallbackType callback)
    { eth_read_callback = callback; }

    double GetReceiveTimeout(void) const { return ReceiveTimeout; }

    void SetReceiveTimeout(double timeSec) { ReceiveTimeout = timeSec; }

    // Return FPGA status related to Ethernet interface
    void GetFpgaStatus(FPGA_Status &status) const { status = FpgaStatus; }

    // Return time required to receive Ethernet packet on FPGA, in seconds.
    double GetFpgaReceiveTime(void) const { return FPGA_RecvTime; }

    // Return time required to receive and respond to Ethernet packet on FPGA, in seconds.
    // This is the last data value placed in the response packet, but it is a little
    // lower (by about 0.7 microseconds) because it does not include the time to write this
    // value to the KSZ8851 or to queue the packet into the transmit buffer.
    double GetFpgaTotalTime(void) const { return FPGA_TotalTime; }

    //****************** Virtual methods ***************************
    // Implementations of pure virtual methods from BasePort

    // For now, always returns 1
    int NumberOfUsers(void) { return 1; }

    unsigned int GetBusGeneration(void) const { return FwBusGeneration; }

    void UpdateBusGeneration(unsigned int gen) { FwBusGeneration = gen; }

    // virtual method in BasePort
    bool CheckFwBusGeneration(const std::string &caller, bool doScan = false);

    /*!
     \brief Write the broadcast packet containing the DAC values and power control
    */
    bool WriteBroadcastOutput(quadlet_t *buffer, unsigned int size);

    /*!
     \brief Write the broadcast read request
    */
    bool WriteBroadcastReadRequest(unsigned int seq);

    /*!
     \brief Wait for broadcast read data to be available
    */
    void WaitBroadcastRead(void);

    bool isBroadcastReadOrdered(void) const;

    // Return clock period used for broadcast read timing measurements
    // 125 MHz for FPGA V3 Ethernet-only; otherwise 49.152 MHz
    double GetBroadcastReadClockPeriod(void) const;

    /*!
     \brief Receive the broadcast read response. This is usually a block read from
            address 0x1000 (Hub memory); in the case of Ethernet-only, it receives
            a response from an FPGA board some time after WriteBroadcastReadRequest
            is issued.
    */
    bool ReceiveBroadcastReadResponse(quadlet_t *rdata, unsigned int nbytes);

    /*!
     \brief Add delay (if needed) for PROM I/O operations
     The delay is non-zero for Ethernet.
    */
    void PromDelay(void) const;

    //****************** Static methods ***************************

    // Reverse the bits in the input
    static uint32_t BitReverse32(uint32_t input);

    // Returns the destination MAC address (6 bytes)
    // The first 3 characters are FA:61:OE, which is the CID assigned to LCSR by IEEE
    // The next 2 characters are 13:94
    // The last character is 0 (set it to the board address)
    static void GetDestMacAddr(unsigned char *macAddr);

    // Returns the destination multicast MAC address (6 bytes)
    // The first 3 characters are FB:61:OE, which is the CID assigned to LCSR by IEEE, with the multicast bit set
    // The last 3 characters are 13:94:FF
    static void GetDestMulticastMacAddr(unsigned char *macAddr);

    // Print MAC address
    static void PrintMAC(std::ostream &outStr, const char* name, const uint8_t *addr, bool swap16 = false);

    // Print IP address
    static void PrintIP(std::ostream &outStr, const char* name, const uint8_t *addr, bool swap16 = false);

    // Check Ethernet header (only for EthRawPort)
    virtual bool CheckEthernetHeader(const unsigned char *packet, bool useEthernetBroadcast);

    // Byteswap quadlets received packet (quadlet or block read response) to make it easier to work with,
    // and to be consistent with CheckFirewirePacket and PrintFirewirePacket. This is typically used to
    // byteswap the Firewire header quadlets.
    void ByteswapQuadlets(unsigned char *packet, unsigned int nbytes);

    // Get information from Firewire packet header
    // Can specify 0 (null pointer) for fields to ignore
    static void GetFirewireHeaderInfo(const unsigned char *packet, nodeid_t *src_node, unsigned int *tcode,
                                      unsigned int *tl);

    // Check if FireWire packet valid
    //   length:  length of data section (for BRESPONSE)
    //   node:    expected source node
    //   tcode:   expected tcode (e.g., QRESPONSE or BRESPONSE)
    //   tl:      transaction label
    bool CheckFirewirePacket(const unsigned char *packet, size_t length, nodeid_t node, unsigned int tcode,
                             unsigned int tl);

    // Process extra data received from FPGA
    void ProcessExtraData(const unsigned char *packet);

    static bool checkCRC(const unsigned char *packet);

    // Print FireWire packet
    static void PrintFirewirePacket(std::ostream &out, const quadlet_t *packet, unsigned int max_quads);

    static void PrintEthernetPacket(std::ostream &out, const quadlet_t *packet, unsigned int max_quads);

    static void PrintStatus(std::ostream &debugStream, uint32_t status);

    // Print Ethernet debug data; clockPeriod is in seconds
    //   CheckDebugHeader   looks for debug header string ("DBGx", where x is the version)
    //   PrintDebugData     from higher-level module (EthernetIO)
    //   PrintDebugDataKSZ  from lower-level module on FPGA V2 (KSZ8851)
    //   PrintDebugDataRTI  from lower-level module on FPGA V3 (EthRtInterface)
    static bool CheckDebugHeader(std::ostream &debugStream, const std::string &caller, const char *header,
                                 unsigned int version);
    static void PrintDebugData(std::ostream &debugStream, const quadlet_t *data, double clockPeriod);
    static void PrintDebugDataKSZ(std::ostream &debugStream, const quadlet_t *data, double clockPeriod);
    static void PrintDebugDataRTL(std::ostream &debugStream, const quadlet_t *data, const char *portName);
    static void PrintDebugDataRTI(std::ostream &debugStream, const quadlet_t *data, double clockPeriod);
};

#endif  // __EthBasePort_H__
