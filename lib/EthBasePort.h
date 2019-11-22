/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
const unsigned int FW_BRESPONSE_HEADER_SIZE = 20;  // Number of bytes in Firewire block read response header (including CRC)
const unsigned int FW_BWRITE_HEADER_SIZE = 20;     // Number of bytes in Firewire block write header (including CRC)
const unsigned int FW_CRC_SIZE = 4;                // Number of bytes in Firewire CRC

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

    typedef bool (*EthCallbackType)(EthBasePort &port, unsigned char boardId, std::ostream &debugStream);

protected:

    uint8_t fw_tl;          // FireWire transaction label (6 bits)

    EthCallbackType eth_read_callback;
    double ReceiveTimeout;      // Ethernet receive timeout (seconds)

    // Default implementation may be sufficient (may not need to override)
    virtual bool ScanNodes(void);

    virtual int ConvertBoardToNode(unsigned char boardId, bool &useEthernetBroadcast, bool &doNotForward) const;

    static void make_1394_header(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tcode, unsigned int tl,
                                 bool doNotForward = false);

    static void make_qread_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tl, bool doNotForward);
    static void make_qwrite_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned int tl, bool doNotForward);
    static void make_bread_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int nBytes, unsigned int tl, bool doNotForward);
    static void make_bwrite_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, quadlet_t *data, unsigned int nBytes, unsigned int tl, bool doNotForward);

public:

    EthBasePort(int portNum, std::ostream &debugStream = std::cerr, EthCallbackType cb = 0);

    ~EthBasePort();

    void SetEthCallback(EthCallbackType callback)
    { eth_read_callback = callback; }

    double GetReceiveTimeout(void) const { return ReceiveTimeout; }

    void SetReceiveTimeout(double timeSec) { ReceiveTimeout = timeSec; }

    //****************** Virtual methods ***************************
    // Implementations of pure virtual methods from BasePort

    // For now, always returns 1
    int NumberOfUsers(void) { return 1; }

    // Read all boards
    bool ReadAllBoards(void);

    // Write to all boards
    bool WriteAllBoards(void);

    /*!
     \brief Write a quadlet to all boards using broadcasting

     \param addr  The register address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param data  The quadlet data to be broadcasted
     \return bool  True on success or False on failure
    */
    bool WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data);

    /*!
     \brief Add delay (if needed) for PROM I/O operations
     The delay is non-zero for Ethernet.
    */
    void PromDelay(void) const;

    //****************** Static methods ***************************

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
    static void PrintMAC(std::ostream &outStr, const char* name, const uint8_t *addr);

    // Print IP address
    static void PrintIP(std::ostream &outStr, const char* name, const uint8_t *addr);

    // Check if FireWire packet valid
    //   length:  length of data sectio (for BRESPONSE)
    //   tcode:   expected tcode (e.g., QRESPONSE or BRESPONSE)
    //   tl:      transaction label
    // PK TODO: Make it static? Need to handle outStr
    bool CheckFirewirePacket(const unsigned char *packet, size_t length, unsigned int tcode, unsigned int tl);

    static bool checkCRC(const unsigned char *packet);

    // Print FireWire packet
    static void PrintFirewirePacket(std::ostream &out, const quadlet_t *packet, unsigned int max_quads);

    static void PrintDebug(std::ostream &debugStream, unsigned short status);

    static void PrintDebugData(std::ostream &debugStream, const quadlet_t *data);
};

#endif  // __EthBasePort_H__
