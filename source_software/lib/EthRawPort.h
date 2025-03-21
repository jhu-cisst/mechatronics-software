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


#ifndef __EthRawPort_H__
#define __EthRawPort_H__

#include "EthBasePort.h"

// Forward declaration
struct pcap;
typedef struct pcap pcap_t;

const unsigned int ETH_FRAME_HEADER_SIZE = 14;    // dest addr (6), src addr (6), length (2)
const unsigned int ETH_FRAME_LENGTH_OFFSET = 12;  // offset to length

//const unsigned int ETH_RAW_FRAME_MAX_SIZE = 1500; // maximum raw Ethernet frame size
const unsigned int ETH_RAW_FRAME_MAX_SIZE = 1024;   // Temporary firmware limit

class EthRawPort : public EthBasePort
{
protected:

    pcap_t *handle;
    uint8_t frame_hdr[ETH_FRAME_HEADER_SIZE];

    bool headercheck(const unsigned char *header, bool toPC) const;

    // Make Ethernet header
    void make_ethernet_header(unsigned char *packet, unsigned int numBytes, nodeid_t node, unsigned char flags);

    // Check Ethernet header
    bool CheckEthernetHeader(const unsigned char *packet, bool useEthernetBroadcast);

    //! Initialize EthRaw port
    bool Init(void);

    //! Cleanup EthRaw port
    void Cleanup(void);

    //! Initialize nodes on the bus; called by ScanNodes
    // \return Maximum number of nodes on bus (0 if error)
    nodeid_t InitNodes(void);

    // Send packet via PCAP
    bool PacketSend(nodeid_t node, unsigned char *packet, size_t nbytes, bool useEthernetBroadcast);

    // Receive packet via PCAP
    int PacketReceive(unsigned char *packet, size_t nbytes);

    // Flush all packets in receive buffer
    int PacketFlushAll(void);

public:
    EthRawPort(int portNum, bool forceFwBridge = false,
               std::ostream &debugStream = std::cerr, EthCallbackType cb = 0);

    ~EthRawPort();

    //****************** BasePort virtual methods ***********************

    PortType GetPortType(void) const { return PORT_ETH_RAW; }

    bool IsOK(void);

    unsigned int GetPrefixOffset(MsgType msg) const;
    unsigned int GetWritePostfixSize(void) const
        { return FW_CRC_SIZE; }
    unsigned int GetReadPostfixSize(void) const
        { return (FW_CRC_SIZE+FW_EXTRA_SIZE); }

    unsigned int GetWriteQuadAlign(void) const
        { return ((ETH_FRAME_HEADER_SIZE+FW_CTRL_SIZE)%sizeof(quadlet_t)); }
    unsigned int GetReadQuadAlign(void) const
        { return (ETH_FRAME_HEADER_SIZE%sizeof(quadlet_t)); }

    // Get the maximum number of data bytes that can be read
    // (via ReadBlock) or written (via WriteBlock).
    unsigned int GetMaxReadDataSize(void) const;
    unsigned int GetMaxWriteDataSize(void) const;
};

#endif  // __EthRawPort_H__
