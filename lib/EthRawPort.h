/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2020 Johns Hopkins University (JHU), All Rights Reserved.

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

const unsigned int ETH_FRAME_HEADER_SIZE = 14;  // dest addr (6), src addr (6), length (2)

class EthRawPort : public EthBasePort
{
protected:

    pcap_t *handle;
    uint8_t frame_hdr[ETH_FRAME_HEADER_SIZE];

    bool headercheck(uint8_t* header, bool toPC) const;

    /**
     * @brief send async read request to a node and wait for response.
     *
     * @param node: target node ID
     * @param addr: address to read from
     * @param length: amount of bytes of data to read
     * @param buffer: pointer to buffer where data will be saved
     *
     * @return int 0 on success, -1 on failure
     */
    int eth1394_read(nodeid_t node, nodeaddr_t addr, size_t length, quadlet_t* buffer, bool useEthernetBroadcast = false);

    int make_ethernet_header(unsigned char *buffer, unsigned int numBytes);

    //! Initialize EthRaw port
    bool Init(void);

    //! Cleanup EthRaw port
    void Cleanup(void);

    //! Initialize nodes on the bus; called by ScanNodes
    // \return Maximum number of nodes on bus (0 if error)
    nodeid_t InitNodes(void);

    //! Read quadlet from node (internal method called by ReadQuadlet)
    bool ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags = 0);

    //! Write quadlet to node (internal method called by WriteQuadlet)
    bool WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags = 0);

    // Send packet via PCAP
    bool PacketSend(char *packet, size_t nbytes, bool useEthernetBroadcast);

public:
    EthRawPort(int portNum, std::ostream &debugStream = std::cerr, EthCallbackType cb = 0);

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

    // ReadQuadlet in EthBasePort

    // WriteQuadlet in EthBasePort

    // Read a block from the specified board
    bool ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata,
                   unsigned int nbytes);

    //****************** Static methods ***************************

    static void PrintFrame(unsigned char* buffer, int length);
};

#endif  // __EthRawPort_H__
