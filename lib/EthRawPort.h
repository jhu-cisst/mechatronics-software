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

    /**
     * \brief Write FireWire packet via Ethernet
     *
     * \param node        destination board number
     * \param buffer      address of pre-allocated buffer (includes space for Ethernet header)
     * \param length_fw   length of FireWire packet, in quadlets
     *
     * \return true on success; false otherwise
     */
    bool eth1394_write(nodeid_t node, quadlet_t* buffer, size_t length_fw, bool useEthernetBroadcast = false);

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

public:
    EthRawPort(int portNum, std::ostream &debugStream = std::cerr, EthCallbackType cb = 0);

    ~EthRawPort();

    //****************** BasePort virtual methods ***********************

    PortType GetPortType(void) const { return PORT_ETH_RAW; }

    bool IsOK(void);

    // Add board to list of boards in use
    bool AddBoard(BoardIO *board);

    // Remove board from list of boards in use
    bool RemoveBoard(unsigned char boardId);

    // ReadQuadlet in EthBasePort

    // WriteQuadlet in EthBasePort

    // Read a block from the specified board
    bool ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata,
                   unsigned int nbytes);

    // Write a block to the specified board
    bool WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata,
                    unsigned int nbytes);

    //****************** Static methods ***************************

    static void PrintFrame(unsigned char* buffer, int length);
};

#endif  // __EthRawPort_H__
