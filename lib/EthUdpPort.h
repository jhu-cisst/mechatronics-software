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


#ifndef __EthUdpPort_H__
#define __EthUdpPort_H__

#include <iostream>
#include "EthBasePort.h"

struct SocketInternals;

#define ETH_UDP_DEFAULT_IP "169.254.0.100"

class EthUdpPort : public EthBasePort
{
protected:
    SocketInternals *sockPtr;   // OS-specific internals
    std::string ServerIP;       // IP address of server (string)
    unsigned long IP_addr;      // IP address of server (32-bit number)
    unsigned short UDP_port;    // Port on server (FPGA)

    //! Initialize EthUdp port
    bool Init(void);

    //! Cleanup EthUdp port
    void Cleanup(void);

    //! Initialize nodes on the bus; called by ScanNodes
    // \return Maximum number of nodes on bus (0 if error)
    nodeid_t InitNodes(void);

    //! Read quadlet from node (internal method called by ReadQuadlet)
    bool ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags = 0);

    //! Write quadlet to node (internal method called by WriteQuadlet)
    bool WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags = 0);

public:

    EthUdpPort(int portNum, const std::string &serverIP = ETH_UDP_DEFAULT_IP,
               std::ostream &debugStream = std::cerr, EthCallbackType cb = 0);

    ~EthUdpPort();

    //****************** BasePort virtual methods ***********************

    PortType GetPortType(void) const { return PORT_ETH_UDP; }

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

    // Convert IP address from uint32_t to string
    static std::string IP_String(uint32_t IPaddr);

    // Convert IP address from string to uint32_t
    static uint32_t IP_ULong(const std::string &IPaddr);
};

#endif  // __EthUdpPort_H__
