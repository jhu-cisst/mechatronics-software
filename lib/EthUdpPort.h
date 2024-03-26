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


#ifndef __EthUdpPort_H__
#define __EthUdpPort_H__

#include <iostream>
#include "EthBasePort.h"

struct SocketInternals;

// Default MTU=1500 (does not count 18 bytes for Ethernet frame header and CRC)
const unsigned int ETH_MTU_DEFAULT = 1500;
// Size of IPv4 header (20) and UDP header (8), counts against MTU limit
const unsigned int ETH_UDP_HEADER = 28;

class EthUdpPort : public EthBasePort
{
protected:
    SocketInternals *sockPtr;   // OS-specific internals
    std::string ServerIP;       // IP address of server (string)
    std::string MulticastIP;    // IP address for multicast (string)
    unsigned short UDP_port;    // Port on server (FPGA)

    //! Initialize EthUdp port
    bool Init(void);

    //! Cleanup EthUdp port
    void Cleanup(void);

    //! Initialize nodes on the bus; called by ScanNodes
    // \return Maximum number of nodes on bus (0 if error)
    nodeid_t InitNodes(void);

    // Send packet via UDP
    bool PacketSend(nodeid_t node, unsigned char *packet, size_t nbytes, bool useEthernetBroadcast);

    // Receive packet via UDP
    int PacketReceive(unsigned char *packet, size_t nbytes);

    // Flush all packets in receive buffer
    int PacketFlushAll(void);

public:

    EthUdpPort(int portNum, const std::string &serverIP = ETH_UDP_DEFAULT_IP,
               bool forceFwBridge = false,
               std::ostream &debugStream = std::cerr, EthCallbackType cb = 0);

    ~EthUdpPort();

    std::string GetMultiCastIP() const { return MulticastIP; }
    bool SetMulticastIP(const std::string &multicast);

    //****************** BasePort virtual methods ***********************

    PortType GetPortType(void) const { return PORT_ETH_UDP; }

    bool IsOK(void);

    unsigned int GetPrefixOffset(MsgType msg) const;
    unsigned int GetWritePostfixSize(void) const  { return FW_CRC_SIZE; }
    unsigned int GetReadPostfixSize(void) const   { return (FW_CRC_SIZE+FW_EXTRA_SIZE); }

    unsigned int GetWriteQuadAlign(void) const    { return (FW_CTRL_SIZE%sizeof(quadlet_t)); }
    unsigned int GetReadQuadAlign(void) const     { return 0; }

    // Get the maximum number of data bytes that can be read
    // (via ReadBlock) or written (via WriteBlock).
    unsigned int GetMaxReadDataSize(void) const;
    unsigned int GetMaxWriteDataSize(void) const;

    //****************** Static methods ***************************

    // Convert IP address from uint32_t to string
    static std::string IP_String(uint32_t IPaddr);

    // Convert IP address from string to uint32_t
    static uint32_t IP_ULong(const std::string &IPaddr);
};

#endif  // __EthUdpPort_H__
