/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet

  (C) Copyright 2014-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "PortFactory.h"

#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "EthRawPort.h"
#endif
#if Amp1394_HAS_EMIO
#include "ZynqEmioPort.h"
#endif
#include "EthUdpPort.h"

BasePort * PortFactory(const char * args, std::ostream & debugStream)
{
    BasePort * port = 0;
    int portNumber = 0;
    std::string IPaddr = ETH_UDP_DEFAULT_IP;
    bool fwBridge = false;

    BasePort::PortType portType = BasePort::DefaultPortType();

    if (!BasePort::ParseOptions(args, portType, portNumber, IPaddr, fwBridge)) {
        debugStream << "PortFactory: Failed to parse option: " << args << std::endl;
        return port;
    }

    switch (portType) {

    case BasePort::PORT_FIREWIRE:
#if Amp1394_HAS_RAW1394
        port = new FirewirePort(portNumber, debugStream);
#else
        debugStream << "PortFactory: FireWire not available (set Amp1394_HAS_RAW1394 in CMake)" << std::endl;
#endif
        break;

    case BasePort::PORT_ETH_UDP:
        port = new EthUdpPort(portNumber, IPaddr, fwBridge, debugStream);
        break;
    
    case BasePort::PORT_ETH_RAW:
#if Amp1394_HAS_PCAP
        port = new EthRawPort(portNumber, fwBridge, debugStream);
#else
        debugStream << "PortFactory: Raw Ethernet not available (set Amp1394_HAS_PCAP in CMake)" << std::endl;
#endif
        break;

    case BasePort::PORT_ZYNQ_EMIO:
#if Amp1394_HAS_EMIO
        port = new ZynqEmioPort(portNumber, debugStream);
#else
        debugStream << "PortFactory: Zynq EMIO not available" << std::endl;
#endif
        break;

    default:
        debugStream << "PortFactory: Unsupported port type" << std::endl;
        break;
    }
    
    return port;
}
