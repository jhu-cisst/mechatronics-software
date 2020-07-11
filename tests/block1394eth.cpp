/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * This is a variation of util/block1394.c that relies on the Amp1394 library,
 * thus it can support connecting via Firewire, Ethernet Raw (PCAP) or UDP.
 *
 ******************************************************************************/

#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <byteswap.h>
#include <stdlib.h>
#include <stdio.h>
#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "EthRawPort.h"
#endif
#include "EthUdpPort.h"
#include <AmpIO.h>

void PrintDebugStream(std::stringstream &debugStream)
{
    char line[80];
    while (!debugStream.eof()) {
        debugStream.getline(line, sizeof(line));
        std::cerr << line << std::endl;
    }
    debugStream.clear();
    debugStream.str("");
}

int main(int argc, char** argv)
{
    int args_found = 0;
    nodeaddr_t addr = 0x0;
    int size = 1;
    quadlet_t data1;
    quadlet_t *data = &data1;
    bool isQuad1394 = (strstr(argv[0], "quad1394eth") != 0);

    int i,j;
    int bid = 0;
    bool verbose = false;
#if Amp1394_HAS_RAW1394
    BasePort::PortType desiredPort = BasePort::PORT_FIREWIRE;
#else
    BasePort::PortType desiredPort = BasePort::PORT_ETH_UDP;
#endif
    int port = 0;
    std::string IPaddr(ETH_UDP_DEFAULT_IP);

    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'p') {
                if (!BasePort::ParseOptions(argv[i]+2, desiredPort, port, IPaddr)) {
                    std::cerr << "Failed to parse option: " << argv[i] << std::endl;
                    return 0;
                }
            }
            else if (argv[i][1] == 'b') {
                bid = atoi(argv[i]+2);
                std::cout << "Selecting board " << bid << "\n";
            }
            else if (argv[i][1] == 'v') {
                verbose = true;
            }
        }
        else {
            if (args_found == 0)
                addr = strtoull(argv[i], 0, 16);
            else if ((args_found == 1) && (isQuad1394))
                data1 = strtoul(argv[i], 0, 16);
            else if ((args_found == 1) && (!isQuad1394)) {
                std::cout << "" << std::endl;
                size = strtoul(argv[i], 0, 10);
                /* Allocate data array, initializing contents to 0 */
                data = (quadlet_t *) calloc(sizeof(quadlet_t), size);
                if (!data) {
                    std::cerr << "Failed to allocate memory for " << size  << " quadlets\n";
                    exit(-1);
                }
            }
            else if (!isQuad1394 && (j < size))
                data[j++] = bswap_32(strtoul(argv[i], 0, 16));
            else
                std::cerr << "Warning: extra parameter: " << argv[i] << "\n";

            args_found++;
        }
    }

    if (args_found < 1) {
        if (isQuad1394)
            std::cout << "Usage: " << argv[0] << " [-pP] [-bN] [-v] <address in hex> [value to write in hex]" << std::endl;
        else
            std::cout << "Usage: " << argv[0] << " [-pP] [-bN] [-v] <address in hex> <size in quadlets> [write data quadlets in hex]" << std::endl;
         std::cout << "       where P = port number, N = board number" << std::endl
                   << "                 can also specify -pfwP, -pethP or -pudp[xx.xx.xx.xx]" << std::endl
                   << "             v specifies verbose mode" << std::endl;
        exit(0);
    }


    BasePort* Port = NULL;
    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);

    if (desiredPort == BasePort::PORT_FIREWIRE) {
#if Amp1394_HAS_RAW1394
        Port = new FirewirePort(port, debugStream);
#else
        std::cerr << "FireWire not available (set Amp1394_HAS_RAW1394 in CMake)" << std::endl;
        return -1;
#endif
    }
    else if (desiredPort == BasePort::PORT_ETH_UDP) {
        Port = new EthUdpPort(port, IPaddr, debugStream);
    }
    else if (desiredPort == BasePort::PORT_ETH_RAW) {
#if Amp1394_HAS_PCAP
        Port = new EthRawPort(port, debugStream);
#else
        std::cerr << "Raw Ethernet not available (set Amp1394_HAS_PCAP in CMake)" << std::endl;
        return -1;
#endif
    }

    if (!Port->IsOK()) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to initialize " << BasePort::PortTypeString(desiredPort) << std::endl;
        return -1;
    }
    else if (verbose) {
        PrintDebugStream(debugStream);
    }

    AmpIO Board(bid);
    Port->AddBoard(&Board);

    // Quadlet R/W
    if (isQuad1394 && (args_found == 1))
    {
        if (Port->ReadQuadlet(bid, addr, (*data)))
            std::cout << "0x" << std::hex << data[0] << "\n";
        else {
            PrintDebugStream(debugStream);
            std::cerr << "ReadQuadlet Failed \n";
        }
    }
    else if (isQuad1394 && (args_found == 2))
    {
        std::cout << "WriteQuadlet\n";
        if (!Port->WriteQuadlet(bid, addr, (*data) )) {
            PrintDebugStream(debugStream);
            std::cerr << "WriteQuadlet Failed\n";
        }
    }

    // Block R/W
    else if (!isQuad1394 && (args_found <= 2))
    {
        if (Port->ReadBlock(bid, addr, data, size * 4)) {
            for (j=0; j<size; j++)
                std::cout << "0x" << std::hex << std::setfill('0') << std::setw(8) << bswap_32(data[j]) << std::endl;
        }
        else {
            PrintDebugStream(debugStream);
            std::cerr << "ReadBlock Failed \n";
        }
    }
    else
    {
        if (!Port->WriteBlock(bid, addr, data, size * 4)) {
            PrintDebugStream(debugStream);
            std::cerr << "WriteBlock Failed \n";
        }
    }

    if (Port->IsOK())
        Port->RemoveBoard(bid);
    delete Port;
    return 0;
}
