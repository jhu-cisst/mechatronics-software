/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * This is a variation of block1394.c that relies on the Amp1394 library.
 * Thus, it probably should not be in this directory, which otherwise contains
 * low-level utility programs that do not depend on Amp1394. Also, it could
 * be used for Firewire, with the caveat that it would duplicate the block1394.c
 * functionality in that case.
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
    bool isQuad1394 = (strstr(argv[0], "quadeth1394") != 0);

    int i,j;
    int bid = BoardIO::MAX_BOARDS;
    bool verbose = false;
    BasePort::PortType desiredPort = BasePort::PORT_ETH_UDP;
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
            std::cout << "Usage: " << argv[0] << " [-pP] [-nN] [-v] <address in hex> [value to write in hex]" << std::endl;
        else
            std::cout << "Usage: " << argv[0] << " [-pP] [-nN] [-v] <address in hex> <size in quadlets> [write data quadlets in hex]" << std::endl;
         std::cout << "       where P = port number, N = node number" << std::endl
                   << "                 can also specify -pethP or -pudp[xx.xx.xx.xx]" << std::endl;
        exit(0);
    }


    BasePort* Port = NULL;
    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);

#if Amp1394_HAS_PCAP
    if (desiredPort == BasePort::PORT_ETH_RAW)
        Port = new EthRawPort(port, debugStream);
    else
        Port = new EthUdpPort(port, IPaddr, debugStream);
#else
     Port = new EthUdpPort(port, IPaddr, debugStream);
#endif

    if (!Port->IsOK()) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to initialize ethernet port " << port << std::endl;
        return -1;
    }
    else if (verbose) {
        PrintDebugStream(debugStream);
    }
    Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW);  // PK TEMP

    std::vector<AmpIO*> BoardList;
    BoardList.push_back(new AmpIO(bid));
    Port->AddBoard(BoardList[0]);


    // Quadlet R/W
    if (isQuad1394 && (args_found == 1))
    {
        if (Port->ReadQuadlet(bid, addr, (*data)))
            std::cout << "0x" << std::hex << data[0] << "\n";
        else
            std::cerr << "ReadQuadlet Failed \n";
    }
    else if (isQuad1394 && (args_found == 2))
    {
        std::cout << "WriteQuadlet\n";
        if (!Port->WriteQuadlet(bid, addr, (*data) )) {
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
        else
            std::cerr << "ReadBlock Failed \n";
    }
    else
    {
        if (!Port->WriteBlock(bid, addr, data, size * 4)) {
            std::cerr << "WriteBlock Failed \n";
        }
    }

    delete Port;
    return 0;
}
