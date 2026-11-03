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
#include <stdlib.h>
#include <stdio.h>
#include "Amp1394BSwap.h"

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"

void PrintDebugStream(std::stringstream &debugStream)
{
    char line[80];
    while (debugStream.getline(line, sizeof(line)))
        std::cerr << line << std::endl;
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

    int i, j = 0;
    int bid = 0;
    bool verbose = false;
    std::string portDescription = BasePort::DefaultPort();

    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'p') {
                portDescription = argv[i]+2;
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


    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);

    BasePort *Port = PortFactory(portDescription.c_str(), debugStream);
    if (!Port) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to create port using: " << portDescription << std::endl;
        return -1;
    }
    if (!Port->IsOK()) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to initialize " << Port->GetPortTypeString() << std::endl;
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
