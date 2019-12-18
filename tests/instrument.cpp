/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * (C) Copyright 2018 Johns Hopkins University (JHU), All Rights Reserved.
 *
 * This program is used to read the Dallas DS2505 chip inside a da Vinci instrument
 * via its 1-wire interface. The 1-wire interface is implemented in the FPGA,
 * using bi-redirection digital port DOUT3, available with QLA Rev 1.4+.
 * It depends on the AmpIO library (which depends on libraw1394 and/or pcap).
 *
 * Usage: instrument [-pP] <board num>
 *        where P is the Firewire port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *
 ******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "Eth1394Port.h"
#endif
#include "AmpIO.h"

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
    int i;
#if Amp1394_HAS_RAW1394
    bool useFireWire = true;
#else
    bool useFireWire = false;
#endif
    int port = 0;
    int board = 0;

    if (argc > 1) {
        int args_found = 0;
        for (i = 1; i < argc; i++) {
            if (argv[i][0] == '-') {
                if (argv[i][1] == 'p') {
                    // -p option can be -pN, -pfwN, or -pethN, where N
                    // is the port number. -pN is equivalent to -pfwN
                    // for backward compatibility.
                    if (strncmp(argv[i]+2, "fw", 2) == 0)
                        port = atoi(argv[i]+4);
                    else if (strncmp(argv[i]+2, "eth", 3) == 0) {
                        useFireWire = false;
                        port = atoi(argv[i]+5);
                    }
                    else
                        port = atoi(argv[i]+2);
                    if (useFireWire)
                        std::cerr << "Selecting FireWire port " << port << std::endl;
                    else
                        std::cerr << "Selecting Ethernet port " << port << std::endl;
                }
                else {
                    std::cerr << "Usage: instrument <board-num> [-pP]" << std::endl
                    << "       where <board-num> = rotary switch setting (0-15)" << std::endl
                    << "             P = port number (default 0)" << std::endl;
                    return 0;
                }
            }
            else {
                if (args_found == 0)
                    board = atoi(argv[i]);
                args_found++;
            }
        }
    }

    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);
    BasePort *Port;
    if (useFireWire) {
#if Amp1394_HAS_RAW1394
        Port = new FirewirePort(port, debugStream);
        if (!Port->IsOK()) {
            PrintDebugStream(debugStream);
            std::cerr << "Failed to initialize firewire port " << port << std::endl;
            return -1;
        }
#else
        std::cerr << "FireWire not available (set Amp1394_HAS_RAW1394 in CMake)" << std::endl;
        return -1;
#endif
    }
    else {
#if Amp1394_HAS_PCAP
        Port = new Eth1394Port(port, debugStream);
        if (!Port->IsOK()) {
            PrintDebugStream(debugStream);
            std::cerr << "Failed to initialize ethernet port " << port << std::endl;
            return -1;
        }
        Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW);  // PK TEMP
#else
        std::cerr << "Ethernet not available (set Amp1394_HAS_PCAP in CMake)" << std::endl;
        return -1;
#endif
    }
    AmpIO Board(board);
    Port->AddBoard(&Board);

    AmpIO_UInt32 fver = Board.GetFirmwareVersion();
    if (fver < 7) {
        std::cerr << "Instrument read requires firmware version 7+ (detected version " << fver << ")" << std::endl;
        return -1;
    }
    AmpIO_UInt32 status = Board.ReadStatus();
    // Check whether bi-directional I/O is available
    if ((status & 0x00300000) != 0x00300000) {
        std::cerr << "QLA does not support bidirectional I/O (QLA Rev 1.4+ required)" << std::endl;
        return -1;
    }

    // Now, we try to read the Dallas chip. This will also populate the status field.
    unsigned char buffer[2048];  // Buffer for entire contents of DS2505 memory (2 Kbytes)
    bool ret = Board.DallasReadMemory(0, (unsigned char *) buffer, sizeof(buffer));
    if (!Board.DallasReadStatus(status)) {
        std::cerr << "Failed to read DS2505 status" << std::endl;
        return -1;
    }
    if ((status & 0x00000001) != 0x00000001) {
        std::cerr << "DS2505 interface not enabled (hardware problem)" << std::endl;
        return -1;
    }
    unsigned char ds_reset = static_cast<unsigned char>((status & 0x00000006)>>1);
    if (ds_reset != 1) {
        std::cerr << "Failed to communicate with DS2505" << std::endl;
        if (ds_reset == 2)
            std::cerr << "  - DOUT3 did not reach high state -- is pullup resistor missing?" << std::endl;
        else if (ds_reset == 3)
            std::cerr << "  - Did not received ACK from DS2505 -- is dMIB signal jumpered?" << std::endl;
        return -1;
    }
    unsigned char family_code = static_cast<unsigned char>((status&0xFF000000)>>24);
    if (family_code != 0x0B) {
        std::cerr << "Unknown device family code: 0x" << std::hex << family_code << " (DS2505 should be 0x0B)" << std::endl;
        return -1;
    }
    unsigned char rise_time = static_cast<unsigned char>((status&0x00FF0000)>>16);
    std::cout << "Measured rise time: " << (rise_time/49.152) << " microseconds" << std::endl;
    if (!ret) {
        std::cerr << "Failed to read instrument memory" << std::endl;
        return -1;
    }
    std::ofstream outFile("instrument.txt", std::ios::binary);
    if (outFile.good()) {
        outFile.write((char *)buffer, sizeof(buffer));
        std::cout << "Data written to instrument.txt" << std::endl;
    }
    else {
        std::cerr << "Failed to open instrument.txt for writing" << std::endl;
        return -1;
    }
    outFile.close();
    Port->RemoveBoard(board);
    return 0;
}
