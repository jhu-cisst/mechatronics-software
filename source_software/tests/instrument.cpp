/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * (C) Copyright 2018-2023 Johns Hopkins University (JHU), All Rights Reserved.
 *
 * This program is used to read the Dallas DS2505 chip inside a da Vinci instrument
 * via its 1-wire interface. The 1-wire interface is implemented in the FPGA,
 * using either the direct 1-wire interface or an external DS2480B driver chip.
 * The direct 1-wire interface uses bi-redirectional digital port DOUT3,
 * which is available with QLA Rev 1.4+. The external DS2480B driver chip is
 * located either on the dMIB (Rev F+) or on an external dongle.
 * This program depends on the AmpIO library (which may depend on libraw1394
 * and/or pcap).
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
    int i;
    int board = 0;

    std::string portDescription = BasePort::DefaultPort();

    if (argc > 1) {
        int args_found = 0;
        for (i = 1; i < argc; i++) {
            if (argv[i][0] == '-') {
                if (argv[i][1] == 'p') {
                    portDescription = argv[i]+2;
                }
                else {
                    std::cerr << "Usage: instrument <board-num> [-pP]" << std::endl
                              << "       where <board-num> = rotary switch setting (0-15)" << std::endl
                              << "             P = port number (default 0)" << std::endl
                              << "       can also specify -pfwP, -pethP or -pudp" << std::endl;
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

    AmpIO Board(board);
    Port->AddBoard(&Board);

    uint32_t fver = Board.GetFirmwareVersion();
    if (fver < 7) {
        std::cerr << "Instrument read requires firmware version 7+ (detected version " << fver << ")" << std::endl;
        Port->RemoveBoard(board);
        delete Port;
        return -1;
    }
    uint32_t status = Board.ReadStatus();

    // Now, we try to read the Dallas chip. This will also populate the status field.
    unsigned char buffer[2048];  // Buffer for entire contents of DS2505 memory (2 Kbytes)
    bool ret = Board.DallasReadMemory(0, (unsigned char *) buffer, sizeof(buffer));
    if (!Board.DallasReadStatus(status)) {
        std::cerr << "Failed to read DS2505 status" << std::endl;
        Port->RemoveBoard(board);
        delete Port;
        return -1;
    }
    // No longer need these
    Port->RemoveBoard(board);
    delete Port;
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
        std::cerr << "Unknown device family code: 0x" << std::hex << static_cast<unsigned int>(family_code)
                  << " (DS2505 should be 0x0B)" << std::endl;
        return -1;
    }
    bool useDS2480B = (status & 0x00008000) == 0x00008000;
    if (!useDS2480B) {
        unsigned char rise_time = static_cast<unsigned char>((status&0x00FF0000)>>16);
        std::cout << "Measured rise time: " << (rise_time/49.152) << " microseconds" << std::endl;
    }
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
    return 0;
}
