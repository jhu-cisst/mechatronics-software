/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/******************************************************************************
 *
 * This program continuously displays the status of the FPGA V3 Ethernet switch
 * It relies on the Amp1394 library (which can depend on libraw1394 and/or pcap)
 * and on the Amp1394Console library (which may depend on curses).
 *
 * Usage: ethswitch [-pP] <board num> [<board_num>]
 *        where P is the port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *        -br or -bw specify to use a broadcast protocol
 *
 ******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <sstream>

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "Amp1394Console.h"

const unsigned int NUM_PORTS = 4;

// Must match DebugData in EthSwitch.v
struct EthSwitchData {
    uint32_t unused0;
    uint16_t numPacketRecv0;
    uint16_t numPacketSent0;
    uint16_t numPacketRecv1;
    uint16_t numPacketSent1;
    uint16_t numPacketRecv2;
    uint16_t numPacketSent2;
    uint16_t numPacketRecv3;
    uint16_t numPacketSent3;
    uint16_t fifo_active;
    uint16_t fifo_underflow;
    uint16_t fifo_empty;
    uint16_t fifo_full;
    uint32_t numCrcErrorInVec;
    uint32_t numCrcErrorOutVec;
    uint32_t TxInfoRegVec;
    uint32_t MacAddrHost0;
    uint32_t MacAddrHost1;
    uint16_t packet_dropped;
    uint16_t packet_truncated;
    uint32_t numIPv4ErrorInVec;
    uint32_t numPacketFwdVec;
};

bool GetBit(uint16_t value, uint16_t inPort, uint16_t outPort)
{
    uint16_t offset = NUM_PORTS*inPort + outPort;
    return value & (1 << offset);
}

void PrintDebugStream(std::stringstream &debugStream)
{
    std::cerr << debugStream.str() << std::endl;
    debugStream.clear();
    debugStream.str("");
}

int main(int argc, char** argv)
{
    const unsigned int lm = 5; // left margin
    unsigned int i, j;
    const int ESC_CHAR = 0x1b;
    int c;

    static const char *PortName[NUM_PORTS] = { "Eth1", "Eth2", "PS", "RT" };
    std::vector<AmpIO*> BoardList;

    std::string portDescription = BasePort::DefaultPort();

    for (i = 1; i < (unsigned int)argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'p') {
                portDescription = argv[i]+2;
            }
        }
        else {
            int bnum = atoi(argv[i]);
            if ((bnum >= 0) && (bnum < BoardIO::MAX_BOARDS)) {
                BoardList.push_back(new AmpIO(bnum));
                std::cerr << "Selecting board " << bnum << std::endl;
            }
            else
                std::cerr << "Invalid board number: " << argv[i] << std::endl;
        }
    }

    if (BoardList.size() < 1) {
        // usage
        std::cerr << "Usage: ethswitch <board-num> [<board-num>] [-pP]" << std::endl
                  << "       where P = port number (default 0)" << std::endl
                  << "                 can also specify -pfw[:P], -peth:P or -pudp[:xx.xx.xx.xx]" << std::endl
                  << std::endl;

        return 0;
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

    if (Port->GetNumOfNodes() == 0) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to find any boards" << std::endl;
        return -1;
    }

    // TODO: support multiple boards
#if 0
    for (i = 0; i < BoardList.size(); i++) {
        Port->AddBoard(BoardList[i]);
    }
#else
    Port->AddBoard(BoardList[0]);
#endif

    AmpIO *board = BoardList[0];

    Amp1394Console console;
    console.Init();
    if (!console.IsOK()) {
        std::cerr << "Failed to initialize console" << std::endl;
        return -1;
    }

    console.Print(1, lm, "Ethernet Switch status for Board %d", board->GetBoardId());
    console.Print(2, lm, "Press ESC to quit, c to clear errors");

    const unsigned int NL = 8;   // Number of lines per port

    for (i = 0; i < NUM_PORTS; i++) {
        console.Print(4+NL*i, lm, "Port %d: %s", i, PortName[i]);
        console.Print(5+NL*i, lm+2, "NumRecv: ");
        console.Print(6+NL*i, lm+2, "NumSent:");
        console.Print(7+NL*i, lm+2, "FIFOs -->");
        console.Print(8+NL*i, lm+2, "FIFOs <--");
        console.Print(9+NL*i, lm+2, "TxInfo:");
        console.Print(10+NL*i, lm+2, "NumFwd: ");
    }
    console.Print(32, lm, "Other:");
    console.Refresh();

    // control loop
    while ((c = console.GetChar()) != ESC_CHAR)
    {
        uint32_t buffer[15];
        uint16_t numPacketRecv[NUM_PORTS];
        uint16_t numPacketSent[NUM_PORTS];

        if (c == 'c') {
            board->WriteEthernetClearErrors(3);
        }

        if (board->ReadEthernetData(buffer, 0x00a0, 15)) {
            EthSwitchData *data = reinterpret_cast<EthSwitchData *>(buffer);
            uint16_t *data16 = reinterpret_cast<uint16_t *>(buffer);
            for (i = 0; i < NUM_PORTS; i++) {
                numPacketSent[i] = data16[3+2*i];
                numPacketRecv[i] = data16[2+2*i];
                console.Print(5+NL*i, lm+14, "%6d", numPacketRecv[i]);
                console.Print(6+NL*i, lm+14, "%6d", numPacketSent[i]);
                uint8_t *txinfo = reinterpret_cast<uint8_t *>(&data->TxInfoRegVec);
                uint8_t *numFwd = reinterpret_cast<uint8_t *>(&data->numPacketFwdVec);
                for (j = 0; j < NUM_PORTS; j++) {      // out ports
                    bool bit;
                    bit = GetBit(data->fifo_empty, i, j);
                    console.Print(7+NL*i, lm+16+14*j, bit ? "e" : " ");
                    bit = GetBit(data->fifo_empty, j, i);
                    console.Print(8+NL*i, lm+16+14*j, bit ? "e" : " ");
                    bit = GetBit(data->fifo_full, i, j);
                    console.Print(7+NL*i, lm+18+14*j, bit ? "f" : " ");
                    bit = GetBit(data->fifo_full, j%4, i);
                    console.Print(8+NL*i, lm+18+14*j, bit ? "f" : " ");
                    bit = GetBit(data->fifo_active, i, j);
                    console.Print(7+NL*i, lm+20+14*j, bit ? "a" : " ");
                    bit = GetBit(data->fifo_active, j, i);
                    console.Print(8+NL*i, lm+20+14*j, bit ? "a" : " ");
                    bit = GetBit(data->fifo_underflow, i, j);
                    console.Print(7+NL*i, lm+22+14*j, bit ? "u" : " ");
                    bit = GetBit(data->fifo_underflow, j, i);
                    console.Print(8+NL*i, lm+22+14*j, bit ? "u" : " ");
                    bit = GetBit(data->packet_dropped, i, j);
                    console.Print(7+NL*i, lm+24+14*j, bit ? "d" : " ");
                    bit = GetBit(data->packet_dropped, j, i);
                    console.Print(8+NL*i, lm+24+14*j, bit ? "d" : " ");
                    bit = GetBit(data->packet_truncated, i, j);
                    console.Print(7+NL*i, lm+28+14*j, bit ? "t" : " ");
                    bit = GetBit(data->packet_truncated, j, i);
                    console.Print(8+NL*i, lm+28+14*j, bit ? "t" : " ");
                    uint8_t TxSt = (txinfo[j]&0xc0)>>6;
                    uint8_t curIn = (txinfo[j]&0x30)>>4;
                    uint8_t errBits = txinfo[j]&0x0f;
                    console.Print(9+NL*i, lm+16+10*j, "[%1x  %1d  %1x]", TxSt, curIn, errBits);
                    if (i == 0) console.Print(10+NL*i, lm+16+10*j, "%8d", numFwd[j]);
                }
            }
            if ((data->numCrcErrorInVec != 0) || (data->numCrcErrorOutVec != 0) ||
                (data->numIPv4ErrorInVec != 0)) {
                console.Print(32, lm+14, "%8x  %8x  %8x", data->numCrcErrorInVec, data->numCrcErrorOutVec,
                                                          data->numIPv4ErrorInVec);
            }
        }
    }

    console.End();

#if 0
    for (j = 0; j < BoardList.size(); j++)
        Port->RemoveBoard(BoardList[j]->GetBoardId());
#else
    Port->RemoveBoard(BoardList[0]->GetBoardId());
#endif

    delete Port;
    return 0;
}
