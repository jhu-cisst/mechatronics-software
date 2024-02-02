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
    char     headerString[4];
    uint16_t PortAttr;
    uint16_t unused1;
    uint16_t numPacketRecv[4];
    uint16_t numPacketSent[4];
    uint8_t  TxInfoReg[4];
    uint8_t  numCrcErrorIn[4];
    uint8_t  numCrcErrorOut[4];
    uint8_t  numIPv4ErrorIn[4];
    uint32_t unused10_11[2];
    uint16_t fifo_active;
    uint16_t fifo_underflow;
    uint16_t fifo_empty;
    uint16_t fifo_full;
    uint16_t data_avail;
    uint16_t info_not_empty;
    uint16_t packet_dropped;
    uint16_t packet_truncated;
    uint8_t  numPacketFwd[4][4];
    uint32_t unused20_23[4];
    uint32_t MacAddrPrimary0High;
    uint32_t MacAddrPrimary01;
    uint32_t MacAddrPrimary1Low;
    uint16_t PortForwardFpga[2];
    uint32_t unused_28_31[4];
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
    bool show_errors = false;

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

    uint32_t debugString;
    Port->ReadQuadlet(board->GetBoardId(), 0x40a0, debugString);
    if (strncmp(reinterpret_cast<char *>(&debugString), "ESW0", 4) != 0) {
        std::cerr << "Firmware does not support Ethernet switch data" << std::endl;
        return -1;
    }

    uint64_t MacAddrPort[2];
    MacAddrPort[0] = 0;
    MacAddrPort[1] = 0;

    Amp1394Console console;
    console.Init();
    if (!console.IsOK()) {
        std::cerr << "Failed to initialize console" << std::endl;
        return -1;
    }

    console.Print(1, lm, "Ethernet Switch status for Board %d", board->GetBoardId());
    console.Print(2, lm, "Press ESC to quit, c to clear errors, e to show/hide CRC errors");

    const unsigned int NL = 8;   // Number of lines per port

    for (i = 0; i < NUM_PORTS; i++) {
        console.Print(4+NL*i, lm, "Port %d: %s", i, PortName[i]);
        console.Print(5+NL*i, lm+2, "NumRecv:");
        console.Print(6+NL*i, lm+2, "NumSent:");
        if (show_errors) {
            console.Print(5+NL*i, lm+26, "CRC/IPv4:");
            console.Print(6+NL*i, lm+26, "CRC:");
        }
        console.Print(7+NL*i, lm+2, "FIFOs -->");
        console.Print(8+NL*i, lm+2, "FIFOs <--");
        console.Print(9+NL*i, lm+2, "TxInfo:");
        console.Print(10+NL*i, lm+2, "NumFwd:");
    }
    console.Print(36, lm, "MAC Addr:");
    console.Print(37, lm, "FPGA Fwd:");
    console.Refresh();

    // control loop
    while ((c = console.GetChar()) != ESC_CHAR)
    {
        uint32_t buffer[32];

        if (c == 'c') {
            board->WriteEthernetClearErrors(3);
        }
        else if (c == 'e') {
            show_errors = !show_errors;
            if (show_errors) {
                for (i = 0; i < NUM_PORTS; i++) {
                    console.Print(5+NL*i, lm+26, "CRC/IPv4:");
                    console.Print(6+NL*i, lm+26, "CRC:");
                }
            }
            else {
                for (i = 0; i < NUM_PORTS; i++) {
                    console.Print(5+NL*i, lm+26, "         ");
                    console.Print(6+NL*i, lm+26, "    ");
                    console.Print(5+NL*i, lm+36, "      ");
                    console.Print(5+NL*i, lm+44, "      ");
                    console.Print(6+NL*i, lm+36, "      ");
                }
            }
        }

        if (board->ReadEthernetData(buffer, 0x00a0, 32)) {
            EthSwitchData *data = reinterpret_cast<EthSwitchData *>(buffer);
            for (i = 0; i < NUM_PORTS; i++) {
                bool portActive = data->PortAttr & (0x0001 << i);
                bool portFast   = data->PortAttr & (0x0010 << i);
                bool recvReady  = data->PortAttr & (0x0100 << i);
                bool dataReady  = data->PortAttr & (0x1000 << i);
                console.Print(4+NL*i, lm+18, portActive ? "on " : "off");
                console.Print(4+NL*i, lm+23, !portActive ? "    " : portFast   ? "fast" : "slow");
                console.Print(4+NL*i, lm+30, portActive&recvReady  ? "rr" : "  ");
                console.Print(4+NL*i, lm+34, portActive&dataReady  ? "dr" : "  ");
                console.Print(5+NL*i, lm+14, "%6d", data->numPacketRecv[i]);
                console.Print(6+NL*i, lm+14, "%6d", data->numPacketSent[i]);
                if (show_errors) {
                    console.Print(5+NL*i, lm+36, "%6d", data->numCrcErrorIn[i]);
                    console.Print(5+NL*i, lm+44, "%6d", data->numIPv4ErrorIn[i]);
                    console.Print(6+NL*i, lm+36, "%6d", data->numCrcErrorOut[i]);
                }
                for (j = 0; j < NUM_PORTS; j++) {      // out ports
                    bool bit, ebit, fbit;
                    ebit = GetBit(data->fifo_empty, i, j);
                    fbit = GetBit(data->fifo_full, i, j);
                    if (ebit && fbit)
                        console.Print(7+NL*i, lm+16+16*j, "?");
                    else if (ebit)
                        console.Print(7+NL*i, lm+16+16*j, "e");
                    else if (fbit)
                        console.Print(7+NL*i, lm+16+16*j, "f");
                    else
                        console.Print(7+NL*i, lm+16+16*j, " ");
                    ebit = GetBit(data->fifo_empty, j, i);
                    fbit = GetBit(data->fifo_full, j, i);
                    if (ebit && fbit)
                        console.Print(8+NL*i, lm+16+16*j, "?");
                    else if (ebit)
                        console.Print(8+NL*i, lm+16+16*j, "e");
                    else if (fbit)
                        console.Print(8+NL*i, lm+16+16*j, "f");
                    else
                        console.Print(8+NL*i, lm+16+16*j, " ");
                    bit = GetBit(data->info_not_empty, i, j);
                    console.Print(7+NL*i, lm+18+16*j, bit ? "i" : " ");
                    bit = GetBit(data->info_not_empty, j, i);
                    console.Print(8+NL*i, lm+18+16*j, bit ? "i" : " ");
                    bit = GetBit(data->fifo_active, i, j);
                    console.Print(7+NL*i, lm+20+16*j, bit ? "a" : " ");
                    bit = GetBit(data->fifo_active, j, i);
                    console.Print(8+NL*i, lm+20+16*j, bit ? "a" : " ");
                    bit = GetBit(data->fifo_underflow, i, j);
                    console.Print(7+NL*i, lm+22+16*j, bit ? "u" : " ");
                    bit = GetBit(data->fifo_underflow, j, i);
                    console.Print(8+NL*i, lm+22+16*j, bit ? "u" : " ");
                    bit = GetBit(data->packet_dropped, i, j);
                    console.Print(7+NL*i, lm+24+16*j, bit ? "d" : " ");
                    bit = GetBit(data->packet_dropped, j, i);
                    console.Print(8+NL*i, lm+24+16*j, bit ? "d" : " ");
                    bit = GetBit(data->packet_truncated, i, j);
                    console.Print(7+NL*i, lm+28+16*j, bit ? "t" : " ");
                    bit = GetBit(data->packet_truncated, j, i);
                    console.Print(8+NL*i, lm+28+16*j, bit ? "t" : " ");
                    uint8_t TxSt = (data->TxInfoReg[j]&0xc0)>>6;
                    uint8_t curIn = (data->TxInfoReg[j]&0x30)>>4;
                    uint8_t errBits = data->TxInfoReg[j]&0x0f;
                    console.Print(9+NL*i, lm+16+16*j, "[%1x  %1d  %1x]", TxSt, curIn, errBits);
                    console.Print(10+NL*i, lm+16+16*j, "%4d", data->numPacketFwd[i][j]);
                }
            }
            uint64_t newMacAddr[2];
            newMacAddr[0] = data->MacAddrPrimary0High;
            newMacAddr[0] <<= 16;
            newMacAddr[0] |= (data->MacAddrPrimary01&0xffff0000)>>16;
            newMacAddr[1] = data->MacAddrPrimary01&0x0000ffff;
            newMacAddr[1] <<= 32;
            newMacAddr[1] |= data->MacAddrPrimary1Low;
            if (newMacAddr[0] != MacAddrPort[0]) {
                MacAddrPort[0] = newMacAddr[0];
                console.Print(36, lm+16, "%llx", MacAddrPort[0]);
            }
            if (newMacAddr[1] != MacAddrPort[1]) {
                MacAddrPort[1] = newMacAddr[1];
                console.Print(36, lm+32, "%llx", MacAddrPort[1]);
            }
            console.Print(37, lm+16, "%6x", data->PortForwardFpga[0]);
            console.Print(37, lm+32, "%6x", data->PortForwardFpga[1]);
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
