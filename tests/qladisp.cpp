/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen, Anton Deguet

  (C) Copyright 2012-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/******************************************************************************
 *
 * This program continuously displays the sensor feedback from the selected
 * board. It relies on the curses library and the AmpIO library (which
 * depends on libraw1394 and/or pcap).
 *
 * Usage: qladisp [-pP] [-b<r|w>] [-v] <board num> [<board_num>]
 *        where P is the Firewire port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *        -br or -bw specify to use a broadcast protocol
 *        -v specifies to display full velocity feedback
 *
 ******************************************************************************/

#include <stdlib.h>
#include <unistd.h>
#include <curses.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "EthRawPort.h"
#endif
#include "EthUdpPort.h"

#include "AmpIO.h"


/*!
 \brief Increment encoder counts
 \param[in] bd FPGA Board
*/
void EncUp(AmpIO &bd)
{
    bd.WriteDigitalOutput(0x03, 0x03);
    bd.WriteDigitalOutput(0x03, 0x02);
    bd.WriteDigitalOutput(0x03, 0x00);
    bd.WriteDigitalOutput(0x03, 0x01);
    bd.WriteDigitalOutput(0x03, 0x03);
}

/*!
 \brief Decrement encoder counts
 \param[in] bd FPGA Board
*/
void EncDown(AmpIO &bd)
{
    bd.WriteDigitalOutput(0x03, 0x03);
    bd.WriteDigitalOutput(0x03, 0x01);
    bd.WriteDigitalOutput(0x03, 0x00);
    bd.WriteDigitalOutput(0x03, 0x02);
    bd.WriteDigitalOutput(0x03, 0x03);
}

void PrintDebugStream(std::stringstream &debugStream)
{
    char line[80];
    while (debugStream.getline(line, sizeof(line)))
        std::cerr << line << std::endl;
    debugStream.clear();
    debugStream.str("");
}

unsigned int collectFileNum = 0;
std::ofstream collectFile;
bool isCollecting = false;

bool CollectCB(quadlet_t *buffer, short nquads, unsigned short readSize)
{
    collectFile.write(reinterpret_cast<const char *>(buffer), nquads*sizeof(quadlet_t));
    if (isCollecting)
        mvwprintw(stdscr, 1, 76, "%4d,%4u", nquads, readSize);
    else {
        collectFile.close();
        mvwprintw(stdscr, 1, 76, "         ");
    }
    return true;
}

bool CollectFileConvert(const char *inFilename, const char *outFilename)
{
    std::cerr << "Converting data collection file " << inFilename << " to " << outFilename << std::endl;
    std::ifstream inFile(inFilename, std::ifstream::binary);
    if (!inFile.good()) {
        std::cerr << "Failed to open input data collection file " << inFilename << std::endl;
        return false;
    }
    std::ofstream outFile(outFilename, std::ofstream::trunc);
    if (!outFile.good()) {
        std::cerr << "Failed to open output data collection file " << outFilename << std::endl;
        return false;
    }
    quadlet_t value;
    while (inFile.good()) {
        inFile.read(reinterpret_cast<char *>(&value), sizeof(quadlet_t));
        outFile << std::dec
                << ((value&0xC0000000)>>30) << ", "    // flag
                << ((value&0x3FF00000)>>20) << ", "    // write index
                << ((value&0x000F0000)>>16) << ", "    // channel
                << std::hex << (value&0x0000FFFF)      // data
                << std::endl;
    }
    inFile.close();
    outFile.close();
    return true;
}

int main(int argc, char** argv)
{
    const unsigned int lm = 5; // left margin
    unsigned int i, j;
#if Amp1394_HAS_RAW1394
    BasePort::PortType desiredPort = BasePort::PORT_FIREWIRE;
#else
    BasePort::PortType desiredPort = BasePort::PORT_ETH_UDP;
#endif
    int port = 0;
    std::string IPaddr(ETH_UDP_DEFAULT_IP);
    int board1 = BoardIO::MAX_BOARDS;
    int board2 = BoardIO::MAX_BOARDS;
    BasePort::ProtocolType protocol = BasePort::PROTOCOL_SEQ_RW;
    bool fullvel = false;  // whether to display full velocity feedback

    // measure time between reads
    AmpIO_UInt32 maxTime = 0;
    AmpIO_UInt32 lastTime = 0;

    int args_found = 0;
    for (i = 1; i < (unsigned int)argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'p') {
                if (!BasePort::ParseOptions(argv[i]+2, desiredPort, port, IPaddr)) {
                    std::cerr << "Failed to parse option: " << argv[i] << std::endl;
                    return 0;
                }
                std::cerr << "Selected port: " << BasePort::PortTypeString(desiredPort) << std::endl;
            }
            else if (argv[i][1] == 'b') {
                // -br -- enable broadcast read/write
                // -bw -- enable broadcast write (sequential read)
                if (argv[i][2] == 'r')
                    protocol = BasePort::PROTOCOL_BC_QRW;
                else if (argv[i][2] == 'w')
                    protocol = BasePort::PROTOCOL_SEQ_R_BC_W;
            }
            else if (argv[i][1] == 'v')
                fullvel = true;
        }
        else {
            if (args_found == 0) {
                board1 = atoi(argv[i]);
                std::cerr << "Selecting board " << board1 << std::endl;
            }
            else if (args_found == 1) {
                board2 = atoi(argv[i]);
                std::cerr << "Selecting board " << board2 << std::endl;
            }
            args_found++;
        }
    }

    if (args_found < 1) {
        // usage
        std::cerr << "Usage: qladisp <board-num> [<board-num>] [-pP] [-b<r|w>] [-v]" << std::endl
                  << "       where P = port number (default 0)" << std::endl
                  << "                 can also specify -pfwP, -pethP or -pudp[xx.xx.xx.xx]" << std::endl
                  << "            -br enables broadcast read/write" << std::endl
                  << "            -bw enables broadcast write" << std::endl
                  << "            -v  displays full velocity feedback" << std::endl
                  << std::endl
                  << "Trying to detect boards on port:" << std::endl;
    }

    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);

    BasePort *Port = 0;
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

    if (!Port || !Port->IsOK()) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to initialize " << BasePort::PortTypeString(desiredPort) << std::endl;
        return -1;
    }

    if (args_found < 1) {
        PrintDebugStream(debugStream);
        // keys
        std::cerr << std::endl << "Keys:" << std::endl
                  << "'r': reset FireWire port" << std::endl
                  << "'d': turn watchdog on/off (0.25 ms, this should be triggered immediately)" << std::endl
                  << "'D': turn watchdog on/off (25.0 ms, can be triggered by unplugging cable to PC)" << std::endl
                  << "'p': turn power on/off (board and axis)" << std::endl
                  << "'o': turn power on/off (board only)" << std::endl
                  << "'i': turn power on/off (axis only, requires board first)" << std::endl
                  << "'w': increment encoders" << std::endl
                  << "'s': decrement encoders" << std::endl
                  << "'=': increase motor current by about 50mA" << std::endl
                  << "'-': increase motor current by about 50mA" << std::endl
                  << "'c': start/stop data collection" << std::endl;
        return 0;
    }

    if (Port->GetNumOfNodes() == 0) {
        std::cerr << "Failed to find any boards" << std::endl;
        return -1;
    }

    // Set protocol; default is PROTOCOL_SEQ_RW (not broadcast), but can be changed to
    // one of the broadcast protocols by specifying -br or -bw command line parameter.
    if (protocol == BasePort::PROTOCOL_BC_QRW)
        std::cerr << "Setting protocol to broadcast read/write" << std::endl;
    else if (protocol == BasePort::PROTOCOL_SEQ_R_BC_W)
        std::cerr << "Setting protocol to broadcast write" << std::endl;
    Port->SetProtocol(protocol);

    // Currently hard-coded for up to 2 boards; initialize at mid-range
    AmpIO_UInt32 MotorCurrents[2][4] = { {0x8000, 0x8000, 0x8000, 0x8000 },
                                         {0x8000, 0x8000, 0x8000, 0x8000 }};

    std::vector<AmpIO*> BoardList;
    std::vector<AmpIO_UInt32> FirmwareVersionList;
    BoardList.push_back(new AmpIO(board1));
    Port->AddBoard(BoardList[0]);
    if (board2 < BoardIO::MAX_BOARDS) {
        BoardList.push_back(new AmpIO(board2));
        Port->AddBoard(BoardList[1]);
    }

    bool allRev7 = true;
    FirmwareVersionList.clear();
    for (j = 0; j < BoardList.size(); j++) {
        AmpIO_UInt32 fver = BoardList[j]->GetFirmwareVersion();
        FirmwareVersionList.push_back(fver);
        if (fver < 7) allRev7 = false;
    }

    for (i = 0; i < 4; i++) {
        for (j = 0; j < BoardList.size(); j++)
            BoardList[j]->WriteEncoderPreload(i, 0x1000*i + 0x1000);
    }
    bool power_board = false;
    bool power_axis = false;
    for (j = 0; j < BoardList.size(); j++) {
        BoardList[j]->WriteSafetyRelay(false);
        BoardList[j]->WritePowerEnable(false);
        BoardList[j]->WriteAmpEnable(0x0f, 0);
    }

    bool watchdog_on = false;

    initscr();
    cbreak();
    keypad(stdscr, TRUE);
    noecho();
    nodelay(stdscr, TRUE);

    if (BoardList.size() > 1) {
       if (protocol == BasePort::PROTOCOL_BC_QRW)
           mvwprintw(stdscr, 1, lm, "Sensor Feedback for Boards %d, %d (hub %d)", board1, board2, Port->GetHubBoardId());
       else
           mvwprintw(stdscr, 1, lm, "Sensor Feedback for Boards %d, %d", board1, board2);
    } else {
        mvwprintw(stdscr, 1, lm, "Sensor Feedback for Board %d", board1);
    }
    mvwprintw(stdscr, 2, lm, "Press ESC to quit, r to reset port, 0-3 to toggle digital output bit, p to enable/disable power,");
    mvwprintw(stdscr, 3, lm, "+/- to increase/decrease commanded current (DAC) by 0x100");

    unsigned int numAxes = (BoardList.size() > 1)?8:4;
    for (i = 0; i < numAxes; i++) {
        mvwprintw(stdscr, 5, lm+8+i*13, "Axis %d", i);
    }
    mvwprintw(stdscr, 6, lm, "Enc:");
    mvwprintw(stdscr, 7, lm, "Pot:");
    mvwprintw(stdscr, 8, lm, "Vel:");
    mvwprintw(stdscr, 9, lm, "Cur:");
    mvwprintw(stdscr, 10, lm, "DAC:");
    if (fullvel) {
        if (allRev7) {
            mvwprintw(stdscr, 11, lm, "Qtr1:");
            mvwprintw(stdscr, 12, lm, "Qtr5:");
            mvwprintw(stdscr, 13, lm, "Run:");
        }
        else
            mvwprintw(stdscr, 11, lm, "Acc:");
    }

    wrefresh(stdscr);

    unsigned char dig_out = 0x0f;
    unsigned char collect_axis = 1;

    int loop_cnt = 0;
    const int STATUS_LINE = fullvel ? 15 : 13;
    const int DEBUG_START_LINE = fullvel ? 22 : 19;
    unsigned int last_debug_line = DEBUG_START_LINE;
    const int ESC_CHAR = 0x1b;
    int c;

    // control loop
    while ((c = getch()) != ESC_CHAR) {
        if (c == 'r') Port->Reset();
        else if ((c >= '0') && (c <= '3')) {
            // toggle digital output bit
            dig_out = dig_out^(1<<(c-'0'));
            for (j = 0; j < BoardList.size(); j++)
                BoardList[j]->WriteDigitalOutput(0x0f, dig_out);
        }
        else if (c == 'w') {
            for (j = 0; j < BoardList.size(); j++)
                EncUp(*(BoardList[j]));
        }
        else if (c == 's') {
            for (j = 0; j < BoardList.size(); j++)
                EncDown(*(BoardList[j]));
        }
        else if (c == 'd'){
            watchdog_on = !watchdog_on;
            for (j = 0; j < BoardList.size(); j++)
                // 50 CNTS = 0.25 ms
                BoardList[j]->WriteWatchdogPeriod(watchdog_on?50:0);
        }
        else if (c == 'D'){
            watchdog_on = !watchdog_on;
            for (j = 0; j < BoardList.size(); j++)
                // 5000 CNTS = 25.00 ms
                BoardList[j]->WriteWatchdogPeriod(watchdog_on?5000:0);
        }
        else if (c == 'p') {
            // Only power on system if completely off (power_board
            // and power_axis false). Otherwise, we power off.
            if (!power_board && !power_axis) {
                power_board = true;
                power_axis = true;
            }
            else {
                power_board = false;
                power_axis = false;
            }
            for (j = 0; j < BoardList.size(); j++) {
                if (power_board && power_axis) {
                    BoardList[j]->SetSafetyRelay(true);
                    // Cannot enable Amp power unless Board power is
                    // already enabled.
                    BoardList[j]->WritePowerEnable(true);
                    //BoardList[j]->SetPowerEnable(true);
                    BoardList[j]->SetAmpEnableMask(0x0f, 0x0f);
                } else {
                    BoardList[j]->SetAmpEnableMask(0x0f, 0x00);
                    BoardList[j]->SetPowerEnable(false);
                    BoardList[j]->SetSafetyRelay(false);
                }
            }
        }
        else if (c == 'o') {
            power_board = !power_board;
            for (j = 0; j < BoardList.size(); j++) {
                if (power_board) {
                    BoardList[j]->SetSafetyRelay(true);
                    BoardList[j]->SetPowerEnable(true);
                }
                else {
                    BoardList[j]->SetPowerEnable(false);
                    BoardList[j]->SetSafetyRelay(false);
                }
            }
        }
        else if (c == 'i') {
            power_axis = !power_axis;
            for (j = 0; j < BoardList.size(); j++) {
                if (power_axis) {
                    BoardList[j]->SetAmpEnableMask(0x0f, 0x0f);
                }
                else {
                    BoardList[j]->SetAmpEnableMask(0x0f, 0x00);
                }
            }
        }
        else if (c == '=') {
            for (j = 0; j < BoardList.size(); j++) {
                for (i = 0; i < 4; i++)
                    MotorCurrents[j][i] += 0x100;   // 0x100 is about 50 mA
            }
        }
        else if (c == '-') {
            for (j = 0; j < BoardList.size(); j++) {
                for (i = 0; i < 4; i++)
                    MotorCurrents[j][i] -= 0x100;   // 0x100 is about 50 mA
            }
        }
        else if (c == 'c') {
            int collect_board = (collect_axis <= 4) ? 0 : 1;
            if (BoardList[collect_board]->IsCollecting()) {
                BoardList[collect_board]->DataCollectionStop();
                mvwprintw(stdscr, 1, 62, "             ");
                if (collect_axis > collectFileNum)
                    collectFileNum = collect_axis;
                collect_axis = (collect_axis == numAxes) ? 1 : collect_axis+1;
                isCollecting = false;
            }
            else {
                char fileName[20];
                sprintf(fileName, "Collect%d.raw", collect_axis);
                collectFile.open(fileName, std::ofstream::binary|std::ofstream::trunc);
                if (!collectFile.good())
                    std::cerr << "Failed to open data collection file " << fileName << std::endl;
                unsigned char collect_chan = collect_axis-4*collect_board;
                if (BoardList[collect_board]->DataCollectionStart(collect_chan, CollectCB)) {
                    isCollecting = true;
                    mvwprintw(stdscr, 1, 62, "Collecting %d:", collect_axis);
                }
            }
        }

        if (!debugStream.str().empty()) {
            int cur_line = DEBUG_START_LINE;
            char line[80];
            memset(line, ' ', sizeof(line)-1);
            line[sizeof(line)-1] = 0;
            for (i = cur_line; i < last_debug_line; i++)
                mvwprintw(stdscr, i, lm, line);
            while (!debugStream.eof()) {
                std::string stringLine;
                std::getline(debugStream, stringLine);
                mvwprintw(stdscr, cur_line++, lm, stringLine.c_str());
            }
            debugStream.clear();
            debugStream.str("");
            last_debug_line = cur_line;
        }

        if (!Port->IsOK()) continue;

        char nodeStr[2][5];
        int node = Port->GetNodeId(board1);
        if (node < BoardIO::MAX_BOARDS)
            sprintf(nodeStr[0], "%4d", node);
        else
            strcpy(nodeStr[0], "none");

        if (BoardList.size() > 1) {
            node = Port->GetNodeId(board2);
            if (node < BasePort::MAX_NODES)
                sprintf(nodeStr[1], "%4d", node);
            else
                strcpy(nodeStr[1], "none");
        }

        Port->ReadAllBoards();
        unsigned int j = 0;
        for (j = 0; j < BoardList.size(); j++) {
            if (BoardList[j]->ValidRead()) {
                for (i = 0; i < 4; i++) {
                    mvwprintw(stdscr, 6, lm+5+(i+4*j)*13, "0x%07X", BoardList[j]->GetEncoderPosition(i)+0x800000);
                    mvwprintw(stdscr, 7, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetAnalogInput(i));
                    if (fullvel)
                        mvwprintw(stdscr, 8, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetEncoderVelocityRaw(i));
                    else
                        mvwprintw(stdscr, 8, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetEncoderVelocity(i));
                    mvwprintw(stdscr, 9, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetMotorCurrent(i));
                    if (fullvel) {
                        if (allRev7) {
                            mvwprintw(stdscr, 11, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetEncoderQtr1(i));
                            mvwprintw(stdscr, 12, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetEncoderQtr5(i));
                            mvwprintw(stdscr, 13, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetEncoderRunningCounter(i));
                        }
                        else
                            mvwprintw(stdscr, 11, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetEncoderAccelerationRaw(i));
                    }
                }
                dig_out = BoardList[j]->GetDigitalOutput();
                mvwprintw(stdscr, STATUS_LINE, lm+58*j, "Status: 0x%08X   Timestamp: %08X  DigOut: 0x%01X",
                          BoardList[j]->GetStatus(), BoardList[j]->GetTimestamp(),
                          (unsigned int)dig_out);
                mvwprintw(stdscr, STATUS_LINE+1, lm+58*j, "NegLim: 0x%01X          PosLim: 0x%01X          Home: 0x%01X",
                          BoardList[j]->GetNegativeLimitSwitches(),
                          BoardList[j]->GetPositiveLimitSwitches(),
                          BoardList[j]->GetHomeSwitches());
                mvwprintw(stdscr, STATUS_LINE+2, lm+58*j, "EncA: 0x%01X            EncB: 0x%01X            EncInd: 0x%01X",
                          BoardList[j]->GetEncoderChannelA(),
                          BoardList[j]->GetEncoderChannelB(),
                          BoardList[j]->GetEncoderIndex());

                mvwprintw(stdscr, STATUS_LINE+4, lm+58*j, "Node: %s", nodeStr[j]);
                mvwprintw(stdscr, STATUS_LINE+4, lm+18+58*j, "Temp:  0x%02X    0x%02X",
                          (unsigned int)BoardList[j]->GetAmpTemperature(0),
                          (unsigned int)BoardList[j]->GetAmpTemperature(1));
                mvwprintw(stdscr, STATUS_LINE+4, lm+40, "Ct: %8d", loop_cnt++);
                if (loop_cnt > 500) {
                    lastTime = BoardList[j]->GetTimestamp();
                    if (lastTime > maxTime) {
                        maxTime = lastTime;
                    }
                }
            }
            for (i = 0; i < 4; i++) {
                mvwprintw(stdscr, 10, lm+8+(i+4*j)*13, "0x%04X", MotorCurrents[j][i]);
                BoardList[j]->SetMotorCurrent(i, MotorCurrents[j][i]);
            }
        }
        Port->WriteAllBoards();

        mvwprintw(stdscr, 1, lm+38, "Gen: %d",  Port->GetBusGeneration());
        mvwprintw(stdscr, 1, lm+52, "dt: %f",  (1.0 / 49125.0) * maxTime);

        wrefresh(stdscr);
        usleep(500);
    }

    for (j = 0; j < BoardList.size(); j++) {
        BoardList[j]->WritePowerEnable(false);      // Turn power off
        BoardList[j]->WriteAmpEnable(0x0f, 0x00);   // Turn power off
        BoardList[j]->WriteSafetyRelay(false);
        Port->RemoveBoard(BoardList[j]->GetBoardId());
    }

    endwin();
    delete Port;
    // Process any data collection files (convert from binary to text)
    for (j = 1; j <= collectFileNum; j++) {
        char inFile[20];
        char outFile[20];
        sprintf(inFile,  "Collect%d.raw", j);
        sprintf(outFile, "Collect%d.csv", j);
        CollectFileConvert(inFile, outFile);
    }
    return 0;
}
