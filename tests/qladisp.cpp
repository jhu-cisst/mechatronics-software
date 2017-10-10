/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen, Anton Deguet
  Created on: 2012

  (C) Copyright 2012-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
 * Usage: sensor [-pP] <board num>
 *        where P is the Firewire port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *
 ******************************************************************************/

#include <stdlib.h>
#include <unistd.h>
#include <curses.h>
#include <iostream>
#include <sstream>
#include <vector>

#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "Eth1394Port.h"
#endif

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
    while (!debugStream.eof()) {
        debugStream.getline(line, sizeof(line));
        std::cerr << line << std::endl;
    }
    debugStream.clear();
    debugStream.str("");
}

int main(int argc, char** argv)
{
    const unsigned int lm = 5; // left margin
    unsigned int i, j;
#if Amp1394_HAS_RAW1394
    bool useFireWire = true;
#else
    bool useFireWire = false;
#endif
    int port = 0;
    int board1 = BoardIO::MAX_BOARDS;
    int board2 = BoardIO::MAX_BOARDS;

    // measure time between reads
    AmpIO_UInt32 maxTime = 0;
    AmpIO_UInt32 lastTime = 0;

    int args_found = 0;
    for (i = 1; i < (unsigned int)argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
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
        std::cerr << "Usage: qladisp <board-num> [<board-num>] [-pP]" << std::endl
                  << "       where P = port number (default 0)" << std::endl
                  << std::endl
                  << "Trying to detect board on default port:" << std::endl;

        if (useFireWire) {
            // try to locate all boards available on default port
            FirewirePort Port(port, std::cerr);
            if (!Port.IsOK()) {
                std::cerr << "Failed to initialize FireWire port " << port << std::endl;
            }
        }

        // keys
        std::cerr << std::endl << "Keys:" << std::endl
                  << "'r': reset FireWire port" << std::endl
                  << "'d': turn watchdog on/off (25.0 ms)" << std::endl
                  << "'p': turn power on/off (board and axis)" << std::endl
                  << "'o': turn power on/off (board only)" << std::endl
                  << "'i': turn power on/off (axis only, requires board first)" << std::endl
                  << "'w': increment encoders" << std::endl
                  << "'s': decrement encoders" << std::endl
                  << "'=': increase motor current by about 50mA" << std::endl
                  << "'-': increase motor current by about 50mA" << std::endl << std::endl;
        return 0;
    }

    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);

    BasePort *Port;
    if (useFireWire) {
        std::cerr << "Attempting to connect using FireWire" << std::endl;
#if Amp1394_HAS_RAW1394
        Port = new FirewirePort(port, debugStream);
        if (!Port->IsOK()) {
            PrintDebugStream(debugStream);
            std::cerr << "Failed to initialize FireWire port " << port << std::endl;
            return -1;
        }
        std::cerr << "FireWire port properly initialized" << std::endl;
#else
        std::cerr << "FireWire not available (set Amp1394_HAS_RAW1394 in CMake)" << std::endl;
        return -1;
#endif
    }
    else {
        std::cerr << "Attempting to connect using Ethernet/PCAP" << std::endl;
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

    FirmwareVersionList.clear();
    for (j = 0; j < BoardList.size(); j++) {
        FirmwareVersionList.push_back(BoardList[j]->GetFirmwareVersion());
    }

    for (i = 0; i < 4; i++) {
        for (j = 0; j < BoardList.size(); j++)
            BoardList[j]->WriteEncoderPreload(i, 0x1000*i + 0x1000);
    }
    bool power_on = false;
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

    wrefresh(stdscr);

    unsigned char dig_out = 0x0f;

    int loop_cnt = 0;
    const int DEBUG_START_LINE = 19;
    unsigned int last_debug_line = DEBUG_START_LINE;
    const int ESC_CHAR = 0x1b;
    int c;

    // control loop
    // Port->WriteAllBoardsBroadcast(); // dummy write to start the pipeline

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
        else if (c == 'p') {
            power_on = !power_on;
            for (j = 0; j < BoardList.size(); j++) {
                if (power_on) {
                    power_board = true;
                    power_axis = true;
                    BoardList[j]->WriteSafetyRelay(true);
                    BoardList[j]->WritePowerEnable(true);
                    BoardList[j]->WriteAmpEnable(0x0f, 0x0f);
                } else {
                    power_board = false;
                    power_axis = false;
                    BoardList[j]->WriteAmpEnable(0x0f, 0x00);
                    BoardList[j]->WritePowerEnable(false);
                    BoardList[j]->WriteSafetyRelay(false);
                }
            }
        }
        else if (c == 'o') {
            power_board = !power_board;
            for (j = 0; j < BoardList.size(); j++) {
                if (power_board) {
                    // if axis are on, it's like having all power on
                    power_on = power_axis;
                    BoardList[j]->WriteSafetyRelay(true);
                    BoardList[j]->WritePowerEnable(true);
                }
                else {
                    power_on = false;
                    BoardList[j]->WritePowerEnable(false);
                    BoardList[j]->WriteSafetyRelay(false);
                }
            }
        }
        else if (c == 'i') {
            power_axis = !power_axis;
            for (j = 0; j < BoardList.size(); j++) {
                if (power_axis) {
                    // if boards are on, it's like having all power on
                    power_on = power_board;
                    BoardList[j]->WriteAmpEnable(0x0f, 0x0f);
                }
                else {
                    power_on = false;
                    BoardList[j]->WriteAmpEnable(0x0f, 0x00);
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
            if (node < FirewirePort::MAX_NODES)
                sprintf(nodeStr[1], "%4d", node);
            else
                strcpy(nodeStr[1], "none");
        }

//        Port->ReadAllBoardsBroadcast();
        Port->ReadAllBoards();
        unsigned int j = 0;
        for (j = 0; j < BoardList.size(); j++) {
            if (BoardList[j]->ValidRead()) {
                for (i = 0; i < 4; i++) {
                    mvwprintw(stdscr, 6, lm+5+(i+4*j)*13, "0x%07X", BoardList[j]->GetEncoderPosition(i)+0x800000);
                    mvwprintw(stdscr, 7, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetAnalogInput(i));
                    mvwprintw(stdscr, 8, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetEncoderVelocity(i));
                    mvwprintw(stdscr, 9, lm+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetMotorCurrent(i));
                }
                dig_out = BoardList[j]->GetDigitalOutput();
                mvwprintw(stdscr, 13, lm+58*j, "Status: 0x%08X   Timestamp: %08X  DigOut: 0x%01X",
                          BoardList[j]->GetStatus(), BoardList[j]->GetTimestamp(),
                          (unsigned int)dig_out);
                mvwprintw(stdscr, 14, lm+58*j, "NegLim: 0x%01X          PosLim: 0x%01X          Home: 0x%01X",
                          BoardList[j]->GetNegativeLimitSwitches(),
                          BoardList[j]->GetPositiveLimitSwitches(),
                          BoardList[j]->GetHomeSwitches());
                mvwprintw(stdscr, 15, lm+58*j, "EncA: 0x%01X            EncB: 0x%01X            EncInd: 0x%01X",
                          BoardList[j]->GetEncoderChannelA(),
                          BoardList[j]->GetEncoderChannelB(),
                          BoardList[j]->GetEncoderIndex());

                mvwprintw(stdscr, 17, lm+58*j, "Node: %s", nodeStr[j]);
                mvwprintw(stdscr, 17, lm+18+58*j, "Temp:  0x%02X    0x%02X",
                          (unsigned int)BoardList[j]->GetAmpTemperature(0),
                          (unsigned int)BoardList[j]->GetAmpTemperature(1));
                mvwprintw(stdscr, 17, lm+40, "Ct: %8d", loop_cnt++);
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
//        Port->WriteAllBoardsBroadcast();
        Port->WriteAllBoards();

        mvwprintw(stdscr, 1, lm+41, "dt: %f",  (1.0 / 49125.0) * maxTime);

        wrefresh(stdscr);
        usleep(500);
    }

    for (j= 0; j < BoardList.size(); j++) {
        BoardList[j]->WritePowerEnable(false);      // Turn power off
        BoardList[j]->WriteAmpEnable(0x0f, 0x00);   // Turn power off
        BoardList[j]->WriteSafetyRelay(false);
        Port->RemoveBoard(BoardList[j]->GetBoardId());
    }

    endwin();
    delete Port;
    return 0;
}
