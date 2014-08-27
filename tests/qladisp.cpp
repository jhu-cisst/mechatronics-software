/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * This program continuously displays the sensor feedback from the selected
 * board. It relies on the curses library and the AmpIO library (which
 * depends on libraw1394).
 *
 * Usage: sensor [-pP] <board num>
 *        where P is the Firewire port number (default 0)
 *
 ******************************************************************************/

#include <stdlib.h>
#include <unistd.h>
#include <curses.h>
#include <iostream>
#include <sstream>
#include <vector>

//#include "FirewirePort.h"

#include "Eth1394Port.h"
#include "AmpIO.h"

void EncUp(AmpIO &bd)
{
    bd.WriteDigitalOutput(0x03, 0x00);
    bd.WriteDigitalOutput(0x03, 0x01);
    bd.WriteDigitalOutput(0x03, 0x03);
    bd.WriteDigitalOutput(0x03, 0x02);
    bd.WriteDigitalOutput(0x03, 0x00);
}

void EncDown(AmpIO &bd)
{
    bd.WriteDigitalOutput(0x03, 0x00);
    bd.WriteDigitalOutput(0x03, 0x02);
    bd.WriteDigitalOutput(0x03, 0x03);
    bd.WriteDigitalOutput(0x03, 0x01);
    bd.WriteDigitalOutput(0x03, 0x00);
}


int main(int argc, char** argv)
{
    unsigned int i, j;
    int port = 0;
    int board1 = BoardIO::MAX_BOARDS;
    int board2 = BoardIO::MAX_BOARDS;

    // measure time between reads
    AmpIO_UInt32 maxTime = 0;
    AmpIO_UInt32 lastTime = 0;

    int args_found = 0;
    for (i = 1; i < (unsigned int)argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            port = atoi(argv[i]+2);
            std::cerr << "Selecting port " << port << std::endl;
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
        std::cerr << "Usage: sensors <board-num> [<board-num>] [-pP]" << std::endl
                  << "       where P = port number (default 0)" << std::endl;
        return 0;
    }

    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);

//    FirewirePort Port(port, debugStream);
    Eth1394Port Port(port, debugStream);

    if (!Port.IsOK()) {
        std::cerr << "Failed to initialize firewire port " << port << std::endl;
        return -1;
    }

    // Currently hard-coded for up to 2 boards; initialize at mid-range
    AmpIO_UInt32 MotorCurrents[2][4] = { {0x8000, 0x8000, 0x8000, 0x8000 },
                                         {0x8000, 0x8000, 0x8000, 0x8000 }};

    std::vector<AmpIO*> BoardList;
    BoardList.push_back(new AmpIO(board1));
    Port.AddBoard(BoardList[0]);
    if (board2 < BoardIO::MAX_BOARDS) {
        BoardList.push_back(new AmpIO(board2));
        Port.AddBoard(BoardList[1]);
    }

    for (i = 0; i < 4; i++) {
        for (j = 0; j < BoardList.size(); j++)
            BoardList[j]->WriteEncoderPreload(i, 0x1000*i + 0x1000);
    }
    bool power_on = false;
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

    if (BoardList.size() > 1)
        mvwprintw(stdscr, 1, 9, "Sensor Feedback for Boards %d, %d", board1, board2);
    else
        mvwprintw(stdscr, 1, 9, "Sensor Feedback for Board %d", board1);
    mvwprintw(stdscr, 2, 9, "Press ESC to quit, r to reset port, 0-3 to toggle digital output bit, p to enable/disable power,");
    mvwprintw(stdscr, 3, 9, "+/- to increase/decrease commanded current (DAC) by 0x100");

    unsigned int numAxes = (BoardList.size() > 1)?8:4;
    for (i = 0; i < numAxes; i++)
        mvwprintw(stdscr, 5, 9+8+i*13, "Axis%d", i);
    mvwprintw(stdscr, 6, 9, "Enc:");
    mvwprintw(stdscr, 7, 9, "Pot:");
    mvwprintw(stdscr, 8, 9, "Vel:");
    mvwprintw(stdscr, 9, 9, "Cur:");
    mvwprintw(stdscr, 10, 9, "DAC:");
    wrefresh(stdscr);

    unsigned char dig_out = 0;

    int loop_cnt = 0;
    const int DEBUG_START_LINE = 18;
    unsigned int last_debug_line = DEBUG_START_LINE;
    const int ESC_CHAR = 0x1b;
    int c;

    // control loop
//    Port.WriteAllBoardsBroadcast(); // dummy write to start the pipeline

    while ((c = getch()) != ESC_CHAR) {
        if (c == 'r') Port.Reset();
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
                    BoardList[j]->WriteSafetyRelay(true);
                    BoardList[j]->WritePowerEnable(true);
                    usleep(40000); // sleep 40 ms
                    BoardList[j]->WriteAmpEnable(0x0f, 0x0f);
                }
                else {
                    BoardList[j]->WriteAmpEnable(0x0f, 0x00);
                    BoardList[j]->WritePowerEnable(false);
                    BoardList[j]->WriteSafetyRelay(false);
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

        mvwprintw(stdscr, 16, 50, "%10d", loop_cnt++);

        if (!debugStream.str().empty()) {
            int cur_line = DEBUG_START_LINE;
            char line[80];
            memset(line, ' ', sizeof(line));
            for (i = cur_line; i < last_debug_line; i++)
                mvwprintw(stdscr, i, 9, line);
            while (!debugStream.eof()) {
                debugStream.getline(line, sizeof(line));
                mvwprintw(stdscr,cur_line++, 9, line);
            }
            debugStream.clear();
            debugStream.str("");
            last_debug_line = cur_line;
        }

        if (!Port.IsOK()) continue;

        char nodeStr[2][5];
        int node = Port.GetNodeId(board1);
        if (node < BoardIO::MAX_BOARDS)
            sprintf(nodeStr[0], "%4d", node);
        else
            strcpy(nodeStr[0], "none");

        if (BoardList.size() > 1) {
            node = Port.GetNodeId(board2);
            if (node < BoardIO::MAX_BOARDS)
                sprintf(nodeStr[1], "%4d", node);
            else
                strcpy(nodeStr[1], "none");
        }

//        Port.ReadAllBoardsBroadcast();
        Port.ReadAllBoards();
        unsigned int j = 0;
        for (j = 0; j < BoardList.size(); j++) {
            if (BoardList[j]->ValidRead()) {
                for (i = 0; i < 4; i++) {
                    mvwprintw(stdscr, 6, 9+5+(i+4*j)*13, "0x%07X", BoardList[j]->GetEncoderPosition(i));
                    mvwprintw(stdscr, 7, 9+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetAnalogInput(i));
                    mvwprintw(stdscr, 8, 9+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetEncoderVelocity(i));
                    mvwprintw(stdscr, 9, 9+8+(i+4*j)*13, "0x%04X", BoardList[j]->GetMotorCurrent(i));
                }
                dig_out = BoardList[j]->GetDigitalOutput();
                mvwprintw(stdscr, 13, 9+58*j, "Status: 0x%08X   Timestamp: %08X  DigOut: 0x%01X", 
                          BoardList[j]->GetStatus(), BoardList[j]->GetTimestamp(),
                          (unsigned int)dig_out);
                mvwprintw(stdscr, 14, 9+58*j, "NegLim: 0x%01X          PosLim: 0x%01X          Home: 0x%01X",
                          BoardList[j]->GetNegativeLimitSwitches(),
                          BoardList[j]->GetPositiveLimitSwitches(),
                          BoardList[j]->GetHomeSwitches());
                mvwprintw(stdscr, 16, 9+58*j, "Node: %s", nodeStr[j]);
                mvwprintw(stdscr, 16, 27+58*j, "Temp:  0x%02X    0x%02X", 
                          (unsigned int)BoardList[j]->GetAmpTemperature(0),
                          (unsigned int)BoardList[j]->GetAmpTemperature(1));
                if (loop_cnt > 500) {
                    lastTime = BoardList[j]->GetTimestamp();
                    if (lastTime > maxTime) {
                        maxTime = lastTime;
                    }
                }
            }
            for (i = 0; i < 4; i++) {
                mvwprintw(stdscr, 10, 9+8+(i+4*j)*13, "0x%04X", MotorCurrents[j][i]);
                BoardList[j]->SetMotorCurrent(i, MotorCurrents[j][i]);
            }
        }
//        Port.WriteAllBoardsBroadcast();
        Port.WriteAllBoards();

        mvwprintw(stdscr, 1, 50, "dt: %f",  (1.0 / 49125.0) * maxTime);

        wrefresh(stdscr);
        usleep(500);
    }

    for (j= 0; j < BoardList.size(); j++) {
        BoardList[j]->WritePowerEnable(false);      // Turn power off
        BoardList[j]->WriteAmpEnable(0x0f, 0x00);   // Turn power off
        BoardList[j]->WriteSafetyRelay(false);
        Port.RemoveBoard(BoardList[j]->GetBoardId());
    }

    endwin();
    return 0;
}
