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

#include "FirewirePort.h"
#include "AmpIO.h"

int main(int argc, char** argv)
{
    int i;
    int port = 0;
    int board = 0;

    int args_found = 0;
    for (i = 1; i < argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            port = atoi(argv[i]+2);
            std::cerr << "Selecting port " << port << std::endl;
        }
        else {
            board = atoi(argv[i]);
            std::cerr << "Selecting board " << board << std::endl;
            args_found++;
        }
    }
    if (args_found < 1) {
        std::cerr << "Usage: sensors <board-num> [-pP]" << std::endl
                  << "       where P = port number (default 0)" << std::endl;
        return 0;
    }

    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);
    FirewirePort Port(port, debugStream);
    if (!Port.IsOK()) {
        std::cerr << "Failed to initialize firewire port " << port << std::endl;
        return -1;
    }
    AmpIO Board(board);
    Port.AddBoard(&Board);

    for (i = 0; i < 4; i++)
        Board.SetEncoderPreload(i, 0x1000*i);

    initscr();
    cbreak();
    keypad(stdscr, TRUE);
    noecho();
    nodelay(stdscr, TRUE);

    mvwprintw(stdscr, 1, 9, "Sensor Feedback for Board %d", board);
    mvwprintw(stdscr, 2, 9, "Press space to quit, r to reset port, 0-3 to toggle digital output bit");
    wrefresh(stdscr);

    for (i = 0; i < 4; i++)
        mvwprintw(stdscr, 4, 9+8+i*10, "Axis%d", i);
    mvwprintw(stdscr, 5, 9, "Enc:");
    mvwprintw(stdscr, 6, 9, "Pot:");
    mvwprintw(stdscr, 7, 9, "Vel:");
    mvwprintw(stdscr, 8, 9, "VelF:");
    mvwprintw(stdscr, 9, 9, "Cur:");
    mvwprintw(stdscr, 13, 9, "Node:");

    unsigned long dig_out = 0;

    int loop_cnt = 0;
    int last_debug_line = 16;
    while (1) {
        int c = getch();
        if (c == ' ') break;
        if (c == 'r') Port.Reset();
        if ((c >= '0') && (c <= '3')) {
            // toggle digital output bit
            dig_out = dig_out^(1<<(c-'0'));
            Board.SetDigitalOutput(dig_out);
        }

        mvwprintw(stdscr, 14, 41, "%10d", loop_cnt++);

        if (!debugStream.str().empty()) {
            int cur_line = 16;
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

        int node = Port.GetNodeId(board);
        if (node < FirewirePort::MAX_NODES)        
            mvwprintw(stdscr, 14, 16, "%d   ", Port.GetNodeId(board));
        else
            mvwprintw(stdscr, 14, 16, "none");

        Port.ReadAllBoards();
        if (Board.ValidRead()) {

            for (i = 0; i < 4; i++) {
                mvwprintw(stdscr, 5, 9+8+i*10, "0x%04X", Board.GetEncoderPosition(i));
                mvwprintw(stdscr, 6, 9+8+i*10, "0x%04X", Board.GetAnalogPosition(i));
                mvwprintw(stdscr, 7, 9+8+i*10, "0x%04X", Board.GetEncoderVelocity(i));
                mvwprintw(stdscr, 8, 9+8+i*10, "0x%04X", Board.GetEncoderFrequency(i));
                mvwprintw(stdscr, 9, 9+8+i*10, "0x%04X", Board.GetMotorCurrent(i));
            }
            dig_out = Board.GetDigitalOutput();
            mvwprintw(stdscr, 11, 9, "Status:  0x%08X    Timestamp: %08X   DigOut: 0x%01X", 
                      Board.GetStatus(), Board.GetTimestamp(), dig_out);
            unsigned long dig_in = Board.GetDigitalInput();
            mvwprintw(stdscr, 12, 9, "NegLim:  0x%01X    PosLim:  0x%01X  Home: 0x%01X",
                      (dig_in&0x0f00)>>8, (dig_in&0x00f0)>>4, dig_in&0x000f);
        }

        wrefresh(stdscr);
        usleep(500);
    }

    Port.RemoveBoard(&Board);

    endwin();
    return 0;
}
