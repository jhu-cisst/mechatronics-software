/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * This is a preliminary curses program for continuously reading the encoder
 * values from Channel 1 of a single connected 1394-based controller.
 *
 * Compile: gcc -Wall -lraw1394 -lcurses <name of this file> -o <name of exec>
 * Usage: <name of exec>
 *
 * Note: Assumes only one node is connected (and its 1394 ID is 0)
 *
 ******************************************************************************/

#include <stdlib.h>
#include <unistd.h>
#include <curses.h>
#include <iostream>


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

    FirewirePort Port(port);
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
    mvwprintw(stdscr, 2, 9, "Press space to quit, r to reset port");
    wrefresh(stdscr);

    for (i = 0; i < 4; i++)
        mvwprintw(stdscr, 4, 9+8+i*10, "Axis%d", i);
    mvwprintw(stdscr, 5, 9, "Enc:");
    mvwprintw(stdscr, 6, 9, "Pot:");
    mvwprintw(stdscr, 7, 9, "Vel:");
    mvwprintw(stdscr, 8, 9, "VelF:");
    mvwprintw(stdscr, 9, 9, "Cur:");
    mvwprintw(stdscr, 13, 9, "Node:");

    int loop_cnt = 0;
    while (1) {
        int c = getch();
        if (c == ' ') break;
        if (c == 'r') Port.Reset();

        mvwprintw(stdscr, 13, 41, "%10d", loop_cnt++);
        if (!Port.IsOK()) continue;

        int node = Port.GetNodeId(board);
        if (node < FirewirePort::MAX_NODES)        
            mvwprintw(stdscr, 13, 16, "%d   ", Port.GetNodeId(board));
        else
            mvwprintw(stdscr, 13, 16, "none");

        Port.ReadAllBoards();
        if (!Board.ValidRead()) continue;

        for (i = 0; i < 4; i++) {
            mvwprintw(stdscr, 5, 9+8+i*10, "0x%04X", Board.GetEncoderPosition(i));
            mvwprintw(stdscr, 6, 9+8+i*10, "0x%04X", Board.GetAnalogPosition(i));
            mvwprintw(stdscr, 7, 9+8+i*10, "0x%04X", Board.GetEncoderVelocity(i));
            mvwprintw(stdscr, 8, 9+8+i*10, "0x%04X", Board.GetEncoderFrequency(i));
            mvwprintw(stdscr, 9, 9+8+i*10, "0x%04X", Board.GetMotorCurrent(i));
        }
        mvwprintw(stdscr, 11, 9, "Status:  0x%08X    Timestamp: %08X", 
                  Board.GetStatus(), Board.GetTimestamp());

        wrefresh(stdscr);
        usleep(500);
    }

    Port.RemoveBoard(&Board);

    endwin();
    return 0;
}
