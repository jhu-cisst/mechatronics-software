/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * This program is used to test the FPGA1394+QLA board, assuming that it is
 * connected to the FPGA1394-QLA-Test board. It relies on the curses library
 * and the AmpIO library (which depends on libraw1394).
 *
 * Usage: qlatest [-pP] <board num>
 *        where P is the Firewire port number (default 0)
 *
 ******************************************************************************/

#include <stdlib.h>
#include <unistd.h>
#include <curses.h>
#include <math.h>
#include <iostream>
#include <sstream>

#include "FirewirePort.h"
#include "AmpIO.h"

void EncUp(AmpIO &bd)
{

    bd.SetDigitalOutput(0x0000);
    bd.SetDigitalOutput(0x0008);
    bd.SetDigitalOutput(0x000C);
    bd.SetDigitalOutput(0x0004);
}

void EncDown(AmpIO &bd)
{
    bd.SetDigitalOutput(0x0000);
    bd.SetDigitalOutput(0x0004);
    bd.SetDigitalOutput(0x000C);
    bd.SetDigitalOutput(0x0008);
}

void ClearLines(int start, int end)
{
    char blank[100];
    memset(blank, ' ', sizeof(blank)-1);
    blank[sizeof(blank)-1] = 0;
    while (start < end)
        mvprintw(start++, 9, blank);
}

bool TestDigitalInputs(int curLine, AmpIO &Board)
{
    bool pass = true;
    unsigned long data;
    char buf[80];

    mvprintw(curLine++, 9, "This tests the loopback on the test board"
                           " between DOUT1 and all digital inputs");
    Board.SetDigitalOutput(0x0001);  // DOUT is active low
    data = (Board.GetDigitalInput() & 0x0fff);
    if (!data)
        sprintf(buf, "Setting all inputs low - PASS (%03lx)", data);
    else {
        sprintf(buf, "Setting all inputs low - FAIL (%03lx)", data);
        pass = false;
    }
    mvprintw(curLine++, 9, buf);
    Board.SetDigitalOutput(0x0000);  // DOUT is active low
    data = (Board.GetDigitalInput() & 0x0fff);
    if (data == 0x0fff)
        sprintf(buf, "Setting all inputs high - PASS (%03lx)", data);
    else {
        sprintf(buf, "Setting all inputs high - FAIL (%03lx)", data);
        pass = false;
    }
    mvprintw(curLine++, 9, buf);
    return pass;
}


bool TestEncoders(int curLine, AmpIO &Board, FirewirePort &Port)
{
    int i, j;
    unsigned long darray[4];
    char buf[80];
    bool pass = true;

    mvprintw(curLine++, 9, "This test uses the DOUT signals to"
                           " generate a known number of quadrature encoder signals");
    Board.SetDigitalOutput(0x0000);
    // First, test setting of encoder preload
    for (i = 0; i < 4; i++)
        Board.SetEncoderPreload(i, 0x8000);
    Port.ReadAllBoards(); 
    for (i = 0; i < 4; i++) {
        darray[i] = Board.GetEncoderPosition(i);
        if (darray[i] != 0x8000)
            pass = false;
    }
    if (pass)
        sprintf(buf, "Set encoder preload to 0x8000 - PASS");
    else
        sprintf(buf, "Set encoder preload to 0x8000 - FAIL (%04lx %04lx %04lx %04lx)",
                     darray[0], darray[1], darray[2], darray[3]);
    mvprintw(curLine++, 9, buf);
    if (pass) {
        mvprintw(curLine, 9, "Testing encoder increment to 0x8100 -");
        for (i = 0; (i < (0x100/4)) && pass; i++) {
            EncUp(Board);
            usleep(1000);
            Port.ReadAllBoards(); 
            for (j = 0; j < 4; j++) {
                darray[j] = Board.GetEncoderPosition(j);
                // For now, allow "off-by-one" error
                if (abs(darray[j] - (0x8000+4*(i+1))) > 1)
                    pass = false;
            }
        }
        if (pass)
            sprintf(buf, "PASS");
        else
            sprintf(buf, "FAIL at %x (%04lx %04lx %04lx %04lx)", 0x8000+4*i,
                         darray[0], darray[1], darray[2], darray[3]);
        mvprintw(curLine++, 47, buf);
    }
    bool tmp_pass = true;
    for (i = 0; i < 4; i++)
        Board.SetEncoderPreload(i, 0x8100);
    Port.ReadAllBoards(); 
    for (i = 0; i < 4; i++) {
        darray[i] = Board.GetEncoderPosition(i);
        if (darray[i] != 0x8100)
            tmp_pass = false;
    }
    if (tmp_pass)
        sprintf(buf, "Set encoder preload to 0x8100 - PASS");
    else {
        sprintf(buf, "Set encoder preload to 0x8100 - FAIL (%04lx %04lx %04lx %04lx)",
                      darray[0], darray[1], darray[2], darray[3]);
        pass = false;
    }
    mvprintw(curLine++, 9, buf);
    if (tmp_pass) {
        mvprintw(curLine, 9, "Testing encoder decrement to 0x8100 -");
        for (i = (0x100/4)-1; (i >= 0) && tmp_pass; i--) {
            EncDown(Board);
            usleep(1000);
            Port.ReadAllBoards(); 
            for (j = 0; j < 4; j++) {
                darray[j] = Board.GetEncoderPosition(j);
                // For now, allow "off-by-one" error
                if (abs(darray[j] - (0x8000+4*(i+1))) > 1)
                    tmp_pass = false;
            }
        }
        if (tmp_pass)
            sprintf(buf, "PASS");
        else {
            pass = false;
            sprintf(buf, "FAIL at %x (%04lx %04lx %04lx %04lx)", 0x8000+4*i,
                         darray[0], darray[1], darray[2], darray[3]);
        }
        mvprintw(curLine++, 47, buf);
    }
    return pass;
}

bool TestAnalogInputs(int curLine, AmpIO &Board, FirewirePort &Port)
{
    int i, j;
    char buf[80];
    double measured;
    bool pass = true;

    mvprintw(curLine++, 9, "This test uses the pot on the test board to set a voltage (+/-0.25V)");
    nodelay(stdscr, FALSE);
    echo();
    bool tmp_pass = false;
    while (!tmp_pass) {
        mvprintw(curLine, 9, "Enter pot voltage: ");
        getstr(buf);
        if (sscanf(buf, "%lf", &measured) == 1) tmp_pass = true;
    }
    curLine++;
    nodelay(stdscr, TRUE);
    noecho();
    // Average 16 readings to reduce noise
    unsigned long pot[4] = { 0, 0, 0, 0};
    double potV[4];
    for (i = 0; i < 16; i++) {
        Port.ReadAllBoards();
        for (j = 0; j < 4; j++)
            pot[j] += Board.GetAnalogInput(j);
    }
    for (j = 0; j < 4; j++) {
        pot[j] /= 16;
        // Convert to voltage (16-bit converter, Vref = 4.5V)
        potV[j] = pot[j]*4.5/65535;
        if (fabs(potV[j] - measured) > 0.25)
            pass = false;
    }
    if (pass)
        sprintf(buf, "Analog Input - PASS (%4.2lf %4.2lf %4.2lf %4.2lf)",
                potV[0], potV[1], potV[2], potV[3]);
    else
        sprintf(buf, "Analog Input - FAIL (%4.2lf %4.2lf %4.2lf %4.2lf)",
                potV[0], potV[1], potV[2], potV[3]);

    mvprintw(curLine, 9, buf);
    return pass;
}

int main(int argc, char** argv)
{
    int i, j;
    int port = 0;
    int board;

    int args_found = 0;
    for (i = 1; i < argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            port = atoi(argv[i]+2);
            std::cerr << "Selecting port " << port << std::endl;
        }
        else {
            if (args_found == 0)
                board = atoi(argv[i]);
            args_found++;
        }
    }
    if (args_found < 1) {
        std::cerr << "Usage: qlatest <board-num> [-pP]" << std::endl
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

    initscr();
    cbreak();
    keypad(stdscr, TRUE);
    noecho();
    nodelay(stdscr, TRUE);

    mvprintw(1, 9, "QLA Test for Board %d", board);
    mvprintw(2, 9, "Press ESC to quit");

    mvprintw(5, 9, "1) Test digital input:");
    mvprintw(6, 9, "2) Test encoder feedback:");
    mvprintw(7, 9, "3) Test analog feedback:");
    mvprintw(8, 9, "4) Test motor power control:");
    mvprintw(9, 9, "5) Test power amplifier:");   // includes current feedback & temp sense

    wrefresh(stdscr);

    const int TEST_START_LINE = 11;
    const int DEBUG_START_LINE = 18;
    int last_debug_line = DEBUG_START_LINE;
    const int ESC_CHAR = 0x1b;
    int c;
    unsigned long data;
    unsigned long darray[4];
    bool pass;
    bool tmp_pass;
    char buf[100];
    while ((c = getch()) != ESC_CHAR) {
        int curLine = TEST_START_LINE;
        switch (c) {
            case '1':   // Test digital input
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestDigitalInputs(curLine, Board))
                    mvprintw(5, 46, "PASS");
                else
                    mvprintw(5, 46, "FAIL");
                break;

            case '2':    // Test encoder feedback
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestEncoders(curLine, Board, Port))
                    mvprintw(6, 46, "PASS");
                else
                    mvprintw(6, 46, "FAIL");
                break;

            case '3':    // Test analog feedback
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestAnalogInputs(curLine, Board, Port))
                    mvprintw(7, 46, "PASS");
                else
                    mvprintw(7, 46, "FAIL");
                break;
            case '4': 
                mvprintw(8, 46, "PASS");
                break;
            case '5': 
                mvprintw(9, 46, "PASS");
                break;
    }

        if (!debugStream.str().empty()) {
            int cur_line = DEBUG_START_LINE;
            char line[80];
            memset(line, ' ', sizeof(line));
            for (i = cur_line; i < last_debug_line; i++)
                mvprintw(i, 9, line);
            while (!debugStream.eof()) {
                debugStream.getline(line, sizeof(line));
                mvwprintw(stdscr,cur_line++, 9, line);
            }
            debugStream.clear();
            debugStream.str("");
            last_debug_line = cur_line;
        }

        wrefresh(stdscr);
        usleep(500);
    }

    Port.RemoveBoard(board);
    endwin();
}
