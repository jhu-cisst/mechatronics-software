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

    bd.SetDigitalOutput(0x0f, 0x00);
    bd.SetDigitalOutput(0x0f, 0x08);
    bd.SetDigitalOutput(0x0f, 0x0C);
    bd.SetDigitalOutput(0x0f, 0x04);
}

void EncDown(AmpIO &bd)
{
    bd.SetDigitalOutput(0x0f, 0x00);
    bd.SetDigitalOutput(0x0f, 0x04);
    bd.SetDigitalOutput(0x0f, 0x0C);
    bd.SetDigitalOutput(0x0f, 0x08);
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
    Board.SetDigitalOutput(0x0f, 0x01);  // DOUT is active low
    data = Board.GetDigitalInput();
    if (!data)
        sprintf(buf, "Setting all inputs low - PASS (%03lx)", data);
    else {
        sprintf(buf, "Setting all inputs low - FAIL (%03lx)", data);
        pass = false;
    }
    mvprintw(curLine++, 9, buf);
    Board.SetDigitalOutput(0x0f, 0x00);  // DOUT is active low
    data = Board.GetDigitalInput();
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
    Board.SetDigitalOutput(0x0f, 0x00);
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
    bool tmp_pass = false;
    while (!tmp_pass) {
        mvprintw(curLine, 9, "Enter pot voltage: ");
        getstr(buf);
        if (sscanf(buf, "%lf", &measured) == 1) tmp_pass = true;
    }
    curLine++;
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

bool TestMotorPowerControl(int curLine, AmpIO &Board, FirewirePort &Port)
{
    char buf[100];
    unsigned long status;
    bool pass = true;

    // Turning power on
    Board.SetPowerEnable(true);
    Board.SetAmpEnable(0x0f, 0x0f);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    if ((status&0x0000000f) != 0x0000000f) {
        sprintf(buf, "Enable motor power - FAIL to set status (%08lx)", status);
        pass = false;
    }
    else if ((status&0x00000f00) != 0x00000f00) {
        sprintf(buf, "Enable motor power - FAIL to enable amps (%08lx) - is motor power connected?",
                status);
        pass = false;
    }
    else
        sprintf(buf, "Enable motor power - PASS (%08lx)", status);
    mvprintw(curLine++, 9, buf);

    refresh();
    sleep(1);    // wait 1 second (LEDs should be ON)

    // Turning power off
    Board.SetPowerEnable(false);
    Board.SetAmpEnable(0x0f, 0);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    if ((status&0x0000000f) != 0) {
        sprintf(buf, "Disable motor power - FAIL to clear status (%08lx)", status);
        pass = false;
    }
    else if ((status&0x000000f00) != 0) {
        sprintf(buf, "Disable motor power - FAIL to disable amps (%08lx)", status);
        pass = false;
    }
    else
        sprintf(buf, "Disable motor power - PASS (%08lx)", status);
    mvprintw(curLine++, 9, buf);

    // Turn on safety relay
    Board.EnableSafetyRelay(true);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    if ((status&0x00030000) != 0x00030000) {
        sprintf(buf, "Enable safety relay - FAIL (%08lx) - is +12V connected to relay?", status);
        pass = false;
    }
    else
        sprintf(buf, "Enable safety relay - PASS (%08lx)", status);
    mvprintw(curLine++, 9, buf);

    refresh();
    sleep(1);

    // Turn off safety relay
    Board.EnableSafetyRelay(false);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    if ((status&0x00030000) != 0) {
        sprintf(buf, "Disable safety relay - FAIL (%08lx)", status);
        pass = false;
    }
    else
        sprintf(buf, "Disable safety relay - PASS (%08lx)", status);
    mvprintw(curLine++, 9, buf);

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

    mvprintw(1, 9, "QLA Test for Board %d", board);

    mvprintw(3, 9, "0) Quit");
    mvprintw(4, 9, "1) Test digital input:");
    mvprintw(5, 9, "2) Test encoder feedback:");
    mvprintw(6, 9, "3) Test analog feedback:");
    mvprintw(7, 9, "4) Test motor power control:");
    mvprintw(8, 9, "5) Test power amplifier:");   // includes current feedback & temp sense

    refresh();

    const int TEST_START_LINE = 12;
    const int DEBUG_START_LINE = 18;
    int last_debug_line = DEBUG_START_LINE;

    bool done = false;
    while (!done) {

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

        mvprintw(10, 9, "Select option: ");
        int c = getch();

        switch (c) {
            case '0':   // Quit
                done = true;
                break;
            case '1':   // Test digital input
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestDigitalInputs(TEST_START_LINE, Board))
                    mvprintw(4, 46, "PASS");
                else
                    mvprintw(4, 46, "FAIL");
                break;

            case '2':    // Test encoder feedback
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestEncoders(TEST_START_LINE, Board, Port))
                    mvprintw(5, 46, "PASS");
                else
                    mvprintw(5, 46, "FAIL");
                break;

            case '3':    // Test analog feedback
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestAnalogInputs(TEST_START_LINE, Board, Port))
                    mvprintw(6, 46, "PASS");
                else
                    mvprintw(6, 46, "FAIL");
                break;

            case '4': 
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestMotorPowerControl(TEST_START_LINE, Board, Port))
                    mvprintw(7, 46, "PASS");
                else
                    mvprintw(7, 46, "FAIL");
                break;

            case '5': 
                mvprintw(8, 46, "NOT IMPLEMENTED");
                break;
        }

        refresh();
    }

    Port.RemoveBoard(board);
    endwin();
}
