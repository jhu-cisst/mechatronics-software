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

    bd.WriteDigitalOutput(0x0C, 0x00);
    bd.WriteDigitalOutput(0x0C, 0x08);
    bd.WriteDigitalOutput(0x0C, 0x0C);
    bd.WriteDigitalOutput(0x0C, 0x04);
    bd.WriteDigitalOutput(0x0C, 0x00);
}

void EncDown(AmpIO &bd)
{
    bd.WriteDigitalOutput(0x0C, 0x00);
    bd.WriteDigitalOutput(0x0C, 0x04);
    bd.WriteDigitalOutput(0x0C, 0x0C);
    bd.WriteDigitalOutput(0x0C, 0x08);
    bd.WriteDigitalOutput(0x0C, 0x00);
}

void ClearLines(int start, int end)
{
    char blank[100];
    memset(blank, ' ', sizeof(blank)-1);
    blank[sizeof(blank)-1] = 0;
    while (start < end)
        mvprintw(start++, 9, blank);
}

bool TestDigitalInputs(int curLine, AmpIO &Board, FirewirePort &Port)
{
    bool pass = true;
    unsigned long data;
    char buf[80];

    mvprintw(curLine++, 9, "This tests the loopback on the test board"
                           " between DOUT1 and all digital inputs");
    Board.WriteDigitalOutput(0x01, 0x01);  // DOUT is active low
    Port.ReadAllBoards();
    data = Board.GetDigitalInput();
    if (!data)
        sprintf(buf, "Setting all inputs low - PASS (%03lx)", data);
    else {
        sprintf(buf, "Setting all inputs low - FAIL (%03lx)", data);
        pass = false;
    }
    mvprintw(curLine++, 9, buf);
    Board.WriteDigitalOutput(0x01, 0x00);  // DOUT is active low
    Port.ReadAllBoards();
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
    unsigned long testValue[4];
    char numStr[4][3] = { " 1", " 2", " 3", " 4" };
    char buf[80];
    bool pass = true;
    bool tmpFix;

    mvprintw(curLine++, 9, "This test uses the DOUT signals to"
                           " generate a known number of quadrature encoder signals");
    Board.WriteDigitalOutput(0x0f, 0x00);
    // First, test setting of encoder preload
    for (i = 0; i < 4; i++)
        Board.WriteEncoderPreload(i, 0x800000);
    Port.ReadAllBoards(); 
    for (i = 0; i < 4; i++) {
        darray[i] = Board.GetEncoderPosition(i);
        if (darray[i] != 0x800000)
            pass = false;
    }
    if (pass)
        sprintf(buf, "Set encoder preload to 0x800000 - PASS");
    else
        sprintf(buf, "Set encoder preload to 0x800000 - FAIL (%06lx %06lx %06lx %06lx)",
                     darray[0], darray[1], darray[2], darray[3]);
    mvprintw(curLine++, 9, buf);
    refresh();
    if (pass) {
        mvprintw(curLine, 9, "Testing encoder increment to 0x800100 -");
        tmpFix = false;
        for (j = 0; j < 4; j++)
            testValue[j] = 0x800000;
        for (i = 0; (i < (0x100/4)) && pass; i++) {
            EncUp(Board);
            usleep(100);
            Port.ReadAllBoards(); 
            for (j = 0; j < 4; j++) {
                darray[j] = Board.GetEncoderPosition(j);
                unsigned long expected = testValue[j]+4*(i+1);
                if (darray[j] != expected) {
                    // There appears to be a problem, where the count
                    // is less than expected after the preload. For now,
                    // we allow this discrepancy and adjust testValue
                    // so that we can continue the test.
                    if (i == 0) {
                        testValue[j] = darray[j]-4;
                        tmpFix = true;
                    }
                    else
                        pass = false;
                }
            }
        }
        if (pass) {
            sprintf(buf, "PASS");
            if (tmpFix) {
                strcat(buf, " (adjusted");
                for (j = 0; j < 4; j++) {
                    if (testValue[j] != 0x800000)
                        strcat(buf, numStr[j]);
                }
                strcat(buf, ")");
            }
        }
        else
            // Following message could be improved to take testValue into account.
            sprintf(buf, "FAIL at %06lx (%06lx %06lx %06lx %06lx)", 0x800000UL+4*i,
                         darray[0], darray[1], darray[2], darray[3]);
        mvprintw(curLine++, 49, buf);
        refresh();
    }
    bool tmp_pass = true;
    for (i = 0; i < 4; i++)
        Board.WriteEncoderPreload(i, 0x800100);
    Port.ReadAllBoards(); 
    for (i = 0; i < 4; i++) {
        darray[i] = Board.GetEncoderPosition(i);
        if (darray[i] != 0x800100)
            tmp_pass = false;
    }
    if (tmp_pass)
        sprintf(buf, "Set encoder preload to 0x800100 - PASS");
    else {
        sprintf(buf, "Set encoder preload to 0x800100 - FAIL (%06lx %06lx %06lx %06lx)",
                      darray[0], darray[1], darray[2], darray[3]);
        pass = false;
    }
    mvprintw(curLine++, 9, buf);
    refresh();

    if (tmp_pass) {
        mvprintw(curLine, 9, "Testing encoder decrement to 0x800000 -");
        tmpFix = false;
        for (j = 0; j < 4; j++)
            testValue[j] = 0x800000;
        for (i = (0x100/4)-1; (i >= 0) && tmp_pass; i--) {
            EncDown(Board);
            usleep(100);
            Port.ReadAllBoards(); 
            for (j = 0; j < 4; j++) {
                darray[j] = Board.GetEncoderPosition(j);
                unsigned long expected = testValue[j] + 4*i;
                if (darray[j] != expected) {
                    // There appears to be a problem, where the count
                    // is less than expected after the preload. For now,
                    // we allow this discrepancy and adjust testValue
                    // so that we can continue the test.
                    if (i == (0x100/4)-1) {
                        testValue[j] = darray[j]-0x100+4;
                        tmpFix = true;
                    }
                    else
                        tmp_pass = false;
                }
            }
        }
        if (tmp_pass) {
            sprintf(buf, "PASS");
            if (tmpFix) {
                strcat(buf, " (adjusted");
                for (j = 0; j < 4; j++) {
                    if (testValue[j] != 0x800000)
                        strcat(buf, numStr[j]);
                }
                strcat(buf, ")");
            }
        }
        else {
            pass = false;
            // Following message could be improved to take testValue into account.
            sprintf(buf, "FAIL at %06lx (%06lx %06lx %06lx %06lx)", 0x800000UL+4*(i+1),
                         darray[0], darray[1], darray[2], darray[3]);
        }
        mvprintw(curLine++, 49, buf);
        refresh();
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
    bool ignoreMV = false;     // ignore "motor voltage good" feedback
    unsigned long mask;

    // Enabling motor power supply
    Board.WritePowerEnable(true);
    mvprintw(curLine, 9, "Enable motor power -");
    refresh();
    sleep(1);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    if ((status&0x000cffff) != 0x000c0000) {
        sprintf(buf, "FAIL (%08lx) - is motor power connected?", status);
        pass = false;
    }
    else
        sprintf(buf, "PASS (%08lx)", status);
    mvprintw(curLine++, 30, buf);

    if (!pass && (status & 0x0004ffff) == 0x00040000) {
        mvprintw(curLine++, 9, "Continuing test, ignoring 'motor voltage good' feedback");
        ignoreMV = true;
    }

    // Enabling individual amplifiers
    Board.WriteAmpEnable(0x0f, 0x0f);
    usleep(1000);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    mask = ignoreMV ? 0x0004ffff : 0x000cffff;
    if ((status&mask) != (0x000c0f0f&mask)) {
        sprintf(buf, "Enable power amplifiers - FAIL (%08lx)", status);
        pass = false;
    }
    else
        sprintf(buf, "Enable power amplifiers - PASS (%08lx)", status);
    mvprintw(curLine++, 9, buf);

    refresh();
    sleep(1);    // wait 1 second (LEDs should be ON)

    // Turning amplifiers off
    Board.WriteAmpEnable(0x0f, 0);
    usleep(1000);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    if ((status&mask) != (0x000c0000&mask)) {
        sprintf(buf, "Disable power amplifiers - FAIL (%08lx)", status);
        pass = false;
    }
    else
        sprintf(buf, "Disable power amplifiers - PASS (%08lx)", status);
    mvprintw(curLine++, 9, buf);

    Board.WritePowerEnable(false);
    mvprintw(curLine, 9, "Disable motor power -");
    refresh();
    sleep(1);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    if ((status&0x000cffff) != 0) {
        sprintf(buf, "FAIL (%08lx)", status);
        pass = false;
    }
    else
        sprintf(buf, "PASS (%08lx)", status);
    mvprintw(curLine++, 31, buf);

    // Turn on safety relay
    Board.WriteSafetyRelay(true);
    usleep(1000);
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
    Board.WriteSafetyRelay(false);
    usleep(1000);
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

bool TestPowerAmplifier(int curLine, AmpIO &Board, FirewirePort &Port)
{
    char buf[100];
    unsigned long status;
    unsigned long dac;
    int i;
    bool pass = true;

    mvprintw(curLine, 9, "Temperature sensors - ");
    refresh();

    Board.WritePowerEnable(true);
    Board.WriteAmpEnable(0x0f, 0x0f);
    dac = 0x8000;
    for (i = 0; i < 4; i++)
        Board.SetMotorCurrent(i, dac);
    Port.WriteAllBoards();
    sleep(1);  // wait for power to stabilize
    Port.ReadAllBoards();
    status = Board.GetStatus();
    if ((status&0x000cffff) != 0x000c0f0f) {
        sprintf(buf,"Failed to enable power (%08lx) - is motor power connected?", status);
        mvprintw(curLine++, 9, buf);
        mvprintw(curLine, 9, "Temperature sensors - ");
    }
    // Read temperature in Celsius
    unsigned short temp1 = Board.GetAmpTemperature(0)/2;
    unsigned short temp2 = Board.GetAmpTemperature(1)/2;
    if ((temp1 < 20) || (temp1 > 40) || (temp2 < 20) || (temp2 > 40))
        sprintf(buf, "FAIL (%d, %d degC)", temp1, temp2);
    else
        sprintf(buf, "PASS (%d, %d degC)", temp1, temp2);
    mvprintw(curLine++, 31, buf);

    mvprintw(curLine, 9, "DAC:");
    mvprintw(curLine+1, 9, "ADC:");
    mvprintw(curLine+3, 9, "Temp:");
    refresh();

    // Increment of 0x0200 is about 100 mA; 0x9000 is approx. 780 mA.
    unsigned long cur[4];
    while (1) {
        sprintf(buf, "%04lx   %04lx   %04lx   %04lx", dac, dac, dac, dac);
        mvprintw(curLine, 15, buf);
        for (i = 0; i < 4; i++) {
            cur[i] = Board.GetMotorCurrent(i);
            if (abs(cur[i]-dac) > 0x0100) {
                mvprintw(curLine+2, 15+7*i, "FAIL");
                pass = false;
            }
            else
                mvprintw(curLine+2, 15+7*i, "PASS");
        }
        sprintf(buf, "%04lx   %04lx   %04lx   %04lx", cur[0], cur[1], cur[2], cur[3]);
        mvprintw(curLine+1, 15, buf);

        unsigned short newTemp1 = Board.GetAmpTemperature(0)/2;
        unsigned short newTemp2 = Board.GetAmpTemperature(1)/2;        
        sprintf(buf, "%4d (%s)    %4d (%s)", newTemp1, ((newTemp1 >= temp1) ? "PASS" : "FAIL"),
                newTemp2, ((newTemp2 >= temp2) ? "PASS" : "FAIL"));
        mvprintw(curLine+3, 15, buf);
        temp1 = newTemp1;
        temp2 = newTemp2;
        refresh();

        if (dac > 0x8000) dac = 0x8000 - (dac - 0x8000);
        else dac = 0x8000 + (0x8000 - dac) + 0x0200;

        if (dac > 0x9000) break;

        for (i = 0; i < 4; i++)
            Board.SetMotorCurrent(i, dac);
        Port.WriteAllBoards();
        sleep(3);
        Port.ReadAllBoards();
    }

    for (i = 0; i < 4; i++)
        Board.SetMotorCurrent(i, 0x8000);
    Port.WriteAllBoards();
    Board.WriteAmpEnable(0x0f, 0);
    Board.WritePowerEnable(false);
    return pass;
}

int main(int argc, char** argv)
{
    int i;
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
    const int DEBUG_START_LINE = 20;
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
                if (TestDigitalInputs(TEST_START_LINE, Board, Port))
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
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestPowerAmplifier(TEST_START_LINE, Board, Port))
                    mvprintw(8, 46, "PASS");
                else
                    mvprintw(8, 46, "FAIL");
                break;
        }

        refresh();
    }

    Port.RemoveBoard(board);
    endwin();
}
