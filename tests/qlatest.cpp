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
#include <fstream>

#include "FirewirePort.h"
#include "AmpIO.h"

/*!
 \brief Increment encoder counts
 \param[in] bd FPGA Board
*/
void EncUp(AmpIO &bd)
{
    bd.WriteDigitalOutput(0x03, 0x00);
    bd.WriteDigitalOutput(0x03, 0x01);
    bd.WriteDigitalOutput(0x03, 0x03);
    bd.WriteDigitalOutput(0x03, 0x02);
    bd.WriteDigitalOutput(0x03, 0x00);
}

/*!
 \brief Decrement encoder counts
 \param[in] bd FPGA Board
*/
void EncDown(AmpIO &bd)
{
    bd.WriteDigitalOutput(0x03, 0x00);
    bd.WriteDigitalOutput(0x03, 0x02);
    bd.WriteDigitalOutput(0x03, 0x03);
    bd.WriteDigitalOutput(0x03, 0x01);
    bd.WriteDigitalOutput(0x03, 0x00);
}

void ClearLines(int start, int end)
{
    char blank[100];
    memset(blank, ' ', sizeof(blank)-1);
    blank[sizeof(blank)-1] = 0;
    while (start < end)
        mvprintw(start++, 9, blank);
}

bool TestDigitalInputs(int curLine, AmpIO &Board, FirewirePort &Port, std::ofstream &logFile)
{
    bool pass = true;
    unsigned long data;
    char buf[80];

    logFile << std::endl << "=== DIGITAL INPUTS ===" << std::endl;
    mvprintw(curLine++, 9, "This tests the loopback on the test board"
                           " between DOUT4 and all digital inputs");
    Board.WriteDigitalOutput(0x08, 0x08);  // DOUT is active high

    usleep(1000);
    Port.ReadAllBoards();
    data = (Board.GetDigitalInput() & 0x0fff);
    logFile << "   All inputs low: " << std::hex << data;
    if (!data) {
        sprintf(buf, "Setting all inputs low - PASS (%03lx)", data);
        logFile << " - PASS" << std::endl;
    }
    else {
        sprintf(buf, "Setting all inputs low - FAIL (%03lx)", data);
        logFile << " - FAIL" << std::endl;
        pass = false;
    }
    mvprintw(curLine++, 9, buf);
    Board.WriteDigitalOutput(0x08, 0x00);  // DOUT is active low

    usleep(1000);
    Port.ReadAllBoards();
    data = (Board.GetDigitalInput() & 0x0fff);
    logFile << "   All inputs high: " << std::hex << data;
    if (data == 0x0fff) {
        sprintf(buf, "Setting all inputs high - PASS (%03lx)", data);
        logFile << " - PASS" << std::endl;
    }
    else {
        sprintf(buf, "Setting all inputs high - FAIL (%03lx)", data);
        logFile << " - FAIL" << std::endl;
        pass = false;
    }
    mvprintw(curLine++, 9, buf);
    return pass;
}


bool TestEncoders(int curLine, AmpIO &Board, FirewirePort &Port, std::ofstream &logFile)
{
    int i, j;
    unsigned long darray[4];
    unsigned long testValue[4];
    char numStr[4][3] = { " 1", " 2", " 3", " 4" };
    char buf[80];
    bool pass = true;
    bool tmpFix;

    logFile << std::endl << "=== ENCODER INPUTS ===" << std::endl;
    mvprintw(curLine++, 9, "This test uses the DOUT signals to"
                           " generate a known number of quadrature encoder signals");
    Board.WriteDigitalOutput(0x0f, 0x00);
    // First, test setting of encoder preload
    logFile << "   Preload to 0x000000: ";
    for (i = 0; i < 4; i++)
        Board.WriteEncoderPreload(i, 0);
    Port.ReadAllBoards(); 
    for (i = 0; i < 4; i++) {
        darray[i] = Board.GetEncoderPosition(i);
        logFile << std::hex << darray[i] << " ";
        if (darray[i] != 0)
            pass = false;
    }
    if (pass) {
        logFile << "- PASS" << std::endl;
        sprintf(buf, "Set encoder preload to 0 - PASS");
    }
    else {
        logFile << "- FAIL" << std::endl;
        sprintf(buf, "Set encoder preload to 0 - FAIL (%06lx %06lx %06lx %06lx)",
                     darray[0], darray[1], darray[2], darray[3]);
    }
    mvprintw(curLine++, 9, buf);
    refresh();
    if (pass) {
        logFile << "   Increment to 0x000100:" << std::endl;
        mvprintw(curLine, 9, "Testing encoder increment to 0x000100 -");
        tmpFix = false;
        for (j = 0; j < 4; j++)
            testValue[j] = 0;
        for (i = 0; (i < (0x100/4)) && pass; i++) {
            EncUp(Board);
            usleep(100);
            logFile << "      ";
            Port.ReadAllBoards(); 
            for (j = 0; j < 4; j++) {
                darray[j] = Board.GetEncoderPosition(j);
                logFile << darray[j];
                unsigned long expected = testValue[j]+4*(i+1);
                if (darray[j] != expected) {
                    // There appears to be a problem, where the count
                    // is less than expected after the preload. For now,
                    // we allow this discrepancy and adjust testValue
                    // so that we can continue the test.
                    if (i == 0) {
                        logFile << "  ";
                        testValue[j] = darray[j]-4;
                        tmpFix = true;
                    }
                    else {
                        logFile << "* ";
                        pass = false;
                    }
                }
                else
                    logFile << "  ";
            }
            logFile << std::endl;
        }
        if (pass) {
            logFile << "   Increment result - PASS" << std::endl;
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
            logFile << "   Increment result - FAIL" << std::endl;
            // Following message could be improved to take testValue into account.
            sprintf(buf, "FAIL at %06lx (%06lx %06lx %06lx %06lx)", 0x800000UL+4*i,
                         darray[0], darray[1], darray[2], darray[3]);
        }
        mvprintw(curLine++, 49, buf);
        refresh();
    }
    bool tmp_pass = true;
    logFile << "   Preload to 0x000100: ";
    for (i = 0; i < 4; i++)
        Board.WriteEncoderPreload(i, 0x000100);
    Port.ReadAllBoards(); 
    for (i = 0; i < 4; i++) {
        darray[i] = Board.GetEncoderPosition(i);
        logFile << std::hex << darray[i] << " ";
        if (darray[i] != 0x000100)
            tmp_pass = false;
    }
    if (tmp_pass) {
        logFile << "- PASS" << std::endl;
        sprintf(buf, "Set encoder preload to 0x000100 - PASS");
    }
    else {
        logFile << "- FAIL" << std::endl;
        sprintf(buf, "Set encoder preload to 0x000100 - FAIL (%06lx %06lx %06lx %06lx)",
                      darray[0], darray[1], darray[2], darray[3]);
        pass = false;
    }
    mvprintw(curLine++, 9, buf);
    refresh();

    if (tmp_pass) {
        logFile << "   Decrement to 0x000000:" << std::endl;
        mvprintw(curLine, 9, "Testing encoder decrement to 0x000000 -");
        tmpFix = false;
        for (j = 0; j < 4; j++)
            testValue[j] = 0;
        for (i = (0x100/4)-1; (i >= 0) && tmp_pass; i--) {
            EncDown(Board);
            usleep(100);
            logFile << "      ";
            Port.ReadAllBoards(); 
            for (j = 0; j < 4; j++) {
                darray[j] = Board.GetEncoderPosition(j);
                logFile << std::hex << darray[j];
                unsigned long expected = testValue[j] + 4*i;
                if (darray[j] != expected) {
                    // There appears to be a problem, where the count
                    // is less than expected after the preload. For now,
                    // we allow this discrepancy and adjust testValue
                    // so that we can continue the test.
                    if (i == (0x100/4)-1) {
                        logFile << "  ";
                        testValue[j] = darray[j]-0x100+4;
                        tmpFix = true;
                    }
                    else {
                        logFile << "* ";
                        tmp_pass = false;
                    }
                }
                else
                    logFile << "  ";
            }
            logFile << std::endl;
        }
        if (tmp_pass) {
            logFile << "   Decrement result - PASS" << std::endl;
            sprintf(buf, "PASS");
            if (tmpFix) {
                strcat(buf, " (adjusted");
                for (j = 0; j < 4; j++) {
                    if (testValue[j] != 0)
                        strcat(buf, numStr[j]);
                }
                strcat(buf, ")");
            }
        }
        else {
            logFile << "   Decrement result - FAIL" << std::endl;
            pass = false;
            // Following message could be improved to take testValue into account.
            sprintf(buf, "FAIL at %06lx (%06lx %06lx %06lx %06lx)", 0x000000UL+4*(i+1),
                         darray[0], darray[1], darray[2], darray[3]);
        }
        mvprintw(curLine++, 49, buf);
        refresh();
    }
    if (pass)
        logFile << "   Overall result: PASS" << std::endl;
    else
        logFile << "   Overall result: FAIL" << std::endl;
    return pass;
}

bool TestAnalogInputs(int curLine, AmpIO &Board, FirewirePort &Port, std::ofstream &logFile)
{
    int i, j;
    char buf[80];
    double measured;
    bool pass = true;

    logFile << std::endl << "=== Analog Inputs ===" << std::endl;
    mvprintw(curLine++, 9, "This test uses the pot on the test board to set a voltage (+/-0.25V)");
    bool tmp_pass = false;
    while (!tmp_pass) {
        mvprintw(curLine, 9, "Enter pot voltage: ");
        getstr(buf);
        if (sscanf(buf, "%lf", &measured) == 1) tmp_pass = true;
    }
    logFile << "   Input voltage: " << measured << std::endl;
    curLine++;
    // Average 16 readings to reduce noise
    unsigned long pot[4] = { 0, 0, 0, 0};
    double potV[4];
    for (i = 0; i < 16; i++) {
        Port.ReadAllBoards();
        for (j = 0; j < 4; j++)
            pot[j] += Board.GetAnalogInput(j);
    }
    logFile << "   Readings: ";
    for (j = 0; j < 4; j++) {
        pot[j] /= 16;
        // Convert to voltage (16-bit converter, Vref = 4.5V)
        potV[j] = pot[j]*4.5/65535;
        logFile << potV[j] << " ";
        if (fabs(potV[j] - measured) > 0.25)
            pass = false;
    }
    if (pass) {
        logFile << "- PASS" << std::endl;
        sprintf(buf, "Analog Input - PASS (%4.2lf %4.2lf %4.2lf %4.2lf)",
                potV[0], potV[1], potV[2], potV[3]);
    }
    else {
        logFile << "- FAIL" << std::endl;
        sprintf(buf, "Analog Input - FAIL (%4.2lf %4.2lf %4.2lf %4.2lf)",
                potV[0], potV[1], potV[2], potV[3]);
    }

    mvprintw(curLine, 9, buf);
    return pass;
}

bool TestMotorPowerControl(int curLine, AmpIO &Board, FirewirePort &Port, std::ofstream &logFile)
{
    char buf[100];
    unsigned long status;
    bool pass = true;
    bool ignoreMV = false;     // ignore "motor voltage good" feedback
    unsigned long mask;

    logFile << std::endl << "=== Motor Power Control ===" << std::endl;
    // Enabling motor power supply
    Board.WritePowerEnable(true);
    mvprintw(curLine, 9, "Enable motor power -");
    refresh();
    sleep(1);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    logFile << "   Enable motor power: " << std::hex << status;
    if ((status&0x000cffff) != 0x000c0000) {
        logFile << " - FAIL" << std::endl;
        sprintf(buf, "FAIL (%08lx) - is motor power connected?", status);
        pass = false;
    }
    else {
        logFile << " - PASS" << std::endl;
        sprintf(buf, "PASS (%08lx)", status);
    }
    mvprintw(curLine++, 30, buf);

    if (!pass && (status & 0x0004ffff) == 0x00040000) {
        logFile << "   Continuing test, ignoring 'motor voltage good' feedback" << std::endl;
        mvprintw(curLine++, 9, "Continuing test, ignoring 'motor voltage good' feedback");
        ignoreMV = true;
    }

    // Enabling individual amplifiers
    Board.WriteAmpEnable(0x0f, 0x0f);
    usleep(1000);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    logFile << "   Amplifier enable: " << std::hex << status;
    mask = ignoreMV ? 0x0004ffff : 0x000cffff;
    if ((status&mask) != (0x000c0f0f&mask)) {
        logFile << " - FAIL" << std::endl;
        sprintf(buf, "Enable power amplifiers - FAIL (%08lx)", status);
        pass = false;
    }
    else {
        logFile << " - PASS" << std::endl;
        sprintf(buf, "Enable power amplifiers - PASS (%08lx)", status);
    }
    mvprintw(curLine++, 9, buf);

    refresh();
    sleep(1);    // wait 1 second (LEDs should be ON)

    // Turning amplifiers off
    Board.WriteAmpEnable(0x0f, 0);
    usleep(1000);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    logFile << "   Amplifier disable: " << std::hex << status;
    if ((status&mask) != (0x000c0000&mask)) {
        logFile << " - FAIL" << std::endl;
        sprintf(buf, "Disable power amplifiers - FAIL (%08lx)", status);
        pass = false;
    }
    else {
        logFile << " - PASS" << std::endl;
        sprintf(buf, "Disable power amplifiers - PASS (%08lx)", status);
    }
    mvprintw(curLine++, 9, buf);

    Board.WritePowerEnable(false);
    mvprintw(curLine, 9, "Disable motor power -");
    refresh();
    sleep(1);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    logFile << "   Disable motor power: " << std::hex << status;
    if ((status&0x000cffff) != 0) {
        logFile << " - FAIL" << std::endl;
        sprintf(buf, "FAIL (%08lx)", status);
        pass = false;
    }
    else {
        logFile << " - PASS" << std::endl;
        sprintf(buf, "PASS (%08lx)", status);
    }
    mvprintw(curLine++, 31, buf);

    // Turn on safety relay
    Board.WriteSafetyRelay(true);
    usleep(10000);  // 10 ms 
    logFile << "   Enable safety relay: " << std::hex << Board.ReadStatus();
    if (!Board.ReadSafetyRelayStatus()) {
        logFile << " - FAIL" << std::endl;
        // Old QLA boards (before Rev 1.3) require +12V to be connected to relay
        sprintf(buf, "Enable safety relay - FAIL (%08x)", Board.ReadStatus());
        pass = false;
    }
    else {
        logFile << " - PASS" << std::endl;
        sprintf(buf, "Enable safety relay - PASS (%08x)", Board.ReadStatus());
    }
    mvprintw(curLine++, 9, buf);

    refresh();
    sleep(1);

    // Turn off safety relay
    Board.WriteSafetyRelay(false);
    usleep(1000);
    Port.ReadAllBoards();
    status = Board.GetStatus();
    logFile << "   Disable safety relay: " << std::hex << status;
    if ((status&0x00030000) != 0) {
        logFile << " - FAIL" << std::endl;
        sprintf(buf, "Disable safety relay - FAIL (%08lx)", status);
        pass = false;
    }
    else {
        logFile << " - PASS" << std::endl;
        sprintf(buf, "Disable safety relay - PASS (%08lx)", status);
    }
    mvprintw(curLine++, 9, buf);

    return pass;
}

bool TestPowerAmplifier(int curLine, AmpIO &Board, FirewirePort &Port, std::ofstream &logFile)
{
    char buf[100];
    unsigned long status;
    unsigned long dac;
    int i;
    bool pass = true;

    // Disable watchdog
    Board.WriteWatchdogPeriod(0x0000);  

    logFile << std::endl << "=== Power Amplifier Test ===" << std::endl;
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
    logFile << "   Motor temperatures: " << std::dec << temp1 << ", " << temp2 << " degC (expected range is 20-40) ";
    if ((temp1 < 20) || (temp1 > 40) || (temp2 < 20) || (temp2 > 40)) {
        logFile << "- FAIL" << std::endl;
        sprintf(buf, "FAIL (%d, %d degC)", temp1, temp2);
    }
    else {
        logFile << "- PASS" << std::endl;
        sprintf(buf, "PASS (%d, %d degC)", temp1, temp2);
    }
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
        logFile << "   Commanded current: " << std::hex << dac << ", Measured currents: ";
        for (i = 0; i < 4; i++) {
            cur[i] = Board.GetMotorCurrent(i);
            logFile << std::hex << cur[i];
            if (abs(cur[i]-dac) > 0x0100) {
                logFile << "* ";
                mvprintw(curLine+2, 15+7*i, "FAIL");
                pass = false;
            }
            else {
                logFile << "  ";
                mvprintw(curLine+2, 15+7*i, "PASS");
            }
        }
        logFile << std::endl;
        sprintf(buf, "%04lx   %04lx   %04lx   %04lx", cur[0], cur[1], cur[2], cur[3]);
        mvprintw(curLine+1, 15, buf);

        unsigned short newTemp1 = Board.GetAmpTemperature(0)/2;
        unsigned short newTemp2 = Board.GetAmpTemperature(1)/2;        
        logFile << "   Motor temperatures: " << std::dec  << newTemp1 << ", " << newTemp2 << " degC (expected to be increasing)";
        if ((newTemp1 >= temp1) && (newTemp2 >= temp2))
            logFile << " - PASS" << std::endl;
        else
            logFile << " - FAIL" << std::endl;
        sprintf(buf, "%4d (%s)    %4d (%s)", newTemp1, ((newTemp1 >= temp1) ? "PASS" : "FAIL"),
                newTemp2, ((newTemp2 >= temp2) ? "PASS" : "FAIL"));
        mvprintw(curLine+3, 15, buf);
        temp1 = newTemp1;
        temp2 = newTemp2;
        refresh();

        if (dac > 0x8000) dac = 0x8000 - (dac - 0x8000);
        else dac = 0x8000 + (0x8000 - dac) + 0x0200;

        if (dac > 0x8600) break;

        for (i = 0; i < 4; i++)
            Board.SetMotorCurrent(i, dac);
        Port.WriteAllBoards();
        sleep(1);
        Port.ReadAllBoards();
    }

    for (i = 0; i < 4; i++)
        Board.SetMotorCurrent(i, 0x8000);
    Port.WriteAllBoards();
    Board.WriteAmpEnable(0x0f, 0);
    Board.WritePowerEnable(false);

    // Reenable watchdog
    Board.WriteWatchdogPeriod(0xFFFF);
    if (pass)
        logFile << "   Overall result: PASS" << std::endl;
    else
        logFile << "   Overall result: FAIL" << std::endl;
    return pass;
}

bool TestEthernet(int curLine, AmpIO &Board, FirewirePort &Port, std::ofstream &logFile)
{
    logFile << std::endl << "=== Ethernet (KSZ8851) Test ===" << std::endl;
    if (Board.GetFirmwareVersion() < 5) {
        logFile << "   No Ethernet controller, firmware version = " << Board.GetFirmwareVersion() << std::endl;
        return false;
    }
    AmpIO_UInt16 status = Board.ReadKSZ8851Status();
    if (!(status&0x8000)) {
        logFile << "   No Ethernet controller, status = " << std::hex << status << std::endl;
        return false;
    }
    logFile << "   Ethernet controller status = " << std::hex << status << std::endl;
    // Reset the board
    Board.ResetKSZ8851();
    // Wait 100 msec
    usleep(100000L);
    // Read the status
    status = Board.ReadKSZ8851Status();
    logFile << "   After reset, status = " << std::hex << status << std::endl;
    // Read the Chip ID (16-bit read)
    AmpIO_UInt16 chipID = Board.ReadKSZ8851ChipID();
    logFile << "   Chip ID = " << std::hex << chipID << std::endl;
    if ((chipID&0xfff0) != 0x8870)
        return false;
#if 0
    // Read Chip ID using 8-bit read
    // TODO: Reading from 0xC0 works (get 0x72), but not from 0xC1 (get 0xC1 instead of 0x88)
    AmpIO_UInt8 chipIDHigh;
    Board.ReadKSZ8851Reg(0xC1, chipIDHigh);
    logFile << "   Chip ID high (8-bit read) = " << std::hex << static_cast<int>(chipIDHigh) << std::endl;
    if (chipIDHigh != 0x88)
        return false;
#endif
    // Walking bit pattern on MAC address register (R/W)
    AmpIO_UInt16 MacAddrOut, MacAddrIn;
    bool ret = true;
    for (MacAddrOut = 0x0001; MacAddrOut != 0; MacAddrOut <<= 1) {
        logFile << "   MAC address: " << std::hex;
        if (Board.WriteKSZ8851Reg(0x10, MacAddrOut))
            logFile << "write = " << MacAddrOut;
        MacAddrIn = 0;
        if (Board.ReadKSZ8851Reg(0x10, MacAddrIn))
            logFile << ", read = " << MacAddrIn;
        logFile << std::endl;
        if (MacAddrOut != MacAddrIn) ret = false;
    }
    return ret;
}

int main(int argc, char** argv)
{
    int i;
    int port = 0;
    int board = 0;
    bool requireQLA_SN = true;

    if (argc > 1) {
        int args_found = 0;
        for (i = 1; i < argc; i++) {
            if (argv[i][0] == '-') {
                if (argv[i][1] == 'p') {
                    port = atoi(argv[i]+2);
                    std::cerr << "Selecting port " << port << std::endl;
                }
                else if (argv[i][1] == 'k')  {
                    requireQLA_SN = false;
                }
                else {
                    std::cerr << "Usage: qlatest [<board-num>] [-pP] [-k]" << std::endl
                    << "       where <board-num> = rotary switch setting (0-15, default 0)" << std::endl
                    << "             P = port number (default 0)" << std::endl
                    << "             -k option means to continue test even if QLA serial number not found" << std::endl;
                    return 0;
                }
            }
            else {
                if (args_found == 0) {
                    board = atoi(argv[i]);
                    std::cerr << "Selecting board " << board << std::endl;
                }
                args_found++;
            }
       }
    }

    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);
    FirewirePort Port(port, debugStream);
    if (!Port.IsOK()) {
        std::cerr << "Failed to initialize firewire port " << port << std::endl;
        return -1;
    }
    AmpIO Board(board);
    Port.AddBoard(&Board);

    std::string logFilename("QLA_");
    std::string QLA_SN = Board.GetQLASerialNumber();
    if (QLA_SN.empty()) {
        if (requireQLA_SN) {
            std::cerr << "Failed to get QLA serial number (specify -k command line option to continue test)" << std::endl;
            return 0;
        }
        QLA_SN.assign("Unknown");
    }
    logFilename.append(QLA_SN);
    logFilename.append(".log");
    std::ofstream logFile(logFilename.c_str());
    if (!logFile.good()) {
        std::cerr << "Failed to open log file " << logFilename << std::endl;
        return 0;
    }
    logFile << "====== TEST REPORT ======" << std::endl << std::endl;
    logFile << "QLA S/N: " << QLA_SN << std::endl;

    std::string FPGA_SN = Board.GetFPGASerialNumber();
    if (FPGA_SN.empty())
        FPGA_SN.assign("Unknown");
    logFile << "FPGA S/N: " << FPGA_SN << std::endl;

    logFile << "FPGA Firmware Version: " << Board.GetFirmwareVersion() << std::endl;

    initscr();
    cbreak();
    keypad(stdscr, TRUE);

    mvprintw(1, 9, "QLA Test for Board %d, S/N %s", board, QLA_SN.c_str());

    mvprintw(3, 9, "0) Quit");
    mvprintw(4, 9, "1) Test digital input:");
    mvprintw(5, 9, "2) Test encoder feedback:");
    mvprintw(6, 9, "3) Test analog feedback:");
    mvprintw(7, 9, "4) Test motor power control:");
    mvprintw(8, 9, "5) Test power amplifier:");   // includes current feedback & temp sense
    mvprintw(9, 9, "6) Test Ethernet controller:");

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

        mvprintw(10, 10, "Select option: ");
        int c = getch();

        switch (c) {
            case '0':   // Quit
                done = true;
                break;
            case '1':   // Test digital input
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestDigitalInputs(TEST_START_LINE, Board, Port, logFile))
                    mvprintw(4, 46, "PASS");
                else
                    mvprintw(4, 46, "FAIL");
                break;

            case '2':    // Test encoder feedback
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestEncoders(TEST_START_LINE, Board, Port, logFile))
                    mvprintw(5, 46, "PASS");
                else
                    mvprintw(5, 46, "FAIL");
                break;

            case '3':    // Test analog feedback
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestAnalogInputs(TEST_START_LINE, Board, Port, logFile))
                    mvprintw(6, 46, "PASS");
                else
                    mvprintw(6, 46, "FAIL");
                break;

            case '4': 
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestMotorPowerControl(TEST_START_LINE, Board, Port, logFile))
                    mvprintw(7, 46, "PASS");
                else
                    mvprintw(7, 46, "FAIL");
                break;

            case '5': 
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestPowerAmplifier(TEST_START_LINE, Board, Port, logFile))
                    mvprintw(8, 46, "PASS");
                else
                    mvprintw(8, 46, "FAIL");
                break;

            case '6':
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestEthernet(TEST_START_LINE, Board, Port, logFile))
                    mvprintw(9, 46, "PASS");
                else
                    mvprintw(9, 46, "FAIL");
                break;
        }

        refresh();
    }

    Port.RemoveBoard(board);
    endwin();
}
