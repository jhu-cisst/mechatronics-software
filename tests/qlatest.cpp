/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * This program is used to test the FPGA1394+QLA board, assuming that it is
 * connected to the FPGA1394-QLA-Test board. It relies on the Amp1394 library
 * (which depends on libraw1394 and/or pcap) and on the Amp1394Console library
 * (which may depend on curses).
 *
 * Usage: qlatest [-pP] <board num>
 *        where P is the Firewire port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *
 ******************************************************************************/

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "EthRawPort.h"
#endif
#include "EthUdpPort.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "Amp1394Console.h"

/*!
 \brief Increment encoder counts
 \param[in] bd FPGA Board
*/
void EncUp(AmpIO &bd, unsigned int qlaNum)
{
    uint8_t dout_mask = (qlaNum == 2) ? 0x30 : 0x03;
    bd.WriteDigitalOutput(dout_mask, 0x33);
    bd.WriteDigitalOutput(dout_mask, 0x22);
    bd.WriteDigitalOutput(dout_mask, 0x00);
    bd.WriteDigitalOutput(dout_mask, 0x11);
    bd.WriteDigitalOutput(dout_mask, 0x33);
}

/*!
 \brief Decrement encoder counts
 \param[in] bd FPGA Board
*/
void EncDown(AmpIO &bd, unsigned int qlaNum)
{
    uint8_t dout_mask = (qlaNum == 2) ? 0x30 : 0x03;
    bd.WriteDigitalOutput(dout_mask, 0x33);
    bd.WriteDigitalOutput(dout_mask, 0x11);
    bd.WriteDigitalOutput(dout_mask, 0x00);
    bd.WriteDigitalOutput(dout_mask, 0x22);
    bd.WriteDigitalOutput(dout_mask, 0x33);
}

void ClearLines(int start, int end)
{
    char blank[100];
    memset(blank, ' ', sizeof(blank)-1);
    blank[sizeof(blank)-1] = 0;
    while (start < end)
        Amp1394Console::Print(start++, 9, blank);
}

bool TestDigitalInputs(int curLine, AmpIO &Board, BasePort *Port, std::ofstream &logFile, unsigned int qlaNum)
{
    bool pass = true;
    unsigned long data;
    char buf[80];

    logFile << std::endl << "=== DIGITAL INPUTS ===" << std::endl;
    Amp1394Console::Print(curLine++, 9, "This tests the loopback on the test board"
                           " between DOUT4 and all digital inputs");

    uint8_t dout4_mask = (qlaNum == 2) ? 0x80 : 0x08;
    uint32_t data_mask = (qlaNum == 2) ? 0x0fff0000 : 0x00000fff;
    int data_shift = (qlaNum == 2) ? 16 : 0;

    logFile << "   All inputs low: ";
    if (Board.WriteDigitalOutput(dout4_mask, 0x00)) {

        Amp1394_Sleep(0.001);
        if (Port->ReadAllBoards()) {
            data = (Board.GetDigitalInput() & data_mask) >> data_shift;
            logFile << std::hex << data;
            if (!data) {
                sprintf(buf, "Setting all inputs low - PASS (%03lx)", data);
                logFile << " - PASS" << std::endl;
            }
            else {
                sprintf(buf, "Setting all inputs low - FAIL (%03lx)", data);
                logFile << " - FAIL" << std::endl;
                pass = false;
            }
        }
        else {
            strcpy(buf, "Failed to read from board");
            logFile << " - Failed to read from board" << std::endl;
            pass = false;
        }
    }
    else {
        strcpy(buf, "Failed to write to board");
        logFile << " - Failed to write to board" << std::endl;
        pass = false;
    }

    Amp1394Console::Print(curLine++, 9, buf);
    logFile << "   All inputs high: ";
    if (Board.WriteDigitalOutput(dout4_mask, dout4_mask)) {

        Amp1394_Sleep(0.001);
        if (Port->ReadAllBoards()) {
            data = (Board.GetDigitalInput() & data_mask) >> data_shift;
            logFile << std::hex << data;
            if (data == 0x0fff) {
                sprintf(buf, "Setting all inputs high - PASS (%03lx)", data);
                logFile << " - PASS" << std::endl;
            }
            else {
                sprintf(buf, "Setting all inputs high - FAIL (%03lx)", data);
                logFile << " - FAIL" << std::endl;
                pass = false;
            }
        }
        else {
            strcpy(buf, "Failed to read from board");
            logFile << " - Failed to read from board" << std::endl;
            pass = false;
        }
    }
    else {
        strcpy(buf, "Failed to write to board");
        logFile << " - Failed to write to board" << std::endl;
        pass = false;
    }

    Amp1394Console::Print(curLine++, 9, buf);
    return pass;
}


bool TestEncoders(int curLine, AmpIO &Board, BasePort *Port, std::ofstream &logFile, unsigned int qlaNum)
{
    int i, j;
    unsigned long darray[4];
    unsigned long testValue[4];
    char numStr[4][3] = { " 1", " 2", " 3", " 4" };
    char buf[250];
    bool pass = true;
    bool tmpFix;

    int encStart = (qlaNum == 2) ? 4 : 0;
    uint8_t enc_mask = (qlaNum == 2) ? 0xf0 : 0x0f;

    logFile << std::endl << "=== ENCODER INPUTS ===" << std::endl;
    Amp1394Console::Print(curLine++, 9, "This test uses the DOUT signals to"
                           " generate a known number of quadrature encoder signals");
    Board.WriteDigitalOutput(enc_mask, 0x33);
    // First, test setting of encoder preload
    logFile << "   Preload to 0x000000: ";
    for (i = 0; i < 4; i++)
        Board.WriteEncoderPreload(encStart+i, 0);
    Amp1394_Sleep(0.0001);
    Port->ReadAllBoards();
    for (i = 0; i < 4; i++) {
        darray[i] = Board.GetEncoderPosition(encStart+i);
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
    Amp1394Console::Print(curLine++, 9, buf);
    Amp1394Console::Refresh();
    if (pass) {
        logFile << "   Increment to 0x000100:" << std::endl;
        Amp1394Console::Print(curLine, 9, "Testing encoder increment to 0x000100 -");
        tmpFix = false;
        for (j = 0; j < 4; j++)
            testValue[j] = 0;
        for (i = 0; (i < (0x100/4)) && pass; i++) {
            EncUp(Board, qlaNum);
            Amp1394_Sleep(0.0001);
            logFile << "      ";
            Port->ReadAllBoards();
            for (j = 0; j < 4; j++) {
                darray[j] = Board.GetEncoderPosition(encStart+j);
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
        Amp1394Console::Print(curLine++, 49, buf);
        Amp1394Console::Refresh();
    }
    bool tmp_pass = true;
    logFile << "   Preload to 0x000100: ";
    for (i = 0; i < 4; i++)
        Board.WriteEncoderPreload(encStart+i, 0x000100);
    Amp1394_Sleep(0.0001);
    Port->ReadAllBoards();
    for (i = 0; i < 4; i++) {
        darray[i] = Board.GetEncoderPosition(encStart+i);
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
    Amp1394Console::Print(curLine++, 9, buf);
    Amp1394Console::Refresh();

    if (tmp_pass) {
        logFile << "   Decrement to 0x000000:" << std::endl;
        Amp1394Console::Print(curLine, 9, "Testing encoder decrement to 0x000000 -");
        tmpFix = false;
        for (j = 0; j < 4; j++)
            testValue[j] = 0;
        for (i = (0x100/4)-1; (i >= 0) && tmp_pass; i--) {
            EncDown(Board, qlaNum);
            Amp1394_Sleep(0.0001);
            logFile << "      ";
            Port->ReadAllBoards();
            for (j = 0; j < 4; j++) {
                darray[j] = Board.GetEncoderPosition(encStart+j);
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
        Amp1394Console::Print(curLine++, 49, buf);
        Amp1394Console::Refresh();
    }

    Amp1394Console::Print(curLine, 9, "Testing direct encoder input (A,B,I)");
    bool directPass = true;

    // Test A channel directly
    uint8_t chanA;
    uint8_t dout_mask = (qlaNum == 2) ? 0x10 : 0x01;
    // Channel A low
    Board.WriteDigitalOutput(dout_mask, 0x00);
    Amp1394_Sleep(0.0001);
    Port->ReadAllBoards();
    chanA = Board.GetEncoderChannelA();
    if ((chanA&enc_mask) != 0x00) {
        logFile << "   Setting channel A = 0, FAIL: " << std::hex << (int)chanA << std::endl;
        directPass = false;
        pass = false;
    }
    // Channel A high
    Board.WriteDigitalOutput(dout_mask, dout_mask);
    Amp1394_Sleep(0.0001);
    Port->ReadAllBoards();
    chanA = Board.GetEncoderChannelA();
    if ((chanA&enc_mask) != enc_mask) {
        logFile << "   Setting channel A = 1, FAIL: " << std::hex << (int)chanA << std::endl;
        directPass = false;
        pass = false;
    }

    // Test B channel directly
    uint8_t chanB;
    dout_mask = (qlaNum == 2) ? 0x20 : 0x02;
    // Channel B low
    Board.WriteDigitalOutput(dout_mask, 0x00);
    Amp1394_Sleep(0.0001);
    Port->ReadAllBoards();
    chanB = Board.GetEncoderChannelB();
    if ((chanB&enc_mask) != 0x00) {
        logFile << "   Setting channel B = 0, FAIL: " << std::hex << (int)chanB << std::endl;
        directPass = false;
        pass = false;
    }
    // Channel B high
    Board.WriteDigitalOutput(dout_mask, dout_mask);
    Amp1394_Sleep(0.0001);
    Port->ReadAllBoards();
    chanB = Board.GetEncoderChannelB();
    if ((chanB&enc_mask) != enc_mask) {
        logFile << "   Setting channel B = 1, FAIL: " << std::hex << (int)chanB << std::endl;
        directPass = false;
        pass = false;
    }

    // Test index
    // Index low
    uint8_t index;
    dout_mask = (qlaNum == 2) ? 0x40 : 0x04;
    Board.WriteDigitalOutput(dout_mask, 0x00);
    Amp1394_Sleep(0.0001);
    Port->ReadAllBoards();
    index = Board.GetEncoderIndex();
    if ((index&enc_mask) != 0x00) {
        logFile << "   Setting index = 0, FAIL: " << std::hex << (int)index << std::endl;
        directPass = false;
        pass = false;
    }
    // Index high
    Board.WriteDigitalOutput(dout_mask, dout_mask);
    Amp1394_Sleep(0.0001);
    Port->ReadAllBoards();
    index = Board.GetEncoderIndex();
    if ((index&enc_mask) != enc_mask) {
        logFile << "   Setting index = 1, FAIL: " << std::hex << (int)index << std::endl;
        directPass = false;
        pass = false;
    }

    if (directPass)
        Amp1394Console::Print(curLine++, 49, "- PASS");
    else
        Amp1394Console::Print(curLine++, 49, "- FAIL");

    if (pass)
        logFile << "   Overall result: PASS" << std::endl;
    else
        logFile << "   Overall result: FAIL" << std::endl;
    return pass;
}

bool TestAnalogInputs(int curLine, AmpIO &Board, BasePort *Port, std::ofstream &logFile, unsigned int qlaNum)
{
    int i, j;
    char buf[80];
    double measured;
    bool pass = true;

    int potStart = (qlaNum == 2) ? 4 : 0;

    logFile << std::endl << "=== Analog Inputs ===" << std::endl;
    Amp1394Console::Print(curLine++, 9, "This test uses the pot on the test board to set a voltage (+/-0.25V)");
    bool tmp_pass = false;
    while (!tmp_pass) {
        Amp1394Console::Print(curLine, 9, "Enter pot voltage: ");
        Amp1394Console::GetString(buf, sizeof(buf));
        if (sscanf(buf, "%lf", &measured) == 1) tmp_pass = true;
    }
    logFile << "   Input voltage: " << measured << std::endl;
    curLine++;
    // Average 16 readings to reduce noise
    unsigned long pot[4] = { 0, 0, 0, 0};
    double potV[4];
    int numValid = 0;
    for (i = 0; i < 16; i++) {
        if (Port->ReadAllBoards()) {
            numValid++;
            for (j = 0; j < 4; j++)
                pot[j] += Board.GetAnalogInput(potStart+j);
        }
    }
    if (numValid > 0) {
        logFile << "   Readings: ";
        for (j = 0; j < 4; j++) {
            pot[j] /= numValid;
            // Convert to voltage (16-bit converter, Vref = 4.5V)
            potV[j] = pot[j]*4.5/65535;
            logFile << potV[j] << " ";
            if (fabs(potV[j] - measured) > 0.25)
                pass = false;
        }
    }
    else {
        logFile << "   No valid readings!" << std::endl;
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

    Amp1394Console::Print(curLine, 9, buf);
    return pass;
}

bool TestMotorPowerControl(int curLine, AmpIO &Board, BasePort *Port, std::ofstream &logFile, unsigned int qlaNum)
{
    char buf[100];
    unsigned long status;
    bool pass = true;
    bool ignoreMV = false;     // ignore "motor voltage good" feedback
    unsigned long mask;

    // Disable watchdog
    Board.WriteWatchdogPeriod(0x0000);

    logFile << std::endl << "=== Motor Power Control ===" << std::endl;
 retry:
    // Enabling motor power supply
    Board.WritePowerEnable(true);
    Amp1394Console::Print(curLine, 9, "Enable motor power -");
    Amp1394Console::Refresh();
    Amp1394_Sleep(1.0);
    Port->ReadAllBoards();
    status = Board.GetStatus();
    logFile << "   Enable motor power: " << std::hex << status;
    // Note: QLA implementation also verifies that amp enable is off
    uint32_t status_mask = (qlaNum == 1) ? 0x0004400 : (qlaNum == 2) ? 0x00048000 : 0x000c0f0f;
    uint32_t status_goal = (qlaNum == 1) ? 0x0004400 : (qlaNum == 2) ? 0x00048000 : 0x000c0000;
    if ((status & status_mask) != status_goal) {
        sprintf(buf, "FAIL (%08lx) - is motor power connected?", status);
        Amp1394Console::Print(curLine++, 30, buf);
        Amp1394Console::Print(curLine, 9, "Do you want to retry (y/n)? ");
        Amp1394Console::GetString(buf, sizeof(buf));
        if ((buf[0] == 'y') || (buf[0] == 'Y')) {
            Amp1394Console::Print(curLine, 9, "                               ");
            curLine -= 1;
            Amp1394Console::Print(curLine, 30, "                                           ");
            goto retry;
        }
        logFile << " - FAIL" << std::endl;
        pass = false;
    }
    else {
        logFile << " - PASS" << std::endl;
        sprintf(buf, "PASS (%08lx)", status);
    }
    Amp1394Console::Print(curLine++, 30, buf);

    uint32_t fver = Board.GetFirmwareVersion();

    status_mask = (qlaNum == 1) ? 0x00004000 : (qlaNum == 2) ? 0x00008000 : 0x0004ffff;
    status_goal = (qlaNum == 1) ? 0x00004000 : (qlaNum == 2) ? 0x00008000 : 0x00040000;
    if (!pass && (status & status_mask) == status_goal) {
        logFile << "   Continuing test, ignoring 'motor voltage good' feedback" << std::endl;
        Amp1394Console::Print(curLine++, 9, "Continuing test, ignoring 'motor voltage good' feedback");
        ignoreMV = true;
    }
    Amp1394Console::Refresh();

    unsigned int i;
    unsigned int motorStart = (qlaNum == 2) ? 4 : 0;

    // Enabling individual amplifiers
    if (fver < 8) {
        Board.WriteAmpEnable(0x0f, 0x0f);
    }
    else {
        for (i = 0; i < 4; i++)
            Board.WriteAmpEnableAxis(motorStart+i, true);
    }
    Amp1394_Sleep(0.006);       // QLA Rev 1.5+ may have delay up to 5.3 ms
    Port->ReadAllBoards();
    if (fver < 8) {
        status = Board.GetStatus();
        logFile << "   Amplifier enable: " << std::hex << status;
        mask = ignoreMV ? 0x00040f0f : 0x000c0f0f;
        if ((status&mask) != (0x000c0f0f&mask)) {
            logFile << " - FAIL" << std::endl;
            sprintf(buf, "Enable power amplifiers - FAIL (%08lx)", status);
            pass = false;
        }
        else {
            logFile << " - PASS" << std::endl;
            sprintf(buf, "Enable power amplifiers - PASS (%08lx)", status);
        }
    }
    else {
        logFile << "   Amplifier enable: ";
        status = 0;
        for (i = 0; i < 4; i++) {
            bool motorEn = Board.GetAmpEnable(motorStart+i);
            if (motorEn) {
                logFile << "1 ";
                status |= (1 << i);
            }
            else {
                logFile << "0 ";
                pass = false;
            }
        }
        if (status != 0x000f) {
            logFile << " - FAIL" << std::endl;
            sprintf(buf, "Enable power amplifiers - FAIL (%04lx)", status);
        }
        else {
            logFile << " - PASS" << std::endl;
            sprintf(buf, "Enable power amplifiers - PASS (%04lx)", status);
        }
    }
    Amp1394Console::Print(curLine++, 9, buf);

    Amp1394Console::Refresh();
    Amp1394_Sleep(1.0);    // wait 1 second (LEDs should be ON)

    // Turning amplifiers off
    if (fver < 8) {
        Board.WriteAmpEnable(0x0f, 0);
    }
    else {
        for (i = 0; i < 4; i++)
            Board.WriteAmpEnableAxis(motorStart+i, false);
    }
    Amp1394_Sleep(0.001);
    Port->ReadAllBoards();
    if (fver < 8) {
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
    }
    else {
        logFile << "   Amplifier disable: ";
        status = 0;
        for (i = 0; i < 4; i++) {
            bool motorEn = Board.GetAmpEnable(motorStart+i);
            if (motorEn) {
                logFile << "1 ";
                status |= (1 << i);
                pass = false;
            }
            else {
                logFile << "0 ";
            }
        }
        if (status != 0) {
            logFile << " - FAIL" << std::endl;
            sprintf(buf, "Disable power amplifiers - FAIL (%04lx)", status);
        }
        else {
            logFile << " - PASS" << std::endl;
            sprintf(buf, "Disable power amplifiers - PASS (%04lx)", status);
        }
    }
    Amp1394Console::Print(curLine++, 9, buf);

    Board.WritePowerEnable(false);
    Amp1394Console::Print(curLine, 9, "Disable motor power -");
    Amp1394Console::Refresh();
    Amp1394_Sleep(1.0);
    Port->ReadAllBoards();
    status = Board.GetStatus();
    logFile << "   Disable motor power: " << std::hex << status;
    status_mask = (qlaNum == 1) ? 0x00044000 : (qlaNum == 2) ? 0x00048000 : 0x000c0f0f;
    if ((status&status_mask) != 0) {
        logFile << " - FAIL" << std::endl;
        sprintf(buf, "FAIL (%08lx)", status);
        pass = false;
    }
    else {
        logFile << " - PASS" << std::endl;
        sprintf(buf, "PASS (%08lx)", status);
    }
    Amp1394Console::Print(curLine++, 31, buf);

    if (qlaNum == 0) {
        // Turn on safety relay
        Board.WriteSafetyRelay(true);
        Amp1394_Sleep(0.01);   // 10 ms
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
        Amp1394Console::Print(curLine++, 9, buf);

        Amp1394Console::Refresh();
        Amp1394_Sleep(1.0);

        // Turn off safety relay
        Board.WriteSafetyRelay(false);
        Amp1394_Sleep(0.001);
        Port->ReadAllBoards();
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
        Amp1394Console::Print(curLine++, 9, buf);
    }
    else {
        logFile << "   Skipping safety relay tests for DQLA" << std::endl;
    }

    // Reenable watchdog
    Board.WriteWatchdogPeriod(0xFFFF);

    return pass;
}

bool TestPowerAmplifier(int curLine, AmpIO &Board, BasePort *Port, std::ofstream &logFile, unsigned int qlaNum)
{
    char buf[100];
    unsigned long status;
    unsigned long dac;
    unsigned int i;
    bool pass = true;

    // Disable watchdog
    Board.WriteWatchdogPeriod(0x0000);

    uint32_t fver = Board.GetFirmwareVersion();

    logFile << std::endl << "=== Power Amplifier Test ===" << std::endl;
    Amp1394Console::Print(curLine, 9, "Temperature sensors - ");
    Amp1394Console::Refresh();

    Board.WritePowerEnable(true);

    unsigned int motorStart = (qlaNum == 2) ? 4 : 0;

    // Enabling individual amplifiers
    if (fver < 8) {
        Board.WriteAmpEnable(0x0f, 0x0f);
    }
    else {
        for (i = 0; i < 4; i++)
            Board.WriteAmpEnableAxis(motorStart+i, true);
    }
    dac = 0x8000;
    for (i = 0; i < 4; i++)
        Board.SetMotorCurrent(motorStart+i, dac);
    Port->WriteAllBoards();
    Amp1394_Sleep(1.0);  // wait for power to stabilize
    Port->ReadAllBoards();
    if (fver < 8) {
        status = Board.GetStatus();
        if ((status&0x000c0f0f) != 0x000c0f0f) {
            sprintf(buf,"Failed to enable power (%08lx) - is motor power connected?", status);
            Amp1394Console::Print(curLine++, 9, buf);
        }
    }
    else {
        for (i = 0; i < 4; i++) {
            bool motorEn = Board.GetAmpEnable(motorStart+i);
            if (!motorEn) {
                sprintf(buf,"Failed to enable power - is motor power connected?");
                Amp1394Console::Print(curLine++, 9, buf);
                break;
            }
        }
    }
    Amp1394Console::Print(curLine, 9, "Temperature sensors - ");
    // Read temperature in Celsius
    unsigned int tempStart = (qlaNum == 2) ? 2 : 0;
    unsigned short temp1 = Board.GetAmpTemperature(tempStart)/2;
    unsigned short temp2 = Board.GetAmpTemperature(tempStart+1)/2;
    logFile << "   Motor temperatures: " << std::dec << temp1 << ", " << temp2 << " degC (expected range is 20-40) ";
    if ((temp1 < 20) || (temp1 > 40) || (temp2 < 20) || (temp2 > 40)) {
        logFile << "- FAIL" << std::endl;
        sprintf(buf, "FAIL (%d, %d degC)", temp1, temp2);
    }
    else {
        logFile << "- PASS" << std::endl;
        sprintf(buf, "PASS (%d, %d degC)", temp1, temp2);
    }
    Amp1394Console::Print(curLine++, 31, buf);

    Amp1394Console::Print(curLine, 9, "DAC:");
    Amp1394Console::Print(curLine+1, 9, "ADC:");
    Amp1394Console::Print(curLine+3, 9, "Temp:");
    Amp1394Console::Refresh();

    // Increment of 0x0200 is about 100 mA; 0x9000 is approx. 780 mA.
    unsigned long cur[4];
    while (1) {
        sprintf(buf, "%04lx   %04lx   %04lx   %04lx", dac, dac, dac, dac);
        Amp1394Console::Print(curLine, 15, buf);
        logFile << "   Commanded current: " << std::hex << dac << ", Measured currents: ";
        for (i = 0; i < 4; i++) {
            cur[i] = Board.GetMotorCurrent(motorStart+i);
            logFile << std::hex << cur[i];
            if ((cur[i] > (dac + 0x0100))
                || ((dac > 0x0100)
                    && (cur[i] < (dac - 0x0100))
                    )
                ) {
                logFile << "* ";
                Amp1394Console::Print(curLine+2, 15+7*i, "FAIL");
                pass = false;
            }
            else {
                logFile << "  ";
                Amp1394Console::Print(curLine+2, 15+7*i, "PASS");
            }
        }
        logFile << std::endl;
        sprintf(buf, "%04lx   %04lx   %04lx   %04lx", cur[0], cur[1], cur[2], cur[3]);
        Amp1394Console::Print(curLine+1, 15, buf);

        unsigned short newTemp1 = Board.GetAmpTemperature(tempStart)/2;
        unsigned short newTemp2 = Board.GetAmpTemperature(tempStart+1)/2;
        logFile << "   Motor temperatures: " << std::dec  << newTemp1 << ", " << newTemp2 << " degC (expected to be increasing)";
        if ((newTemp1 >= temp1) && (newTemp2 >= temp2))
            logFile << " - PASS" << std::endl;
        else
            logFile << " - fail" << std::endl;
        sprintf(buf, "%4d (%s)    %4d (%s)", newTemp1, ((newTemp1 >= temp1) ? "PASS" : "FAIL"),
                newTemp2, ((newTemp2 >= temp2) ? "PASS" : "FAIL"));
        Amp1394Console::Print(curLine+3, 15, buf);
        temp1 = newTemp1;
        temp2 = newTemp2;
        Amp1394Console::Refresh();

        if (dac > 0x8000) dac = 0x8000 - (dac - 0x8000);
        else dac = 0x8000 + (0x8000 - dac) + 0x0200;

        if (dac > 0x8600) break;

        for (i = 0; i < 4; i++)
            Board.SetMotorCurrent(motorStart+i, dac);
        Port->WriteAllBoards();
        Amp1394_Sleep(1.0);
        Port->ReadAllBoards();
    }

    for (i = 0; i < 4; i++)
        Board.SetMotorCurrent(motorStart+i, 0x8000);
    Port->WriteAllBoards();
    if (fver < 8) {
        Board.WriteAmpEnable(0x0f, 0);
    }
    else {
        for (i = 0; i < 4; i++)
            Board.WriteAmpEnableAxis(motorStart+i, false);
    }
    Board.WritePowerEnable(false);

    // Reenable watchdog
    Board.WriteWatchdogPeriod(0xFFFF);
    if (pass)
        logFile << "   Overall result: PASS" << std::endl;
    else
        logFile << "   Overall result: FAIL" << std::endl;
    return pass;
}

bool TestEthernetV2(int curLine, AmpIO &Board, BasePort *Port, std::ofstream &logFile)
{
    logFile << std::endl << "=== Ethernet (KSZ8851) Test ===" << std::endl;
    if (Board.GetFirmwareVersion() < 5) {
        logFile << "   No Ethernet controller, firmware version = " << Board.GetFirmwareVersion() << std::endl;
        return false;
    }
    uint16_t status = Board.ReadKSZ8851Status();
    if (!(status&0x8000)) {
        logFile << "   No Ethernet controller, status = " << std::hex << status << std::endl;
        return false;
    }
    logFile << "   Ethernet controller status = " << std::hex << status << std::endl;
    // Reset the board
    Board.ResetKSZ8851();
    // Wait 100 msec
    Amp1394_Sleep(0.1);
    // Read the status
    status = Board.ReadKSZ8851Status();
    logFile << "   After reset, status = " << std::hex << status << std::endl;
    // Read the Chip ID (16-bit read)
    uint16_t chipID = Board.ReadKSZ8851ChipID();
    logFile << "   Chip ID = " << std::hex << chipID << std::endl;
    if ((chipID&0xfff0) != 0x8870)
        return false;
#if 0
    // Read Chip ID using 8-bit read
    // TODO: Reading from 0xC0 works (get 0x72), but not from 0xC1 (get 0xC1 instead of 0x88)
    uint8_t chipIDHigh;
    Board.ReadKSZ8851Reg(0xC1, chipIDHigh);
    logFile << "   Chip ID high (8-bit read) = " << std::hex << static_cast<int>(chipIDHigh) << std::endl;
    if (chipIDHigh != 0x88)
        return false;
#endif
    // Walking bit pattern on MAC address registers (R/W)
    // 0x10 is MARL (MAC Address Register Low)
    // 0x12 is MARM (MAC Address Register Middle)
    uint16_t MacAddrOutLow, MacAddrOutMid, MacAddrIn;
    bool ret = true;
    for (MacAddrOutLow = 0x0001, MacAddrOutMid = 0x8000;
         (MacAddrOutLow != 0) && (MacAddrOutMid != 0);
         MacAddrOutLow <<= 1, MacAddrOutMid >>= 1) {
        logFile << "   MAC address: " << std::hex;
        if (Board.WriteKSZ8851Reg(0x10, MacAddrOutLow))
            logFile << "write low = " << MacAddrOutLow;
        if (Board.WriteKSZ8851Reg(0x12, MacAddrOutMid))
            logFile << ", write mid = " << MacAddrOutMid;
        MacAddrIn = 0;
        bool failed = false;
        if (Board.ReadKSZ8851Reg(0x10, MacAddrIn))
            logFile << ", read low = " << MacAddrIn;
        if (MacAddrOutLow != MacAddrIn) failed = true;
        if (Board.ReadKSZ8851Reg(0x12, MacAddrIn))
            logFile << ", read mid = " << MacAddrIn;
        if (MacAddrOutMid != MacAddrIn) failed = true;
        if (failed) {
            logFile << " ***";
            ret = false;
        }
        logFile << std::endl;
    }
    // Reset the board again (to restore MAC address)
    Board.ResetKSZ8851();
    return ret;
}

bool TestEthernetV3(int curLine, AmpIO &Board, BasePort *Port, std::ofstream &logFile)
{
    logFile << std::endl << "=== Ethernet (RTL8211F) Test ===" << std::endl;
    bool ret = true;
    for (unsigned int i = 1; i < 2; i++) {
        // TODO: Figure out why PHY2 has a default address of 0
        unsigned int phyAddr = (i == 2) ? FpgaIO::PHY_BROADCAST : FpgaIO::PHY_RTL8211F;
        uint16_t phyid1 = 0, phyid2 = 0;
        if (!Board.ReadRTL8211F_Register(i, phyAddr, FpgaIO::RTL8211F_PHYID1, phyid1))
            logFile << "Failed to read PHY" << i << " PHYID1" << std::endl;
        if (!Board.ReadRTL8211F_Register(i, phyAddr, FpgaIO::RTL8211F_PHYID2, phyid2))
            logFile << "Failed to read PHY" << i << " PHYID2" << std::endl;
        logFile << "PHY" << i << " PHYID = " << std::hex
                << phyid1 << ", " << phyid2;
        if ((phyid1 == 0x001c) && (phyid2 == 0xc916)) {
            logFile << " - PASS" << std::dec << std::endl;
        }
        else {
            logFile << " - FAIL (should be 1c, c916)" << std::dec << std::endl;
            ret = false;
        }
    }
    return ret;
}

void PrintDebugStream(std::stringstream &debugStream)
{
    std::cerr << debugStream.str() << std::endl;
    debugStream.clear();
    debugStream.str("");
}

int main(int argc, char** argv)
{
    int i;
#if Amp1394_HAS_RAW1394
    BasePort::PortType desiredPort = BasePort::PORT_FIREWIRE;
#else
    BasePort::PortType desiredPort = BasePort::PORT_ETH_UDP;
#endif
    int port = 0;
    int board = 0;
    unsigned int qlaNum = 0;    // QLA number (1 or 2 for DQLA)
    bool requireQLA_SN = true;
    std::string IPaddr(ETH_UDP_DEFAULT_IP);

    if (argc > 1) {
        int args_found = 0;
        for (i = 1; i < argc; i++) {
            if (argv[i][0] == '-') {
                if (argv[i][1] == 'p') {
                    if (!BasePort::ParseOptions(argv[i]+2, desiredPort, port, IPaddr)) {
                        std::cerr << "Failed to parse option: " << argv[i] << std::endl;
                        return 0;
                    }
                    std::cerr << "Selected port: " << BasePort::PortTypeString(desiredPort) << std::endl;
                }
                else if (argv[i][1] == 'q') {
                    qlaNum = argv[i][2]-'0';
                }
                else if (argv[i][1] == 'k')  {
                    requireQLA_SN = false;
                }
                else {
                    std::cerr << "Usage: qlatest [<board-num>] [-pP] [-qN] [-k]" << std::endl
                    << "       where <board-num> = rotary switch setting (0-15, default 0)" << std::endl
                    << "             P = port number (default 0)" << std::endl
                    << "                 can also specify -pfwP, -pethP or -pudp" << std::endl
                    << "             N = QLA number (1 or 2) on DQLA" << std::endl
                    << "            -k option means to continue test even if QLA serial number not found" << std::endl;
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

    bool isDQLA = (Port->GetHardwareVersion(board) == DQLA_String);
    if (isDQLA) {
        if (!((qlaNum == 1) || (qlaNum == 2))) {
            std::cerr << "Must specify -q1 or -q2 to select QLA number for DQLA" << std::endl;
            delete Port;
            return 0;
        }
    }
    else if (qlaNum != 0) {
        // If not DQLA, force qlaNum to 0
        qlaNum = 0;
    }

    AmpIO Board(board);
    Port->AddBoard(&Board);

    std::string logFilename("QLA_");
    std::string QLA_SN = Board.GetQLASerialNumber(qlaNum);
    if (QLA_SN.empty()) {
        if (requireQLA_SN) {
            std::cerr << "Failed to get QLA serial number (specify -k command line option to continue test)" << std::endl;
            Port->RemoveBoard(board);
            delete Port;
            return 0;
        }
        QLA_SN.assign("Unknown");
    }
    logFilename.append(QLA_SN);
    logFilename.append(".log");
    std::ofstream logFile(logFilename.c_str());
    if (!logFile.good()) {
        std::cerr << "Failed to open log file " << logFilename << std::endl;
        Port->RemoveBoard(board);
        delete Port;
        return 0;
    }
    logFile << "====== TEST REPORT ======" << std::endl << std::endl;
    logFile << "QLA S/N: " << QLA_SN;
    if (qlaNum != 0)
        logFile << " (QLA " << qlaNum << " of DQLA)";
    logFile << std::endl;

    unsigned int fpgaVer = Board.GetFPGAVersionMajor();
    if (fpgaVer == 3) {
        logFile << "FPGA V3" << std::endl;
    }
    else {
        std::string FPGA_SN = Board.GetFPGASerialNumber();
        if (FPGA_SN.empty())
            FPGA_SN.assign("Unknown");
        logFile << "FPGA S/N: " << FPGA_SN << std::endl;
    }

    logFile << "FPGA Firmware Version: " << Board.GetFirmwareVersion() << std::endl;

    Amp1394Console console(Amp1394Console::FLAG_ECHO|Amp1394Console::FLAG_BLOCKING);
    console.Init();
    if (!console.IsOK()) {
        std::cerr << "Failed to initialize console" << std::endl;
        Port->RemoveBoard(board);
        delete Port;
        return -1;
    }

    console.Print(1, 9, "QLA Test for Board %d, S/N %s", board, QLA_SN.c_str());

    console.Print(3, 9, "0) Quit");
    console.Print(4, 9, "1) Test digital input:");
    console.Print(5, 9, "2) Test encoder feedback:");
    console.Print(6, 9, "3) Test analog feedback:");
    console.Print(7, 9, "4) Test motor power control:");
    console.Print(8, 9, "5) Test power amplifier:");   // includes current feedback & temp sense
    if ((desiredPort == BasePort::PORT_FIREWIRE) && (fpgaVer > 1))
        console.Print(9, 9, "6) Test Ethernet controller:");

    console.Refresh();

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
                console.Print(i, 9, line);
            while (!debugStream.eof()) {
                std::string stringLine;
                std::getline(debugStream, stringLine);
                console.Print(cur_line++, 9, stringLine.c_str());
            }
            debugStream.clear();
            debugStream.str("");
            last_debug_line = cur_line;
        }

        console.Print(10, 10, "Select option: ");
        int c = console.GetChar();

        switch (c) {
            case '0':   // Quit
                done = true;
                break;
            case '1':   // Test digital input
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestDigitalInputs(TEST_START_LINE, Board, Port, logFile, qlaNum))
                    console.Print(4, 46, "PASS");
                else
                    console.Print(4, 46, "FAIL");
                break;

            case '2':    // Test encoder feedback
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestEncoders(TEST_START_LINE, Board, Port, logFile, qlaNum))
                    console.Print(5, 46, "PASS");
                else
                    console.Print(5, 46, "FAIL");
                break;

            case '3':    // Test analog feedback
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestAnalogInputs(TEST_START_LINE, Board, Port, logFile, qlaNum))
                    console.Print(6, 46, "PASS");
                else
                    console.Print(6, 46, "FAIL");
                break;

            case '4':
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestMotorPowerControl(TEST_START_LINE, Board, Port, logFile, qlaNum))
                    console.Print(7, 46, "PASS");
                else
                    console.Print(7, 46, "FAIL");
                break;

            case '5':
                ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                if (TestPowerAmplifier(TEST_START_LINE, Board, Port, logFile, qlaNum))
                    console.Print(8, 46, "PASS");
                else
                    console.Print(8, 46, "FAIL");
                break;

            case '6':
                if ((desiredPort == BasePort::PORT_FIREWIRE) && (fpgaVer > 1)) {
                    ClearLines(TEST_START_LINE, DEBUG_START_LINE);
                    if ((fpgaVer == 2) && TestEthernetV2(TEST_START_LINE, Board, Port, logFile))
                        console.Print(9, 46, "PASS");
                    else if ((fpgaVer == 3) && TestEthernetV3(TEST_START_LINE, Board, Port, logFile))
                        console.Print(9, 46, "PASS");
                    else
                        console.Print(9, 46, "FAIL");
                }
                break;

        }

        console.Refresh();
    }

    Port->RemoveBoard(board);
    console.End();
    delete Port;
}
