/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen, Anton Deguet

  (C) Copyright 2012-2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/******************************************************************************
 *
 * This program continuously displays SUJ-Si potentiometer and presence feedback
 * from the selected dRAC board. It relies on the Amp1394 library (which depends
 * on libraw1394 and/or pcap) and on the Amp1394Console library (which may depend
 * on curses).
 *
 * Usage: sujsi [-pP] [-hH] [-b<r|w>] [-r] <board num> [<board num>]
 *        where P is the port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *        -br or -bw specify to use a broadcast protocol
 *        -r specifies read-only mode
 *
 ******************************************************************************/

#include <stdlib.h>
#include <array>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "Amp1394Console.h"

const unsigned int NUM_SISUJ_POTS = 10;
const nodeaddr_t SISUJ_Z_STATUS_ADDR = 0xb030;
const nodeaddr_t SISUJ_ESSJ_ADC0_ADDR = 0xa039;
const nodeaddr_t SISUJ_ESSJ_STATUS_ADDR = 0xa03c;
const uint32_t SISUJ_POT_MASK = 0x00000fff;
const uint32_t SISUJ_DSIB_SI_PRESENT = 0x00001000;
const uint32_t SISUJ_DSIB_Z_SI_PRESENT = 0x00002000;
const uint32_t SISUJ_ESSJ_PRESENT = 0x00000001;
const uint32_t SISUJ_ESSJ_ADC_VALID = 0x00000002;
const uint32_t SISUJ_ESSJ_ESPM_PRESENT = 0x00000004;

class SiSUJData {
public:
    SiSUJData()
    {
        positions.fill(-1);
    }

    bool registerReadOk = false;
    bool dsib_z_si_present = false;
    bool dsib_si_present = false;
    unsigned int suj_z_id = 0;
    unsigned int essj_fw_version = 0;
    bool espm_present = false;
    bool essj_adc_valid = false;
    bool essj_present = false;
    std::array<int16_t, NUM_SISUJ_POTS> positions;
};

void PrintDebugStream(std::stringstream &debugStream)
{
    std::cerr << debugStream.str() << std::endl;
    debugStream.clear();
    debugStream.str("");
}

void PrintUsage(const char *programName)
{
    std::cerr << "Usage: " << programName << " <board-num> [<board-num>] [-pP] [-hH] [-b<r|w>] [-r]" << std::endl
              << "       where P = port number (default 0)" << std::endl
              << "                 can also specify -pfw[:P], -peth:P or -pudp[:xx.xx.xx.xx]" << std::endl
              << "             H = additional supported hardware versions" << std::endl
              << "            -br enables broadcast read/write" << std::endl
              << "            -bw enables broadcast write" << std::endl
              << "            -r  read-only (disables power/brake writes)" << std::endl;
}

static int16_t UnpackSiSUJAdc(const std::array<uint32_t, 3> &adcData, unsigned int sampleIndex)
{
    const unsigned int bitIndex = 12 * sampleIndex;
    const unsigned int quadIndex = bitIndex / 32;
    const unsigned int shift = bitIndex % 32;
    uint32_t value = adcData[quadIndex] >> shift;
    if ((shift > 20) && (quadIndex < 2)) {
        value |= adcData[quadIndex + 1] << (32 - shift);
    }
    return static_cast<int16_t>(value & SISUJ_POT_MASK);
}

static std::string FormatPot(int16_t value)
{
    if (value < 0) {
        return "----";
    }
    std::stringstream ss;
    ss << value;
    return ss.str();
}

static const char *BoolString(bool value)
{
    return value ? "1" : "0";
}

bool ReadSiSUJData(BasePort *port, AmpIO *board, SiSUJData &data)
{
    data = SiSUJData();
    if (!port || !board) {
        return false;
    }

    uint32_t sujZStatus = 0;
    uint32_t essjStatus = 0;
    std::array<uint32_t, 3> adcData = {{0, 0, 0}};

    bool success = port->ReadQuadlet(board->GetBoardId(), SISUJ_Z_STATUS_ADDR, sujZStatus);
    for (unsigned int i = 0; i < adcData.size(); i++) {
        success &= port->ReadQuadlet(board->GetBoardId(), SISUJ_ESSJ_ADC0_ADDR + i, adcData[i]);
    }
    success &= port->ReadQuadlet(board->GetBoardId(), SISUJ_ESSJ_STATUS_ADDR, essjStatus);
    data.registerReadOk = success;
    if (!success) {
        return false;
    }

    data.dsib_si_present = (sujZStatus & SISUJ_DSIB_SI_PRESENT) != 0;
    data.dsib_z_si_present = data.dsib_si_present && ((sujZStatus & SISUJ_DSIB_Z_SI_PRESENT) != 0);
    data.suj_z_id = (sujZStatus >> 28) & 0x0f;
    if (data.dsib_z_si_present) {
        data.positions[0] = static_cast<int16_t>(sujZStatus & SISUJ_POT_MASK);
        data.positions[1] = static_cast<int16_t>((sujZStatus >> 16) & SISUJ_POT_MASK);
    }

    data.essj_present = (essjStatus & SISUJ_ESSJ_PRESENT) != 0;
    data.essj_adc_valid = (essjStatus & SISUJ_ESSJ_ADC_VALID) != 0;
    data.espm_present = (essjStatus & SISUJ_ESSJ_ESPM_PRESENT) != 0;
    data.essj_fw_version = (essjStatus >> 16) & 0xffff;
    if (data.essj_present && data.essj_adc_valid) {
        for (unsigned int i = 0; i < 4; i++) {
            data.positions[2 + 2*i] = UnpackSiSUJAdc(adcData, i);
            data.positions[3 + 2*i] = UnpackSiSUJAdc(adcData, i + 4);
        }
    }

    return true;
}

void PrintBoardData(int startLine, unsigned int leftMargin, AmpIO *board, const SiSUJData &data)
{
    const uint8_t digOut = board->GetDigitalOutput();
    const bool sujBrakeReleased = (digOut & 0x01) != 0;
    const bool powerEnabled = board->GetPowerEnable();

    Amp1394Console::Print(startLine, leftMargin, "Board %d  HW: %-4s  FW: %-8s  Read: %s",
                          board->GetBoardId(),
                          board->GetHardwareVersionString().c_str(),
                          std::to_string(board->GetFirmwareVersion()).c_str(),
                          data.registerReadOk ? "ok " : "err");

    Amp1394Console::Print(startLine + 2, leftMargin, "Joint      pot1   pot2");
    Amp1394Console::Print(startLine + 3, leftMargin, "Z       %6s %6s",
                          FormatPot(data.positions[0]).c_str(),
                          FormatPot(data.positions[1]).c_str());
    Amp1394Console::Print(startLine + 4, leftMargin, "rot1    %6s %6s",
                          FormatPot(data.positions[2]).c_str(),
                          FormatPot(data.positions[3]).c_str());
    Amp1394Console::Print(startLine + 5, leftMargin, "rot2    %6s %6s",
                          FormatPot(data.positions[4]).c_str(),
                          FormatPot(data.positions[5]).c_str());
    Amp1394Console::Print(startLine + 6, leftMargin, "rot3    %6s %6s",
                          FormatPot(data.positions[6]).c_str(),
                          FormatPot(data.positions[7]).c_str());
    Amp1394Console::Print(startLine + 7, leftMargin, "rot4    %6s %6s",
                          FormatPot(data.positions[8]).c_str(),
                          FormatPot(data.positions[9]).c_str());

    Amp1394Console::Print(startLine + 9, leftMargin,
                          "dSIB:  dsib_z_si_present %s  dsib_si_present %s  suj_z_id %u     ",
                          BoolString(data.dsib_z_si_present),
                          BoolString(data.dsib_si_present),
                          data.suj_z_id);
    Amp1394Console::Print(startLine + 10, leftMargin,
                          "ESSJ:  FW_VERSION 0x%04X  espm_present %s  adc_valid %s  essj_present %s     ",
                          data.essj_fw_version,
                          BoolString(data.espm_present),
                          BoolString(data.essj_adc_valid),
                          BoolString(data.essj_present));
    Amp1394Console::Print(startLine + 11, leftMargin,
                          "Power: power_enabled %s  suj_brake_released %s  DigOut 0x%02X  Status 0x%08X     ",
                          BoolString(powerEnabled),
                          BoolString(sujBrakeReleased),
                          static_cast<unsigned int>(digOut),
                          board->GetStatus());
    Amp1394Console::Print(startLine + 12, leftMargin,
                          "Err(r/w): %2d %2d     ",
                          board->GetReadErrors(),
                          board->GetWriteErrors());
}

int main(int argc, char** argv)
{
    const unsigned int lm = 5;
    BasePort::ProtocolType protocol = BasePort::PROTOCOL_SEQ_RW;
    bool readOnly = false;

    std::vector<AmpIO*> BoardList;
    std::string portDescription = BasePort::DefaultPort();
    std::string hardwareList;

    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'p') {
                portDescription = argv[i] + 2;
            }
            else if (argv[i][1] == 'h') {
                hardwareList = argv[i] + 2;
            }
            else if (argv[i][1] == 'b') {
                if (argv[i][2] == 'r') {
                    protocol = BasePort::PROTOCOL_BC_QRW;
                }
                else if (argv[i][2] == 'w') {
                    protocol = BasePort::PROTOCOL_SEQ_R_BC_W;
                }
            }
            else if (argv[i][1] == 'r') {
                readOnly = true;
            }
            else if ((argv[i][1] == 'v') || (argv[i][1] == 't') || (argv[i][1] == 'm')) {
                // Accepted for qladisp command-line compatibility; not applicable here.
            }
            else {
                std::cerr << "Unknown option: " << argv[i] << std::endl;
            }
        }
        else {
            int bnum = atoi(argv[i]);
            if ((bnum >= 0) && (bnum < BoardIO::MAX_BOARDS)) {
                BoardList.push_back(new AmpIO(bnum));
                std::cerr << "Selecting board " << bnum << std::endl;
            }
            else {
                std::cerr << "Invalid board number: " << argv[i] << std::endl;
            }
        }
    }

    if (BoardList.empty()) {
        PrintUsage(argv[0]);
        return 0;
    }

    std::stringstream debugStream(std::stringstream::out | std::stringstream::in);
    BasePort::AddHardwareVersionStringList(hardwareList);

    BasePort *Port = PortFactory(portDescription.c_str(), debugStream);
    if (!Port) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to create port using: " << portDescription << std::endl;
        return -1;
    }
    if (!Port->IsOK()) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to initialize " << Port->GetPortTypeString() << std::endl;
        delete Port;
        return -1;
    }

    if (Port->GetNumOfNodes() == 0) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to find any boards" << std::endl;
        delete Port;
        return -1;
    }

    for (size_t i = 0; i < BoardList.size(); i++) {
        Port->AddBoard(BoardList[i]);
    }

    if (protocol == BasePort::PROTOCOL_BC_QRW) {
        std::cerr << "Setting protocol to broadcast read/write" << std::endl;
    }
    else if (protocol == BasePort::PROTOCOL_SEQ_R_BC_W) {
        std::cerr << "Setting protocol to broadcast write" << std::endl;
    }
    if (!Port->SetProtocol(protocol)) {
        protocol = Port->GetProtocol();
    }

    Amp1394Console console;
    console.Init();
    if (!console.IsOK()) {
        std::cerr << "Failed to initialize console" << std::endl;
        delete Port;
        return -1;
    }

    const unsigned int hubBoard = Port->GetHubBoardId();
    if (protocol == BasePort::PROTOCOL_BC_QRW) {
        console.Print(1, lm, "SUJ-Si Feedback (hub %d)", hubBoard);
    }
    else {
        console.Print(1, lm, "SUJ-Si Feedback");
    }
    if (readOnly) {
        console.Print(2, lm, "Press ESC to quit, r to reset port (READ ONLY)");
    }
    else {
        console.Print(2, lm, "Press ESC to quit, r to reset port, p to enable/disable power, SPACE to release/engage SUJ brake");
    }
    console.Refresh();

    const int ESC_CHAR = 0x1b;
    int c;
    std::string message;
    unsigned int loopCount = 0;

    while ((c = console.GetChar()) != ESC_CHAR) {
        if (c == 'r') {
            Port->Reset();
            Port->SetProtocol(protocol);
            message = "Port reset";
        }
        else if (c == 'z') {
            for (size_t j = 0; j < BoardList.size(); j++) {
                BoardList[j]->ClearReadErrors();
                BoardList[j]->ClearWriteErrors();
            }
            message = "Errors cleared";
        }
        else if (readOnly) {
            if ((c == 'p') || (c == ' ')) {
                message = "Read-only mode: write ignored";
            }
        }
        else if (c == 'p') {
            Port->ReadAllBoards();
            bool anyPowered = false;
            for (size_t j = 0; j < BoardList.size(); j++) {
                anyPowered = anyPowered || BoardList[j]->GetPowerEnable();
            }
            const bool enablePower = !anyPowered;
            for (size_t j = 0; j < BoardList.size(); j++) {
                if (!enablePower) {
                    BoardList[j]->WriteDigitalOutput(0x01, 0x00);
                }
                BoardList[j]->WritePowerEnable(enablePower);
            }
            message = enablePower ? "Power enabled" : "Power disabled; SUJ brake engaged";
        }
        else if (c == ' ') {
            Port->ReadAllBoards();
            bool anyPowered = false;
            bool anyBrakeReleased = false;
            for (size_t j = 0; j < BoardList.size(); j++) {
                anyPowered = anyPowered || BoardList[j]->GetPowerEnable();
                anyBrakeReleased = anyBrakeReleased || ((BoardList[j]->GetDigitalOutput() & 0x01) != 0);
            }
            if (anyPowered) {
                const bool releaseBrake = !anyBrakeReleased;
                for (size_t j = 0; j < BoardList.size(); j++) {
                    if (BoardList[j]->GetPowerEnable()) {
                        BoardList[j]->WriteDigitalOutput(0x01, releaseBrake ? 0x01 : 0x00);
                    }
                }
                message = releaseBrake ? "SUJ brake released" : "SUJ brake engaged";
            }
            else {
                message = "Power is disabled; brake command ignored";
            }
        }

        if (!debugStream.str().empty()) {
            PrintDebugStream(debugStream);
        }

        if (Port->IsOK()) {
            Port->ReadAllBoards();
        }

        int line = 4;
        for (size_t j = 0; j < BoardList.size(); j++) {
            SiSUJData data;
            ReadSiSUJData(Port, BoardList[j], data);
            PrintBoardData(line, lm, BoardList[j], data);
            line += 15;
        }

        console.Print(line, lm, "Message: %-80s", message.c_str());
        console.Print(1, lm + 40, "Ct: %8u", loopCount++);
        console.Refresh();

        if (Port->GetPortType() != BasePort::PORT_ZYNQ_EMIO) {
            Amp1394_Sleep(0.0005);
        }
    }

    if (!readOnly) {
        for (size_t j = 0; j < BoardList.size(); j++) {
            BoardList[j]->WriteDigitalOutput(0x01, 0x00);
            BoardList[j]->WritePowerEnable(false);
        }
    }

    console.End();

    for (size_t j = 0; j < BoardList.size(); j++) {
        Port->RemoveBoard(BoardList[j]->GetBoardId());
        delete BoardList[j];
    }

    delete Port;
    return 0;
}
