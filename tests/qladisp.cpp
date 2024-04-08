/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen, Anton Deguet

  (C) Copyright 2012-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/******************************************************************************
 *
 * This program continuously displays the sensor feedback from the selected
 * board. It relies on the Amp1394 library (which depends on libraw1394 and/or pcap)
 * and on the Amp1394Console library (which may depend on curses).
 *
 * Usage: qladisp [-pP] [-b<r|w>] [-v] <board num> [<board_num>]
 *        where P is the port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *        -br or -bw specify to use a broadcast protocol
 *        -v specifies to display full velocity feedback
 *
 ******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>   // for std::max

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "Amp1394Console.h"

#include "EthBasePort.h"

// Maximum number of axes (channels) to display
const unsigned int MAX_AXES = 11;
const uint32_t current_incr = 0x100;   // 0x100 is about 50 mA

class AxisInfo {
public:
    enum AxisType { AXIS_UNKNOWN, MOTOR_ENCODER, MOTOR_ONLY, ENCODER_ONLY };

    AxisInfo() : axisType(AXIS_UNKNOWN) {}
    AxisInfo(AxisType ax) : axisType(ax) {}
    ~AxisInfo() {}

    bool HasMotor(void) const   { return (axisType == MOTOR_ENCODER) || (axisType == MOTOR_ONLY); }
    bool HasEncoder(void) const { return (axisType == MOTOR_ENCODER) || (axisType == ENCODER_ONLY); }
protected:
    AxisType axisType;
};

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
    std::cerr << debugStream.str() << std::endl;
    debugStream.clear();
    debugStream.str("");
}

unsigned int collectFileNum = 0;
std::ofstream collectFile;
bool isCollecting = false;

bool CollectCB(quadlet_t *buffer, short nquads)
{
    collectFile.write(reinterpret_cast<const char *>(buffer), nquads*sizeof(quadlet_t));
    if (!isCollecting)
        collectFile.close();
    return true;
}

bool CollectFileConvert(const char *inFilename, const char *outFilename)
{
    std::cerr << "Converting data collection file " << inFilename << " to " << outFilename << std::endl;
    std::ifstream inFile(inFilename, std::ifstream::binary);
    if (!inFile.good()) {
        std::cerr << "Failed to open input data collection file " << inFilename << std::endl;
        return false;
    }
    std::ofstream outFile(outFilename, std::ofstream::trunc);
    if (!outFile.good()) {
        std::cerr << "Failed to open output data collection file " << outFilename << std::endl;
        return false;
    }
    quadlet_t value;
    while (inFile.good()) {
        inFile.read(reinterpret_cast<char *>(&value), sizeof(quadlet_t));
        outFile << std::dec
                << ((value&0x80000000)>>31) << ", "    // type (0->commanded current, 1-> measured current)
                << ((value&0x40000000)>>30) << ", "    // timer overflow
                << ((value&0x3FFF0000)>>16) << ", "    // timer (14-bits)
                << (value&0x0000FFFF)                  // data
                << std::endl;
    }
    inFile.close();
    outFile.close();
    return true;
}

void UpdateStatusStrings(char *statusStr1, char *statusStr2, uint32_t statusChanged, uint32_t status)
{
    if (statusChanged&0x00080000) {  // power (mv-good)
        if (status&0x00080000) {
            statusStr1[0] = 'P';
            statusStr1[1] = '+';
        }
        else {
            statusStr1[3] = 'P';
            statusStr1[4] = '-';
        }
    }
    if (statusChanged&0x00020000) {  // safety relay
        if (status&0x00020000) {
            statusStr1[6] = 'S';
            statusStr1[7] = '+';
        }
        else {
            statusStr1[9] = 'S';
            statusStr1[10] = '-';
        }
    }
    if (statusChanged&0x00800000) {  // watchdog
        if (status&0x00800000) {
            statusStr1[12] = 'W';
            statusStr1[13] = '+';
        }
        else {
            statusStr1[15] = 'W';
            statusStr1[16] = '-';
        }
    }
    // NOTE: hard-coded for 4 amplifiers
    for (unsigned int i = 0; i < 4; i++) {  // amplifier status
        uint32_t mask = (0x00000100 << i);
        if (statusChanged&mask) {
            if (status&mask) {
                statusStr2[0] = 'A';
                statusStr2[2+4*i] = '+';
            }
            else {
                statusStr2[0] = 'A';
                statusStr2[2+4*i+1] = '-';
            }
        }
    }
}

int main(int argc, char** argv)
{
    const unsigned int lm = 5; // left margin
    unsigned int i, j;
    BasePort::ProtocolType protocol = BasePort::PROTOCOL_SEQ_RW;
    bool fullvel = false;  // whether to display full velocity feedback
    bool showTime = false; // whether to display time information
    bool useMaxAxis = false; // true --> use max(NumEncoders, NumMotors); false --> NumEncoders
    bool readOnly = false;   // true --> program does not call WriteAllBoards

    std::vector<AmpIO*> BoardList;
    std::vector<uint32_t> BoardStatusList;

    // measure time between reads
    uint32_t maxTime = 0;
    uint32_t lastTime = 0;

    unsigned int curAxis = 0;                 // Current axis (0=all, 1-MAX_AXES)
    unsigned int Axis2BoardNum[MAX_AXES+1];   // Maps curAxis (1-MAX_AXES) to board number (0 or 1)
    unsigned int Axis2BoardAxis[MAX_AXES+1];  // Maps curAxis (1-MAX_AXES) to board axis (0 to NumEncoders)
    AxisInfo AxisData[MAX_AXES+1];            // Indicates whether motor, encoder or both
    unsigned int curBoardIndex = 0;           // Current board (when curAxis != 0)
    unsigned int curAxisIndex = 0;            // Current axis on current board (when curAxis != 0)
    char axisString[4] = "all";
    std::string portDescription = BasePort::DefaultPort();
    std::string hardwareList;

    for (i = 1; i < (unsigned int)argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'p') {
                portDescription = argv[i]+2;
            }
            else if (argv[i][1] == 'h') {
                hardwareList = argv[i]+2;
            }
            else if (argv[i][1] == 'b') {
                // -br -- enable broadcast read/write
                // -bw -- enable broadcast write (sequential read)
                if (argv[i][2] == 'r')
                    protocol = BasePort::PROTOCOL_BC_QRW;
                else if (argv[i][2] == 'w')
                    protocol = BasePort::PROTOCOL_SEQ_R_BC_W;
            }
            else if (argv[i][1] == 'v')
                fullvel = true;
            else if (argv[i][1] == 't')
                showTime = true;
            else if (argv[i][1] == 'm')
                useMaxAxis = true;
            else if (argv[i][1] == 'r')
                readOnly = true;
        }
        else {
            int bnum = atoi(argv[i]);
            if ((bnum >= 0) && (bnum < BoardIO::MAX_BOARDS)) {
                BoardList.push_back(new AmpIO(bnum));
                std::cerr << "Selecting board " << bnum << std::endl;
            }
            else
                std::cerr << "Invalid board number: " << argv[i] << std::endl;
        }
    }

    if (BoardList.size() < 1) {
        // usage
        std::cerr << "Usage: qladisp <board-num> [<board-num>] [-pP] [-hH] [-b<r|w>] [-v] [-t] [-m]" << std::endl
                  << "       where P = port number (default 0)" << std::endl
                  << "                 can also specify -pfw[:P], -peth:P or -pudp[:xx.xx.xx.xx]" << std::endl
                  << "             H = additional supported hardware versions" << std::endl
                  << "            -br enables broadcast read/write" << std::endl
                  << "            -bw enables broadcast write" << std::endl
                  << "            -v  displays full velocity feedback" << std::endl
                  << "            -t  displays time information" << std::endl
                  << "            -m  include motors without encoders (if any)" << std::endl
                  << "            -r  read-only (does not write to boards)" << std::endl
                  << std::endl
                  << "Trying to detect boards on port:" << std::endl;
    }

    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);
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
        return -1;
    }

    if (BoardList.size() < 1) {
        PrintDebugStream(debugStream);
        // keys
        std::cerr << std::endl << "Keys:" << std::endl
                  << "'r': reset FireWire port" << std::endl
                  << "'a': axis to address (1-N) or all (0, default)" << std::endl;
        if (!readOnly) {
            std::cerr
                  << "'d': turn watchdog on/off (0.25 ms, this should be triggered immediately)" << std::endl
                  << "'D': turn watchdog on/off (25.0 ms, can be triggered by unplugging cable to PC)" << std::endl
                  << "'p': turn power on/off (board and axis)" << std::endl
                  << "'o': turn power on/off (board only)" << std::endl
                  << "'i': turn power on/off (axis only, requires board first)" << std::endl
                  << "'v': toggle voltage/current mode (QLA 1.5+)" << std::endl
                  << "'w': increment encoders" << std::endl
                  << "'s': decrement encoders" << std::endl
                  << "'=': increase motor current by about 50mA" << std::endl
                  << "'-': increase motor current by about 50mA" << std::endl
                  << "'c': start/stop data collection" << std::endl
                  << "'z': clear status events and r/w errors" << std::endl
                  << "'0'-'3': toggle digital output bit" << std::endl;
        }
        else {
            std::cerr
                  << "'z': clear status events and r/w errors" << std::endl;
        }
        return 0;
    }

    if (Port->GetNumOfNodes() == 0) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to find any boards" << std::endl;
        return -1;
    }

    // Check if an Ethernet port
    EthBasePort *ethPort = dynamic_cast<EthBasePort *>(Port);

    // Number of boards to display (currently 1 or 2)
    unsigned int numDisp = 0;
    // Total number of axes to display
    unsigned int numAxes = 0;
    // Note that Axis2BoardNum and Axis2BoardAxis start at 1 because
    // curAxis 0 is used to select all axes. The first entry should
    // not be used, but we initialize it to something reasonable (0).
    Axis2BoardNum[0] = 0;
    Axis2BoardAxis[0] = 0;
    AxisData[0] = AxisInfo::AXIS_UNKNOWN;
    for (i = 0; i < BoardList.size(); i++) {
        Port->AddBoard(BoardList[i]);
        int numEnc = BoardList[i]->GetNumEncoders();
        int numMot = BoardList[i]->GetNumMotors();
        int numThisBoard = useMaxAxis ? std::max(numEnc,numMot) : numEnc;
        if (numAxes + numThisBoard <= MAX_AXES) {
            for (int j = 0; j < numThisBoard; j++) {
                Axis2BoardNum[1+numAxes+j] = i;
                Axis2BoardAxis[1+numAxes+j] = j;
                if ((j < numEnc) && (j < numMot))
                    AxisData[1+numAxes+j] = AxisInfo(AxisInfo::MOTOR_ENCODER);
                else if (j < numEnc)
                    AxisData[1+numAxes+j] = AxisInfo(AxisInfo::ENCODER_ONLY);
                else
                    AxisData[1+numAxes+j] = AxisInfo(AxisInfo::MOTOR_ONLY);
            }
            numDisp++;
            numAxes += numThisBoard;
        }
    }

    // Set protocol; default is PROTOCOL_SEQ_RW (not broadcast), but can be changed to
    // one of the broadcast protocols by specifying -br or -bw command line parameter.
    if (protocol == BasePort::PROTOCOL_BC_QRW)
        std::cerr << "Setting protocol to broadcast read/write" << std::endl;
    else if (protocol == BasePort::PROTOCOL_SEQ_R_BC_W)
        std::cerr << "Setting protocol to broadcast write" << std::endl;
    if (!Port->SetProtocol(protocol))
        protocol = Port->GetProtocol();  // on failure, get current protocol

    // Initialize motor currents at mid-range
    uint32_t MotorCurrents[MAX_AXES];

    bool VoltageModeAvail[MAX_AXES];    // Whether voltage control available (QLA 1.5+)
    bool VoltageMode[MAX_AXES];         // Whether voltage control active
    for (i = 0; i < MAX_AXES; i++) {
        MotorCurrents[i] = 0x8000;
        VoltageModeAvail[i] = false;
        VoltageMode[i] = false;
    }

    bool someRev7plus = false;
    bool someRev8plus = false;
    BoardStatusList.clear();
    for (j = 0; j < BoardList.size(); j++) {
        uint32_t fver = BoardList[j]->GetFirmwareVersion();
        if (fver >= 7) someRev7plus = true;
        if (fver >= 8) someRev8plus = true;
    }

    if (!readOnly) {
        for (j = 0; j < numDisp; j++) {
            for (i = 0; i < BoardList[j]->GetNumEncoders(); i++)
                BoardList[j]->WriteEncoderPreload(i, 0x1000*i + 0x1000);
        }
        for (j = 0; j < numDisp; j++) {
            BoardList[j]->WriteSafetyRelay(false);
            BoardList[j]->WritePowerEnable(false);
            BoardList[j]->WriteAmpEnable(0x0f, 0);
        }
    }

    for (j = 0; j < numDisp; j++) {
        uint32_t bstat = BoardList[j]->ReadStatus();
        BoardStatusList.push_back(bstat);
    }

    bool watchdog_on = false;

    Amp1394Console console;
    console.Init();
    if (!console.IsOK()) {
        std::cerr << "Failed to initialize console" << std::endl;
        return -1;
    }

    int board1 = BoardList[0]->GetBoardId();
    unsigned int hub_board = Port->GetHubBoardId();
    if (numDisp > 1) {
        int board2 = BoardList[1]->GetBoardId();
        if (protocol == BasePort::PROTOCOL_BC_QRW)
            console.Print(1, lm, "Sensor Feedback for Boards %d, %d (hub %d)", board1, board2, hub_board);
        else
            console.Print(1, lm, "Sensor Feedback for Boards %d, %d", board1, board2);
    } else {
        if (protocol == BasePort::PROTOCOL_BC_QRW)
            console.Print(1, lm, "Sensor Feedback for Board %d (hub %d)", board1, hub_board);
        else
            console.Print(1, lm, "Sensor Feedback for Board %d", board1);
    }
    if (readOnly) {
        console.Print(2, lm, "Press ESC to quit, r to reset port (READ ONLY)");
    }
    else {
        console.Print(2, lm, "Press ESC to quit, r to reset port, 0-3 to toggle digital output bit, p to enable/disable power,");
        console.Print(3, lm, "+/- to increase/decrease commanded current (DAC) by 0x100");
    }

    for (i = 1; i <= numAxes; i++) {
        unsigned int dx = lm+8+(i-1)*13;
        if (i >= 10) dx--;
        unsigned int bd = Axis2BoardNum[i];
        unsigned int ax = Axis2BoardAxis[i];
        uint32_t mconfig;
        if (BoardList[bd]->ReadMotorConfig(ax, mconfig))
            VoltageModeAvail[i-1] = mconfig & AmpIO::MCFG_VOLTAGE_CONTROL;
        if (VoltageModeAvail[i-1])
            console.Print(5, dx-2, "Axis %d I", i);
        else
            console.Print(5, dx, "Axis %d", i);
    }
    console.Print(6, lm, "Enc:");
    console.Print(7, lm, "Pot:");
    console.Print(8, lm, "Vel:");
    console.Print(9, lm, "Cur:");
    console.Print(10, lm, "DAC:");
    int nextLine = 11;
    if (someRev8plus)
        console.Print(nextLine++, lm, "MSt:");   // Motor status
    if (fullvel) {
        if (someRev7plus) {
            console.Print(nextLine++, lm, "Qtr1:");
            console.Print(nextLine++, lm, "Qtr5:");
            console.Print(nextLine++, lm, "Run:");
        }
        else {
            console.Print(nextLine++, lm, "Acc:");
        }
    }

    console.Refresh();

    unsigned char dig_out = 0x0f;
    unsigned char collect_axis = 1;
    uint32_t status;
    uint32_t statusChanged = 0;
    const unsigned int STATUS_STR_LENGTH = 18;
    char statusStr1[2][STATUS_STR_LENGTH];
    memset(statusStr1[0], ' ', STATUS_STR_LENGTH-1);
    memset(statusStr1[1], ' ', STATUS_STR_LENGTH-1);
    statusStr1[0][STATUS_STR_LENGTH-1] = 0;
    statusStr1[1][STATUS_STR_LENGTH-1] = 0;
    char statusStr2[2][STATUS_STR_LENGTH];
    memset(statusStr2[0], ' ', STATUS_STR_LENGTH-1);
    memset(statusStr2[1], ' ', STATUS_STR_LENGTH-1);
    statusStr2[0][STATUS_STR_LENGTH-1] = 0;
    statusStr2[1][STATUS_STR_LENGTH-1] = 0;

    unsigned int loop_cnt = 0;
    const int STATUS_LINE = fullvel ? 17 : 14;
    int timeLines = 0;   // how many lines for timing info
    if (showTime) {
        timeLines++;
        if (protocol == BasePort::PROTOCOL_BC_QRW)
            timeLines += 2;
    }
    else if (ethPort)
        timeLines++;
    const int DEBUG_START_LINE = fullvel ? (23+timeLines) : (20+timeLines);
    unsigned int last_debug_line = DEBUG_START_LINE;
    const int ESC_CHAR = 0x1b;
    int c;

    if (showTime)
        console.Print(STATUS_LINE+5, lm, "Time (s):");
    else if (ethPort)
        console.Print(STATUS_LINE+5, lm, "Ethernet:");

    double startTime = -1.0;   // indicates that startTime not yet set
    double pcTime = 0.0;

    // control loop
    while ((c = console.GetChar()) != ESC_CHAR)
    {

        unsigned int startIndex = (curAxis == 0) ? 0 : curBoardIndex;
        unsigned int endIndex = (curAxis == 0) ? numDisp : curBoardIndex+1;

        if (c == 'r') {
            Port->Reset();
            Port->SetProtocol(protocol);
        }
        else if (c == 'a') {
            if (curAxis < numAxes) {
                curAxis++;
                curBoardIndex = Axis2BoardNum[curAxis];
                curAxisIndex = Axis2BoardAxis[curAxis];
                sprintf(axisString, "%d", curAxis);
            }
            else {
                curAxis = 0;
                strcpy(axisString, "all");
            }
        }
        else if (c == 'z') {
            for (j = 0; j < numDisp; j++) {
                memset(statusStr1[j], ' ', STATUS_STR_LENGTH-1);
                statusStr1[j][STATUS_STR_LENGTH-1] = 0;
                memset(statusStr2[j], ' ', STATUS_STR_LENGTH-1);
                statusStr2[j][STATUS_STR_LENGTH-1] = 0;
                console.Print(STATUS_LINE+4, lm+45+58*j, "            ");
            }
            for (j = 0; j < BoardList.size(); j++) {
                BoardList[j]->ClearReadErrors();
                BoardList[j]->ClearWriteErrors();
            }
            if (showTime && (protocol == BasePort::PROTOCOL_BC_QRW)) {
                Port->ClearBroadcastReadInfo();
            }
        }
        else if (readOnly) {
            // Options below not available in readOnly mode
        }
        else if ((c >= '0') && (c <= '3')) {
            // toggle digital output bit
            dig_out = dig_out^(1<<(c-'0'));
            for (j = startIndex; j < endIndex; j++)
                BoardList[j]->WriteDigitalOutput(0x0f, dig_out);
        }
        else if (c == 'v') {
            if (curAxis == 0) {
                for (unsigned int axis = 0; axis < numAxes; axis++) {
                    if (VoltageModeAvail[axis]) {
                        VoltageMode[axis] = !VoltageMode[axis];
                        unsigned int dx = lm+6+axis*13;
                        if (axis >= 9) dx--;
                        console.Print(5, dx, "Axis %d %c", axis+1, VoltageMode[axis] ? 'V' : 'I');
                    }
                }
            }
            else if (VoltageModeAvail[curAxis-1]) {
                VoltageMode[curAxis-1] = !VoltageMode[curAxis-1];
                unsigned int dx = lm+6+(curAxis-1)*13;
                if (curAxis >= 10) dx--;
                console.Print(5, dx, "Axis %d %c", curAxis, VoltageMode[curAxis-1] ? 'V' : 'I');
            }
        }
        else if (c == 'w') {
            for (j = startIndex; j < endIndex; j++)
                EncUp(*(BoardList[j]));
        }
        else if (c == 's') {
            for (j = startIndex; j < endIndex; j++)
                EncDown(*(BoardList[j]));
        }
        else if (c == 'd') {
            watchdog_on = !watchdog_on;
            for (j = startIndex; j < endIndex; j++)
                // 0.25 ms
                BoardList[j]->WriteWatchdogPeriodInSeconds(watchdog_on?(0.00025):0);
        }
        else if (c == 'D') {
            watchdog_on = !watchdog_on;
            for (j = startIndex; j < endIndex; j++)
                // 25.00 ms
                BoardList[j]->WriteWatchdogPeriodInSeconds(watchdog_on?(0.025):0);
        }
        else if (c == 'p') {
            // Only power on system if completely off; otherwise, we power off.
            if (curAxis == 0) {       // All axes on all boards
                bool anyBoardPowered = false;
                for (j = 0; j < numDisp; j++) {
                    // Also use safety relay for power status because GetPowerStatus relies on the motor
                    // power supply being connected to the QLA. Check GetPowerEnable in addition to
                    // GetPowerStatus because GetPowerStatus returns true if the QLA is not connected.
                    if (BoardList[j]->GetSafetyRelayStatus() || (BoardList[j]->GetPowerEnable() && BoardList[j]->GetPowerStatus()))
                        anyBoardPowered = true;
                }
                for (j = 0; j < numDisp; j++) {
                    if (anyBoardPowered) {
                        BoardList[j]->SetPowerEnable(false);
                        BoardList[j]->SetSafetyRelay(false);
                        for (i = 0; i < BoardList[j]->GetNumMotors(); i++)
                            BoardList[j]->SetAmpEnable(i, false);
                    }
                    else {
                        BoardList[j]->SetSafetyRelay(true);
                        // Cannot enable Amp power unless Board power is
                        // already enabled.
                        BoardList[j]->WritePowerEnable(true);
                        //BoardList[j]->SetPowerEnable(true);
                        for (i = 0; i < BoardList[j]->GetNumMotors(); i++)
                            BoardList[j]->SetAmpEnable(i, true);
                    }
                }
            }
            else {
                bool isPowered = BoardList[curBoardIndex]->GetAmpEnable(curAxisIndex);
                if (isPowered) {
                    // Note that all axes will be disabled when board power is removed
                    BoardList[curBoardIndex]->SetAmpEnable(curAxisIndex, false);
                    BoardList[curBoardIndex]->SetPowerEnable(false);
                    BoardList[curBoardIndex]->SetSafetyRelay(false);
                }
                else {
                    BoardList[curBoardIndex]->SetSafetyRelay(true);
                    // Cannot enable Amp power unless Board power is
                    // already enabled.
                    BoardList[curBoardIndex]->WritePowerEnable(true);
                    //BoardList[curBoardIndex]->SetPowerEnable(true);
                    BoardList[curBoardIndex]->SetAmpEnable(curAxisIndex, true);
                }
            }
        }
        else if (c == 'o') {
            bool anyBoardPowered = false;
            for (j = startIndex; j < endIndex; j++) {
                // Also use safety relay for power status because GetPowerStatus relies on the motor
                // power supply being connected to the QLA. Check GetPowerEnable in addition to
                // GetPowerStatus because GetPowerStatus returns true if the QLA is not connected.
                if (BoardList[j]->GetSafetyRelayStatus() || (BoardList[j]->GetPowerEnable() && BoardList[j]->GetPowerStatus()))
                    anyBoardPowered = true;
            }
            for (j = startIndex; j < endIndex; j++) {
                if (anyBoardPowered) {
                    BoardList[j]->SetPowerEnable(false);
                    BoardList[j]->SetSafetyRelay(false);
                }
                else {
                    BoardList[j]->SetSafetyRelay(true);
                    BoardList[j]->SetPowerEnable(true);
                }
            }
        }
        else if (c == 'i') {
            if (curAxis == 0) {
                bool anyAxisPowered = false;
                for (j = 0; j < numDisp; j++) {
                    if (BoardList[j]->GetAmpEnableMask() != 0)
                        anyAxisPowered = true;
                }
                for (unsigned int axisNum = 1; axisNum <= numAxes; axisNum++) {
                    j = Axis2BoardNum[axisNum];
                    i = Axis2BoardAxis[axisNum];
                    if (i < BoardList[j]->GetNumMotors())
                        BoardList[j]->SetAmpEnable(i, anyAxisPowered ? 0 : 1);
                }
            }
            else {
                bool isPowered = BoardList[curBoardIndex]->GetAmpEnable(curAxisIndex);
                BoardList[curBoardIndex]->SetAmpEnable(curAxisIndex, !isPowered);
            }
        }
        else if (c == '=') {
            if (curAxis == 0) {
                for (unsigned int axis = 0; axis < numAxes; axis++)
                    MotorCurrents[axis] += current_incr;
            }
            else {
                MotorCurrents[curAxis-1] += current_incr;
            }
        }
        else if (c == '-') {
            if (curAxis == 0) {
                for (unsigned int axis = 0; axis < numAxes; axis++)
                    MotorCurrents[axis] -= current_incr;
            }
            else {
                MotorCurrents[curAxis-1] -= current_incr;
            }
        }
        else if (c == 'c') {
            int collect_board = (collect_axis <= BoardList[0]->GetNumEncoders()) ? 0 : 1;
            if (BoardList[collect_board]->IsCollecting()) {
                BoardList[collect_board]->DataCollectionStop();
                console.Print(STATUS_LINE-1, lm, "             ");
                console.Print(STATUS_LINE-1, lm+15, "                ");
                if (collect_axis > collectFileNum)
                    collectFileNum = collect_axis;
                collect_axis = (collect_axis == numAxes) ? 1 : collect_axis+1;
                isCollecting = false;
            }
            else {
                char fileName[20];
                sprintf(fileName, "Collect%d.raw", collect_axis);
                collectFile.open(fileName, std::ofstream::binary|std::ofstream::trunc);
                if (!collectFile.good())
                    std::cerr << "Failed to open data collection file " << fileName << std::endl;
                unsigned char collect_chan = collect_axis-(BoardList[0]->GetNumEncoders())*collect_board;
                if (BoardList[collect_board]->DataCollectionStart(collect_chan, CollectCB)) {
                    isCollecting = true;
                    console.Print(STATUS_LINE-1, lm, "Collecting %d:", collect_axis);
                }
            }
        }

        if (!debugStream.str().empty()) {
            int cur_line = DEBUG_START_LINE;
            char line[120];
            memset(line, ' ', sizeof(line)-1);
            line[sizeof(line)-1] = 0;
            for (i = cur_line; i < last_debug_line; i++)
                console.Print(i, lm, line);
            while (!debugStream.eof()) {
                std::string stringLine;
                std::getline(debugStream, stringLine);
                console.Print(cur_line++, lm, stringLine.c_str());
            }
            debugStream.clear();
            debugStream.str("");
            last_debug_line = cur_line;
        }

        if (!Port->IsOK()) continue;

        char nodeStr[2][3];
        int node = Port->GetNodeId(BoardList[0]->GetBoardId());
        if (node < BoardIO::MAX_BOARDS)
            sprintf(nodeStr[0], "%2d", node);
        else
            strcpy(nodeStr[0], "none");

        if (numDisp > 1) {
            node = Port->GetNodeId(BoardList[1]->GetBoardId());
            if (node < BasePort::MAX_NODES)
                sprintf(nodeStr[1], "%2d", node);
            else
                strcpy(nodeStr[1], "??");
        }

        EncoderVelocity encVelData;
        Port->ReadAllBoards();
        unsigned int j;
        if (showTime) {
            if (startTime < 0.0) {
                startTime = Amp1394_GetTime();
                pcTime = 0.0;
                for (j = 0; j < numDisp; j++)
                    BoardList[j]->SetFirmwareTime(0.0);
            }
            else {
                pcTime = Amp1394_GetTime()-startTime;
            }
        }
        // Check if hub board may have been updated
        if (protocol == BasePort::PROTOCOL_BC_QRW) {
            unsigned int new_hub_board = Port->GetHubBoardId();
            if (new_hub_board != hub_board) {
                hub_board = new_hub_board;
                if (numDisp > 1) {
                    int board2 = BoardList[1]->GetBoardId();
                    console.Print(1, lm, "Sensor Feedback for Boards %d, %d (hub %d)", board1, board2, hub_board);
                }
                else {
                    console.Print(1, lm, "Sensor Feedback for Board %d (hub %d)", board1, hub_board);
                }
            }
        }
        bool allValid = true;
        for (unsigned int axisNum = 1; axisNum <= numAxes; axisNum++) {
            j = Axis2BoardNum[axisNum];
            unsigned int dx = (axisNum-1)*13;   // offset between columns
            if (BoardList[j]->ValidRead()) {
                i = Axis2BoardAxis[axisNum];
                uint32_t fver = BoardList[j]->GetFirmwareVersion();
                if (AxisData[axisNum].HasEncoder()) {
                    console.Print(6, lm+7+dx, "%07X",
                              BoardList[j]->GetEncoderPosition(i)+BoardList[j]->GetEncoderMidRange());
                    console.Print(7, lm+10+dx, "%04X", BoardList[j]->GetAnalogInput(i));
                    if (fullvel)
                        console.Print(8, lm+6+dx, "%08X", BoardList[j]->GetEncoderVelocityRaw(i));
                    else {
                        BoardList[j]->GetEncoderVelocityData(i, encVelData);
                        console.Print(8, lm+6+dx, "%08X", encVelData.GetEncoderVelocityPeriod());
                    }
                }
                if (AxisData[axisNum].HasMotor()) {
                    console.Print(9, lm+10+dx, "%04X", BoardList[j]->GetMotorCurrent(i));
                    console.Print(10, lm+10+dx, "%04X", MotorCurrents[axisNum-1]);
                    if (VoltageMode[axisNum-1])
                        BoardList[j]->SetMotorVoltage(i, MotorCurrents[axisNum-1]);
                    else
                        BoardList[j]->SetMotorCurrent(i, MotorCurrents[axisNum-1]);
                    if (someRev8plus) {
                        if (fver >= 8)
                            console.Print(11, lm+6+dx, "%08X", BoardList[j]->GetMotorStatus(i));
                    }
                }
                nextLine = someRev8plus ? 12 : 11;
                if (AxisData[axisNum].HasEncoder()) {
                    if (fullvel) {
                        if (someRev7plus) {
                            if (fver >= 7) {
                                console.Print(nextLine++, lm+6+dx, "%08X", BoardList[j]->GetEncoderQtr1Raw(i));
                                console.Print(nextLine++, lm+6+dx, "%08X", BoardList[j]->GetEncoderQtr5Raw(i));
                                console.Print(nextLine++, lm+6+dx, "%08X", BoardList[j]->GetEncoderRunningCounterRaw(i));
                            }
                        }
                        else
                            console.Print(nextLine++, lm+6+dx, "%08X", BoardList[j]->GetEncoderAccelerationRaw(i));
                    }
                }
            }
            else {
                allValid = false;
            }
        }
        unsigned int bdx = 0;
        for (j = 0; j < numDisp; j++) {
            if (BoardList[j]->ValidRead()) {
                if (isCollecting) {
                    bool fpgaCollecting;
                    unsigned char fpgaChan;
                    unsigned short fpgaAddr;
                    if (BoardList[j]->GetCollectionStatus(fpgaCollecting, fpgaChan, fpgaAddr)) {
                        console.Print(STATUS_LINE-1, lm+15, "writeAddr: %4d", fpgaAddr);
                    }
                }
                dig_out = BoardList[j]->GetDigitalOutput();
                status = BoardList[j]->GetStatus();
                if (status != BoardStatusList[j]) {
                    statusChanged = status^BoardStatusList[j];
                    BoardStatusList[j] = status;
                    UpdateStatusStrings(statusStr1[j], statusStr2[j], statusChanged, status);
                }
                console.Print(STATUS_LINE, lm+bdx, "Status: %08X   Timestamp: %08X   DigOut: %01X",
                          status, BoardList[j]->GetTimestamp(),
                          (unsigned int)dig_out);
                console.Print(STATUS_LINE+1, lm+bdx, "%17s  NegLim: %01X  PosLim: %01X  Home: %01X",
                          statusStr1[j],
                          BoardList[j]->GetNegativeLimitSwitches(),
                          BoardList[j]->GetPositiveLimitSwitches(),
                          BoardList[j]->GetHomeSwitches());
                console.Print(STATUS_LINE+2, lm+bdx, "%17s  EncA: %01X    EncB: %01X    EncI: %01X",
                          statusStr2[j],
                          BoardList[j]->GetEncoderChannelA(),
                          BoardList[j]->GetEncoderChannelB(),
                          BoardList[j]->GetEncoderIndex());

                console.Print(STATUS_LINE+4, lm+bdx, "Node: %s", nodeStr[j]);
                if (BoardList[j]->GetHardwareVersion() == DQLA_String) {
                    console.Print(STATUS_LINE+4, lm+14+bdx, "Temp:  %02X    %02X    %02X    %02X ",
                              (unsigned int)BoardList[j]->GetAmpTemperature(0),
                              (unsigned int)BoardList[j]->GetAmpTemperature(1),
                              (unsigned int)BoardList[j]->GetAmpTemperature(2),
                              (unsigned int)BoardList[j]->GetAmpTemperature(3));
                }
                else {
                    console.Print(STATUS_LINE+4, lm+14+bdx, "Temp:  %02X    %02X",
                              (unsigned int)BoardList[j]->GetAmpTemperature(0),
                              (unsigned int)BoardList[j]->GetAmpTemperature(1));
                }
                if (loop_cnt > 500) {
                    lastTime = BoardList[j]->GetTimestamp();
                    if (lastTime > maxTime) {
                        maxTime = lastTime;
                    }
                }
            }
            int err_dx = 35+bdx;
            if (BoardList[j]->GetHardwareVersion() == DQLA_String)
                err_dx += 12;
            console.Print(STATUS_LINE+4, lm+err_dx, "Err(r/w): %2d %2d",
                      BoardList[j]->GetReadErrors(),
                      BoardList[j]->GetWriteErrors());
            if (showTime)
                console.Print(STATUS_LINE+5, lm+20+bdx, "%9.3lf (%7.4lf)", BoardList[j]->GetFirmwareTime(),
                          pcTime-BoardList[j]->GetFirmwareTime());
            int numEnc = BoardList[j]->GetNumEncoders();
            int numMot = BoardList[j]->GetNumMotors();
            int numTot = useMaxAxis ? std::max(numEnc,numMot) : numEnc;
            bdx += (6 + numTot*13);
            loop_cnt++;
        }
        if (showTime) {
            console.Print(STATUS_LINE+5, lm+53, "%8.3lf", pcTime);
            if ((protocol == BasePort::PROTOCOL_BC_QRW) && allValid) {
                BasePort::BroadcastReadInfo bcReadInfo;
                bcReadInfo = Port->GetBroadcastReadInfo();
                std::stringstream timingStr;
                bcReadInfo.PrintTiming(timingStr);
                char bcBuf[256];
                timingStr.getline(bcBuf, 256);
                strcat(bcBuf, "   ");
                console.Print(STATUS_LINE+6, lm, bcBuf);
                timingStr.getline(bcBuf, 256);
                strcat(bcBuf, "   ");
                console.Print(STATUS_LINE+7, lm, bcBuf);
            }
        }
        else if (ethPort) {
            EthBasePort::FPGA_Status fpgaStatus;
            ethPort->GetFpgaStatus(fpgaStatus);
            char flagStr[8];
            flagStr[0] = fpgaStatus.FwBusReset ? 'R' : ' ';
            flagStr[1] = ' ';
            flagStr[2] = fpgaStatus.FwPacketDropped ? 'D' : ' ';
            flagStr[3] = ' ';
            flagStr[4] = fpgaStatus.EthInternalError ? 'I' : ' ';
            flagStr[5] = ' ';
            flagStr[6] = fpgaStatus.EthSummaryError ? 'S' : ' ';
            flagStr[7] = 0;
            console.Print(STATUS_LINE+5, lm+12, "%s   StateError: %3d   PacketError: %3d",
                          flagStr, fpgaStatus.numStateInvalid, fpgaStatus.numPacketError);
        }
        if (!readOnly)
            Port->WriteAllBoards();

        unsigned int readErrors = 0;
        unsigned int writeErrors = 0;
        for (j = 0; j < BoardList.size(); j++) {
            readErrors += BoardList[j]->GetReadErrors();
            writeErrors += BoardList[j]->GetWriteErrors();
        }

        console.Print(1, lm+42, "Gen: %d",  Port->GetBusGeneration());
        console.Print(1, lm+54, "Axis: %4s", axisString);
        console.Print(1, lm+70, "dt: %.3f",  (1.0 / 49125.0) * maxTime);
        console.Print(1, lm+85, "Ct: %8u", loop_cnt++);
        console.Print(1, lm+100, "Err(r/w): %2d %2d", readErrors, writeErrors);

        console.Refresh();
        // Skip sleep when using Zynq EMIO because it is already slow
        if (Port->GetPortType() != BasePort::PORT_ZYNQ_EMIO)
            Amp1394_Sleep(0.0005);  // 500 usec
    }

    if (!readOnly) {
        for (j = 0; j < numDisp; j++) {
            BoardList[j]->WritePowerEnable(false);      // Turn power off
            BoardList[j]->WriteAmpEnable(0x0f, 0x00);   // Turn power off
            BoardList[j]->WriteSafetyRelay(false);
        }

        // Reset encoder preloads to default
        for (j = 0; j < numDisp; j++) {
            for (i = 0; i < BoardList[j]->GetNumEncoders(); i++) {
                BoardList[j]->WriteEncoderPreload(i, 0);
            }
        }
    }


    console.End();

    for (j = 0; j < BoardList.size(); j++)
        Port->RemoveBoard(BoardList[j]->GetBoardId());

    delete Port;
    // Process any data collection files (convert from binary to text)
    for (j = 1; j <= collectFileNum; j++) {
        char inFile[20];
        char outFile[20];
        sprintf(inFile,  "Collect%d.raw", j);
        sprintf(outFile, "Collect%d.csv", j);
        CollectFileConvert(inFile, outFile);
    }
    return 0;
}
