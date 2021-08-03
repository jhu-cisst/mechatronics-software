/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen, Anton Deguet

  (C) Copyright 2012-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/******************************************************************************
 *
 * This program continuously displays the sensor feedback from the selected
 * board. It relies on the curses library and the AmpIO library (which
 * depends on libraw1394 and/or pcap).
 *
 * Usage: qladisp [-pP] [-b<r|w>] [-v] <board num> [<board_num>]
 *        where P is the Firewire port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *        -br or -bw specify to use a broadcast protocol
 *        -v specifies to display full velocity feedback
 *
 ******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"
#include "Amp1394Console.h"

#include "EthBasePort.h"

class BUF_CONFIG_MAP {
public:
    std::map<std::string, const AmpIO_UInt32> forward_map;
    std::map<const AmpIO_UInt32, std::string> backward_map;

    BUF_CONFIG_MAP() {
        forward_map.insert(std::make_pair("BUF_POT_DATA", 0x00001000)); // pot feedback
        forward_map.insert(std::make_pair("BUF_CUR_DATA", 0x00002000)); // cur feedback
        forward_map.insert(std::make_pair("BUF_DAC_DATA", 0x00003000)); // cur command
        forward_map.insert(std::make_pair("BUF_ENC_DATA", 0x00004000)); // encoder edge count
        forward_map.insert(std::make_pair("BUF_ENC_PER",  0x00005000)); // encoder period
        forward_map.insert(std::make_pair("BUF_ENC_QTR1", 0x00006000)); // encoder period recent quarter
        forward_map.insert(std::make_pair("BUF_ENC_QTR5", 0x00007000)); // encoder period old quarter
        forward_map.insert(std::make_pair("BUF_ENC_RUN",  0x00008000)); // encoder running data
        forward_map.insert(std::make_pair("BUF_UINT16",   0x00000001)); // save in uint16 format
        forward_map.insert(std::make_pair("BUF_UINT32",   0x00000002)); // save in uint32 format

        backward_map.insert(std::make_pair(0x00001000, "BUF_POT_DATA"));
        backward_map.insert(std::make_pair(0x00002000, "BUF_CUR_DATA"));
        backward_map.insert(std::make_pair(0x00003000, "BUF_DAC_DATA"));
        backward_map.insert(std::make_pair(0x00004000, "BUF_ENC_DATA"));
        backward_map.insert(std::make_pair(0x00005000, "BUF_ENC_PER"));
        backward_map.insert(std::make_pair(0x00006000, "BUF_ENC_QTR1"));
        backward_map.insert(std::make_pair(0x00007000, "BUF_ENC_QTR5"));
        backward_map.insert(std::make_pair(0x00008000, "BUF_ENC_RUN"));
        backward_map.insert(std::make_pair(0x00000001, "BUF_UINT16"));
        backward_map.insert(std::make_pair(0x00000002, "BUF_UINT32"));
    }

    ~BUF_CONFIG_MAP() {
        forward_map.clear();
        backward_map.clear();
    }
};

const AmpIO_UInt32 BUF_MODE_CONTINUOUS  = 0x00000300; // switch to continuous capture
const AmpIO_UInt32 BUF_MODE_SAMPLE      = 0x00000400; // switch to sample number based capture

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
    char line[80];
    while (debugStream.getline(line, sizeof(line)))
        std::cerr << line << std::endl;
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

bool CollectFileConvert(const char *inFilename, const char *outFilename, std::vector<SignalConfig> &DataFrame, BUF_CONFIG_MAP &configMap)
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
    unsigned int row_idx = 0;
    while (inFile.good()) {
        inFile.read(reinterpret_cast<char *>(&value), sizeof(quadlet_t));
        if (row_idx == 0) {
            outFile << "time stamp"  << ", " << std::dec
                    << "overflow: "  << ((value&0x00004000) >> 14) << ", "    // timer overflow
                    << "timer: "     << (value&0x00003FFF) << ", "    // timer (14-bits)
                    << std::endl;
        } else {
            // TODO: add format mask

            /*<data buffer debug only>*/
            std::cerr << configMap.backward_map.find(DataFrame[row_idx-1].tgt_sig_type)->second << ","
                      << "chan: " << DataFrame[row_idx-1].tgt_sig_chan << ", "
                      << "value = " << std::dec << (value&0xFFFFFFFF) << ", " // data
                      << std::endl;/*<data buffer debug only>*/

            outFile << configMap.backward_map.find(DataFrame[row_idx-1].tgt_sig_type)->second << ","
                    << DataFrame[row_idx-1].tgt_sig_chan << ", "
                    << std::dec << (value&0xFFFFFFFF) << ", " // data
                    << std::endl;
        }
        row_idx = (row_idx == DataFrame.size()) ? 0 : row_idx + 1;
    }
    inFile.close();
    outFile.close();
    return true;
}

void UpdateStatusStrings(char *statusStr1, char *statusStr2, AmpIO_UInt32 statusChanged, AmpIO_UInt32 status)
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
    for (unsigned int i = 0; i < 4; i++) {  // amplifier status
        AmpIO_UInt32 mask = (0x00000100 << i);
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

    std::vector<AmpIO*> BoardList;
    std::vector<AmpIO_UInt32> BoardStatusList;

    // measure time between reads
    AmpIO_UInt32 maxTime = 0;
    AmpIO_UInt32 lastTime = 0;

    unsigned int curAxis = 0;     // Current axis (0=all, 1-8)
    unsigned int curBoardIndex = 0;
    unsigned int curAxisIndex = 0;
    char axisString[4] = "all";
    std::string portDescription;

    for (i = 1; i < (unsigned int)argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'p') {
                portDescription = argv[i]+2;
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
        std::cerr << "Usage: qladisp <board-num> [<board-num>] [-pP] [-b<r|w>] [-v] [-t]" << std::endl
                  << "       where P = port number (default 0)" << std::endl
                  << "                 can also specify -pfw[:P], -peth:P or -pudp[:xx.xx.xx.xx]" << std::endl
                  << "            -br enables broadcast read/write" << std::endl
                  << "            -bw enables broadcast write" << std::endl
                  << "            -v  displays full velocity feedback" << std::endl
                  << "            -t  displays time information" << std::endl
                  << std::endl
                  << "Trying to detect boards on port:" << std::endl;
    }

    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);

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
                  << "'a': axis to address (1-4 or 8) or all (0, default)" << std::endl
                  << "'d': turn watchdog on/off (0.25 ms, this should be triggered immediately)" << std::endl
                  << "'D': turn watchdog on/off (25.0 ms, can be triggered by unplugging cable to PC)" << std::endl
                  << "'p': turn power on/off (board and axis)" << std::endl
                  << "'o': turn power on/off (board only)" << std::endl
                  << "'i': turn power on/off (axis only, requires board first)" << std::endl
                  << "'w': increment encoders" << std::endl
                  << "'s': decrement encoders" << std::endl
                  << "'=': increase motor current by about 50mA" << std::endl
                  << "'-': increase motor current by about 50mA" << std::endl
                  << "'c': start/stop data collection" << std::endl
                  << "'z': clear status events and r/w errors" << std::endl
                  << "'0'-'3': toggle digital output bit" << std::endl;
        return 0;
    }

    if (Port->GetNumOfNodes() == 0) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to find any boards" << std::endl;
        return -1;
    }

    // Check if an Ethernet port
    EthBasePort *ethPort = dynamic_cast<EthBasePort *>(Port);

    for (i = 0; i < BoardList.size(); i++)
        Port->AddBoard(BoardList[i]);

    // Set protocol; default is PROTOCOL_SEQ_RW (not broadcast), but can be changed to
    // one of the broadcast protocols by specifying -br or -bw command line parameter.
    if (protocol == BasePort::PROTOCOL_BC_QRW)
        std::cerr << "Setting protocol to broadcast read/write" << std::endl;
    else if (protocol == BasePort::PROTOCOL_SEQ_R_BC_W)
        std::cerr << "Setting protocol to broadcast write" << std::endl;
    if (!Port->SetProtocol(protocol))
        protocol = Port->GetProtocol();  // on failure, get current protocol

    // Number of boards to display (currently 1 or 2)
    unsigned int numDisp = (BoardList.size() >= 2) ? 2 : 1;

    // Currently hard-coded for up to 2 boards; initialize at mid-range
    AmpIO_UInt32 MotorCurrents[2][4] = { {0x8000, 0x8000, 0x8000, 0x8000 },
                                         {0x8000, 0x8000, 0x8000, 0x8000 }};

    bool allRev7 = true;
    BoardStatusList.clear();
    for (j = 0; j < BoardList.size(); j++) {
        AmpIO_UInt32 fver = BoardList[j]->GetFirmwareVersion();
        if (fver < 7) allRev7 = false;
    }

    for (i = 0; i < 4; i++) {
        for (j = 0; j < numDisp; j++)
            BoardList[j]->WriteEncoderPreload(i, 0x1000*i + 0x1000);
    }
    for (j = 0; j < numDisp; j++) {
        BoardList[j]->WriteSafetyRelay(false);
        BoardList[j]->WritePowerEnable(false);
        BoardList[j]->WriteAmpEnable(0x0f, 0);
        AmpIO_UInt32 bstat = BoardList[j]->ReadStatus();
        BoardStatusList.push_back(bstat);
    }

    bool watchdog_on = false;

    Amp1394Console console;
    if (!console.IsOK()) {
        std::cerr << "Failed to initialize console" << std::endl;
        return -1;
    }

    int board1 = BoardList[0]->GetBoardId();
    if (numDisp > 1) {
        int board2 = BoardList[1]->GetBoardId();
        if (protocol == BasePort::PROTOCOL_BC_QRW)
            console.Print(1, lm, "Sensor Feedback for Boards %d, %d (hub %d)", board1, board2, Port->GetHubBoardId());
        else
            console.Print(1, lm, "Sensor Feedback for Boards %d, %d", board1, board2);
    } else {
        console.Print(1, lm, "Sensor Feedback for Board %d", board1);
    }
    console.Print(2, lm, "Press ESC to quit, r to reset port, 0-3 to toggle digital output bit, p to enable/disable power,");
    console.Print(3, lm, "+/- to increase/decrease commanded current (DAC) by 0x100");

    unsigned int numAxes = (numDisp > 1) ? 8 : 4;
    for (i = 0; i < numAxes; i++) {
        console.Print(5, lm+8+i*13, "Axis %d", i);
    }
    console.Print(6, lm, "Enc:");
    console.Print(7, lm, "Pot:");
    console.Print(8, lm, "Vel:");
    console.Print(9, lm, "Cur:");
    console.Print(10, lm, "DAC:");
    if (fullvel) {
        if (allRev7) {
            console.Print(11, lm, "Qtr1:");
            console.Print(12, lm, "Qtr5:");
            console.Print(13, lm, "Run:");
        }
        else
            console.Print(11, lm, "Acc:");
    }

    console.Refresh();

    // initialize configuration map
    BUF_CONFIG_MAP bufConfigMap;
    // initialize data frame
    std::vector<SignalConfig> DataFrame;

    unsigned char dig_out = 0x0f;
    unsigned char collect_axis = 1;
    AmpIO_UInt32 status;
    AmpIO_UInt32 statusChanged = 0;
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
    const int STATUS_LINE = fullvel ? 16 : 13;
    int timeLines = 0;   // how many lines for timing info
    if (showTime) {
        timeLines++;
        if (protocol == BasePort::PROTOCOL_BC_QRW)
            timeLines++;
    }
    else if (ethPort)
        timeLines++;
    const int DEBUG_START_LINE = fullvel ? (22+timeLines) : (19+timeLines);
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
                curBoardIndex = (curAxis-1)/4;
                curAxisIndex = (curAxis-1)%4;
                sprintf(axisString, "%d", curAxis);
            }
            else {
                curAxis = 0;
                strcpy(axisString, "all");
            }
        }
        else if ((c >= '0') && (c <= '3')) {
            // toggle digital output bit
            dig_out = dig_out^(1<<(c-'0'));
            for (j = startIndex; j < endIndex; j++)
                BoardList[j]->WriteDigitalOutput(0x0f, dig_out);
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
                    // Note that all axes will be disabled when board power is removed
                    if (curAxis == 0)
                        BoardList[j]->SetAmpEnableMask(0x0f, 0x00);
                    else
                        BoardList[j]->SetAmpEnable(curAxisIndex, false);
                    BoardList[j]->SetPowerEnable(false);
                    BoardList[j]->SetSafetyRelay(false);
                }
                else {
                    BoardList[j]->SetSafetyRelay(true);
                    // Cannot enable Amp power unless Board power is
                    // already enabled.
                    BoardList[j]->WritePowerEnable(true);
                    //BoardList[j]->SetPowerEnable(true);
                    if (curAxis == 0)
                        BoardList[j]->SetAmpEnableMask(0x0f, 0x0f);
                    else
                        BoardList[j]->SetAmpEnable(curAxisIndex, true);
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
                for (j = 0; j < numDisp; j++) {
                    if (anyAxisPowered) {
                        BoardList[j]->SetAmpEnableMask(0x0f, 0x00);
                    }
                    else {
                        BoardList[j]->SetAmpEnableMask(0x0f, 0x0f);
                    }
                }
            }
            else {
                bool isPowered = BoardList[curBoardIndex]->GetAmpEnable(curAxisIndex);
                BoardList[curBoardIndex]->SetAmpEnable(curAxisIndex, !isPowered);
            }
        }
        else if (c == '=') {
            if (curAxis == 0) {
                for (j = 0; j < numDisp; j++) {
                    for (i = 0; i < 4; i++)
                        MotorCurrents[j][i] += 0x100;   // 0x100 is about 50 mA
                }
            }
            else {
                MotorCurrents[curBoardIndex][curAxisIndex] += 0x100;
            }
        }
        else if (c == '-') {
            if (curAxis == 0) {
                for (j = 0; j < numDisp; j++) {
                    for (i = 0; i < 4; i++)
                        MotorCurrents[j][i] -= 0x100;   // 0x100 is about 50 mA
                }
            }
            else {
                MotorCurrents[curBoardIndex][curAxisIndex] -= 0x100;
            }
        }
        else if (c == 'c') {
            // MODE_CONTINUOUS: continuously capture data frame
            // NOTE: need to manually stop collection
            for (j = 0; j < numDisp; j++) {
                if (BoardList[j]->IsCollecting()) {
                    if (BoardList[j]->DataCollectionStop()) {
                        console.Print(STATUS_LINE + 21, lm, "Collection stopped.......");
                        isCollecting = false;
                    }
                } else {
                    char fileName[20];
                    sprintf(fileName, "CollectCONT.raw");
                    collectFile.open(fileName, std::ofstream::binary | std::ofstream::trunc);
                    if (!collectFile.good())
                        std::cerr << "Failed to open data collection file " << fileName << std::endl;
                    if (BoardList[j]->DataCollectionStart(BUF_MODE_CONTINUOUS, CollectCB, 0)) {
                        isCollecting = true;
                        console.Print(STATUS_LINE + 21, lm, "Continuously collecting...");
                    }
                }
            }
        }
        else if (c == 'x') {
            // MODE_SAMPLE: capture data frame based on configured sample number
            // NOTE: collection would stop automatically, or be stopped manually
            for (j = 0; j < numDisp; j++) {
                if (BoardList[j]->IsCollecting()) {
                    if (BoardList[j]->DataCollectionStop()) {
                        console.Print(STATUS_LINE + 20, lm, "Collection forced to stop........");
                        isCollecting = false;
                    }
                } else {
                    std::ifstream fsmp;
                    fsmp.open("num_sample.txt");
                    if (fsmp.fail()) {
                        std::cerr << "Failed to find data buffer sample number configuration file..." << std::endl;
                    } else {
                        char fileName[20];
                        sprintf(fileName, "CollectSAMP.raw");
                        collectFile.open(fileName, std::ofstream::binary | std::ofstream::trunc);
                        if (!collectFile.good())
                            std::cerr << "Failed to open data collection file " << fileName << std::endl;
                        double num;
                        fsmp >> num;
                        if (BoardList[j]->DataCollectionStart(BUF_MODE_SAMPLE, CollectCB, static_cast<AmpIO_UInt32>(num))) {
                            isCollecting = true;
                            console.Print(STATUS_LINE + 20, lm, "Collecting by configured sample number... Collection would stop automatically");
                        }
                    }
                    fsmp.close();
                }
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
        }
        else if (c == 'f') {
            // configure data frame to be collected in data buffer
            for (j = 0; j < numDisp; j++) {
                std::ifstream fdbuf;
                fdbuf.open("data_frame.txt");
                if (fdbuf.fail()) {
                    std::cerr << "Failed to find data frame configuration file..." << std::endl;
                } else {
                    // reset data frame
                    DataFrame.clear();
                    // template signal configuration
                    SignalConfig signalConfig;
                    std::string sig_type, sig_chan, sig_fmat;
                    while (fdbuf >> sig_type >> sig_chan >> sig_fmat) {
                        signalConfig.tgt_sig_type = bufConfigMap.forward_map.find(sig_type)->second;
                        signalConfig.tgt_sig_chan = std::stoi(sig_chan);
                        signalConfig.tgt_sig_fmat = bufConfigMap.forward_map.find(sig_fmat)->second;
                        DataFrame.push_back(signalConfig);
                    }
                    if (BoardList[j]->ConfigDataFrame(DataFrame)) {
                        console.Print(STATUS_LINE+30, lm+25, "data frame configuration completed...");
                    }
                }
            }
        }

        /*<data buffer debug only>*/
        for (j = 0; j < numDisp; j++) {
            AmpIO_UInt32 type_data = 0xffffffff;
            AmpIO_UInt32 chan_data = 0xffffffff;
            if (BoardList[j]->ReadBufConfig(type_data, chan_data)) {
                console.Print(STATUS_LINE + 31, lm + 25, "type config = %08X", type_data);
                console.Print(STATUS_LINE + 32, lm + 25, "chan config = %08X", chan_data);
            }
        }/*<data buffer debug only>*/

        /*<data buffer debug only>*/
        for (j = 0; j < numDisp; j++) {
            bool fpgaCollecting;
            unsigned char fpgaCollectNum;
            unsigned short collect_windex;
            unsigned short lreadAddr;
            BoardList[j]->ReadCollectionStatus(fpgaCollecting, fpgaCollectNum, collect_windex, lreadAddr);
            console.Print(STATUS_LINE+34, lm+25, "lreadAddr = %d", lreadAddr);
            console.Print(STATUS_LINE+35, lm+25, "collect_windex = %d", collect_windex);
            console.Print(STATUS_LINE+36, lm+25, "fpgaCollectNum = %d", fpgaCollectNum);
            int tmp_is_Collecting = 0;
            if (fpgaCollecting) {
                tmp_is_Collecting = 1;
            } else {
                tmp_is_Collecting = 0;
                isCollecting = false;
            }
            console.Print(STATUS_LINE+37, lm+25, "fpgaCollect = %d", tmp_is_Collecting);
            if (isCollecting) {
                console.Print(STATUS_LINE+38, lm+25, "pcCollect = %d", 1);
            } else {
                console.Print(STATUS_LINE+38, lm+25, "pcCollect = %d", 0);
            }
        }/*<data buffer debug only>*/

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

        AmpIO::EncoderVelocityData encVelData;
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
        for (j = 0; j < numDisp; j++) {
            if (BoardList[j]->ValidRead()) {
                for (i = 0; i < 4; i++) {
                    console.Print(6, lm+7+(i+4*j)*13, "%07X",
                                  BoardList[j]->GetEncoderPosition(i)+BoardList[j]->GetEncoderMidRange());
                    console.Print(7, lm+10+(i+4*j)*13, "%04X", BoardList[j]->GetAnalogInput(i));
                    if (fullvel)
                        console.Print(8, lm+6+(i+4*j)*13, "%08X", BoardList[j]->GetEncoderVelocityRaw(i));
                    else {
                        BoardList[j]->GetEncoderVelocityData(i, encVelData);
                        console.Print(8, lm+6+(i+4*j)*13, "%08X", encVelData.velPeriod);
                    }
                    console.Print(9, lm+10+(i+4*j)*13, "%04X", BoardList[j]->GetMotorCurrent(i));
                    if (fullvel) {
                        if (allRev7) {
                            console.Print(11, lm+6+(i+4*j)*13, "%08X", BoardList[j]->GetEncoderQtr1Raw(i));
                            console.Print(12, lm+6+(i+4*j)*13, "%08X", BoardList[j]->GetEncoderQtr5Raw(i));
                            console.Print(13, lm+6+(i+4*j)*13, "%08X", BoardList[j]->GetEncoderRunningCounterRaw(i));
                        }
                        else
                            console.Print(11, lm+6+(i+4*j)*13, "%08X", BoardList[j]->GetEncoderAccelerationRaw(i));
                    }
                }
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
                console.Print(STATUS_LINE, lm+58*j, "Status: %08X   Timestamp: %08X   DigOut: %01X",
                              status, BoardList[j]->GetTimestamp(),
                              (unsigned int)dig_out);
                console.Print(STATUS_LINE+1, lm+58*j, "%17s  NegLim: %01X  PosLim: %01X  Home: %01X",
                              statusStr1[j],
                              BoardList[j]->GetNegativeLimitSwitches(),
                              BoardList[j]->GetPositiveLimitSwitches(),
                              BoardList[j]->GetHomeSwitches());
                console.Print(STATUS_LINE+2, lm+58*j, "%17s  EncA: %01X    EncB: %01X    EncI: %01X",
                              statusStr2[j],
                              BoardList[j]->GetEncoderChannelA(),
                              BoardList[j]->GetEncoderChannelB(),
                              BoardList[j]->GetEncoderIndex());

                console.Print(STATUS_LINE+4, lm+58*j, "Node: %s", nodeStr[j]);
                console.Print(STATUS_LINE+4, lm+14+58*j, "Temp:  %02X    %02X",
                              (unsigned int)BoardList[j]->GetAmpTemperature(0),
                              (unsigned int)BoardList[j]->GetAmpTemperature(1));
                if (loop_cnt > 500) {
                    lastTime = BoardList[j]->GetTimestamp();
                    if (lastTime > maxTime) {
                        maxTime = lastTime;
                    }
                }
            }
            console.Print(STATUS_LINE+4, lm+35+58*j, "Err(r/w): %2d %2d",
                          BoardList[j]->GetReadErrors(),
                          BoardList[j]->GetWriteErrors());
            if (showTime)
                console.Print(STATUS_LINE+5, lm+20+58*j, "%9.3lf (%7.4lf)", BoardList[j]->GetFirmwareTime(),
                              pcTime-BoardList[j]->GetFirmwareTime());
            for (i = 0; i < 4; i++) {
                console.Print(10, lm+10+(i+4*j)*13, "%04X", MotorCurrents[j][i]);
                BoardList[j]->SetMotorCurrent(i, MotorCurrents[j][i]);
            }
            loop_cnt++;
        }
        if (showTime) {
            console.Print(STATUS_LINE+5, lm+53, "%8.3lf", pcTime);
            if (protocol == BasePort::PROTOCOL_BC_QRW) {
                BasePort::BroadcastReadInfo bcReadInfo;
                bcReadInfo = Port->GetBroadcastReadInfo();
                std::stringstream timingStr;
                bcReadInfo.PrintTiming(timingStr, false);  // false --> no std::endl
                timingStr << "   ";
                console.Print(STATUS_LINE+6, lm, timingStr.str().c_str());
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
        Amp1394_Sleep(0.0005);  // 500 usec
    }

    for (j = 0; j < numDisp; j++) {
        BoardList[j]->WritePowerEnable(false);      // Turn power off
        BoardList[j]->WriteAmpEnable(0x0f, 0x00);   // Turn power off
        BoardList[j]->WriteSafetyRelay(false);
    }

    // Reset encoder preloads to default
    for (i = 0; i < 4; i++) {
        for (j = 0; j < numDisp; j++)
            BoardList[j]->WriteEncoderPreload(i, 0);
    }

    console.End();

    for (j = 0; j < BoardList.size(); j++)
        Port->RemoveBoard(BoardList[j]->GetBoardId());

    delete Port;

    // Process any data collection files (convert from binary to text)
    // NOTE: maximum number of collected data file is 2 for struct data buffer
    char inFile[20];
    char outFile[20];
    /*<convert mode continuous output if there is any>*/
    sprintf(inFile,  "CollectCONT.raw", j);
    sprintf(outFile, "CollectCONT.csv", j);
    CollectFileConvert(inFile, outFile, DataFrame, bufConfigMap);
    /*<convert mode sample output if there is any>*/
    sprintf(inFile,  "CollectSAMP.raw", j);
    sprintf(outFile, "CollectSAMP.csv", j);
    CollectFileConvert(inFile, outFile, DataFrame, bufConfigMap);

    return 0;
}
