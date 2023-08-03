/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * Usage: dvrktest [-pP] <controller sn> <board num> [<board num2>]
 *        where P is the Firewire port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *
 ******************************************************************************/

//#include <stdlib.h>
//#include <unistd.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <thread>
#include <array>
#include <vector>

#include <Amp1394/AmpIORevision.h>

#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "EthRawPort.h"
#endif
#include "EthUdpPort.h"

#include "AmpIO.h"
#include "MotorVoltage.h"

enum Result {
    pass, fail, fatal_fail
};

int NUM_FPGA = 1;
int NUM_CHANNEL_PER_FPGA = 8;
bool is_dqla = true;

char* controller_sn;

//**************************** Find dVRK Controllers **************************************

struct ControllerInfo {
    enum ControllerType    { MTML, MTMR, PSM1, PSM2, PSM3, ECM, SUJ, UNSUPPORTED };

    unsigned int board_id;     // Base FPGA board id (first id if more than one FPGA board)
    ControllerType armType;    // Arm type (e.g., MTML, PSM1), based on board id setting
    unsigned long  hwVersion;  // Hardware version (e.g., QLA, DQLA, dRAC)
};

std::string ControllerTypeString[] = { "MTML", "MTMR", "PSM1", "PSM2", "PSM3", "ECM", "SUJ", "UNSUPPORTED" };

ControllerInfo::ControllerType BoardIdMap[16] = {
                   ControllerInfo::MTML, ControllerInfo::MTML,   // 0,1
                   ControllerInfo::MTMR, ControllerInfo::MTMR,   // 2,3
                   ControllerInfo::ECM,  ControllerInfo::ECM,    // 4,5
                   ControllerInfo::PSM1, ControllerInfo::PSM1,   // 6,7
                   ControllerInfo::PSM2, ControllerInfo::PSM2,   // 8,9
                   ControllerInfo::PSM3, ControllerInfo::PSM3,   // 10,11
                   ControllerInfo::SUJ,  ControllerInfo::UNSUPPORTED,        // 12,13
                   ControllerInfo::UNSUPPORTED, ControllerInfo::UNSUPPORTED  // 14,15
               };

double ControllerMotorSupplyVoltage[16] = {
                   24.0, 12.0,   // MTML
                   24.0, 12.0,   // MTMR
                   36.0, 36.0,   // ECM
                   24.0, 24.0,   // PSM1
                   24.0, 24.0,   // PSM2
                   24.0, 24.0,   // PSM3
                   48.0,  0.0,   // SUJ
                    0.0,  0.0
               };

// Return vector of all dVRK Controllers on the specified port
std::vector<ControllerInfo> GetControllerList(const BasePort *port)
{
    std::vector<ControllerInfo> infoList;
    ControllerInfo info;

    for (unsigned int i = 0; i < BoardIO::MAX_BOARDS; i++) {
        info.board_id = i;
        info.hwVersion = port->GetHardwareVersion(i);
        info.armType = ControllerInfo::UNSUPPORTED;
        if (info.hwVersion == dRA1_String) {
            // dRAC setup uses even numbers; for now an odd number is UNSUPPORTED
            info.armType = (i&1) ? ControllerInfo::UNSUPPORTED : BoardIdMap[i];
            // Also, dRAC does not currently support MTMs or (separate) SUJ
            if ((info.armType == ControllerInfo::MTML) || (info.armType == ControllerInfo::MTMR) ||
                (info.armType == ControllerInfo::SUJ))
                info.armType = ControllerInfo::UNSUPPORTED;
        }
        else if (info.hwVersion == DQLA_String) {
            // DQLA uses even numbers; for now, an odd number is UNSUPPORTED
            info.armType = (i&1) ? ControllerInfo::UNSUPPORTED : BoardIdMap[i];
            // Also, DQLA does not support SUJ
            if (info.armType == ControllerInfo::SUJ)
                info.armType = ControllerInfo::UNSUPPORTED;
        }
        else if (info.hwVersion == QLA1_String) {
            // QLA setup requires 2 boards, except for SUJ. First QLA must have an
            // even board id
            info.armType = BoardIdMap[i];
            if (info.armType != ControllerInfo::SUJ) {
                if (!(i&1)) {  // even board id
                    unsigned long hver2 = port->GetHardwareVersion(i+1);
                    if (hver2 == QLA1_String) {
                        i++;  // skip next board
                    }
                    else {    // next board not QLA
                        info.armType = ControllerInfo::UNSUPPORTED;
                    }
                }
                else {  // odd board id
                    info.armType = ControllerInfo::UNSUPPORTED;
                }
            }
        }
        if (info.hwVersion != 0)
            infoList.push_back(info);
    }
    return infoList;
}

//******************************** Test Functions *****************************************

const unsigned long POT_TEST_ADC_COUNT[8] = {0x4000, 0x4000, 0x4000, 0x4000, 0x8000, 0x8000, 0x8000, 0x8000};
const unsigned long POT_TEST_ADC_ERROR_TOLERANCE = 0x500;

std::array<uint8_t, 3> DIGITAL_IN_TEST_LOW_STATE{0x08, 0x03, 0x07};
std::array<uint8_t, 3> DIGITAL_IN_TEST_HIGH_STATE{0x0f, 0x0f, 0x0f};
const unsigned long DIGITAL_IN_TEST_WAIT_TIME_MS = 1000;

const unsigned long ENCODER_TEST_WAIT_TIME_MS = 1000;
const unsigned long ENCODER_TEST_INCREMENT = 10;
const unsigned long ENCODER_TEST_INCREMENT_TOLERANCE = 2;

const unsigned long POWER_AMP_TEST_DAC = 0x8500;
const unsigned long POWER_AMP_TEST_TOLERANCE = 0x200;

const auto PASS = " ... \033[0;32m[PASS]\033[0m";
const auto FAIL = " ... \033[1;91m[FAIL]\033[0m";

const std::string COLOR_OFF = "\033[0m";
const std::string COLOR_ERROR = "\033[0;91m";
const std::string COLOR_GOOD = "\033[0;32m";

Result TestSerialNumberAndVersion(AmpIO **Board, BasePort *Port) {
    Result result = pass;
    if (Port->ReadAllBoards()) {
        for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
            std::cout << "Board " << board_index << " - ";
            std::string FPGA_SN = Board[board_index]->GetFPGASerialNumber();
            std::cout << "FPGA_sn=" << FPGA_SN << " ";
            std::string HW_STRING = Board[board_index]->GetHardwareVersionString();
            std::cout << "HW_ver=" << HW_STRING << " ";
            if (is_dqla) {
                std::string QLA1_SN = Board[board_index]->GetQLASerialNumber(1);
                std::cout << "QLA1_sn=" << QLA1_SN << " ";
                std::string QLA2_SN = Board[board_index]->GetQLASerialNumber(2);
                std::cout << "QLA2_sn=" << QLA2_SN << " ";
            } else {
                std::string QLA_SN = Board[board_index]->GetQLASerialNumber();
                std::cout << "QLA_sn=" << QLA_SN << " ";
            }
            uint32_t FPGA_VER = Board[board_index]->GetFirmwareVersion();
            std::cout << "FPGA_ver=" << FPGA_VER << std::endl;
            uint32_t hwver = Board[board_index]->GetHardwareVersion();
            if (hwver == dRA1_String || hwver == BCFG_String) {
                std::cout << COLOR_ERROR << "(Unexpected board type.)" << COLOR_OFF << std::endl;
                result = fatal_fail;
                break;
            }
        }
    } else {
        std::cout << COLOR_ERROR << "(Can't talk to controller.)" << COLOR_OFF
                  << std::endl;
        result = fatal_fail;
    }
    return result;
}

Result TestAnalogInputs(AmpIO **Board, BasePort *Port) {
    Result result = pass;
    if (Port->ReadAllBoards()) {
        for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
            for (int channel_index = 0; channel_index < NUM_CHANNEL_PER_FPGA; channel_index++) {
                unsigned long reading = Board[board_index]->GetAnalogInput(channel_index);
                std::cout << std::hex << "board " << board_index << " pot channel " << channel_index << " - "
                          << " expected="
                          << POT_TEST_ADC_COUNT[board_index * 4 + channel_index] << " measured=" << reading;
                if (std::fabs((long) reading - (long) POT_TEST_ADC_COUNT[board_index * 4 + channel_index]) > POT_TEST_ADC_ERROR_TOLERANCE) {
                    std::cout << FAIL << std::endl;
                    result = fail;
                    // check for swapped cable
                    if (std::fabs((long) reading - (long) POT_TEST_ADC_COUNT[(1 ^ board_index)  * 4 + channel_index]) <
                        POT_TEST_ADC_ERROR_TOLERANCE) {
                        std::cout << COLOR_ERROR << "(Looks like the 68-pin cable is crossed!)" << COLOR_OFF
                                  << std::endl;
                        result = fatal_fail;
                    }

                } else {
                    std::cout << PASS << std::endl;
                }
            }
        }
    }
    return result;
}

Result TestDigitalInputs(AmpIO **Board, BasePort *Port) {
    Result result = pass;
    std::array<uint8_t, 3> before;
    std::array<uint8_t, 3> after;
    Port->ReadAllBoards();
    before[0] = Board[0]->GetHomeSwitches() & 0xf;
    if (is_dqla) {
        before[1] = Board[0]->GetNegativeLimitSwitches() >> 4;
        before[2] = Board[0]->GetPositiveLimitSwitches() >> 4;
    } else {
        before[1] = Board[1]->GetNegativeLimitSwitches();
        before[2] = Board[1]->GetPositiveLimitSwitches();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(DIGITAL_IN_TEST_WAIT_TIME_MS));
    Port->ReadAllBoards();
    after[0] = Board[0]->GetHomeSwitches() & 0xf;
    if (is_dqla) {
        after[1] = Board[0]->GetNegativeLimitSwitches() >> 4;
        after[2] = Board[0]->GetPositiveLimitSwitches() >> 4;
    } else {
        after[1] = Board[1]->GetNegativeLimitSwitches();
        after[2] = Board[1]->GetPositiveLimitSwitches();
    }

    std::cout << "digital inputs - ";

    for (int i = 0; i < 3; i++) {
        std::cout << std::hex << (int) before[i] << "->" << (int) after[i] << " ";
    }

    if ((before == DIGITAL_IN_TEST_HIGH_STATE && after == DIGITAL_IN_TEST_LOW_STATE) ||
        (before == DIGITAL_IN_TEST_LOW_STATE && after == DIGITAL_IN_TEST_HIGH_STATE)) {
        std::cout << PASS << std::endl;
    } else {
        result = fail;
        std::cout << FAIL << std::endl;
        std::cout << COLOR_ERROR << "(False-positive failure is possible. Consider it pass if retry OK.)" << COLOR_OFF
                  << std::endl;
    }

    return result;

}

Result TestEncoders(AmpIO **Board, BasePort *Port) {
    Result result = pass;

    int64_t encoder_count_before[8];
    int64_t encoder_count_after[8];

    if (Port->ReadAllBoards()) {
        for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
            for (int channel_index = 0; channel_index < NUM_CHANNEL_PER_FPGA; channel_index++) {
                encoder_count_before[board_index * 4 + channel_index] = Board[board_index]->GetEncoderPosition(
                        channel_index);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(ENCODER_TEST_WAIT_TIME_MS));

        if (Port->ReadAllBoards()) {
            for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
                for (int channel_index = 0; channel_index < NUM_CHANNEL_PER_FPGA; channel_index++) {
                    encoder_count_after[board_index * 4 + channel_index] = Board[board_index]->GetEncoderPosition(
                            channel_index);
                }
            }
        }
    }

    uint8_t each_encoder_result = 0;

    for (int i = 0; i < 7; i++) {
        auto increment = encoder_count_after[i] - encoder_count_before[i];

        std::cout << std::dec << "encoder " << i << " increment - expected=10 measured=" << increment;
        if (std::fabs((long) increment - (long) ENCODER_TEST_INCREMENT) < ENCODER_TEST_INCREMENT_TOLERANCE) {
            std::cout << PASS << std::endl;
            each_encoder_result |= 1 << i;
        } else {
            std::cout << FAIL << std::endl;
            result = fatal_fail;
        }
    }

    if (each_encoder_result == 0b0001111 || each_encoder_result == 0b1110000 || each_encoder_result == 0) {
        std::cout << COLOR_ERROR << "(Check SCSI (68-pin) cables!)" << COLOR_OFF <<
                  std::endl;
        result = fatal_fail;
    }

    if (result == fatal_fail) {
        std::cout << COLOR_ERROR << "(Skipped motor tests due to failed encoder test.)" << COLOR_OFF <<
                  std::endl;
    }

    return result;
}

Result TestMotorPowerControl(AmpIO **Board, BasePort *Port) {

    Result result = pass;
    int mv_good_fail_count = 0;

    for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
        Board[board_index]->WriteSafetyRelay(true);
        Board[board_index]->WriteWatchdogPeriod(0x0000);
        Board[board_index]->WritePowerEnable(true);
        Board[board_index]->WriteAmpEnable(0xff, 0xff);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    Port->ReadAllBoards();

    for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
        auto status = Board[board_index]->GetStatus();
        auto safety_relay_status = Board[board_index]->GetSafetyRelayStatus();
        std::cout << std::hex << "board " << board_index << " power - status=" << status << " safety_relay="
                  << safety_relay_status;

        if (safety_relay_status) {
            std::cout << std::endl << "    ";
            if (Board[board_index]->GetPowerStatus()) {
                std::cout << " mv_good=ok";
                bool board_failed = false;
                std::cout << " amp_power=";

                for (int channel_index = 0; channel_index < NUM_CHANNEL_PER_FPGA; channel_index++) {
                    if (Board[board_index]->GetAmpStatus(channel_index)) {
                        std::cout << 1;
                    } else {
                        std::cout << 0;
                        board_failed = true;
                    }

                }
                if (board_failed) {
                    std::cout << " amp_power=bad";
                    result = fatal_fail;
                    std::cout << FAIL << std::endl;
                } else {
                    std::cout << PASS << std::endl;
                }
            } else {
                std::cout << " mv_good=bad";
                mv_good_fail_count++;
                result = fatal_fail;
                std::cout << FAIL << std::endl;
            }
        } else {
            result = fatal_fail;
            std::cout << FAIL << std::endl;
        }
    }


    if (mv_good_fail_count == NUM_FPGA) {
        // When both boards don't have mv_good
        std::cout << COLOR_ERROR << "(None of the board has motor power. Is the safety chain open?)" << COLOR_OFF <<
                  std::endl;
    }

    return result;
}

Result TestPowerAmplifier(AmpIO **Board, BasePort *Port) {
    int crossed_db9_error_count = 0;
    Result result = pass;

    for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
        for (int channel_index = 0; channel_index < NUM_CHANNEL_PER_FPGA; channel_index++) {
            if (board_index == 1 && channel_index == 3) break;
            if (board_index == 0 && channel_index == 7) break;

            Port->ReadAllBoards();

            if (std::fabs((long) Board[0]->GetAnalogInput(0) - (long) POT_TEST_ADC_COUNT[0] / 2) <
                POT_TEST_ADC_ERROR_TOLERANCE) {
                std::cout << COLOR_ERROR << "(unexpected current detected on channel 0)" << COLOR_OFF << std::endl;
                return fatal_fail;
            }

            Board[board_index]->SetMotorCurrent(channel_index, POWER_AMP_TEST_DAC);
            Port->WriteAllBoards();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            Port->ReadAllBoards();
            auto motor_current = Board[board_index]->GetMotorCurrent(channel_index);
            bool amp_working = std::fabs((long) motor_current - (long) POWER_AMP_TEST_DAC) <
                               POWER_AMP_TEST_TOLERANCE;
            bool channel_0_load_driven =
                    std::fabs((long) Board[0]->GetAnalogInput(0) - (long) POT_TEST_ADC_COUNT[0] / 2) <
                    POT_TEST_ADC_ERROR_TOLERANCE;

            if (board_index == 0 && channel_index == 0 && amp_working && !channel_0_load_driven) {
                crossed_db9_error_count++;
            }

            if (board_index == 1 && channel_index == 0 && amp_working && channel_0_load_driven) {
                crossed_db9_error_count++;
            }

            if (board_index == 0 && channel_index == 4 && amp_working && channel_0_load_driven) {
                crossed_db9_error_count++;
            }

            std::cout << std::hex << "board " << board_index << " amp channel " << channel_index << " - "
                      << " expected="
                      << POWER_AMP_TEST_DAC << " measured=" << motor_current;

            if (!amp_working) {
                result = fail;
                std::cout << FAIL << std::endl;
            } else {
                std::cout << PASS << std::endl;
            }

            Board[board_index]->SetMotorCurrent(channel_index, 0x8000);
            Port->WriteAllBoards();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));


        }
    }
    if (crossed_db9_error_count == 2) {
        result = fatal_fail;
        std::cout << COLOR_ERROR << "(DB9 cables are likely crossed!)" << COLOR_OFF << std::endl;
    }

    if (result == fail) {
        std::cout << COLOR_ERROR << "(Are the DB9 cables plugged in?)" << COLOR_OFF << std::endl;
    }

    return result;

}

Result TestDallas(AmpIO **Board, BasePort *Port) {
    std::cout << "dallas ";
    AmpIO *board;
    if (is_dqla) {
        board = Board[0];
    } else {
        board = Board[1];
    }
    unsigned char buffer[2048];
    bool ret = board->DallasReadMemory(0, (unsigned char *) buffer, sizeof(buffer));
    if (!ret) {
        std::cout << FAIL << std::endl;
        std::cout << COLOR_ERROR << "(DallasReadMemory failed)" << COLOR_OFF << std::endl;
        return fail;
    } else {
        std::cout << PASS << std::endl;
        return pass;
    }
}

//*********************************** Main ********************************************

void PrintDebugStream(std::stringstream &debugStream)
{
    std::cerr << debugStream.str() << std::endl;
    debugStream.clear();
    debugStream.str("");
}

int main(int argc, char **argv)
{
#if Amp1394_HAS_RAW1394
    BasePort::PortType desiredPort = BasePort::PORT_FIREWIRE;
#else
    BasePort::PortType desiredPort = BasePort::PORT_ETH_UDP;
#endif
    int port = 0;
    int board1 = BoardIO::MAX_BOARDS;
    int board2 = BoardIO::MAX_BOARDS;
    std::string IPaddr(ETH_UDP_DEFAULT_IP);

    int args_found = 0;
    for (int i = 1; i < argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            // -p option can be -pN, -pfwN, or -pethN, where N
            // is the port number. -pN is equivalent to -pfwN
            // for backward compatibility.
            if (!BasePort::ParseOptions(argv[i]+2, desiredPort, port, IPaddr)) {
                std::cerr << "Failed to parse option: " << argv[i] << std::endl;
                return 0;
            }
            std::cerr << "Selected port: " << BasePort::PortTypeString(desiredPort) << std::endl;
        } else {
            if (args_found == 0) {
                board1 = atoi(argv[i]);
            } else if (args_found == 1) {
                board2 = atoi(argv[i]);
            }
            args_found++;
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

    // Now that Port is initialized, look for dVRK Controllers
    std::vector<ControllerInfo> ControllerList = GetControllerList(Port);
    if (ControllerList.size() > 0) {
        std::cout << "Controllers on " << BasePort::PortTypeString(desiredPort) << ":" << std::endl;
        std::vector<ControllerInfo>::const_iterator it;
        for (it = ControllerList.begin(); it != ControllerList.end(); it++) {
            std::cout << "   " << ControllerTypeString[it->armType] << ", " << Port->GetHardwareVersionString(it->board_id);
            if (it->hwVersion == QLA1_String)
                std::cout << ", boards " << it->board_id << ", " << it->board_id+1 << std::endl;
            else
                std::cout << ", board " << it->board_id << std::endl;
        }
        std::cout << std::endl;
    }
    else {
        std::cout << "No controllers found on " << BasePort::PortTypeString(desiredPort) << ", exiting" << std::endl;
        return -1;
    }

    if (ControllerList.size() != 1) {
        std::cout << "More than one controller found, please remove all controllers except unit under test" << std::endl;
        return -1;
    }

    if (ControllerList[0].hwVersion == dRA1_String) {
        std::cout << "dVRK-Si Controller detected -- this program is only for testing a dVRK (Classic) controller" << std::endl;
        return -1;
    }

    if (ControllerList[0].hwVersion == BCFG_String) {
        std::cout << "Controller boot error -- please confirm that the MicroSD card is inserted into the controller" << std::endl;
        return -1;
    }

    if ((ControllerList[0].hwVersion != QLA1_String) && (ControllerList[0].hwVersion != DQLA_String)) {
        // Should not happen, since not dRA1_String or BCFG_String, but checking just in case
        std::cout << "Unexpected controller hardware: " << std::hex << ControllerList[0].hwVersion << std::dec << std::endl;
        return -1;
    }

    std::cout << "*********************** WARNING **************************" << std::endl;
    std::cout << "This program tests dVRK Classic controllers using a special test board." << std::endl;
    std::cout << "Running the test with a robot (PSM/MTM) can cause injury and damage the robot." << std::endl;
    std::cout << "Do not use this program with a controller connected to a robot." << std::endl;
    std::cout << "Press [Enter] to run the test. Press [Ctrl-C] to exit." << std::endl;
    std::cin.ignore();

    // If user did not specify board id, use the one obtained by scanning the bus.
    // Perhaps we will remove ability to specify board id(s) on command line.
    if (board1 == BoardIO::MAX_BOARDS) {
        board1 = ControllerList[0].board_id;
        if (ControllerList[0].hwVersion == QLA1_String) {
            board2 = board1+1;
            is_dqla = false;
            NUM_FPGA = 2;
            NUM_CHANNEL_PER_FPGA = 4;
        }
    }

    std::vector<AmpIO *> BoardList;
    BoardList.push_back(new AmpIO(board1));
    Port->AddBoard(BoardList[0]);
    if (!is_dqla) {
        BoardList.push_back(new AmpIO(board2));
        Port->AddBoard(BoardList[1]);
    }

    // Check motor supply voltage
    double V1 = 0.0;   // QLA 1 measured motor supply voltage
    double V2 = 0.0;   // QLA 2 measured motor supply voltage
    if (is_dqla) {
        V1 = MeasureMotorSupplyVoltage(Port, BoardList[0], 1);
        V2 = MeasureMotorSupplyVoltage(Port, BoardList[0], 2);
    }
    else {
        V1 = MeasureMotorSupplyVoltage(Port, BoardList[0]);
        V2 = MeasureMotorSupplyVoltage(Port, BoardList[1]);
    }
    // Nominal motor supply voltages
    double V1Nom = ControllerMotorSupplyVoltage[board1];
    double V2Nom = ControllerMotorSupplyVoltage[board1+1];
    // Check that measured supply voltage not more than 10% different from nominal voltage
    if ((fabs(V1-V1Nom) > 0.1*V1Nom) || (fabs(V2-V2Nom) > 0.1*V2Nom)) {
        std::cout << "WARNING: Motor supply voltages do not match controller type" << std::endl;
        std::cout << "   V1 = " << V1 << " (should be " << V1Nom << "), V2 = " << V2
                  << " (should be " << V2Nom << ")" << std::endl;
        // Could add prompt whether to continue or exit
    }

    for (int i = 0; i < NUM_CHANNEL_PER_FPGA; i++) {
        for (auto &j : BoardList)
            j->WriteEncoderPreload(i, 0x1000 * i + 0x1000);
    }

    std::cout << "Board ID selected: Board 0=" << board1;
    if (!is_dqla) std::cout << " Board 1=" << board2;
    std::cout << std::endl;


    bool pass = true;

    std::vector<Result (*)(AmpIO **, BasePort *)> test_functions;
    test_functions.push_back(TestSerialNumberAndVersion);
    test_functions.push_back(TestEncoders);
    test_functions.push_back(TestAnalogInputs);
    test_functions.push_back(TestDigitalInputs);
    test_functions.push_back(TestDallas);
    test_functions.push_back(TestMotorPowerControl);
    test_functions.push_back(TestPowerAmplifier);

    for (auto test_function : test_functions) {
        auto result = test_function(&BoardList[0], Port);
        std::cout << std::endl;
        if (result == fatal_fail) {
            pass = false;
            break;
        } else if (result == fail) {
            pass = false;
        }
    }

    if (pass) {
        std::cout << COLOR_GOOD << "PASS" << COLOR_OFF << ": all tests passed." <<std::endl;
    } else {
        std::cout << COLOR_ERROR << "FAIL" << COLOR_OFF << ": one or more tests failed." << std::endl;
    }


    for (auto &j : BoardList) {
        j->WritePowerEnable(false);
        j->WriteSafetyRelay(false);
        Port->RemoveBoard(j);
    }

    return pass ? 0 : -1;

}
