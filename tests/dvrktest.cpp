/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/******************************************************************************
 *
 * Usage: dvrktest [-pP]
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
#include <string>
#include <chrono>
#include <thread>
#include <array>
#include <vector>
#include <iomanip>

#ifdef _MSC_VER   // Windows
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include <Amp1394/AmpIORevision.h>
#include "PortFactory.h"
#include "AmpIO.h"
#include "MotorVoltage.h"

enum Result {
    pass, fail, fatal_fail
};

int NUM_FPGA = 1;
int NUM_CHANNEL_PER_FPGA = 8;
bool is_dqla = true;

std::ofstream logfile;
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


/****************************************************************
*   @brief: This function logs debug information to console and logfile simultaneously.
*  
*   @details: A variadic template function that accepts any number of streamable 
*   arguments, combines them into a single string, and outputs the result to both 
*   standard output and the logfile with a trailing newline.
*
*   @param Args (&&... args): Variable number of arguments of any streamable type
*                             that can be inserted into an output stream with the
*                             << operator
*
*   @return none
****************************************************************/
template<typename... Args>
void logDebugInfo(Args&&... args) {
    std::stringstream ss;
    
    (void) std::initializer_list<int> { (ss << std::forward<Args>(args), 0)...};
    
    std::cout << ss.str();
    logfile << ss.str();
}

/****************************************************************
*   @brief: This function logs a passing test result to console and logfile.
*  
*   @details: Outputs a standardized PASS message to standard output and 
*   logs a corresponding pass message to the logfile.
*   @return none
****************************************************************/
void logPass() {
    std::cout << PASS << std::endl;
    logfile << "... PASS" << std::endl;
}

/****************************************************************
*   @brief: This function logs a failing test result to console and logfile.
*  
*   @details: Outputs a standardized FAIL message to standard output, logs a 
*   corresponding fail message to the logfile, and sets the result parameter 
*   to the specified failure type.
*
*   @param result (Result &): Reference to the result variable to be updated
*   @param resultType (Result): Type of failure (fail or fatal_fail) to assign
*
*   @return none
****************************************************************/
void logFail(Result &result, Result resultType) {
    std::cout << FAIL << std::endl;
    logfile << "... FAIL" << std::endl;
    result = resultType;
}


/****************************************************************
*   @brief: Function to test FPGA and QLA serial numbers and test FPGA and board hardware 
*   versions and firmware version of each board in the dVRK.
*
*   @details: This function outputs the serial number and hardware version of the FPGAs 
*   and the QLAs on each board as well as the firmware version and hardware version of 
*   each board to standard out and the logfile.
*
*   @param AmpIO  (**Board): Array of board objects to have this function performed on
*   @param BasePort (*Port): Communication interface object that provides access to the 
*                            physical connection to the controller boards     
*
*   @return testResult (Result): result object indicating pass, fail, or fatal_fail 
*                                status of the test (see note below)
*   
*   @note: This functions' return does not indicate success or failure for the FPGA, QLA, 
*   or board. It indicates if there is an unexpected board type or inability to communicate
*   with the board.
*   @note: This test is for classic dVRK controllers. If tested on a non classic one, 
*   the function will return a result of fatal failure. Additionally, if unable to read
*   from the board, it will also return a fatal failure.
****************************************************************/
Result TestSerialNumberAndVersion(AmpIO **Board, BasePort *Port) {
    Result result = pass;
    if (Port->ReadAllBoards()) {
        for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
            // Read Board Number, FGPA serial number and hardware version
            logDebugInfo("Board ", std::to_string(board_index), " - ");
            std::string FPGA_SN = Board[board_index]->GetFPGASerialNumber();
            logDebugInfo("FPGA_sn=", FPGA_SN, " ");
            std::string HW_STRING = Board[board_index]->GetHardwareVersionString();
            logDebugInfo("HW_ver=", HW_STRING, " ");
            std::cout << std::endl;

            // Read QLA hardware version
            if (is_dqla) {
                std::string QLA1_SN = Board[board_index]->GetQLASerialNumber(1);
                logDebugInfo("QLA1_sn=", QLA1_SN, " ");
                std::string QLA2_SN = Board[board_index]->GetQLASerialNumber(2);
                logDebugInfo("QLA2_sn=", QLA2_SN, " ");
            } else {
                std::string QLA_SN = Board[board_index]->GetQLASerialNumber();
                logDebugInfo("QLA_sn=", QLA_SN, " ");
            }

            // Read FPGA firmware version
            uint32_t FPGA_VER = Board[board_index]->GetFirmwareVersion();
            logDebugInfo("FPGA_ver=", FPGA_VER, "");
            std::cout << std::endl;
            logfile << std::endl;

            // Check for unsupported board type
            uint32_t hwver = Board[board_index]->GetHardwareVersion();
            logfile << "HW_ver=" << Board[board_index]->GetHardwareVersionString() << std::endl;
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

/****************************************************************
*   @brief: Function to test analog inputs of the dVRK.
*
*   @details: This function tests each channel on each FPGA board to ensure the analog
*   inputs (potentiometers that measure the positions of the robotic arms) from the dVRK 
*   have their expected values (within a specified tolerance) when the motors are adjusted.
*   
*   @param AmpIO  (**Board): Array of board objects to have this function performed on
*   @param BasePort (*Port): Communication interface object that provides access to the 
*                            physical connection to the controller boards     
*
*   @return testResult (Result): result object indicating pass, fail, or fatal_fail 
*                                status of the test
*
*   @note: This function outputs debug information to standard out and the logfile; it
*   outputs the board, potentiometer channel, the expected and actual measured inputs.
*   @note: This test also outputs debugging info if one of the measured potentiometer
*   values match the expected value for a different potentiometer. It output that there
*   a DB68 SCSI cable is potentially crossed.
****************************************************************/
Result TestAnalogInputs(AmpIO **Board, BasePort *Port) {
    Result result = pass;
    Port->ReadAllBoards();
    for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
        for (int channel_index = 0; channel_index < NUM_CHANNEL_PER_FPGA; channel_index++) {
            // Get actual potentiometer values
            unsigned long reading = Board[board_index]->GetAnalogInput(channel_index);
            logDebugInfo(std::hex, "board ", board_index, " pot channel ", channel_index, " - ", " expected=", 
                        POT_TEST_ADC_COUNT[board_index * 4 + channel_index], " measured=", reading);
                
            // check potentiometer within range of expected value
            if (std::fabs((long) reading - (long) POT_TEST_ADC_COUNT[board_index * 4 + channel_index]) > POT_TEST_ADC_ERROR_TOLERANCE) {
                logFail(result, fail);
                logfile << "Tolerance=" << POT_TEST_ADC_ERROR_TOLERANCE << std::endl;
                // check for swapped cable
                if (std::fabs((long) reading - (long) POT_TEST_ADC_COUNT[(1 ^ board_index)  * 4 + channel_index]) <
                    POT_TEST_ADC_ERROR_TOLERANCE) {
                    std::cout << COLOR_ERROR << "(Looks like the 68-pin cable is crossed!)" << COLOR_OFF
                                << std::endl;
                    logfile << "Crossed scsi?" << std::endl;
                    result = fatal_fail;
                }

            } else {
                logPass();
            }
        }
    }
    
    return result;
}

/****************************************************************
*   @brief: Function to test digital inputs of the dVRK.
*
*   @details: This function tests the home switches of the dVRK device by checking 
*   that they switch when they are adjusted. It checks that if it was in its default
*   position initially (the home position), it is no longer there, and if it was not 
*   in default position initially, it now is in its default position. It also checks 
*   that if an arm is at a limit (i.e. where it's not allowed to move any further in 
*   some direction), it is no longer at a limit after being moved, and that if it was 
*   not at a limit, it is at a limit after being moved.
*
*   @param AmpIO  (**Board): Array of board objects to have this function performed on
*   @param BasePort (*Port): Communication interface object that provides access to the 
*                            physical connection to the controller boards     
*
*   @return testResult (Result): result object indicating pass, fail, or fatal_fail 
*                                status of the test
*
*   @note: This function outputs debug information to standard out and the logfile; it
*   outputs the digital inputs.
*   @note: This test also notes that false positive readings can cause failure and might
*   need to run the test again.
****************************************************************/
Result TestDigitalInputs(AmpIO **Board, BasePort *Port) {
    Result result = pass;
    std::array<uint8_t, 3> before;
    std::array<uint8_t, 3> after;
    Port->ReadAllBoards();

    // read all initial switch values
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

    // read all switch values after waiting
    after[0] = Board[0]->GetHomeSwitches() & 0xf;
    if (is_dqla) {
        after[1] = Board[0]->GetNegativeLimitSwitches() >> 4;
        after[2] = Board[0]->GetPositiveLimitSwitches() >> 4;
    } else {
        after[1] = Board[1]->GetNegativeLimitSwitches();
        after[2] = Board[1]->GetPositiveLimitSwitches();
    }

    logDebugInfo("digital inputs - ");
    for (int i = 0; i < 3; i++) {
        logDebugInfo(std::hex, (int) before[i], "->", (int) after[i], " ");
    }

    // check all switches changed after waiting
    if ((before == DIGITAL_IN_TEST_HIGH_STATE && after == DIGITAL_IN_TEST_LOW_STATE) ||
        (before == DIGITAL_IN_TEST_LOW_STATE && after == DIGITAL_IN_TEST_HIGH_STATE)) {
        logPass();
    } else {
        logFail(result, fail);
        std::cout << COLOR_ERROR << "(False-positive failure is possible. Consider it pass if retry OK.)" << COLOR_OFF
                  << std::endl;
    }

    return result;

}

/****************************************************************
*   @brief: Function to test encoders' measurements on the dVRK.
*   
*   @details: This function tests if the encoders properly measure the expected change in
*   position after a set amount of time has passed (1000 milliseconds). It also outputs
*   the results to standard out and the logfile. 
*
*   @param AmpIO  (**Board): Array of board objects to have this function performed on
*   @param BasePort (*Port): Communication interface object that provides access to the 
*                            physical connection to the controller boards     
*
*   @return testResult (Result): result object indicating pass, fail, or fatal_fail 
*                                status of the test
*
*   @note: This function outputs debug information to standard out and the logfile; it
*   primarily outputs the measured difference in position and the expected difference.
*   @note: This test also outputs debugging info if specific patterns of encoders fail, 
*   such as encoders 0 through 3 working but 4 through 6 are not, which might indicate 
*   an issue with a cable.
****************************************************************/
Result TestEncoders(AmpIO **Board, BasePort *Port) {
    Result result = pass;

    int64_t encoder_count_before[8];
    int64_t encoder_count_after[8];

    // read initial encoder values
    if (Port->ReadAllBoards()) {
        for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
            for (int channel_index = 0; channel_index < NUM_CHANNEL_PER_FPGA; channel_index++) {
                encoder_count_before[board_index * 4 + channel_index] = Board[board_index]->GetEncoderPosition(
                        channel_index);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(ENCODER_TEST_WAIT_TIME_MS));

        // read encoder values after waiting
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

    // check encoder values change is within expected range of expected value
    for (int i = 0; i < 7; i++) {
        auto increment = encoder_count_after[i] - encoder_count_before[i];

        logDebugInfo(std::dec, "encoder ", i, " increment - expected=10 measured=",increment);
        if (std::fabs((long) increment - (long) ENCODER_TEST_INCREMENT) < ENCODER_TEST_INCREMENT_TOLERANCE) {
            logPass();
            each_encoder_result |= 1 << i;
        } else {
            logFail(result, fatal_fail);
        }
    }

    // extra debugging information if needed
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

/****************************************************************
*   @brief: Function to test power supplies of boards and channels in the dVRK.
*   
*   @details: This function goes through each board and checks its motor power 
*   supply/voltage. Then, it proceeds to check each of the channels on the board for 
*   their amp status (essentially equivalent to the channel's power supply). 
*
*   @param AmpIO  (**Board): Array of board objects to have this function performed on
*   @param BasePort (*Port): Communication interface object that provides access to the 
*                            physical connection to the controller boards     
*
*   @return testResult (Result): result object indicating pass, fail, or fatal_fail 
*                                status of the test
*
*   @note: This function outputs debug information to standard out and the logfile. It
*   outputs if the issue is the board's power supply or one of the amplifiers.
*   @note: This function first checks the relay status on the board to ensure that there 
*   is a connection for the power supplies to flow. If there is not, the function will fail 
*   and note that none of the board has power and output debug information to standard 
*   out accordingly.
****************************************************************/
Result TestMotorPowerControl(AmpIO **Board, BasePort *Port) {

    Result result = pass;
    int mv_good_fail_count = 0;

    // set initial conditions (enable relays, shutdown watchdog, allow power control)
    for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
        Board[board_index]->WriteSafetyRelay(true);
        Board[board_index]->WriteWatchdogPeriod(0x0000);
        Board[board_index]->WritePowerEnable(true);
        Board[board_index]->WriteAmpEnable(0xff, 0xff);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    Port->ReadAllBoards();

    // check power status and amp status on each board
    for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
        auto status = Board[board_index]->GetStatus();
        auto safety_relay_status = Board[board_index]->GetSafetyRelayStatus();
        logDebugInfo(std::hex, "board ", board_index, " power - status=", status, " safety_relay=", safety_relay_status);

        if (safety_relay_status) {
            std::cout << std::endl << "    ";
            // check power status
            if (Board[board_index]->GetPowerStatus()) {
                logDebugInfo(" mv_good=ok");
                bool board_failed = false;
                logDebugInfo(" amp_power=");

                // check amp status
                for (int channel_index = 0; channel_index < NUM_CHANNEL_PER_FPGA; channel_index++) {
                    if (Board[board_index]->GetAmpStatus(channel_index)) {
                        logDebugInfo(1);
                    } else {
                        logDebugInfo(0);
                        board_failed = true;
                    }

                }
                if (board_failed) {
                    logDebugInfo(" amp_power=bad");
                    logFail(result, fatal_fail);
                } else {
                    logPass();
                }
            } else {
                logDebugInfo(" mv_good=bad");
                mv_good_fail_count++;
                logFail(result, fatal_fail);
            }
        } else {
            logFail(result, fatal_fail);
        }
    }


    if (mv_good_fail_count == NUM_FPGA) {
        // When both boards don't have mv_good
        std::cout << COLOR_ERROR << "(None of the board has motor power. Is the safety chain open?)" << COLOR_OFF <<
                  std::endl;
    }

    return result;
}

/****************************************************************
*   @brief: Function to test motor current operation. 
*
*   @details: This function goes through each FPGA and each channel on the FPGA and
*   tests the motor current operation. It first tests that the motor has no current
*   when it is meant to be off. It then sets the motor current and reads that the 
*   motor current is as expected. Then it tests if the current drives the actual
*   analog devices (the analog device performance is read from the potentiometers)
*   as expected.
*
*   @param AmpIO  (**Board): Array of board objects to have this function performed on
*   @param BasePort (*Port): Communication interface object that provides access to the 
*                            physical connection to the controller boards      
*
*   @return testResult (Result): result object indicating pass, fail, or fatal_fail 
*                                status of the test
*
*   @note: This function outputs debug information to standard out and the logfile. It
*   outputs the board, amplifier channel, expected and actual motor current values.
*   @note: This function has extra logic to test if the DB9 cables might be crossed or
*   even not plugged in.
*   
****************************************************************/
Result TestPowerAmplifier(AmpIO **Board, BasePort *Port) {
    int crossed_db9_error_count = 0;
    Result result = pass;

    for (int board_index = 0; board_index < NUM_FPGA; board_index++) {
        for (int channel_index = 0; channel_index < NUM_CHANNEL_PER_FPGA; channel_index++) {
            if (board_index == 1 && channel_index == 3) break;
            if (board_index == 0 && channel_index == 7) break;

            Port->ReadAllBoards();

            // check for no current
            if (std::fabs((long) Board[0]->GetAnalogInput(0) - (long) POT_TEST_ADC_COUNT[0] / 2) <
                POT_TEST_ADC_ERROR_TOLERANCE) {
                std::cout << COLOR_ERROR << "(unexpected current detected on channel 0)" << COLOR_OFF << std::endl;
                logfile << "unexpected current detected on channel 0" << std::endl;
                return fatal_fail;
            }

            // write motor current
            Board[board_index]->SetMotorCurrent(channel_index, POWER_AMP_TEST_DAC);
            Port->WriteAllBoards();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            Port->ReadAllBoards();

            // check motor current is within range of expected motor current
            // also check motor current is driving analog inputs correctly
            auto motor_current = Board[board_index]->GetMotorCurrent(channel_index);
            bool amp_working = std::fabs((long) motor_current - (long) POWER_AMP_TEST_DAC) <
                               POWER_AMP_TEST_TOLERANCE;
            bool channel_0_load_driven =
                    std::fabs((long) Board[0]->GetAnalogInput(0) - (long) POT_TEST_ADC_COUNT[0] / 2) <
                    POT_TEST_ADC_ERROR_TOLERANCE;

            // extra debugging info if needed (if db0 cables are crossed)
            if (board_index == 0 && channel_index == 0 && amp_working && !channel_0_load_driven) {
                crossed_db9_error_count++;
            }

            if (board_index == 1 && channel_index == 0 && amp_working && channel_0_load_driven) {
                crossed_db9_error_count++;
            }

            if (board_index == 0 && channel_index == 4 && amp_working && channel_0_load_driven) {
                crossed_db9_error_count++;
            }

            logDebugInfo(std::hex, "board ", board_index, " amp channel ", channel_index, " - ", " expected=", 
                        POWER_AMP_TEST_DAC, " measured=", motor_current);

            if (!amp_working) {
                logFail(result, fail);
            } else {
                logPass();
            }

            Board[board_index]->SetMotorCurrent(channel_index, 0x8000);
            Port->WriteAllBoards();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));


        }
    }
    if (crossed_db9_error_count == 2) {
        result = fatal_fail;
        std::cout << COLOR_ERROR << "(DB9 cables are likely crossed!)" << COLOR_OFF << std::endl;
        logfile << "DB9 cables are likely crossed!" << std::endl;
    }

    if (result == fail) {
        std::cout << COLOR_ERROR << "(Are the DB9 cables plugged in?)" << COLOR_OFF << std::endl;
    }

    return result;

}

/****************************************************************
*   @brief: Function to test the Dallas Chip used in the dVRK.
*   
*   @details: This function checks whether the dallas chip can be read from and if it has
*   the proper family code (checks it is the expected chip). 
*
*   @param AmpIO  (**Board): Array of board objects to have this function performed on
*   @param BasePort (*Port): Communication interface object that provides access to the 
*                            physical connection to the controller boards  
*
*   @return testResult (Result): result object indicating pass, fail, or fatal_fail 
*                                status of the test
*
*   @note: This function outputs debug information to standard out and the logfile, but 
*   only what the test failed on.
****************************************************************/
Result TestDallas(AmpIO **Board, BasePort *Port) {
    Result result = pass;
    std::cout << "dallas ";
    logfile << "dallas ";
    AmpIO *board;
    if (is_dqla) {
        board = Board[0];
    } else {
        board = Board[1];
    }
    unsigned char buffer[2048];
    uint32_t status = 0;

    // read from the Dallas chip and read family code
    bool ret = board->DallasReadMemory(0, (unsigned char *) buffer, sizeof(buffer));
    board->DallasReadStatus(status);
    unsigned char family_code = static_cast<unsigned char>((status&0xFF000000)>>24);

    // check everything is as expected
    if (!ret) {
        logFail(result, fail);
        std::cout << COLOR_ERROR << "(Can't talk to DS2480B. Check SCSI cable.)" << COLOR_OFF << std::endl;
    } else if (family_code != 0x0B) {
        logFail(result, fail);
        std::cout << COLOR_ERROR << "(Can talk to DS2480B but not DS2505. Check dMIB jumper J42.)" << COLOR_OFF << std::endl;        
    } else {
        logPass();
    }
    return result;
}

//*********************************** Main ********************************************

void PrintDebugStream(std::stringstream &debugStream)
{
    std::cerr << debugStream.str() << std::endl;
    debugStream.clear();
    debugStream.str("");
}

/****************************************************************
*   @brief: Function to check for problems with controllers.
*
*   @details: This function checks for any issues with the controllers in the dVRK 
*   before running the tests.
*
*   @param ControllerList (std::vector<ControllerInfo>): A vector of the controllers
*                         that were found in the dVRK and information about them (The 
*                         ControllerInfo struct can be found in this file).
*                           
*   @return int: this int indicates if there was an error found. -1 indicates that an 
*           error was found with the controllers and 0 indicates no error
*
*   @note: This function also outputs to standard out what the issue found was.
****************************************************************/
int controllerErrorCheck(std::vector<ControllerInfo> &ControllerList) {
    if (ControllerList.size() != 1) {
        std::cout << "More than one controller found, please remove all controllers except unit under test" << std::endl;
        std::cout << "Press [Enter] to exit" << std::endl;
        std::cin.ignore();
        return -1;
    }

    if (ControllerList[0].hwVersion == dRA1_String) {
        std::cout << "dVRK-Si Controller detected -- this program is only for testing a dVRK (Classic) controller" << std::endl;
        std::cout << "Press [Enter] to exit" << std::endl;
        std::cin.ignore();
        return -1;
    }

    if (ControllerList[0].hwVersion == BCFG_String) {
        std::cout << "Controller boot error -- please confirm that the MicroSD card is inserted into the controller" << std::endl;
        std::cout << "Press [Enter] to exit" << std::endl;
        std::cin.ignore();
        return -1;
    }

    if ((ControllerList[0].hwVersion != QLA1_String) && (ControllerList[0].hwVersion != DQLA_String)) {
        // Should not happen, since not dRA1_String or BCFG_String, but checking just in case
        std::cout << "Unexpected controller hardware: " << std::hex << ControllerList[0].hwVersion << std::dec << std::endl;
        std::cout << "Press [Enter] to exit" << std::endl;
        std::cin.ignore();
        return -1;
    }

    if (ControllerList[0].armType == ControllerInfo::UNSUPPORTED) {
        std::cout << "Unsupported arm type -- check board id" << std::endl;
        std::cout << "Press [Enter] to exit" << std::endl;
        std::cin.ignore();
        return -1;
    }

    return 0;
}

/****************************************************************
*   @brief: Function to setup the port and controllers.
*
*   @details: This function does the initial setup of the port and controllers in
*   the dVRK, including calling the function to check for any errors with the
*   controllers.
*
*   @param argc (int): number of command line arguments in the terminal.
*   @param argv (char**): pointer to array of strings of the command line arguments
*                         provided in the terminal.
*   @param BasePort (*Port): Communication interface object that provides access to the 
*                            physical connection to the controller boards 
*   @param ControllerList (std::vector<ControllerInfo>): A vector of the controllers
*                         that were found in the dVRK and information about them.
*                   
*   @return int: this int indicates if there was an error found in the setup. -1 
*                indicates that an error was found and 0 indicates no error.
*
*   @note: This function also outputs to standard out or standard error some debug
*   information on if the port failed to initialize or no controllers are found.
****************************************************************/
int portAndControllerSetup(int argc, char **argv, BasePort **Port, std::vector<ControllerInfo> &ControllerList) {
    std::string portDescription = BasePort::DefaultPort();

    for (int i = 1; i < argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            portDescription = argv[i]+2;
        }
    }

    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);
    *Port = PortFactory(portDescription.c_str(), debugStream);
    if (!(*Port) || !(*Port)->IsOK()) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to initialize " << (*Port)->GetPortTypeString() << std::endl;
        std::cerr << "Press [Enter] to exit" << std::endl;
        std::cin.ignore();
        return -1;
    }

    // Now that Port is initialized, look for dVRK Controllers
    ControllerList = GetControllerList(*Port);
    if (ControllerList.size() > 0) {
        std::cout << "Controllers on " << (*Port)->GetPortTypeString() << ":" << std::endl;
        std::vector<ControllerInfo>::const_iterator it;
        for (it = ControllerList.begin(); it != ControllerList.end(); it++) {
            std::cout << "   " << ControllerTypeString[it->armType] << ", " << (*Port)->GetHardwareVersionString(it->board_id);
            if (it->hwVersion == QLA1_String)
                std::cout << ", boards " << it->board_id << ", " << it->board_id+1 << std::endl;
            else
                std::cout << ", board " << it->board_id << std::endl;
        }
        std::cout << std::endl;
    }
    else {
        std::cout << "No controllers found on " << (*Port)->GetPortTypeString() << ", exiting" << std::endl;
        std::cout << "Press [Enter] to exit" << std::endl;
        std::cin.ignore();
        return -1;
    }

    return controllerErrorCheck(ControllerList);
}

/****************************************************************
*   @brief: Function to check motor supply voltages of the dVRK.
*
*   @details: This function measures the motor supply voltages on the connected
*   QLA boards and compares them against the expected nominal values based on the
*   controller type. It handles both DQLA (dual QLA) and standard QLA configurations.
*
*   @param BasePort (*Port): Communication interface object that provides access to the 
*                           physical connection to the controller boards 
*   @param BoardList (std::vector<AmpIO *>): Vector of AmpIO board objects 
*                       representing the hardware interface to the controller boards
*   @param board1 (int): The board ID of the primary FPGA board
*
*   @return none
*
*   @note: This function outputs the measured voltage and expected nominal voltage to
*   standard out and the logfile. 
****************************************************************/
void checkMotorSupplyVoltage(BasePort *Port, std::vector<AmpIO *> BoardList, int board1) {
    double V[2] = { 0.0, 0.0};   // QLA 1,2 measured motor supply voltage
    if (is_dqla) {
        V[0] = MeasureMotorSupplyVoltage(Port, BoardList[0], 1);
        V[1] = MeasureMotorSupplyVoltage(Port, BoardList[0], 2);
    } else {
        V[0] = MeasureMotorSupplyVoltage(Port, BoardList[0]);
        V[1] = MeasureMotorSupplyVoltage(Port, BoardList[1]);
    }
    // Nominal motor supply voltages
    double VNom[2];
    VNom[0] = ControllerMotorSupplyVoltage[board1];
    VNom[1] = ControllerMotorSupplyVoltage[board1+1];
    // Check that measured supply voltage (if non-zero) is not more than
    // 10% different from nominal voltage. If measured supply voltage is 0,
    // we assume that hardware cannot measure motor supply voltage.
    bool V_mismatch = false;
    for (int v = 0; v < 2; v++) {
        if (V[v] > 0.0) {
            logDebugInfo("QLA ", (v+1), " Motor Supply Voltage: ", V[v], " (should be ", VNom[v], ")", "\n");
            if (fabs(V[v]-VNom[v]) > 0.1*VNom[v])
                V_mismatch = true;
        }
    }
    if (V_mismatch) {
        logDebugInfo("WARNING: Motor supply voltages do not match controller type", "\n");
        std::cout << "Press [Enter] to continue. Press [Ctrl-C] to exit." << std::endl;
        std::cin.ignore();
    }
}

/****************************************************************
*   @brief: Function to execute all test functions in sequence.
*
*   @details: This function creates a vector of test function pointers and runs each
*   test in order. It records the results of each test and handles fatal failures by
*   immediately terminating the test sequence.
*
*   @param Port (*BasePort): Communication interface object that provides access to the 
*                            physical connection to the controller boards
*   @param BoardList (std::vector<AmpIO *>): Vector of AmpIO board objects representing
*                            the hardware interface to the controller boards
*
*   @return all_pass (bool): Returns true if all tests passed, false if any test failed
*
*   @note: This function logs the result of each test section to the logfile.
*   @note: If any test results in a fatal failure, the function returns immediately without
*   running the remaining tests.
****************************************************************/
bool runTests(BasePort *Port, std::vector<AmpIO *> BoardList) {
    std::vector<Result (*)(AmpIO **, BasePort *)> test_functions;
    test_functions.push_back(TestSerialNumberAndVersion);
    test_functions.push_back(TestEncoders);
    test_functions.push_back(TestAnalogInputs);
    test_functions.push_back(TestDigitalInputs);
    test_functions.push_back(TestDallas);
    test_functions.push_back(TestMotorPowerControl);
    test_functions.push_back(TestPowerAmplifier);

    bool all_pass = true;

    for (auto test_function : test_functions) {
        auto result = test_function(&BoardList[0], Port);
        logfile << "Section result: " << (result == pass ? "PASS" : "FAIL") << std::endl;
        logDebugInfo("\n");
        if (result == fatal_fail) {
            return false;
        } else if (result == fail) {
            all_pass = false;
        }
    }

    return all_pass;
}

/****************************************************************
*   @brief: Function to set up the AmpIO board objects based on controller configuration.
*
*   @details: This function creates and initializes the AmpIO board objects based on
*   the detected controller type (DQLA or standard QLA). For standard QLA, it sets up
*   two separate board objects, while for DQLA it sets up a single board object.
*
*   @param Port (BasePort**): Pointer to Communication interface object that provides 
*                           access to the physical connection to the controller boards
*   @param board1 (int&): Reference to the board ID of the primary FPGA board
*   @param board2 (int&): Reference to the board ID of the secondary FPGA board (used only
*                         for standard QLA configuration)
*
*   @return BoardList (std::vector<AmpIO *>): Vector of initialized AmpIO board objects
*
*   @note: If board1 is set to MAX_BOARDS, indicating that the user did not specify a
*   board ID, this function uses the board ID obtained from scanning the bus.
*   @note: The function also updates global variables is_dqla, NUM_FPGA, and 
*   NUM_CHANNEL_PER_FPGA based on the detected controller type.
****************************************************************/
std::vector<AmpIO *> boardSetUp(BasePort **Port, int &board1, int &board2, std::vector<AmpIO *> BoardList) {
    if (board1 == BoardIO::MAX_BOARDS) {
        auto ControllerList = GetControllerList(*Port);
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
    (*Port)->AddBoard(BoardList[0]);
    if (!is_dqla) {
        BoardList.push_back(new AmpIO(board2));
        (*Port)->AddBoard(BoardList[1]);
    }

    return BoardList;
}

int main(int argc, char **argv) {
    int board1 = BoardIO::MAX_BOARDS;
    int board2 = BoardIO::MAX_BOARDS;
    BasePort *Port;
    std::vector<ControllerInfo> ControllerList;
    
    int setup_result = portAndControllerSetup(argc, argv, &Port, ControllerList);
    if (setup_result != 0) {
        std::cout << "Error in port and controller setup" << std::endl;
        return setup_result;
    }
   
    std::cout << "*********** dVRK  Classic Controller Test ***********" << std::endl;
    std::cout << "This program tests dVRK Classic controllers using a special test board." << std::endl;

    std::vector<AmpIO *> BoardList = boardSetUp(&Port, board1, board2);

    Port->ReadAllBoards();
    auto controller_sn = BoardList[0]->GetFPGASerialNumber();
    if (controller_sn.empty()) {
        controller_sn = "unknown";
    }
    
    std::stringstream filename;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    #ifdef _MSC_VER   // Windows
        _mkdir("dVRK_test_results");
    #else
        mkdir("dVRK_test_results", 0777);
    #endif
    filename << "dVRK_test_results/dVRK_" << controller_sn << "_" << std::put_time(&tm, "%Y-%m-%d-%H-%M-%S") << ".txt";
    logfile.open(filename.str(), std::fstream::out);

    // Check motor supply voltage
    checkMotorSupplyVoltage(Port, BoardList, board1);

    for (int i = 0; i < NUM_CHANNEL_PER_FPGA; i++) {
        for (auto &j : BoardList)
            j->WriteEncoderPreload(i, 0x1000 * i + 0x1000);
    }

    logDebugInfo("Board ID selected: Board 0=", board1);
    if (!is_dqla) {
        logDebugInfo(" Board 1=", board2);
    }
    logDebugInfo("\n");

    bool all_pass = runTests(Port, BoardList);

    if (all_pass) {
        std::cout << COLOR_GOOD << "PASS" << COLOR_OFF << ": all tests passed." <<std::endl;
    } else {
        std::cout << COLOR_ERROR << "FAIL" << COLOR_OFF << ": one or more tests failed." << std::endl;
    }
    logfile << "Final result: " << (all_pass ? "PASS" : "FAIL") << std::endl;

    for (auto &j : BoardList) {
        j->WritePowerEnable(false);
        j->WriteSafetyRelay(false);
        Port->RemoveBoard(j);
    }

    std::cout << "Press [Enter] to exit." << std::endl;
    std::cin.ignore();

    return all_pass ? 0 : -1;
}