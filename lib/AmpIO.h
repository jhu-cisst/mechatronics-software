/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Jie Ying Wu

  (C) Copyright 2011-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef __AMPIO_H__
#define __AMPIO_H__

#include <Amp1394/AmpIORevision.h>

#include "FpgaIO.h"
#include "EncoderVelocity.h"
#include <iostream>

class ostream;

/*! See Interface Spec: https://github.com/jhu-cisst/mechatronics-software/wiki/Interface-Specification */
class AmpIO : public FpgaIO
{
public:
    AmpIO(AmpIO_UInt8 board_id);
    ~AmpIO();

    // Return number of motors (4 for QLA, 10 for dRAC)
    unsigned int GetNumMotors(void) const { return NumMotors; }

    // Return number of encoders (4 for QLA, 7 for dRAC)
    unsigned int GetNumEncoders(void) const { return NumEncoders; }

    // Return number of digital outputs (4 for QLA)
    unsigned int GetNumDouts(void) const { return NumDouts; }

    // Return QLA serial number (empty string if not found)
    std::string GetQLASerialNumber(void);
    void DisplayReadBuffer(std::ostream &out = std::cout) const;

    // *********************** GET Methods ***********************************
    // The GetXXX methods below return data from local buffers that were filled
    // by BasePort::ReadAllBoards. To read data immediately from the boards,
    // use the ReadXXX methods, but note that individual reads are less
    // efficient than block transfers.

    AmpIO_UInt32 GetStatus(void) const;

    AmpIO_UInt32 GetTimestamp(void) const;

    // Get timestamp in seconds (time between two consecutive reads)
    double GetTimestampSeconds(void) const;

    // Return digital output state
    AmpIO_UInt8 GetDigitalOutput(void) const;

    /*! Utility method, uses result of GetDigitalInput but performs
      required shift and mask. */
    AmpIO_UInt8 GetNegativeLimitSwitches(void) const;

    /*! Utility method, uses result of GetDigitalInput but performs
      required shift and mask. */
    AmpIO_UInt8 GetPositiveLimitSwitches(void) const;

    /*! Utility method, uses result of GetDigitalInput but performs
      required shift and mask. */
    AmpIO_UInt8 GetHomeSwitches(void) const;

    /*! Utility methods, use result of GetDigitalInput but perform
      required shift and mask. */
    //@{
    AmpIO_UInt8 GetEncoderChannelA(void) const;
    bool GetEncoderChannelA(unsigned int index) const;

    AmpIO_UInt8 GetEncoderChannelB(void) const;
    bool GetEncoderChannelB(unsigned int index) const;

    AmpIO_UInt8 GetEncoderIndex(void) const;
    //@}

    AmpIO_UInt32 GetDigitalInput(void) const;

    AmpIO_UInt8 GetAmpTemperature(unsigned int index) const;

    AmpIO_UInt32 GetMotorCurrent(unsigned int index) const;

    AmpIO_UInt32 GetMotorStatus(unsigned int index) const;

    double GetMotorVoltageRatio(unsigned int index) const;

    AmpIO_UInt32 GetAnalogInput(unsigned int index) const;

    //********************** Encoder position/velocity/acceleration *****************************

    /*! Returns the encoder position in counts. */
    AmpIO_Int32 GetEncoderPosition(unsigned int index) const;

    /*! Returns true if encoder position has overflowed. */
    bool GetEncoderOverflow(unsigned int index) const;

    /*! Returns the encoder clock period, in seconds. Note that this value depends on the
        firmware version. */
    double GetEncoderClockPeriod(void) const;

    /*! Returns the encoder velocity, in counts per second, based on the FPGA measurement
        of the encoder period (i.e., time between two consecutive edges). Specifically, the
        velocity is given by 4/(period*clk) counts/sec, where clk is the period of the clock used to measure
        the encoder period. The numerator is 4 because an encoder period is equal to 4 counts (quadrature).
        If the counter overflows, the velocity is set to 0. This method should work with all versions of firmware,
        but has improved performance starting with Version 6. For a better velocity estimate, see the
        GetEncoderVelocityPredicted method.

        This method was previously called GetEncoderVelocityCountsPerSecond to distinguish it from an obsolete
        GetEncoderVelocity that actually returned the encoder period as an integer (AmpIO_Int32). The velocity
        period (velPeriod) and other information can now be obtained via the GetEncoderVelocityData method.
    */
    double GetEncoderVelocity(unsigned int index) const;

    /*! Returns the predicted encoder velocity, in counts per second, based on the FPGA measurement of the
        encoder period (i.e., time between two consecutive edges), with compensation for the measurement delay.
        For firmware Rev 6+, the predicted encoder velocity uses the estimated acceleration to predict the
        velocity at the current time. For Rev 7+, the prediction also uses the encoder running counter (time
        since last edge). There are two limits enforced:
        1) The predicted velocity will not change sign (i.e., be in the opposite direction from the measured velocity).
        2) The predicted velocity will not be larger than the velocity that would have caused one encoder count to
           occur during the time measured by the running counter.
        For an explanation of percent_threshold, see GetEncoderAcceleration.
    */
    double GetEncoderVelocityPredicted(unsigned int index, double percent_threshold = 1.0) const;

    /*! Returns the raw encoder period (velocity) value.
        This method is provided for internal use and testing. */
    AmpIO_UInt32 GetEncoderVelocityRaw(unsigned int index) const;

    /*! Returns midrange value of encoder position. */
    static AmpIO_Int32 GetEncoderMidRange(void);

    /*! Returns the encoder acceleration in counts per second**2, based on the scaled difference
        between the most recent full cycle and the previous full cycle. If the encoder counter overflowed
        (i.e., velocity was very slow or zero), the acceleration is set to 0. Since velocity is averaged
        over an entire cycle, acceleration can be applied over half the cycle to reduce delays.
        The percent_threshold is between 0 and 1 and essentially controls the maximum velocity for
        which acceleration is estimated. Specifically, if T is the most recent quarter-cycle period,
        then the acceleration is set to 0 if the magnitude of 1/T is greater than percent_threshold.
        This can avoid noisy acceleration estimates in those cases. Setting percent_threshold to 1.0
        effectively disables this feature. */
    double GetEncoderAcceleration(unsigned int index, double percent_threshold = 1.0) const;

    /*! Returns the raw encoder acceleration value. For firmware prior to Version 6, this was actually the
        encoder "frequency" (i.e., number of pulses in specified time period, which can be used to estimate velocity).
        This method is provided for internal use and testing in firmware Rev 6 and deprecated in >6. */
    AmpIO_UInt32 GetEncoderAccelerationRaw(unsigned int index) const;

    /*! Get the most recent encoder quarter cycle period for internal use and testing (Rev 7+). */
    AmpIO_UInt32 GetEncoderQtr1Raw(unsigned int index) const;

    /*! Get the encoder quarter cycle period from 5 cycles ago (i.e., 4 cycles prior to the one returned
        by GetEncoderQtr1) for internal use and testing (Rev 7+). */
    AmpIO_UInt32 GetEncoderQtr5Raw(unsigned int index) const;

    /*! Get the encoder running counter, which measures the elasped time since the last encoder edge;
        for internal use and testing (Rev 7+). */
    AmpIO_UInt32 GetEncoderRunningCounterRaw(unsigned int index) const;

    /*! Get the encoder running counter, in seconds. This is primarily used for Firmware Rev 7+, but
        also supports the running counter in Firmware Rev 4-5.
    */
    double GetEncoderRunningCounterSeconds(unsigned int index) const;

    /*! Returns the data available for computing encoder velocity (and acceleration). */
    bool GetEncoderVelocityData(unsigned int index, EncoderVelocity &data) const;

    /*! Returns the number of encoder errors (invalid transitions on the A or B channel). The errors
        are detected on the FPGA, with Firmware V7+, which sets an error bit in the encoder period used
        for velocity estimation. The error bit is cleared with the next valid transition.
        The number of errors reported here is only the number of errors received by the PC; it does
        not include any errors detected on the FPGA that were cleared before being read by the PC. */
    unsigned int GetEncoderErrorCount(unsigned int index) const;

    /*! Clears the count of encoder errors for the specified channel. Specifying MAX_CHANNELS (default
        value) clears the counter for all channels. */
    bool ClearEncoderErrorCount(unsigned int index = MAX_CHANNELS);

    //********************************************************************************************

    // GetPowerEnable: return power enable control
    bool GetPowerEnable(void) const;

    // GetPowerStatus: returns true if motor power supply voltage
    // is present on the QLA. If not present, it could be because
    // power is disabled or the power supply is off.
    bool GetPowerStatus(void) const;

    // GetPowerFault: returns true if motor power fault is detected.
    // This is supported from Rev 6 firmware and requires a QLA 1.4+
    bool GetPowerFault(void) const;

    // GetSafetyRelay: returns desired safety relay state
    bool GetSafetyRelay(void) const;

    // GetSafetyRelayStatus: returns true if safety relay contacts are closed
    bool GetSafetyRelayStatus(void) const;

    // GetWatchdogTimeoutStatus: returns true if watchdog timeout
    bool GetWatchdogTimeoutStatus(void) const;

    // GetAmpEnable: returns true if system is requesting amplifier to
    // be enabled (but, amplifier might be in fault state)
    bool GetAmpEnable(unsigned int index) const;

    // GetAmpEnableMask: returns mask where each set bit indicates that
    // the corresponding amplifier is requested to be enabled
    AmpIO_UInt8 GetAmpEnableMask(void) const;

    // GetAmpStatus: returns true if amplifier is enabled; false if
    // amplifier is in fault state.
    bool GetAmpStatus(unsigned int index) const;

    // GetSafetyAmpDisable: returns true if current safety module in FPGA trip
    AmpIO_UInt32 GetSafetyAmpDisable(void) const;

    AmpIO_UInt32 GetAmpFaultCode(unsigned int index) const;

    // *********************** SET Methods ***********************************
    // The SetXXX methods below write data to local buffers that are sent over
    // the bus via BasePort::WriteAllBoards. To immediately write to
    // the boards, you can use a WriteXXX method.
    // Note that SetAmpEnableMask is equivalent to WriteAmpEnable, whereas
    // SetAmpEnable affects only the specified amplifier index.
    // Also, due to the firmware implementation, SetAmpEnable/SetAmpEnableMask
    // cannot enable the amplifiers unless board power is already enabled,
    // i.e., via an earlier SetPowerEnable(true) followed by WriteAllBoards, or
    // via a call to WritePowerEnable(true).

    void SetPowerEnable(bool state);
    bool SetAmpEnable(unsigned int index, bool state);
    bool SetAmpEnableMask(AmpIO_UInt32 mask, AmpIO_UInt32 state);
    void SetSafetyRelay(bool state);

    bool SetMotorCurrent(unsigned int index, AmpIO_UInt32 mcur);
    bool SetMotorVoltageRatio(unsigned int index, double ratio);

    // ********************** READ Methods ***********************************
    // The ReadXXX methods below read data directly from the boards via the
    // bus using quadlet reads. In most cases, it is better to use the
    // GetXXX methods which read data from a buffer that is read from the board
    // via the more efficient block transfer.

    bool ReadPowerStatus(void) const;
    bool ReadSafetyRelayStatus(void) const;
    AmpIO_UInt32 ReadSafetyAmpDisable(void) const;

    bool ReadEncoderPreload(unsigned int index, AmpIO_Int32 &sdata) const;
    bool IsEncoderPreloadMidrange(unsigned int index, bool & isMidrange) const;

    // Read the watchdog period (16 bit number).
    // If applyMask is true, the upper 16 bits are cleared. This is the default
    // behavior for backward compatibility.
    // Starting with Firmware Rev 8, the upper bit (bit 31) indicates whether
    // LED1 on the QLA displays the watchdog period status.
    AmpIO_Int32 ReadWatchdogPeriod(bool applyMask = true) const;
    double ReadWatchdogPeriodInSeconds(void) const;

    AmpIO_UInt32 ReadDigitalIO(void) const;

    /*! \brief Read DOUT control register (e.g., for PWM, one-shot modes).

        \param index which digital output bit (0-3, which correspond to OUT1-OUT4)
        \param countsHigh counter value for high part of pulse (0 --> indefinite)
        \param countsLow counter value for low part of waveform (0 --> indefinite)
        \returns true if successful (results in countsHigh and countsLow)

        \sa WriteDoutControl
        \note Firmware Version 5+
     */
    bool ReadDoutControl(unsigned int index, AmpIO_UInt16 &countsHigh, AmpIO_UInt16 &countsLow);

    /*! \brief Read the status of the waveform output (i.e., whether waveform table is driving any
               digital outputs and, if so, the current table index).
        \param active true if the waveform table is actively driving any digital output
        \param tableIndex index into waveform table (see WriteWaveformTable)
        \returns true if successful (results in active and tableIndex)
    */
    bool ReadWaveformStatus(bool &active, AmpIO_UInt32 &tableIndex);

    // ********************** WRITE Methods **********************************

    // Enable motor power to the entire board (it is still necessary
    // to enable power to the individual amplifiers).
    bool WritePowerEnable(bool state);

    // Enable individual amplifiers
    bool WriteAmpEnable(AmpIO_UInt32 mask, AmpIO_UInt32 state);

    bool WriteSafetyRelay(bool state);

    bool WriteEncoderPreload(unsigned int index, AmpIO_Int32 enc);

    // Reset DOUT configuration bit, which causes firmware to repeat DOUT configuration check
    // (i.e., to detect QLA Rev 1.4+ with bidirectional DOUT).
    bool WriteDoutConfigReset(void);

    // Set digital output state
    bool WriteDigitalOutput(AmpIO_UInt8 mask, AmpIO_UInt8 bits);

    // Start/stop driving waveform for specified digital outputs (mask)
    bool WriteWaveformControl(AmpIO_UInt8 mask, AmpIO_UInt8 bits);

    // Write the watchdog period in counts. Starting with Firmware Rev 8, setting the upper
    // bit (bit 31) will cause LED1 on the QLA to display the watchdog period status.
    bool WriteWatchdogPeriod(AmpIO_UInt32 counts);
    // Write the watchdog period in seconds. Starting with Firmware Rev 8, setting ledDisplay
    // true will cause LED1 on the QLA to display the watchdog period status.
    bool WriteWatchdogPeriodInSeconds(const double seconds, bool ledDisplay = false);

    /*! \brief Write DOUT control register to set digital output mode (e.g., PWM, one-shot (pulse), general out).

        The modes are as follows:
          General DOUT:       high_time = low_time = 0
          Pulse high for DT:  high_time = DT, low_time = 0
          Pulse low for DT:   high_time = 0, low_time = DT
          PWM mode:           high_time = DTH, low_time = DTL (period = DTH+DTL)

        This is a low level-interface that specifies counts relative to the FPGA sysclk, which is 49.152 MHz.
        Thus, 1 count is approximately 20 nsec. The maximum 16-bit unsigned value, 65535, corresponds to
        about 1.33 msec.

        Note that regardless of the mode, writing to the digital output (e.g., using WriteDigitalOutput)
        will cause it to change to the new value. It will also reset the timer. For example, if we set
        the control register to pulse high for 100 clocks, but then set DOUT to 1 after 50 clocks, it will
        stay high for a total of 150 clocks. Writing to the control register also resets the time, so the
        same behavior would occur if we write to the control register after 50 clocks (assuming we do not
        also change the high_time).

        For the one-shot modes, the effect of calling WriteDoutControl is that the digital output will
        transition to the inactive state. For example, WriteDoutControl(0, 1000, 0) will have no effect
        on the digital output if it is already low. If the digital output is high, then the above call
        will cause the digital output to remain high for 1000 counts and then transition low and remain low.
        Subsequently, any call WriteDigitalOutput(0x01, 0x01) will cause the digital output to transition
        high, remain there for 1000 counts, and then transition low (i.e., a positive pulse of 1000 counts
        duration).

        For the PWM mode (high time and low time both non-zero), the effect of calling WriteDoutControl
        is to start PWM mode immediately.

        \param index which digital output bit (0-3, which correspond to OUT1-OUT4)
        \param countsHigh counter value for high part of pulse (0 --> indefinite)
        \param countsLow counter value for low part of waveform (0 --> indefinite)
        \returns true if successful

        \note Firmware Version 5+
    */
    bool WriteDoutControl(unsigned int index, AmpIO_UInt16 countsHigh, AmpIO_UInt16 countsLow);

    /*! \brief Write PWM parameters. This is a convenience function that calls WriteDoutControl.
               Note that the function can fail (and return false) if the input parameters are
               not feasible. The actual duty cycle will be as close as possible to the desired duty cycle.

        \param index which digital output bit (0-3, which correspond to OUT1-OUT4)
        \param freq PWM frequency, in Hz (375.08 Hz - 24.56 MHz)
        \param duty PWM duty cycle (0.0 - 1.0); 0.75 means PWM signal should be high 75% of time
        \returns true if successful; possible failures include firmware version < 5, failure to write
                 over FireWire, or specified frequency or duty cycle not possible.
    */
    bool WritePWM(unsigned int index, double freq, double duty);

    /*! \brief Get digital output time (in counts) corresponding to specified time in seconds.

        \param time Time, in seconds
        \returns Time, in counts
    */
    AmpIO_UInt32 GetDoutCounts(double timeInSec) const;

    // **************** Static WRITE Methods (for broadcast) ********************

    static bool WritePowerEnableAll(BasePort *port, bool state);

    static bool WriteAmpEnableAll(BasePort *port, AmpIO_UInt32 mask, AmpIO_UInt32 state);

    static bool WriteSafetyRelayAll(BasePort *port, bool state);

    static bool WriteEncoderPreloadAll(BasePort *port, unsigned int index, AmpIO_Int32 sdata);

    // ********************** Dallas DS2505 (1-wire) Methods **************************

    // DallasReadTool is the recommended method to obtain the tool information for the dVRK
    // and dVRK-S.
    //
    // Parameters:
    //    model              Tool model (part number)
    //    version            Tool version
    //    name               Tool name (currently only for dVRK)
    //    timeoutSec         Timeout, in seconds, for obtaining data (dVRK only, default is 10)
    //
    // Common return values (see below for additional dVRK return values):
    //    DALLAS_OK          On success
    //    DALLAS_IO_ERROR    I/O error (on Firewire or Ethernet)
    //
    // For the dVRK (QLA-based systems), this function should be called multiple times to
    // advance through an internal state machine. The function will return DALLAS_WAIT while
    // progressing through the state machine and then DALLAS_OK when the result is valid.
    // In addition to the two return values above, dVRK-specific returns are:
    //    DALLAS_WAIT        Data read still in process (call again later)
    //    DALLAS_NONE        Interface not supported (e.g., firmware to old)
    //    DALLAS_TIMEOUT     Timeout waiting for data (see DallasTimeoutSec)
    //    DALLAS_DATA_ERROR  Invalid data (i.e., did not read "997" in copyright string)
    //
    // For the dVRK-S (dRAC-based systems), there is no internal state machine and the
    // function will return either DALLAS_OK or DALLAS_IO_ERROR. Note that the tool name
    // is not supported (only model and version are set).

    enum DallasStatus { DALLAS_NONE, DALLAS_IO_ERROR, DALLAS_TIMEOUT, DALLAS_DATA_ERROR, DALLAS_WAIT, DALLAS_OK};
    DallasStatus DallasReadTool(AmpIO_UInt32 &model, AmpIO_UInt8 &version, std::string &name,
                                double timeoutSec = 10.0);

    bool DallasWriteControl(AmpIO_UInt32 ctrl);
    bool DallasReadStatus(AmpIO_UInt32 &status);
    bool DallasWaitIdle();
    bool DallasReadBlock(unsigned char *data, unsigned int nbytes) const;
    bool DallasReadMemory(unsigned short addr, unsigned char *data, unsigned int nbytes);

    /*! \brief Reads model number of the tool on an S PSM
        \returns Model number. 0xffffffff when no tool is present.
    */
    AmpIO_UInt32 SPSMReadToolModel(void) const;

    /*! \brief Reads version of the tool on an S PSM
        \returns Version. 0xff when no tool is present.
    */
    AmpIO_UInt8 SPSMReadToolVersion(void) const;

    // ********************** Waveform Generator Methods *****************************
    // FPGA Firmware Version 7 introduced a Waveform table that can be used to drive
    // any combination of the 4 digital outputs. The waveform table length is 1024,
    // but can be updated while the waveform is active; to do that, call ReadWaveformStatus
    // to obtain the current readIndex on the FPGA. Note that the FPGA will automatically
    // wrap when reading/writing the table. For example, writing 10 quadlets to offset 1020
    // will write to table[1020]-table[1023] and then table[0]-table[5].

    /*! \brief Read the contents of the waveform table.
        \param buffer Buffer for storing contents of waveform table
        \param offset Offset into waveform table (0-1023)
        \param nquads Number of quadlets to read (1-1024)
        \note Cannot read table while waveform is active.
    */
    bool ReadWaveformTable(quadlet_t *buffer, unsigned short offset, unsigned short nquads);

    /*! \brief Write the contents of the waveform table.
        \param buffer Buffer containing data to write to waveform table
        \param offset Offset into waveform table (0-1023)
        \param nquads Number of quadlets to write (1-1024)
    */
    bool WriteWaveformTable(const quadlet_t *buffer, unsigned short offset, unsigned short nquads);

    // *********************** Data Collection Methods *******************************

    /*! \brief User-supplied callback function for data collection
        \param buffer pointer to collected data
        \param nquads number of elements in buffer
        \returns returning false stops data collection (same as calling DataCollectionStop)
    */
    typedef bool (*CollectCallback)(quadlet_t *buffer, short nquads);

    /*! \brief Start data collection on FPGA (Firmware Rev 7+)
        \param chan which channel to collect (1-4)
        \param collectCB optional callback function
        \returns true   if data collection available (Firmware Rev 7+) and parameters are valid
        \note  If callback not specified, must call ReadCollectedData to read data
    */
    bool DataCollectionStart(unsigned char chan, CollectCallback collectCB = 0);
    /*! \brief Stop data collection on FPGA */
    void DataCollectionStop();
    /*! \brief Returns true if data collection is active */
    bool IsCollecting() const;

    /*! \brief Gets FPGA data collection status, from real-time block read (Firmware Rev 7+)
        \param collecting Whether FPGA is currently collecting data
        \param chan Channel being collected (1-4)
        \param writeAddr Buffer address being written by FPGA (can read up to writeAddr-1)
        \returns true if successful
        \sa ReadCollectionStatus  */
    bool GetCollectionStatus(bool &collecting, unsigned char &chan, unsigned short &writeAddr) const;

    /*! \brief Reads FPGA data collection status, via quadlet read command (Firmware Rev 7+)
        \param collecting Whether FPGA is currently collecting data
        \param chan Channel being collected (1-4)
        \param writeAddr Buffer address being written by FPGA (can read up to writeAddr-1)
        \returns true if successful
        \sa GetCollectionStatus  */
    bool ReadCollectionStatus(bool &collecting, unsigned char &chan, unsigned short &writeAddr) const;

    /*! \brief Read collected data from FPGA memory buffer */
    bool ReadCollectedData(quadlet_t *buffer, unsigned short offset, unsigned short nquads);

    // Motor control mode for Firmware Rev 8+
    enum MotorControlMode {
        RESET = 2,
        VOLTAGE = 1,
        CURRENT = 0
    };

    bool WriteMotorControlMode(unsigned int index, AmpIO_UInt16 val);
    bool WriteCurrentKpRaw(unsigned int index, AmpIO_UInt32 val);
    bool WriteCurrentKiRaw(unsigned int index, AmpIO_UInt32 val);
    bool WriteCurrentITermLimitRaw(unsigned int index, AmpIO_UInt16 val);
    bool WriteDutyCycleLimit(unsigned int index, AmpIO_UInt16 val);

    AmpIO_UInt16 ReadMotorControlMode(unsigned int index) const;
    AmpIO_UInt32 ReadCurrentKpRaw(unsigned int index) const;
    AmpIO_UInt32 ReadCurrentKiRaw(unsigned int index) const;
    AmpIO_UInt16 ReadCurrentITermLimitRaw(unsigned int index) const;
    AmpIO_UInt16 ReadDutyCycleLimit(unsigned int index) const;

    AmpIO_Int16 ReadDutyCycle(unsigned int index) const;
    AmpIO_Int16 ReadCurrentITerm(unsigned int index) const;
    AmpIO_Int16 ReadFault(unsigned int index) const;

protected:
    // NumMotors specifies the number of motors/brakes and NumEncoders specifies the
    // number of encoders. Motors and encoders are paired up to T = min(NumMotors, NumEncoders),
    // e.g., motor[i] is paired with encoder[i] for i = 0 ... (T-1).
    //   QLA (QLA1):  NumMotors=NumEncoders=4, so there are 4 motor/encoder pairs
    //   dRAC (dRA1): NumMotors=10, NumEncoders=7, so there are 7 motor/encoder pairs and
    //                the last 3 motors (brakes) do not have encoders
    // NumDouts specifies the number of digital outputs, which can also be driven in PWM mode.
    // The number of digital inputs is not specified, but a 32-bit register is allocated so there
    // may be up to 32 digital inputs.
    unsigned int NumMotors;    // Number of motors/brakes
    unsigned int NumEncoders;  // Number of encoders
    unsigned int NumDouts;     // Number of digital outputs

    // Maximum number of channels (avoids need to dynamically allocate memory)
    enum { MAX_CHANNELS = 16 };

    // Maximum read and write buffer sizes (in quadlets)
    enum { ReadBufSize_Max = 64,
           WriteBufSize_Max = 64 };

    // Buffer for real-time block reads. The Port class calls SetReadData to copy the
    // most recent data into this buffer, while also byteswapping.
    quadlet_t ReadBuffer[ReadBufSize_Max];

    // Buffer for real-time block writes. The Port class calls GetWriteData to copy from
    // this buffer, while also byteswapping if needed.
    quadlet_t WriteBuffer[WriteBufSize_Max];

    // Encoder velocity data (per axis)
    EncoderVelocity encVelData[MAX_CHANNELS];

    // Counts received encoder errors
    unsigned int encErrorCount[MAX_CHANNELS];

    // Dallas interface (for QLA)
    enum DallasStateType { ST_DALLAS_START, ST_DALLAS_WAIT, ST_DALLAS_READ };
    DallasStateType dallasState;         // Current state
    DallasStateType dallasStateNext;     // Next state (only used by ST_DALLAS_WAIT)
    double dallasTimeoutSec;             // Timeout in seconds
    double dallasWaitStart;              // Start time (in seconds); used to check for timeout
    bool   dallasUseDS2480B;             // True if DS2480B driver used; false if direct 1-wire interface
    // Offsets into Dallas memory
    enum { DALLAS_START_READ = 0x80, DALLAS_MODEL_OFFSET = 0xa4, DALLAS_VERSION_OFFSET = 0xa8,
           DALLAS_NAME_OFFSET = 0x160, DALLAS_NAME_END = 0x17c };

    // Data collection
    // The FPGA firmware contains a data collection buffer of 1024 quadlets.
    // Data collection is enabled by setting the COLLECT_BIT when writing the desired motor current.
    // Data can only be collected on one channel, specified by collect_chan.
    enum { COLLECT_BUFSIZE = 1024,     // must match firmware buffer size
           COLLECT_MAX = 512           // maximum read request size (at 400 MBit/sec)
         };
    quadlet_t collect_data[COLLECT_MAX];
    bool collect_state;                // true if collecting data
    unsigned char collect_chan;        // which channel is being collected
    CollectCallback collect_cb;        // user-supplied callback (if non-zero)
    unsigned short collect_rindex;     // current read index

    // Virtual methods

    // InitBoard sets the number of motors and encoders, based on the hardware
    // (e.g., QLA or dRA1) and firmware version. It assumes that the port member
    // data has already been set.
    void InitBoard(void);

    unsigned int GetReadNumBytes() const;
    void SetReadData(const quadlet_t *buf);

    unsigned int GetWriteNumBytes(void) const;
    bool GetWriteData(quadlet_t *buf, unsigned int offset, unsigned int numQuads, bool doSwap = true) const;
    void InitWriteBuffer(void);

    // Test if the current write buffer contains commands that will
    // reset the watchdog on the board.  In practice, checks if
    // there's any valid bit on the 4 requested currents.
    bool WriteBufferResetsWatchdog(void) const;

    /*! Extract the data used for velocity estimation */
    bool SetEncoderVelocityData(unsigned int index);

    /*! \brief If user-supplied callback is not NULL, read data collection buffer and then call callback.
        \note Called by relevant Port class.
    */
    void CheckCollectCallback();

    // Offsets of real-time read buffer contents, in quadlets
    // Offsets from TIMESTAMP_OFFSET to ANALOG_POS_OFFSET have remained stable through
    // all releases of firmware and for both QLA1 and dRA1. The other offsets are related
    // to velocity (and acceleration) estimation and have varied based on the firmware.
    // ENC_VEL_OFFSET has always measured the encoder period (time between consecutive
    // encoder edges of the same type), though the resolution of that measurement (i.e.,
    // clock ticks per second) varies between different firmware versions.
    // ENC_FRQ_OFFSET originally measured the number of encoder counts over a fixed
    // time period (about 8.5 ms), but was never used by the higher-level software.
    // It was later changed to ENC_QTR1_OFFSET, which measures the number of encoder
    // counts over the last quarter-cycle, which is used for acceleration estimation.
    // Firmware V7 added ENC_QTR5_OFFSET and ENC_RUN_OFFSET; in V6, the QTR5 data
    // was stuffed into unused bits in other fields.
    // Firmware V8 added MOTOR_STATUS_OFFSET.
    enum {
        TIMESTAMP_OFFSET  = 0,    // one quadlet
        STATUS_OFFSET     = 1,    // one quadlet
        DIGIO_OFFSET      = 2,    // digital I/O (one quadlet)
        TEMP_OFFSET       = 3,    // temperature (one quadlet)
        MOTOR_CURR_OFFSET = 4,    // half quadlet per channel (lower half)
        ANALOG_POS_OFFSET = 4     // half quadlet per channel (upper half)
    };
    unsigned int ENC_POS_OFFSET;  // one quadlet per channel
    unsigned int ENC_VEL_OFFSET;  // one quadlet per channel
    unsigned int ENC_FRQ_OFFSET;  // one quadlet per channel
    unsigned int ENC_QTR1_OFFSET; // one quadlet per channel
    unsigned int ENC_QTR5_OFFSET; // one quadlet per channel
    unsigned int ENC_RUN_OFFSET;  // one quadlet per channel
    unsigned int MOTOR_STATUS_OFFSET;

    // offsets of real-time write buffer contents
    unsigned int WB_HEADER_OFFSET; // write header (Firmware Rev 8+)
    unsigned int WB_CURR_OFFSET;   // one quadlet per channel
    unsigned int WB_CTRL_OFFSET;   // control register (power control)

    // Hardware device address offsets, not to be confused with buffer offsets.
    // Format is [4-bit channel address (1-7) | 4-bit device offset]
    // Applies only to quadlet transactions--block transactions use fixed
    // real-time formats described above
    enum {
        ADC_DATA_OFFSET = 0,       // adc data register
        DAC_CTRL_OFFSET = 1,       // dac control register
        POT_CTRL_OFFSET = 2,       // pot control register
        POT_DATA_OFFSET = 3,       // pot data register
        ENC_LOAD_OFFSET = 4,       // enc control register (preload)
        POS_DATA_OFFSET = 5,       // enc data register (position)
        VEL_DATA_OFFSET = 6,       // enc data register (velocity, 4/DT method)
        VEL_DP_DATA_OFFSET = 7,    // enc data register (velocity, DP/1 method)
        DOUT_CTRL_OFFSET = 8       // digital output control (PWM, one-shot)
    };

    // Following offsets are for dRA1
    enum {
        OFF_MOTOR_CONTROL_MODE = 0,
        OFF_CURRENT_KP = 1,
        OFF_CURRENT_KI = 2,
        OFF_CURRENT_I_TERM_LIMIT = 3,
        OFF_DUTY_CYCLE_LIMIT = 4,
        OFF_DUTY_CYCLE = 10,
        OFF_FAULT = 11,
        OFF_CURRENT_I_TERM = 12 // awaiting new assignment
    };

    enum {
        ADDR_MOTOR_CONTROL = 9
    };
};

#endif // __AMPIO_H__
