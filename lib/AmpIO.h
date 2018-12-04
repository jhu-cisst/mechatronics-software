/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Jie Ying Wu

  (C) Copyright 2011-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef __AMPIO_H__
#define __AMPIO_H__

#include <Amp1394/AmpIORevision.h>

#include "BoardIO.h"
#ifdef _MSC_VER
typedef unsigned __int8  uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef __int32          int32_t;
#else
#include <stdint.h>
#endif
#include <iostream>

class ostream;


typedef int32_t  AmpIO_Int32;
typedef uint32_t AmpIO_UInt32;
typedef uint16_t AmpIO_UInt16;
typedef uint8_t  AmpIO_UInt8;

class AmpIO : public BoardIO
{
public:
    AmpIO(AmpIO_UInt8 board_id, unsigned int numAxes = 4);
    ~AmpIO();

    AmpIO_UInt32 GetFirmwareVersion(void) const;    
    // Return FPGA serial number (empty string if not found)
    std::string GetFPGASerialNumber(void);
    // Return QLA serial number (empty string if not found)
    std::string GetQLASerialNumber(void);
    void DisplayReadBuffer(std::ostream &out = std::cout) const;

    // Returns true if FPGA has Ethernet (Rev 2.0+)
    bool HasEthernet(void) const;

    // *********************** GET Methods ***********************************
    // The GetXXX methods below return data from local buffers that were filled
    // by FirewirePort::ReadAllBoards. To read data immediately from the boards,
    // use the ReadXXX methods, but note that individual reads via IEEE-1394 are
    // less efficient than block transfers.

    AmpIO_UInt32 GetStatus(void) const;

    AmpIO_UInt32 GetTimestamp(void) const;

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

    bool GetEncoderOverflow(unsigned int index) const;

    AmpIO_UInt8 GetEncoderIndex(void) const;
    //@}

    AmpIO_UInt32 GetDigitalInput(void) const;

    AmpIO_UInt8 GetAmpTemperature(unsigned int index) const;

    AmpIO_UInt32 GetMotorCurrent(unsigned int index) const;

    AmpIO_UInt32 GetAnalogInput(unsigned int index) const;

    AmpIO_Int32 GetEncoderPosition(unsigned int index) const;

    /*! Returns the encoder velocity, in counts per second, based on the FPGA measurement
        of the encoder period (i.e., time between two consecutive edges). Specifically, the
        velocity is given by 4/(period*clk) counts/sec, where clk is the period of the clock used to measure
        the encoder period. The numerator is 4 because an encoder period is equal to 4 counts (quadrature).
        If the counter overflows, the velocity is set to 0. This method should work with all versions of firmware,
        but has improved performance starting with Version 6. */
    double GetEncoderVelocityCountsPerSecond(unsigned int index) const;

    /*! Returns the time delay of the encoder velocity measurement, in seconds.
        Currently, this is equal to half the measured period, based on the assumption that measuring the
        period over a full cycle (4 quadrature counts) estimates the velocity in the middle of that cycle. */
    double GetEncoderVelocityDelay(unsigned int index) const;

    /*! Returns the encoder period, which is the time between two consecutive edges, as a signed value.
        Specific details depend on the version of FPGA firmware. This value can be used to estimate the encoder
        velocity (see GetEncoderVelocityCountsPerSecond). This method probably should have been called GetEncoderPeriod.
        \note In prior versions of this library (up through 1.3.0), there was an optional "islatch"
        boolean parameter, which defaulted to true. This is no longer supported by FPGA Firmware
        Version 6+ and thus has been removed. If this library is used with prior versions of firmware,
        it will return the estimated period corresponding to a true "islatch", and that period will
        occupy the lower 16-bits. */
    AmpIO_Int32 GetEncoderVelocity(unsigned int index) const;

    /*! Returns the raw encoder period (velocity) value.
        This method is provided for internal use and testing. */
    AmpIO_UInt32 GetEncoderVelocityRaw(unsigned int index) const;

    /*! Returns midrange value of encoder position. */
    AmpIO_Int32 GetEncoderMidRange(void) const;

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
        This method is provided for internal use and testing. */
    AmpIO_UInt32 GetEncoderAccelerationRaw(unsigned int index) const;
    
    // GetPowerStatus: returns true if motor power supply voltage
    // is present on the QLA. If not present, it could be because
    // power is disabled or the power supply is off.
    bool GetPowerStatus(void) const;

    // GetSafetyRelayStatus: returns true if safety relay contacts are closed
    bool GetSafetyRelayStatus(void) const;

    // GetWatchdogTimeoutStatus: returns true if watchdog timeout
    bool GetWatchdogTimeoutStatus(void) const;

    // GetAmpEnable: returns true if system is requesting amplifier to
    // be enabled (but, amplifier might be in fault state)
    bool GetAmpEnable(unsigned int index) const;

    // GetAmpStatus: returns true if amplifier is enabled; false if
    // amplifier is in fault state.
    bool GetAmpStatus(unsigned int index) const;

    // GetSafetyAmpDisable: returns true if current safety module in FPGA trip
    AmpIO_UInt32 GetSafetyAmpDisable(void) const;

    // *********************** SET Methods ***********************************
    // The SetXXX methods below write data to local buffers that are sent over
    // IEEE-1394 via FirewirePort::WriteAllBoards. To immediately write to
    // the boards, you can use a WriteXXX method.

    void SetPowerEnable(bool state);
    bool SetAmpEnable(unsigned int index, bool state);
    void SetSafetyRelay(bool state);

    bool SetMotorCurrent(unsigned int index, AmpIO_UInt32 mcur);

    // ********************** READ Methods ***********************************
    // The ReadXXX methods below read data directly from the boards via IEEE-1394
    // (Firewire) using quadlet reads. In most cases, it is better to use the
    // GetXXX methods which read data from a buffer that is read from the board
    // via the more efficient IEEE-1394 block transfer.

    AmpIO_UInt32 ReadStatus(void) const;
    bool ReadPowerStatus(void) const;
    bool ReadSafetyRelayStatus(void) const;
    AmpIO_UInt32 ReadSafetyAmpDisable(void) const;

    bool ReadEncoderPreload(unsigned int index, AmpIO_Int32 &sdata) const;

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

    // ********************** WRITE Methods **********************************

    // Enable motor power to the entire board (it is still necessary
    // to enable power to the individual amplifiers).
    bool WritePowerEnable(bool state);

    // Enable individual amplifiers
    bool WriteAmpEnable(AmpIO_UInt8 mask, AmpIO_UInt8 state);

    bool WriteSafetyRelay(bool state);

    bool WriteEncoderPreload(unsigned int index, AmpIO_Int32 enc);

    // Set digital output state
    bool WriteDigitalOutput(AmpIO_UInt8 mask, AmpIO_UInt8 bits);

    bool WriteWatchdogPeriod(AmpIO_UInt32 counts);

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

    // ********************** PROM Methods ***********************************
    // Methods for reading or programming
    //   1 - the FPGA configuration PROM (M25P16)
    //   2 - QLA PROM (25AA128)
    // NOTE:
    //   - M25P16 & 25AA128 Have exact same command table, except M25P16 has
    //     a few extra commands (e.g. ReadID, DeepSleep)
    //   - address length
    //     - M25P16: 24-bit
    //     - 25AA128: 16-bit (2 MSB are ignored)
    enum PromType{
        PROM_M25P16 = 1,
        PROM_25AA128 = 2
    };

    /*!
     \brief Get PROM read/write address (1394 space) based on prom type

     \param type   PROM type e.g. M25P16/25AA128
     \param isWrite  write address or read address
     \return nodeaddr_t  return address in FPGA 1394 address space
    */
    nodeaddr_t GetPromAddress(PromType type = PROM_M25P16, bool isWrite = true);

    // User-supplied callback, called when software needs to wait for an
    // action, or when an error occurs. In the latter case, the error message
    // is passed as a parameter (NULL if no error). If the callback returns
    // false, the current wait operation is aborted.
    typedef bool (*ProgressCallback)(const char *msg);

    // Returns the PROM ID (M25P15 ONLY)
    // should be 0x00202015 for the M25P16
    // returns 0 on error
    AmpIO_UInt32 PromGetId(void);

    // PromGetStatus (General)
    // returns the status register from the M25P16 PROM
    // (this is different than the interface status read from address offset 8)
    // The following masks are for the useful status register bits.
    enum { MASK_WIP = 1, MASK_WEL = 2 };   // status register bit masks
    bool PromGetStatus(AmpIO_UInt32 &status, PromType type = PROM_M25P16);

    // PromGetResult (General)
    // returns the result (if any) from the last command sent
    bool PromGetResult(AmpIO_UInt32 &result, PromType type = PROM_M25P16);

    // Returns nbytes data read from the specified address
    bool PromReadData(AmpIO_UInt32 addr, AmpIO_UInt8 *data,
                      unsigned int nbytes);

    // Enable programming commands (erase and program page) (General)
    // This sets the WEL bit in the status register.
    // This mode is automatically cleared after a programming command is executed.
    bool PromWriteEnable(PromType type = PROM_M25P16);

    // Disable programming commands (General)
    // This clears the WEL bit in the status register.
    bool PromWriteDisable(PromType type = PROM_M25P16);

    // Erase a sector (64K) at the specified address. (M25P16 ONLY)
    // This command calls PromWriteEnable.
    // If non-zero, the callback (cb) is called while the software is waiting
    // for the PROM to be erased, or if there is an error.
    bool PromSectorErase(AmpIO_UInt32 addr, const ProgressCallback cb = 0);

    // Program a page (up to 256 bytes) at the specified address.
    // This command calls PromWriteEnable.
    // nbytes must be a multiple of 4.
    // If non-zero, the callback (cb) is called while the software is waiting
    // for the PROM to be programmed, or if there is an error.
    // Returns the number of bytes programmed (-1 if error).
    int PromProgramPage(AmpIO_UInt32 addr, const AmpIO_UInt8 *bytes,
                        unsigned int nbytes, const ProgressCallback cb = 0);


    // ********************** QLA PROM ONLY Methods ***********************************
    // ZC: meta data only, so don't care speed that much
    bool PromReadByte25AA128(AmpIO_UInt16 addr, AmpIO_UInt8 &data);
    bool PromWriteByte25AA128(AmpIO_UInt16 addr, const AmpIO_UInt8 &data);
    bool PromReadBlock25AA128(AmpIO_UInt16 addr, quadlet_t* data, unsigned int nquads);
    bool PromWriteBlock25AA128(AmpIO_UInt16 addr, quadlet_t* data, unsigned int nquads);

    // ********************** Dallas DS2505 (1-wire) Methods **************************
    bool DallasWriteControl(AmpIO_UInt32 ctrl);
    bool DallasReadStatus(AmpIO_UInt32 &status);
    bool DallasWaitIdle();
    bool DallasReadMemory(unsigned short addr, unsigned char *data, unsigned int nbytes);

    // ************************ Ethernet Methods *************************************
    // Following functions enable access to the KSZ8851 Ethernet controller on the
    // FPGA V2 board via FireWire. They are provided for testing/debugging.
    // Note that both 8-bit and 16-bit transactions are supported.
    bool ResetKSZ8851();   // Reset the chip (requires ~60 msec)
    bool WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt8 &data);
    bool WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt16 &data);
    bool ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt8 &rdata);
    bool ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt16 &rdata);
    // Following are for DMA access (assumes chip has been placed in DMA mode)
    bool WriteKSZ8851DMA(const AmpIO_UInt16 &data);
    bool ReadKSZ8851DMA(AmpIO_UInt16 &rdata);
    // Read Chip ID from register 0xC0
    AmpIO_UInt16 ReadKSZ8851ChipID();
    // Get KSZ8851 status; format is:  VALID(1) 0(6) ERROR(1) PME(1) IRQ(1) STATE(4)
    //    VALID=1 indicates that Ethernet is present
    //    ERROR=1 indicates that last command had an error (i.e., state machine was not idle)
    //    PME is the state of the Power Management Event pin
    //    IRQ is the state of the Interrupt Request pin (active low)
    //    STATE is a 4-bit value that encodes the FPGA state machine (0=IDLE)
    // Returns 0 on error (i.e., if Ethernet not present, or read fails)
    AmpIO_UInt16 ReadKSZ8851Status();

protected:
    unsigned int NumAxes;   // not currently used

    // Number of channels in the node (4 for QLA)
    enum { NUM_CHANNELS = 4 };

    // Sizes of real-time read and write buffers (see below for offsets into these buffers)
    enum { ReadBufSize = 4+4*NUM_CHANNELS,
           WriteBufSize = NUM_CHANNELS+1 };

    // The (internal) read buffer; the base class ReadBuffer pointer
    // is initialized to this buffer in the constructor.
    quadlet_t read_buffer[ReadBufSize];     // buffer for real-time reads

    // Internal write buffer; the base class WriteBuffer and WriteBufferData pointers are
    // initialized to this buffer in the constructor, so that they always point to valid memory.
    // WriteBuffer and WriteBufferData can be changed by calling InitWriteBuffer
    // (see Eth1394Port).
    quadlet_t write_buffer_internal[WriteBufSize];

    // Virtual method
    void InitWriteBuffer(quadlet_t *buf, size_t data_offset);

    // Test if the current write buffer contains commands that will
    // reset the watchdog on the board.  In practice, checks if
    // there's any valid bit on the 4 requested currents.
    bool WriteBufferResetsWatchdog(void) const;
 
    /*! Returns the encoder period (counter) of the previous full cycle.
        Used internally to calculate acceleration. */
    AmpIO_Int32 GetEncoderPrevCounter(unsigned int index) const;

    /*! Returns whether the full cycle counter has overflows (22 bits) */
    bool GetEncoderVelocityOverflow(unsigned int index) const;

    /*! Returns the direction the encoder is moving in. */
    bool GetEncoderDir(unsigned int index) const;

    /*! Returns the latched period of five encoder quarter cycles
        ago. Used internally to calculate acceleration. */
    AmpIO_Int32 GetEncoderAccPrev(unsigned int index) const;

    /*! Returns the latched period of the most recent encoder
      quarter cycle. Used internally to calculate acceleration. */
    AmpIO_Int32 GetEncoderAccRec(unsigned int index) const;

    // Offsets of real-time read buffer contents, 20 = 4 + 4 * 4 quadlets
    // Note that there are two velocity measurements. The first one (ENC_VEL_OFFSET)
    // measures the time between consecutive encoder edges of the same type.
    // The second one (ENC_FRQ_OFFSET) returns the number of encoder counts over
    // a fixed time period (currently, about 8.5 ms, based on 11.72 Hz clock on FPGA).
    // Only the first one (ENC_VEL_OFFSET) is currently used.
    enum {
        TIMESTAMP_OFFSET  = 0,    // one quadlet
        STATUS_OFFSET     = 1,    // one quadlet
        DIGIO_OFFSET      = 2,    // digital I/O (one quadlet)
        TEMP_OFFSET       = 3,    // temperature (one quadlet)
        MOTOR_CURR_OFFSET = 4,    // half quadlet per channel (lower half)
        ANALOG_POS_OFFSET = 4,    // half quadlet per channel (upper half)
        ENC_POS_OFFSET    = 4+NUM_CHANNELS,    // one quadlet per channel
        ENC_VEL_OFFSET    = 4+2*NUM_CHANNELS,  // one quadlet per channel
        ENC_FRQ_OFFSET    = 4+3*NUM_CHANNELS   // one quadlet per channel
    };

    // offsets of real-time write buffer contents
    enum {
        WB_CURR_OFFSET = 0,             // one quadlet per channel
        WB_CTRL_OFFSET = NUM_CHANNELS   // control register (power control)
    };

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
};

#endif // __AMPIO_H__
