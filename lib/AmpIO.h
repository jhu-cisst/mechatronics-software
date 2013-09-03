/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen

  (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __AMPIO_H__
#define __AMPIO_H__

#include "BoardIO.h"
#include <stdint.h> // for uint32_t
#include <iostream>

class ostream;

typedef uint32_t AmpIO_UInt32;
typedef uint8_t AmpIO_UInt8;

class AmpIO : public BoardIO
{
public:
    AmpIO(AmpIO_UInt8 board_id, unsigned int numAxes = 4);
    ~AmpIO();

    AmpIO_UInt32 GetFirmwareVersion(void) const;
    void DisplayReadBuffer(std::ostream &out = std::cout) const;

    // *********************** GET Methods ***********************************
    // The GetXXX methods below return data from local buffers that were filled
    // by FirewirePort::ReadAllBoards. To read data immediately from the boards,
    // use the ReadXXX methods, but note that individual reads via IEEE-1394 are
    // less efficient than block transfers.

    AmpIO_UInt32 GetStatus(void) const;

    AmpIO_UInt32 GetTimestamp(void) const;

    AmpIO_UInt8 GetDigitalOutput(void) const;

    /*! Utility method, uses result of GetDigitalOutput but performs
      required shift and mask. */
    AmpIO_UInt8 GetNegativeLimitSwitches(void) const;

    /*! Utility method, uses result of GetDigitalOutput but performs
      required shift and mask. */
    AmpIO_UInt8 GetPositiveLimitSwitches(void) const;

    /*! Utility method, uses result of GetDigitalOutput but performs
      required shift and mask. */
    AmpIO_UInt8 GetHomeSwitches(void) const;

    AmpIO_UInt32 GetDigitalInput(void) const;

    AmpIO_UInt8 GetAmpTemperature(unsigned int index) const;

    AmpIO_UInt32 GetMotorCurrent(unsigned int index) const;

    AmpIO_UInt32 GetAnalogInput(unsigned int index) const;

    AmpIO_UInt32 GetEncoderPosition(unsigned int index) const;

    AmpIO_UInt32 GetEncoderVelocity(unsigned int index) const;

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

    bool SetMotorCurrent(unsigned int index, AmpIO_UInt32 mcur, const bool isbroadcast = false);

    // ********************** READ Methods ***********************************
    // The ReadXXX methods below read data directly from the boards via IEEE-1394
    // (Firewire) using quadlet reads. In most cases, it is better to use the
    // GetXXX methods which read data from a buffer that is read from the board
    // via the more efficient IEEE-1394 block transfer.

    AmpIO_UInt32 ReadStatus(void) const;
    bool ReadPowerStatus(void) const;
    bool ReadSafetyRelayStatus(void) const;
    AmpIO_UInt32 ReadSafetyAmpDisable(void) const;

    // ********************** WRITE Methods **********************************

    // Enable motor power to the entire board (it is still necessary
    // to enable power to the individual amplifiers).
    bool WritePowerEnable(bool state);

    // Enable individual amplifiers
    bool WriteAmpEnable(AmpIO_UInt8 mask, AmpIO_UInt8 state);

    bool WriteSafetyRelay(bool state);

    bool WriteEncoderPreload(unsigned int index, AmpIO_UInt32 enc);

    bool WriteDigitalOutput(AmpIO_UInt8 mask, AmpIO_UInt8 bits);

    bool WriteWatchdogPeriod(AmpIO_UInt32 counts);

    // ********************** PROM Methods ***********************************
    // Methods for reading or programming the FPGA configuration PROM (M25P16)

    // User-supplied callback, called when software needs to wait for an
    // action, or when an error occurs. In the latter case, the error message
    // is passed as a parameter (NULL if no error). If the callback returns
    // false, the current wait operation is aborted.
    typedef bool (*ProgressCallback)(const char *msg);

    // Returns the PROM ID (should be 0x00202015 for the M25P16)
    AmpIO_UInt32 PromGetId(void);

    // PromGetStatus returns the status register from the M25P16 PROM
    // (this is different than the interface status read from address offset 8)
    // The following masks are for the useful status register bits.
    enum { MASK_WIP = 1, MASK_WEL = 2 };   // status register bit masks
    AmpIO_UInt32 PromGetStatus(void);

    // PromGetResult returns the result (if any) from the last command sent
    AmpIO_UInt32 PromGetResult(void);

    // Returns nbytes data read from the specified address
    bool PromReadData(AmpIO_UInt32 addr, AmpIO_UInt8 *data,
                      unsigned int nbytes);

    // Enable programming commands (erase and program page)
    // This sets the WEL bit in the status register.
    // This mode is automatically cleared after a programming command is executed.
    bool PromWriteEnable(void);

    // Disable programming commands
    // This clears the WEL bit in the status register.
    bool PromWriteDisable(void);

    // Erase a sector (64K) at the specified address.
    // This command calls PromWriteEnable.
    // If non-zero, the callback (cb) is called while the software is waiting
    // for the PROM to be erased, or if there is an error.
    bool PromSectorErase(AmpIO_UInt32 addr, const ProgressCallback cb = 0);

    // Program a page (up to 256 bytes) at the specified address.
    // This command calls PromWriteEnable.
    // If non-zero, the callback (cb) is called while the software is waiting
    // for the PROM to be programmed, or if there is an error.
    // Returns the number of bytes programmed (-1 if error).
    int PromProgramPage(AmpIO_UInt32 addr, const AmpIO_UInt8 *bytes,
                        unsigned int nbytes, const ProgressCallback cb = 0);

protected:
    unsigned int NumAxes;   // not currently used

    // Number of channels in the node (4 for QLA)
    enum { NUM_CHANNELS = 4 };

    enum { ReadBufSize = 4+3*NUM_CHANNELS,
           WriteBufSize = NUM_CHANNELS+1 };

    quadlet_t read_buffer[ReadBufSize];     // buffer for real-time reads
    quadlet_t write_buffer[WriteBufSize];   // buffer for real-time writes

    // Offsets of real-time read buffer contents, 16 = 4 + 2 * 4 quadlets
    enum {
        TIMESTAMP_OFFSET  = 0,    // one quadlet
        STATUS_OFFSET     = 1,    // one quadlet
        DIGIO_OFFSET      = 2,    // digital I/O (one quadlet)
        TEMP_OFFSET       = 3,    // temperature (one quadlet)
        MOTOR_CURR_OFFSET = 4,    // half quadlet per channel (lower half)
        ANALOG_POS_OFFSET = 4,    // half quadlet per channel (upper half)
        ENC_POS_OFFSET    = 4+NUM_CHANNELS,    // one quadlet per channel
        ENC_VEL_OFFSET    = 4+2*NUM_CHANNELS   // one quadlet per channel
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
        VEL_DATA_OFFSET = 6        // enc data register (velocity)
    };
};

#endif // __AMPIO_H__
