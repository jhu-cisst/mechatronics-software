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

class ostream;

class AmpIO : public BoardIO
{
public:
    AmpIO(unsigned char board_id, unsigned int numAxes = 4);
    ~AmpIO();

    unsigned long GetFirmwareVersion() const;

    void DisplayReadBuffer(std::ostream &out = std::cout) const;

    //  Interface methods
    unsigned long GetStatus() const;
    unsigned long GetTimestamp() const;
    unsigned char GetDigitalOutput() const;
    unsigned long GetDigitalInput() const;
    unsigned char GetAmpTemperature(unsigned int ) const;
    unsigned long GetMotorCurrent(    unsigned int ) const;
    unsigned long GetAnalogInput(  unsigned int ) const;
    unsigned long GetEncoderPosition( unsigned int ) const;
    unsigned long GetEncoderVelocity( unsigned int ) const;
    unsigned long GetEncoderFrequency(unsigned int ) const;
    // GetAmpEnable: returns true if system is requesting amplifier to
    // be enabled (but, amplifier might be in fault state)
    bool GetAmpEnable( unsigned int ) const;
    // GetAmpStatus: returns true if amplifier is enabled; false if
    // amplifier is in fault state.
    bool GetAmpStatus( unsigned int ) const;

    bool SetPowerEnable(                      bool state );
    bool SetAmpEnable( unsigned char mask, unsigned char state);
    bool EnableSafetyRelay(                   bool state );
    bool SetMotorCurrent(    unsigned int, unsigned long );
    bool SetEncoderPreload(  unsigned int, unsigned long );
    bool SetDigitalOutput(unsigned char mask, unsigned char bits);

    // Methods for reading or programming the FPGA configuration PROM (M25P16)

    // User-supplied callback, called when software needs to wait for an
    // action, or when an error occurs. In the latter case, the error message
    // is passed as a parameter (NULL if no error). If the callback returns
    // false, the current wait operation is aborted.
    typedef bool (*ProgressCallback)(const char *msg);

    // Returns the PROM ID (should be 0x00202015 for the M25P16)
    unsigned long PromGetId();

    // PromGetStatus returns the status register from the M25P16 PROM
    // (this is different than the interface status read from address offset 8)
    // The following masks are for the useful status register bits.
    enum { MASK_WIP = 1, MASK_WEL = 2 };   // status register bit masks
    unsigned long PromGetStatus();

    // PromGetResult returns the result (if any) from the last command sent
    unsigned long PromGetResult();

    // Returns nbytes data read from the specified address
    bool PromReadData(unsigned long addr, unsigned char *data,
                      unsigned int nbytes);

    // Enable programming commands (erase and program page)
    // This sets the WEL bit in the status register.
    // This mode is automatically cleared after a programming command is executed.
    bool PromWriteEnable();

    // Disable programming commands
    // This clears the WEL bit in the status register.    
    bool PromWriteDisable();

    // Erase a sector (64K) at the specified address.
    // This command calls PromWriteEnable.
    // If non-zero, the callback (cb) is called while the software is waiting
    // for the PROM to be erased, or if there is an error.
    bool PromSectorErase(unsigned long addr, const ProgressCallback cb = 0);

    // Program a page (up to 256 bytes) at the specified address.
    // This command calls PromWriteEnable.
    // If non-zero, the callback (cb) is called while the software is waiting
    // for the PROM to be programmed, or if there is an error.
    // Returns the number of bytes programmed (-1 if error).
    int PromProgramPage(unsigned long addr, const unsigned char *bytes,
                        unsigned int nbytes, const ProgressCallback cb = 0);

protected:
    unsigned int NumAxes;   // not currently used

    // Number of channels in the node (4 for QLA)
    enum { NUM_CHANNELS = 4 };

    enum { ReadBufSize = 4+4*NUM_CHANNELS,
           WriteBufSize = NUM_CHANNELS };

    quadlet_t read_buffer[ReadBufSize];     // buffer for real-time reads
    quadlet_t write_buffer[WriteBufSize];   // buffer for real-time writes

    // Offsets of real-time read buffer contents, 30 quadlets
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
        WB_CURR_OFFSET = 0         // one quadlet per channel
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
        VEL_DATA_OFFSET = 6,       // enc data register (velocity)
        FRQ_DATA_OFFSET = 7        // enc data register (frequency)
    };
};

#endif // __AMPIO_H__
