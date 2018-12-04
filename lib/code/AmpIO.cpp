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

#include <iostream>
#include <sstream>

#include "AmpIO.h"
#include "FirewirePort.h"
#include "Amp1394Time.h"

#ifdef _MSC_VER
#include <stdlib.h>
inline quadlet_t bswap_32(quadlet_t data) { return _byteswap_ulong(data); }
#else
#include <byteswap.h>
#endif

const AmpIO_UInt32 VALID_BIT        = 0x80000000;  /*!< High bit of 32-bit word */
const AmpIO_UInt32 MIDRANGE_ADC     = 0x00008000;  /*!< Midrange value of ADC bits */
const AmpIO_UInt32 ENC_PRELOAD      = 0x007fffff;  /*!< Encoder position preload value */
const AmpIO_Int32  ENC_MIDRANGE     = 0x00800000;  /*!< Encoder position midrange value */

const AmpIO_UInt32 PWR_ENABLE       = 0x000c0000;  /*!< Turn pwr_en on             */
const AmpIO_UInt32 PWR_DISABLE      = 0x00080000;  /*!< Turn pwr_en off            */
const AmpIO_UInt32 RELAY_ON         = 0x00030000;  /*!< Turn safety relay on       */
const AmpIO_UInt32 RELAY_OFF        = 0x00020000;  /*!< Turn safety relay off      */
const AmpIO_UInt32 ENABLE_MASK      = 0x0000ffff;  /*!< Mask for power enable bits */
const AmpIO_UInt32 MOTOR_CURR_MASK  = 0x0000ffff;  /*!< Mask for motor current adc bits */
const AmpIO_UInt32 ANALOG_POS_MASK  = 0xffff0000;  /*!< Mask for analog pot ADC bits */
const AmpIO_UInt32 ADC_MASK         = 0x0000ffff;  /*!< Mask for right aligned ADC bits */
const AmpIO_UInt32 DAC_MASK         = 0x0000ffff;  /*!< Mask for 16-bit DAC values */
const AmpIO_UInt32 ENC_POS_MASK     = 0x00ffffff;  /*!< Encoder position mask (24 bits) */
const AmpIO_UInt32 ENC_OVER_MASK    = 0x01000000;  /*!< Encoder bit overflow mask */
const AmpIO_UInt32 ENC_VEL_MASK_16  = 0x0000ffff;  /*!< Mask for encoder velocity (period) bits, Firmware Version <=5 (16 bits) */
const AmpIO_UInt32 ENC_VEL_MASK_22  = 0x003fffff;  /*!< Mask for encoder velocity (period) bits, Firmware Version >=6 (22 bits) */

// Following masks are for the most recent quarter-cycle period and the previous one of the same type (i.e., four cycles ago).
// These are used for estimating acceleration using Firmware Rev 6.
const AmpIO_UInt32 ENC_ACC_REC_MS_MASK   = 0xfff00000;   /*!< Mask (into encoder period) for upper 12 bits of most recent quarter-cycle period */
const AmpIO_UInt32 ENC_ACC_REC_LS_MASK   = 0x3fc00000;   /*!< Mask (into encoder freq/acc) for lower 8 bits of most recent quarter-cycle period */
const AmpIO_UInt32 ENC_ACC_PREV_MASK     = 0x000fffff;   /*!< Mask (into encoder freq/acc) for all 20 bits of previous quarter-cycle period */

// Following offsets are for FPGA Firmware Version 6+ (22 bits)
// (Note that older versions of software assumed that Firmware Version 6 would have different bit assignments)
const AmpIO_UInt32 ENC_VEL_OVER_MASK   = 0x80000000;  /*!< Mask for encoder velocity (period) overflow bit */
const AmpIO_UInt32 ENC_DIR_MASK        = 0x40000000;  /*!< Mask for encoder velocity (period) direction bit */

const AmpIO_UInt32 DAC_WR_A         = 0x00300000;  /*!< Command to write DAC channel A */

const double FPGA_sysclk_MHz        = 49.152;      /* FPGA sysclk in MHz (from FireWire) */
const double VEL_PERD               = 1.0/3072000;    /* Slower clock for velocity measurements (Rev 6 firmware) */
const double VEL_PERD_OLD           = 1.0/768000;     /* Slower clock for velocity measurements (prior to Rev 6 firmware) */

// PROGRESS_CALLBACK: inform the caller when the software is busy waiting: in this case,
//                    the parameter is NULL, but the function returns an error if
//                    the callback returns false.
// ERROR_CALLBACK:    inform the caller of an error; in this case, the error message
//                    (char *) is passed as a parameter, and the return value is ignored.

#define PROGRESS_CALLBACK(CB, ERR)             \
    if (CB) { if (!(*CB)(0)) return ERR; }     \
    else std::cout << '.';

#define ERROR_CALLBACK(CB, MSG)         \
    if (CB) (*CB)(MSG.str().c_str());   \
    else { std::cerr << MSG.str() << std::endl; }


AmpIO_UInt8 BitReverse4[16] = { 0x0, 0x8, 0x4, 0xC,         // 0000, 0001, 0010, 0011
                                0x2, 0xA, 0x6, 0xE,         // 0100, 0101, 0110, 0111
                                0x1, 0x9, 0x5, 0xD,         // 1000, 1001, 1010, 1011
                                0x3, 0xB, 0x7, 0xF };       // 1100, 1101, 1110, 1111

AmpIO::AmpIO(AmpIO_UInt8 board_id, unsigned int numAxes) : BoardIO(board_id), NumAxes(numAxes)
{
    memset(read_buffer, 0, sizeof(read_buffer));
    memset(write_buffer_internal, 0, sizeof(write_buffer_internal));

    // Set members in base class.
    ReadBufferSize = sizeof(read_buffer);
    ReadBuffer = read_buffer;
    WriteBufferSize = sizeof(write_buffer_internal);
    InitWriteBuffer(0, 0);
}

AmpIO::~AmpIO()
{
    if (port) {
        std::cerr << "Warning: AmpIO being destroyed while still in use by FirewirePort" << std::endl;
        port->RemoveBoard(this);
    }
}

void AmpIO::InitWriteBuffer(quadlet_t *buf, size_t data_offset)
{
    if (buf) {
        WriteBuffer = buf;
        WriteBufferData = buf + data_offset;
        memset(WriteBufferData, 0, WriteBufSize*sizeof(quadlet_t));
    }
    else {
        WriteBuffer = write_buffer_internal;
        WriteBufferData = write_buffer_internal;
    }
}

bool AmpIO::WriteBufferResetsWatchdog(void) const
{
    return
        (bswap_32(WriteBufferData[WB_CURR_OFFSET + 0]) & VALID_BIT)
        | (bswap_32(WriteBufferData[WB_CURR_OFFSET + 1]) & VALID_BIT)
        | (bswap_32(WriteBufferData[WB_CURR_OFFSET + 2]) & VALID_BIT)
        | (bswap_32(WriteBufferData[WB_CURR_OFFSET + 3]) & VALID_BIT);
}

AmpIO_UInt32 AmpIO::GetFirmwareVersion(void) const
{
    return (port ? port->GetFirmwareVersion(BoardId) : 0);
}

std::string AmpIO::GetFPGASerialNumber(void)
{
    // Format: FPGA 1234-56 (12 bytes) or FPGA 1234-567 (13 bytes).
    // Note that on PROM, the string is terminated by 0xff because the sector
    // is first erased (all bytes set to 0xff) before the string is written.
    AmpIO_UInt32 address = 0x001FFF00;
    char data[20];
    std::string sn;
    const size_t FPGASNSize = 13;
    const size_t bytesToRead = (FPGASNSize+3)&0xFC;  // must be multiple of 4

    data[FPGASNSize] = 0;    // Make sure null-terminated
    if (PromReadData(address, (AmpIO_UInt8 *)data, bytesToRead)) {
        if (strncmp(data, "FPGA ", 5) == 0) {
            char *p = strchr(data+5, 0xff);
            if (p) *p = 0;      // Null terminate at first 0xff
            sn.assign(data+5);
        }
    }
    else
        std::cerr << "AmpIO::GetFPGASerialNumber: failed to read FPGA Serial Number" << std::endl;
    return sn;
}

std::string AmpIO::GetQLASerialNumber(void)
{
    // Format: QLA 1234-56 or QLA 1234-567.
    // String is terminated by 0 or 0xff.
    AmpIO_UInt16 address = 0x0000;
    AmpIO_UInt8 data[20];
    const size_t QLASNSize = 12;
    std::string sn;

    data[QLASNSize] = 0;  // make sure null-terminated
    for (size_t i = 0; i < QLASNSize; i++) {
        if (!PromReadByte25AA128(address, data[i])) {
            std::cerr << "AmpIO::GetQLASerialNumber: failed to get QLA Serial Number" << std::endl;
            break;
        }
        if (data[i] == 0xff)
            data[i] = 0;
        address += 1;
    }
    if (strncmp((char *)data, "QLA ", 4) == 0)
        sn.assign((char *)data+4);
    return sn;
}

void AmpIO::DisplayReadBuffer(std::ostream &out) const
{
    // first two quadlets are timestamp and status, resp.
    out << std::hex << bswap_32(read_buffer[0]) << std::endl;
    out << std::hex << bswap_32(read_buffer[1]) << std::endl;
    // next two quadlets are digital I/O and amplifier temperature
    out << std::hex << bswap_32(read_buffer[2]) << std::endl;
    out << std::hex << bswap_32(read_buffer[3]) << std::endl;

    // remaining quadlets are in 4 groups of NUM_CHANNELS as follows:
    //   - motor current and analog pot per channel
    //   - encoder position per channel
    //   - encoder velocity per channel
    //   - encoder frequency per channel
    for (int i=4; i<ReadBufSize; i++) {
        out << std::hex << bswap_32(read_buffer[i]) << " ";
        if (!((i-1)%NUM_CHANNELS)) out << std::endl;
    }
    out << std::dec;
}

bool AmpIO::HasEthernet(void) const
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 12, read_data))
        return false;
    // Bit 31 indicates whether Ethernet is present
    return (read_data&0x80000000);
}

AmpIO_UInt32 AmpIO::GetStatus(void) const
{
    return bswap_32(read_buffer[STATUS_OFFSET]);
}

AmpIO_UInt32 AmpIO::GetTimestamp(void) const
{
    return bswap_32(read_buffer[TIMESTAMP_OFFSET]);
}

AmpIO_UInt32 AmpIO::GetDigitalInput(void) const
{
    return bswap_32(read_buffer[DIGIO_OFFSET]);
}

AmpIO_UInt8 AmpIO::GetDigitalOutput(void) const
{
    // Starting with Version 1.3.0 of this library, the digital outputs are inverted
    // before being returned to the caller because they are inverted in hardware and/or firmware.
    // This way, the digital output state matches the hardware state (i.e., 0 means digital output
    // is at 0V).
    AmpIO_UInt8 dout = static_cast<AmpIO_UInt8>((~(bswap_32(read_buffer[DIGIO_OFFSET])>>12))&0x000f);
    // Firmware versions < 5 have bits in reverse order with respect to schematic
    if (GetFirmwareVersion() < 5)
        dout = BitReverse4[dout];
    return dout;
}

AmpIO_UInt8 AmpIO::GetNegativeLimitSwitches(void) const
{
    return (this->GetDigitalInput()&0x0f00)>>8;
}

AmpIO_UInt8 AmpIO::GetPositiveLimitSwitches(void) const
{
    return (this->GetDigitalInput()&0x00f0)>>4;
}

AmpIO_UInt8 AmpIO::GetHomeSwitches(void) const
{
    return (this->GetDigitalInput()&0x00f);
}

AmpIO_UInt8 AmpIO::GetEncoderChannelA(void) const
{
    return (this->GetDigitalInput()&0x0f000000)>>24;
}

bool AmpIO::GetEncoderChannelA(unsigned int index) const
{
    const AmpIO_UInt8 mask = (0x0001 << index);
    return GetEncoderChannelA()&mask;
}

AmpIO_UInt8 AmpIO::GetEncoderChannelB(void) const
{
    return (this->GetDigitalInput()&0x00f00000)>>20;
}

bool AmpIO::GetEncoderChannelB(unsigned int index) const
{
    const AmpIO_UInt8 mask = (0x0001 << index);
    return GetEncoderChannelB()&mask;
}

AmpIO_UInt8 AmpIO::GetEncoderIndex(void) const
{
    return (this->GetDigitalInput()&0x000f0000)>>16;
}

bool AmpIO::GetEncoderOverflow(unsigned int index) const
{
    if (index < NUM_CHANNELS) {
        return bswap_32(read_buffer[index+ENC_POS_OFFSET]) & ENC_OVER_MASK;
    }
    else {
        std::cerr << "AmpIO::GetEncoderOverflow: index out of range " << index
                  << ", nb channels is " << NUM_CHANNELS << std::endl;
    }
    return true; // send error "code"
}

AmpIO_UInt8 AmpIO::GetAmpTemperature(unsigned int index) const
{
    AmpIO_UInt8 temp = 0;
    if (index == 0)
        temp = (bswap_32(read_buffer[TEMP_OFFSET])>>8) & 0x000000ff;
    else if (index == 1)
        temp = bswap_32(read_buffer[TEMP_OFFSET]) & 0x000000ff;
    return temp;
}

AmpIO_UInt32 AmpIO::GetMotorCurrent(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+MOTOR_CURR_OFFSET]);
    buff &= MOTOR_CURR_MASK;       // mask for applicable bits

    return static_cast<AmpIO_UInt32>(buff) & ADC_MASK;
}

AmpIO_UInt32 AmpIO::GetAnalogInput(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+ANALOG_POS_OFFSET]);
    buff &= ANALOG_POS_MASK;       // mask for applicable bits
    buff >>= 16;                   // shift to lsb alignment

    return static_cast<AmpIO_UInt32>(buff) & ADC_MASK;
}

AmpIO_Int32 AmpIO::GetEncoderPosition(unsigned int index) const
{
    if (index < NUM_CHANNELS) {
        return static_cast<AmpIO_Int32>(bswap_32(read_buffer[index + ENC_POS_OFFSET]) & ENC_POS_MASK) - ENC_MIDRANGE;
    }
    return 0;
}

// Returns encoder velocity in counts/sec -> 4/period
// For clarity and efficiency, this duplicates some code rather than calling GetEncoderVelocity.
double AmpIO::GetEncoderVelocityCountsPerSecond(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff = GetEncoderVelocityRaw(index);

    AmpIO_Int32 cnter;
    double vel;
    AmpIO_UInt32 fver = GetFirmwareVersion();
    if (fver < 6) {
        // Prior to Firmware Version 6, the latched counter value is returned
        // as the lower 16 bits. The upper 16 bits are the free-running counter,
        // which is not used in this implementation, but could be used to better
        // handle deceleration (i.e., when the free-running counter value is greater than
        // the latched counter value). But, for firmware prior to Version 6, the
        // returned free-running counter value is for the last encoder edge type rather
        // than for the next expected encoder edge, so it will not work as well.
        cnter = buff & ENC_VEL_MASK_16;
        if (cnter == 0x00008000)  // if overflow
            vel = 0.0;
        else {
            // Sign extend if necessary
            if (cnter & 0x00008000)
                cnter |= 0xffff0000;
            vel = 4.0 * ((double)cnter*VEL_PERD_OLD);
        }
    } else if (fver == 6) {
        // buff[31] = whether full cycle period has overflowed
        // buff[30] = direction of the encoder
        // buff[29:22] = upper 8 bits of most recent quarter-cycle period (for acceleration)
        // buff[21:0] = velocity (22 bits)
        // Clock = 3.072 MHz
        
        // mask and convert to signed
        cnter = buff & ENC_VEL_MASK_22;
        
        if (GetEncoderVelocityOverflow(index)) {
            vel = 0.0;
        } else if (!GetEncoderDir(index)) {
            vel = -4.0/((double)cnter*VEL_PERD);
        } else {
            vel = 4.0/((double)cnter*VEL_PERD);
        }
    }
    else {
        // Not sure what later firmware versions will do
        vel = 0.0;
    }
    return vel;
}

// Returns the time delay of the encoder velocity measurement, in seconds.
// Currently, this is equal to half the measured period, based on the assumption that measuring the
// period over a full cycle (4 quadrature counts) estimates the velocity in the middle of that cycle.
double AmpIO::GetEncoderVelocityDelay(unsigned int index) const
{
    double delay = 0.0;
    AmpIO_Int32 cnter;
    AmpIO_UInt32 fver = GetFirmwareVersion();
    if (fver < 6) {
        cnter = GetEncoderVelocityRaw(index) & ENC_VEL_MASK_16;
        // This is a 16-bit signed value, but we want an unsigned period
        if (cnter & 0x00008000) {
            cnter |= 0xffff0000;   // sign extend
            cnter = -cnter;        // negate to get a positive number
        }
        delay = ((double)cnter * VEL_PERD_OLD)/2.0;
    }
    else if (fver == 6) {
        cnter = GetEncoderVelocityRaw(index) & ENC_VEL_MASK_22;
        delay = ((double)cnter * VEL_PERD)/2.0;
    }
    return delay;
}

// Deprecated: returns encoder period; encoder velocity is 4/period.
// Note that number of bits used for encoder period has changed in later firmware,
// so this function should not be used.
AmpIO_Int32 AmpIO::GetEncoderVelocity(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff = GetEncoderVelocityRaw(index);

    AmpIO_UInt32 fver = GetFirmwareVersion();
    if (fver < 6) {
        // buff = [cnter_now, cnter_latch]
        // cnter_latch: latched counter value
        // cnter_now  : free-running counter value
        // both are signed 16-bit data
        // Clock = 768 kHz
        // PROGRAMMER NOTE: the 16-bit signed value is not sign extended
        //                  to 32 bits (backward compatible behavior)
        return (buff & ENC_VEL_MASK_16);
    }
    else if (fver == 6) {
        AmpIO_Int32 cnter;

        // mask and convert to signed
        cnter = buff & ENC_VEL_MASK_22;
        if (!(buff & ENC_DIR_MASK))
            cnter = -cnter;

        return  cnter;
    }
    else {
        // Not sure what later firmware versions will do
        return buff;
    }
}

// Returns previous encoder period counter (previous full cycle period);
// Valid for firmware version 6.
AmpIO_Int32 AmpIO::GetEncoderPrevCounter(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    AmpIO_Int32 cnter = GetEncoderVelocityRaw(index) & ENC_VEL_MASK_22;
    AmpIO_Int32 prev_perd = GetEncoderAccPrev(index);
    AmpIO_Int32 rec_perd = GetEncoderAccRec(index);

    return cnter - rec_perd + prev_perd;    
}

// Estimate acceleration from two quarters of the same type; units are counts/second**2
// Valid for firmware version 6.
double AmpIO::GetEncoderAcceleration(unsigned int index, double percent_threshold) const
{
    
    if (index >= NUM_CHANNELS)
        return 0L;

    double acc = 0.0;
    if ((GetFirmwareVersion() == 6)) {
        // Not sure how later firmware versions will work

        if (!GetEncoderVelocityOverflow(index)) {
            AmpIO_Int32 prev_perd = GetEncoderAccPrev(index);
            AmpIO_Int32 rec_perd = GetEncoderAccRec(index);
            AmpIO_Int32 prev_cnter = GetEncoderPrevCounter(index);
            AmpIO_Int32 cnter = GetEncoderVelocityRaw(index) & ENC_VEL_MASK_22;

            if ((1.0/rec_perd <= percent_threshold) && (1.0/rec_perd >= -percent_threshold)) {
                acc = 8.0*((double) (prev_perd - rec_perd)/(prev_perd + rec_perd))/((double) cnter * VEL_PERD * (double) prev_cnter * VEL_PERD);
                if (!GetEncoderDir(index))
                    acc = -acc;
            }
        }
    }
    return acc;
}

// Counter over full cycle has overflowed; only valid for Rev 6 firmware
bool AmpIO::GetEncoderVelocityOverflow(unsigned int index) const
{
    quadlet_t buff = GetEncoderVelocityRaw(index);
    return buff & ENC_VEL_OVER_MASK;
}

// Direction of encoder at last velocity reading
bool AmpIO::GetEncoderDir(unsigned int index) const
{
    quadlet_t buff = GetEncoderVelocityRaw(index);
    return buff & ENC_DIR_MASK;
}

// Latch from 5 quarter cycles ago for accleration calculation
// Valid for firmware version 6.
AmpIO_Int32 AmpIO::GetEncoderAccPrev(unsigned int index) const
{
    quadlet_t buff = bswap_32(read_buffer[index+ENC_FRQ_OFFSET]);
    AmpIO_Int32 prev_perd = buff & ENC_ACC_PREV_MASK;
    return prev_perd;
}

// Latch last quarter cycle for acceleration calculation
// Valid for firmware version 6.
AmpIO_Int32 AmpIO::GetEncoderAccRec(unsigned int index) const
{
    AmpIO_UInt32 ms_buff = bswap_32(read_buffer[index+ENC_FRQ_OFFSET]);
    AmpIO_UInt32 ls_buff = GetEncoderVelocityRaw(index);
    AmpIO_Int32 cur_perd = (((ms_buff & ENC_ACC_REC_MS_MASK) >> 12) | ((ls_buff & ENC_ACC_REC_LS_MASK) >> 22)) & ENC_ACC_PREV_MASK;
    return cur_perd;
}

// Raw velocity field; includes period of velocity and other data, depending on firmware version.
// For firmware version 6, includes part of AccRec.
AmpIO_UInt32 AmpIO::GetEncoderVelocityRaw(unsigned int index) const
{
    quadlet_t buff;
    buff = bswap_32(read_buffer[index+ENC_VEL_OFFSET]);
    return buff;
}

// Raw acceleration field; for firmware prior to Version 6, this was actually the encoder "frequency"
// (i.e., number of pulses in specified time period, which can be used to estimate velocity), but was never used.
// Starting with Firmware Version 6, it has been reused to return some data that can be used to estimate
// the acceleration. For testing only.
AmpIO_UInt32 AmpIO::GetEncoderAccelerationRaw(unsigned int index) const
{
    quadlet_t buff;
    buff = bswap_32(read_buffer[index+ENC_FRQ_OFFSET]);
    return buff;
}

AmpIO_Int32 AmpIO::GetEncoderMidRange(void) const
{
    return ENC_MIDRANGE;
}

bool AmpIO::GetPowerStatus(void) const
{
    // Bit 19: MV_GOOD
    return (GetStatus()&0x00080000);
}

bool AmpIO::GetSafetyRelayStatus(void) const
{
    // Bit 17
    return (GetStatus()&0x00020000);
}

bool AmpIO::GetWatchdogTimeoutStatus(void) const
{
    // Bit 23
    return (GetStatus()&0x00800000);
}

bool AmpIO::GetAmpEnable(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return false;
    AmpIO_UInt32 mask = (0x00000001 << index);
    return GetStatus()&mask;
}

bool AmpIO::GetAmpStatus(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return false;
    AmpIO_UInt32 mask = (0x00000100 << index);
    return GetStatus()&mask;
}

AmpIO_UInt32 AmpIO::GetSafetyAmpDisable(void) const
{
    AmpIO_UInt32 mask = 0x000000F0;
    return (GetStatus() & mask) >> 4;
}


/*******************************************************************************
 * Set commands
 */

void AmpIO::SetPowerEnable(bool state)
{
    AmpIO_UInt32 enable_mask = 0x00080000;
    WriteBufferData[WB_CTRL_OFFSET] |=  enable_mask;
    AmpIO_UInt32 state_mask  = 0x00040000;
    if (state)
        WriteBufferData[WB_CTRL_OFFSET] |=  state_mask;
    else
        WriteBufferData[WB_CTRL_OFFSET] &= ~state_mask;
}

bool AmpIO::SetAmpEnable(unsigned int index, bool state)
{
    if (index < NUM_CHANNELS) {
        AmpIO_UInt32 enable_mask = 0x00000100 << index;
        WriteBufferData[WB_CTRL_OFFSET] |=  enable_mask;
        AmpIO_UInt32 state_mask  = 0x00000001 << index;
        if (state)
            WriteBufferData[WB_CTRL_OFFSET] |=  state_mask;
        else
            WriteBufferData[WB_CTRL_OFFSET] &= ~state_mask;
        return true;
    }
    return false;
}

void AmpIO::SetSafetyRelay(bool state)
{
    AmpIO_UInt32 enable_mask = 0x00020000;
    WriteBufferData[WB_CTRL_OFFSET] |=  enable_mask;
    AmpIO_UInt32 state_mask  = 0x00010000;
    if (state)
        WriteBufferData[WB_CTRL_OFFSET] |=  state_mask;
    else
        WriteBufferData[WB_CTRL_OFFSET] &= ~state_mask;
}

bool AmpIO::SetMotorCurrent(unsigned int index, AmpIO_UInt32 sdata)
{
    quadlet_t data = 0x00;
    data = VALID_BIT | ((BoardId & 0x0F) << 24) | DAC_WR_A | (sdata & DAC_MASK);

    if (index < NUM_CHANNELS) {
        WriteBufferData[index+WB_CURR_OFFSET] = bswap_32(data);
        return true;
    }
    else
        return false;
}

/*******************************************************************************
 * Read commands
 */

AmpIO_UInt32 AmpIO::ReadStatus(void) const
{
    AmpIO_UInt32 read_data = 0;
    if (port) port->ReadQuadlet(BoardId, 0, read_data);
    return read_data;
}

bool AmpIO::ReadPowerStatus(void) const
{
    return (ReadStatus()&0x00080000);

}

bool AmpIO::ReadSafetyRelayStatus(void) const
{
    return (ReadStatus()&0x00020000);
}

AmpIO_UInt32 AmpIO::ReadSafetyAmpDisable(void) const
{
    AmpIO_UInt32 read_data = 0;
    // 11: quadlet read address for Safety Amp Disable
    if (port) port->ReadQuadlet(BoardId, 11, read_data);
    return read_data & 0x0000000F;
}

bool AmpIO::ReadEncoderPreload(unsigned int index, AmpIO_Int32 &sdata) const
{
    bool ret = false;
    if (port && (index < NUM_CHANNELS)) {
        AmpIO_UInt32 read_data;
        unsigned int channel = (index+1) << 4;
        ret = port->ReadQuadlet(BoardId, channel | ENC_LOAD_OFFSET, read_data);
        if (ret) sdata = static_cast<AmpIO_Int32>(read_data);
    }
    return ret;
}

AmpIO_UInt32 AmpIO::ReadDigitalIO(void) const
{
    AmpIO_UInt32 read_data = 0;
    if (port) port->ReadQuadlet(BoardId, 0x0a, read_data);
    return read_data;
}

bool AmpIO::ReadDoutControl(unsigned int index, AmpIO_UInt16 &countsHigh, AmpIO_UInt16 &countsLow)
{
    countsHigh = 0;
    countsLow = 0;
    if (GetFirmwareVersion() < 5) {
        std::cerr << "AmpIO::ReadDoutControl: requires firmware 5 or above" << std::endl;
        return false;
    }

    AmpIO_UInt32 read_data;
    unsigned int channel = (index+1) << 4;
    if (port && (index < NUM_CHANNELS)) {
        if (port->ReadQuadlet(BoardId, channel | DOUT_CTRL_OFFSET, read_data)) {
            // Starting with Version 1.3.0 of this library, we swap the high and low times
            // because the digital outputs are inverted in hardware.
            countsLow = static_cast<AmpIO_UInt16>(read_data >> 16);
            countsHigh  = static_cast<AmpIO_UInt16>(read_data);
            return true;
        }
    }
    return false;
}

/*******************************************************************************
 * Write commands
 */

bool AmpIO::WritePowerEnable(bool state)
{
    AmpIO_UInt32 write_data = state ? PWR_ENABLE : PWR_DISABLE;
    return (port ? port->WriteQuadlet(BoardId, 0, write_data) : false);
}

bool AmpIO::WriteAmpEnable(AmpIO_UInt8 mask, AmpIO_UInt8 state)
{
    quadlet_t write_data = (mask << 8) | state;
    return (port ? port->WriteQuadlet(BoardId, 0, write_data) : false);
}

bool AmpIO::WriteSafetyRelay(bool state)
{
    AmpIO_UInt32 write_data = state ? RELAY_ON : RELAY_OFF;
    return (port ? port->WriteQuadlet(BoardId, 0, write_data) : false);
}

bool AmpIO::WriteEncoderPreload(unsigned int index, AmpIO_Int32 sdata)
{
    unsigned int channel = (index+1) << 4;

    if ((sdata >= ENC_MIDRANGE)
            || (sdata < -ENC_MIDRANGE)) {
        std::cerr << "AmpIO::WriteEncoderPreload, preload out of range " << sdata << std::endl;
        return false;
    }
    if (port && (index < NUM_CHANNELS)) {
        return port->WriteQuadlet(BoardId, channel | ENC_LOAD_OFFSET, static_cast<AmpIO_UInt32>(sdata + ENC_MIDRANGE));
    } else {
        return false;
    }
}

bool AmpIO::WriteDigitalOutput(AmpIO_UInt8 mask, AmpIO_UInt8 bits)
{
    // Firmware versions < 5 have bits in reverse order with respect to schematic
    if (GetFirmwareVersion() < 5) {
        mask = BitReverse4[mask&0x0f];
        bits = BitReverse4[bits&0x0f];
    }
    // Starting with Version 1.3.0 of this library, the digital outputs are inverted
    // before being sent because they are inverted in hardware and/or firmware.
    // This way, the digital output state matches the hardware state (i.e., 0 means digital output
    // is at 0V).
    quadlet_t write_data = (mask << 8) | ((~bits)&0x0f);
    return port->WriteQuadlet(BoardId, 6, write_data);
}

bool AmpIO::WriteWatchdogPeriod(AmpIO_UInt32 counts)
{
    // period = counts(16 bits) * 5.208333 us (default = 0 = no timeout)
    return port->WriteQuadlet(BoardId, 3, counts);
}

bool AmpIO::WriteDoutControl(unsigned int index, AmpIO_UInt16 countsHigh, AmpIO_UInt16 countsLow)
{
    if (GetFirmwareVersion() < 5) {
        std::cerr << "AmpIO::WriteDoutControl: requires firmware 5 or above" << std::endl;
        return false;
    }

    // Counter frequency = 49.152 MHz --> 1 count is about 0.02 uS
    //    Max high/low time = (2^16-1)/49.152 usec = 1333.3 usec = 1.33 msec
    //    The max PWM period with full adjustment of duty cycle (1-65535) is (2^16-1+1)/49.152 usec = 1.33 msec
    unsigned int channel = (index+1) << 4;
    if (port && (index < NUM_CHANNELS)) {
        // Starting with Version 1.3.0 of this library, we swap the high and low times
        // because the digital outputs are inverted in hardware.
        AmpIO_UInt32 counts = (static_cast<AmpIO_UInt32>(countsLow) << 16) | countsHigh;
        return port->WriteQuadlet(BoardId, channel | DOUT_CTRL_OFFSET, counts);
    } else {
        return false;
    }
}

bool AmpIO::WritePWM(unsigned int index, double freq, double duty)
{
    // Check for valid frequency (also avoid divide by 0)
    if (freq <= 375.0) return false;
    // Check for valid duty cycle (0-1)
    if ((duty < 0.0) || (duty > 1.0)) return false;
    // Compute high time and low time (in counts). Note that we return false
    // if either time is greater than 16 bits, rather than attempting to adjust.
    // Starting with Version 1.3.0 of this library, digital outputs are inverted
    // to match the actual output. This function does not need to be changed,
    // however,  because the inversion is performed in WriteDoutControl and
    // WriteDigitalOutput.
    AmpIO_UInt32 highTime = GetDoutCounts(duty/freq);
    if (highTime > 65535L) return false;
    AmpIO_UInt32 lowTime = GetDoutCounts((1.0-duty)/freq);
    if (lowTime > 65535L) return false;
    // Following can occur if frequency is too high
    if ((highTime == 0) && (lowTime == 0)) return false;
    bool ret;
    // If highTime is 0, then turn off PWM at low value
    if (highTime == 0) {
        ret = WriteDoutControl(index, 0, 0);
        ret &= WriteDigitalOutput((1<<index), 0);
    }
    // If lowTime is 0, then turn off PWM at high value
    else if (lowTime == 0) {
        ret = WriteDoutControl(index, 0, 0);
        ret &= WriteDigitalOutput((1<<index), 1);
    }
    else
        ret = WriteDoutControl(index, static_cast<AmpIO_UInt16>(highTime), static_cast<AmpIO_UInt16>(lowTime));
    return ret;
}

AmpIO_UInt32 AmpIO::GetDoutCounts(double time) const
{
    return static_cast<AmpIO_UInt32>((FPGA_sysclk_MHz*1e6)*time + 0.5);
}

/*******************************************************************************
 * PROM commands (M25P16)
 *    data e.g. 0x9f000000 the higher 8-bit is M25P16 cmd
 *    see DataSheet Table 5: Command Set Codes
 */

AmpIO_UInt32 AmpIO::PromGetId(void)
{
    AmpIO_UInt32 id = 0;
    quadlet_t data = 0x9f000000;
    if (port->WriteQuadlet(BoardId, 0x08, data)) {
        port->PromDelay();
        // Should be ready by now...
        PromGetResult(id);
    }
    return id;
}

bool AmpIO::PromGetStatus(AmpIO_UInt32 &status, PromType type)
{
    quadlet_t data = 0x05000000;
    nodeaddr_t address = GetPromAddress(type, true);

    bool ret = port->WriteQuadlet(BoardId, address, data);
    if (ret) {
        port->PromDelay();
        // Should be ready by now...
        ret = PromGetResult(status, type);
    }
    return ret;
}

bool AmpIO::PromGetResult(AmpIO_UInt32 &result, PromType type)
{
    quadlet_t read_data;
    nodeaddr_t address = GetPromAddress(type, false);

    bool ret = port->ReadQuadlet(BoardId, address, read_data);
    if (ret)
        result = static_cast<AmpIO_UInt32>(read_data);
    return ret;
}

bool AmpIO::PromReadData(AmpIO_UInt32 addr, AmpIO_UInt8 *data,
                         unsigned int nbytes)
{
    AmpIO_UInt32 addr24 = addr&0x00ffffff;
    if (addr24+nbytes > 0x00ffffff)
        return false;
    quadlet_t write_data = 0x03000000|addr24;  // 03h = Read Data Bytes
    AmpIO_UInt32 page = 0;
    while (page < nbytes) {
        const unsigned int maxReadSize = 64u;
        unsigned int bytesToRead = ((nbytes-page)<maxReadSize) ? (nbytes-page) : maxReadSize;
        if (!port->WriteQuadlet(BoardId, 0x08, write_data))
            return false;
        // Read FPGA status register; if 4 LSB are 0, command has finished.
        // The IEEE-1394 clock is 24.576 MHz, so it should take
        // about 256*8*(1/24.576) = 83.3 microseconds to read 256 bytes.
        // Experimentally, 1 iteration of the loop below is sufficient
        // most of the time, with 2 iterations required occasionally.
        quadlet_t read_data = 0x000f;
        int i;
        const int MAX_LOOP_CNT = 8;
        for (i = 0; (i < MAX_LOOP_CNT) && read_data; i++) {
            Amp1394_Sleep(0.00001);   // 10 usec
            if (!port->ReadQuadlet(BoardId, 0x08, read_data)) return false;
            read_data = read_data&0x000f;
        }
        if (i == MAX_LOOP_CNT) {
            std::cout << "PromReadData: command failed to finish, status = "
                      << std::hex << read_data << std::dec << std::endl;
            return false;
        }
        // Now, read result. This should be the number of quadlets written.
        AmpIO_UInt32 nRead;
        if (!PromGetResult(nRead)) {
            std::cout << "PromReadData: failed to get PROM result" << std::endl;
            return false;
        }
        nRead *= 4;
        // should never happen, as for now firmware always reads 256 bytes
        // and saves to local registers in FPGA
        if (nRead != 256) {
            std::cout << "PromReadData: incorrect number of bytes = "
                      << nRead << std::endl;
            return false;
        }

        AmpIO_UInt32 fver = GetFirmwareVersion();
        nodeaddr_t address;
        if (fver >= 4) {address = 0x2000;}
        else {address = 0xc0;}
        if (!port->ReadBlock(BoardId, address, (quadlet_t *)(data+page), bytesToRead))
            return false;
        write_data += bytesToRead;
        page += bytesToRead;
    }
    return true;
}

bool AmpIO::PromWriteEnable(PromType type)
{
    quadlet_t write_data = 0x06000000;
    nodeaddr_t address = GetPromAddress(type, true);
    return port->WriteQuadlet(BoardId, address, write_data);
}

bool AmpIO::PromWriteDisable(PromType type)
{
    quadlet_t write_data = 0x04000000;
    nodeaddr_t address = GetPromAddress(type, true);
    return port->WriteQuadlet(BoardId, address, write_data);
}

bool AmpIO::PromSectorErase(AmpIO_UInt32 addr, const ProgressCallback cb)
{
    PromWriteEnable();
    quadlet_t write_data = 0xd8000000 | (addr&0x00ffffff);
    if (!port->WriteQuadlet(BoardId, 0x08, write_data))
        return false;
    // Wait for erase to finish
    AmpIO_UInt32 status;
    if (!PromGetStatus(status))
        return false;
    while (status) {
        PROGRESS_CALLBACK(cb, false);
        if (!PromGetStatus(status))
            return false;
    }
    return true;
}

int AmpIO::PromProgramPage(AmpIO_UInt32 addr, const AmpIO_UInt8 *bytes,
                           unsigned int nbytes, const ProgressCallback cb)
{
    const unsigned int MAX_PAGE = 256;  // 64 quadlets
    if (nbytes > MAX_PAGE) {
        std::ostringstream msg;
        msg << "AmpIO::PromProgramPage: error, nbytes = " << nbytes
            << " (max = " << MAX_PAGE << ")";
        ERROR_CALLBACK(cb, msg);
        return -1;
    }
    PromWriteEnable();
    // Block write of the data (+1 quad for command)
    AmpIO_UInt8 page_data[MAX_PAGE+sizeof(quadlet_t)];
    quadlet_t *data_ptr = reinterpret_cast<quadlet_t *>(page_data);
    // First quadlet is the "page program" instruction (0x02)
    data_ptr[0] = bswap_32(0x02000000 | (addr & 0x00ffffff));
    // Remaining quadlets are the data to be programmed. These do not
    // need to be byte-swapped.
    memcpy(page_data+sizeof(quadlet_t), bytes, nbytes);

    // select address based on firmware version number
    AmpIO_UInt32 fver = GetFirmwareVersion();
    nodeaddr_t address;
    if (fver >= 4) {address = 0x2000;}
    else {address = 0xc0;}
    if (!port->WriteBlock(BoardId, address, data_ptr, nbytes+sizeof(quadlet_t))) {
        std::ostringstream msg;
        msg << "AmpIO::PromProgramPage: failed to write block, nbytes = " << nbytes;
        ERROR_CALLBACK(cb, msg);
        return -1;
    }
    // Read FPGA status register; if 4 LSB are 0, command has finished
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 0x08, read_data)) return -1;
    while (read_data&0x000f) {
        PROGRESS_CALLBACK(cb, -1);
        if (!port->ReadQuadlet(BoardId, 0x08, read_data)) return -1;
    }
    if (read_data & 0xff000000) { // shouldn't happen
        std::ostringstream msg;
        msg << "AmpIO::PromProgramPage: FPGA error = " << read_data;
        ERROR_CALLBACK(cb, msg);
    }
    // Now, read result. This should be the number of quadlets written.
    AmpIO_UInt32 nWritten;
    if (!PromGetResult(nWritten)) {
        std::ostringstream msg;
        msg << "AmpIO::PromProgramPage: could not get PROM result";
        ERROR_CALLBACK(cb, msg);
    }
    if (nWritten > 0)
        nWritten = 4*(nWritten-1);  // convert from quadlets to bytes
    if (nWritten != nbytes) {
        std::ostringstream msg;
        msg << "AmpIO::PromProgramPage: wrote " << nWritten << " of "
            << nbytes << " bytes";
        ERROR_CALLBACK(cb, msg);
    }
    // Wait for "Write in Progress" bit to be cleared
    AmpIO_UInt32 status;
    bool ret = PromGetStatus(status);
    while (status&MASK_WIP) {
        if (ret) {
            PROGRESS_CALLBACK(cb, 0);
        }
        else {
            std::ostringstream msg;
            msg << "AmpIO::PromProgramPage: could not get PROM status" << std::endl;
            ERROR_CALLBACK(cb, msg);
        }
        ret = PromGetStatus(status);
    }
    return nWritten;
}


nodeaddr_t AmpIO::GetPromAddress(PromType type, bool isWrite)
{
    if (type == PROM_M25P16 && isWrite)
        return 0x0008;
    else if (type == PROM_M25P16 && !isWrite)
        return 0x0009;
    else if (type == PROM_25AA128 && isWrite)
        return 0x3000;
    else if (type == PROM_25AA128 && !isWrite)
        return 0x3002;
    else
        std::cerr << "AmpIO::GetPromAddress: unsupported PROM type " << type << std::endl;

    return 0x00;
}


// ********************** QLA PROM ONLY Methods ***********************************
bool AmpIO::PromReadByte25AA128(AmpIO_UInt16 addr, AmpIO_UInt8 &data)
{
    // 8-bit cmd + 16-bit addr (2 MSBs ignored)
    AmpIO_UInt32 result = 0x00000000;
    quadlet_t write_data = 0x03000000|(addr << 8);
    nodeaddr_t address = GetPromAddress(PROM_25AA128, true);

    if (port->WriteQuadlet(BoardId, address, write_data)) {
        port->PromDelay();
        // Should be ready by now...
        if (!PromGetResult(result, PROM_25AA128))
            return false;
        // Get the last 8-bit of result
        data = result & 0xFF;
        return true;
    } else {
        data = 0x00;
        return false;
    }
}

bool AmpIO::PromWriteByte25AA128(AmpIO_UInt16 addr, const AmpIO_UInt8 &data)
{
    // enable write
    PromWriteEnable(PROM_25AA128);
    Amp1394_Sleep(0.0001);   // 100 usec

    // 8-bit cmd + 16-bit addr + 8-bit data
    quadlet_t write_data = 0x02000000|(addr << 8)|data;
    nodeaddr_t address = GetPromAddress(PROM_25AA128, true);
    if (port->WriteQuadlet(BoardId, address, write_data)) {
        // wait 5ms for the PROM to be ready to take new commands
        Amp1394_Sleep(0.005);
        return true;
    }
    else
        return false;
}


// Read block data (quadlet)
bool AmpIO::PromReadBlock25AA128(AmpIO_UInt16 addr, quadlet_t *data, unsigned int nquads)
{
    // nquads sanity check
    if (nquads == 0 || nquads > 16) {
        std::cout << "invalid number of quadlets" << std::endl;
        return false;
    }

    // trigger read
    quadlet_t write_data = 0xFE000000|(addr << 8)|(nquads-1);
    nodeaddr_t address = GetPromAddress(PROM_25AA128, true);
    if (!port->WriteQuadlet(BoardId, address, write_data))
        return false;

    // get result
    if (!port->ReadBlock(BoardId, address, data, nquads * 4))
        return false;
    else
        return true;
}


// Write block data (quadlet)
bool AmpIO::PromWriteBlock25AA128(AmpIO_UInt16 addr, quadlet_t *data, unsigned int nquads)
{
    // address sanity check
    if (nquads == 0 || nquads > 16) {
        std::cout << "invalid number of quadlets" << std::endl;
        return false;
    }

    // block write data to buffer
    if (!port->WriteBlock(BoardId, 0x3100, data, nquads*4))
        return false;

    // enable write
    PromWriteEnable(PROM_25AA128);

    // trigger write
    quadlet_t write_data = 0xFF000000|(addr << 8)|(nquads-1);
    nodeaddr_t address = GetPromAddress(PROM_25AA128, true);
    return port->WriteQuadlet(BoardId, address, write_data);
}

// ********************** Dallas DS2505 (1-wire) Methods ****************************************
bool AmpIO::DallasWriteControl(AmpIO_UInt32 ctrl)
{
    if (GetFirmwareVersion() < 7) return false;
    return port->WriteQuadlet(BoardId, 13, ctrl);
}


bool AmpIO::DallasReadStatus(AmpIO_UInt32 &status)
{
    status = 0;
    if (GetFirmwareVersion() < 7) return false;
    return port->ReadQuadlet(BoardId, 13, status);
}

bool AmpIO::DallasWaitIdle()
{
    int i;
    AmpIO_UInt32 status;
    // Wait up to 500 msec. Based on measurements, approximate wait time is 250-300 msec.
    for (i = 0; i < 500; i++) {
        // Wait 1 msec
        Amp1394_Sleep(0.001);
        if (!DallasReadStatus(status)) return false;
        // Done when in idle state
        if ((status&0x000000F0) == 0)
            break;
    }
    //std::cerr << "Wait time = " << i << " milliseconds" << std::endl;
    return (i < 500);
}

bool AmpIO::DallasReadMemory(unsigned short addr, unsigned char *data, unsigned int nbytes)
{
    if (GetFirmwareVersion() < 7) return false;
    AmpIO_UInt32 status = ReadStatus();
    // Check whether bi-directional I/O is available
    if ((status & 0x00300000) != 0x00300000) return false;
    if (!DallasWriteControl((addr<<16)|2)) return false;
    if (!DallasWaitIdle()) return false;
    if (!DallasReadStatus(status)) return false;
    // Check family_code, dout_cfg_bidir, ds_reset, and ds_enable
    if ((status & 0xFF00000F) != 0x0B00000B) return false;
    nodeaddr_t address = 0x6000;
    unsigned char *ptr = data;
    // Read first block of data (up to 256 bytes)
    unsigned int nb = (nbytes>256) ? 256 : nbytes;
    if (!port->ReadBlock(BoardId, address, reinterpret_cast<quadlet_t *>(ptr), nb)) return false;
    ptr += nb;
    nbytes -= nb;
    // Read additional blocks of data if necessary
    while (nbytes > 0) {
        if (!DallasWriteControl(3)) return false;
        if (!DallasWaitIdle()) return false;
        nb = (nbytes>256) ? 256 : nbytes;
        if (!port->ReadBlock(BoardId, address, reinterpret_cast<quadlet_t *>(ptr), nb)) return false;
        ptr += nb;
        nbytes -= nb;
    }
    return true;
}

// ********************** KSZ8851 Ethernet Controller Methods ***********************************

bool AmpIO::ResetKSZ8851()
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x04000000;
    return port->WriteQuadlet(BoardId, 12, write_data);
}

bool AmpIO::WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt8 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x02000000 | (static_cast<quadlet_t>(addr) << 16) | data;
    return port->WriteQuadlet(BoardId, 12, write_data);
}

bool AmpIO::WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt16 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x03000000 | (static_cast<quadlet_t>(addr) << 16) | data;
    return port->WriteQuadlet(BoardId, 12, write_data);
}

bool AmpIO::ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt8 &rdata)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = (static_cast<quadlet_t>(addr) << 16) | rdata;
    if (!port->WriteQuadlet(BoardId, 12, write_data))
        return false;
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 12, read_data))
        return false;
    // Bit 31 indicates whether Ethernet is present
    if (!(read_data&0x80000000)) return false;
    // Bit 30 indicates whether last command had an error
    if (read_data&0x40000000) return false;
    rdata = static_cast<AmpIO_UInt8>(read_data);
    return true;
}

bool AmpIO::ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt16 &rdata)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x01000000 | (static_cast<quadlet_t>(addr) << 16) | rdata;
    if (!port->WriteQuadlet(BoardId, 12, write_data)) {
        std::cout << "WriteQuadlet failed" << std::endl;
        return false;
    }
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 12, read_data)) {
        std::cout << "ReadQuadlet failed" << std::endl;
        return false;
    }
    // Bit 31 indicates whether Ethernet is present
    if (!(read_data&0x80000000)) return false;
    // Bit 30 indicates whether last command had an error
    if (read_data&0x40000000) return false;
    rdata = static_cast<AmpIO_UInt16>(read_data);
    return true;
}

// DMA access (no address specified)
// This assumes that the chip has already been placed in DMA mode
// (e.g., by writing to register 0x82).

bool AmpIO::WriteKSZ8851DMA(const AmpIO_UInt16 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x0B000000 | data;
    return port->WriteQuadlet(BoardId, 12, write_data);
}

bool AmpIO::ReadKSZ8851DMA(AmpIO_UInt16 &rdata)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x09000000 | rdata;
    if (!port->WriteQuadlet(BoardId, 12, write_data))
        return false;
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 12, read_data))
        return false;
    // Bit 31 indicates whether Ethernet is present
    if (!(read_data&0x80000000)) return false;
    // Bit 30 indicates whether last command had an error
    if (read_data&0x40000000) return false;
    rdata = static_cast<AmpIO_UInt16>(read_data);
    return true;
}

AmpIO_UInt16 AmpIO::ReadKSZ8851ChipID()
{
    AmpIO_UInt16 data;
    if (ReadKSZ8851Reg(0xC0, data))
        return data;
    else
        return 0;
}

AmpIO_UInt16 AmpIO::ReadKSZ8851Status()
{
    if (GetFirmwareVersion() < 5) return 0;
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 12, read_data))
        return 0;
    return static_cast<AmpIO_UInt16>(read_data>>16);
}
