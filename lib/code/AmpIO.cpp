/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Jie Ying Wu

  (C) Copyright 2011-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>
#include <sstream>
#include <algorithm>   // for std::max

#include "AmpIO.h"
#include "BasePort.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"

const AmpIO_UInt32 VALID_BIT        = 0x80000000;  /*!< High bit of 32-bit word */
const AmpIO_UInt32 COLLECT_BIT      = 0x40000000;  /*!< Enable data collection on FPGA */
const AmpIO_UInt32 MIDRANGE_ADC     = 0x00008000;  /*!< Midrange value of ADC bits */
const AmpIO_UInt32 ENC_PRELOAD      = 0x007fffff;  /*!< Encoder position preload value */
const AmpIO_Int32  ENC_MIDRANGE     = 0x00800000;  /*!< Encoder position midrange value */

const AmpIO_UInt32 DOUT_CFG_RESET   = 0x01000000;  /*!< Reset DOUT config (Rev 7+) */
const AmpIO_UInt32 REBOOT_FPGA      = 0x00300000;  /*!< Reboot FPGA (Rev 7+)       */
const AmpIO_UInt32 PWR_ENABLE       = 0x000c0000;  /*!< Turn pwr_en on             */
const AmpIO_UInt32 PWR_DISABLE      = 0x00080000;  /*!< Turn pwr_en off            */
const AmpIO_UInt32 RELAY_ON         = 0x00030000;  /*!< Turn safety relay on       */
const AmpIO_UInt32 RELAY_OFF        = 0x00020000;  /*!< Turn safety relay off      */
const AmpIO_UInt32 ENABLE_MASK      = 0x0000ffff;  /*!< Mask for power enable bits */
const AmpIO_UInt32 MOTOR_CURR_MASK  = 0x0000ffff;  /*!< Mask for motor current adc bits */
const AmpIO_UInt32 ANALOG_POS_MASK  = 0xffff0000;  /*!< Mask for analog pot ADC bits */
const AmpIO_UInt32 ADC_MASK         = 0x0000ffff;  /*!< Mask for right aligned ADC bits */
const AmpIO_UInt32 DAC_MASK         = 0x0000ffff;  /*!< Mask for 16-bit DAC values */
const AmpIO_UInt32 RESET_KSZ8851    = 0x04000000;  /*!< Mask to reset KSZ8851 Ethernet chip */
const AmpIO_UInt32 ENC_POS_MASK     = 0x00ffffff;  /*!< Encoder position mask (24 bits) */
const AmpIO_UInt32 ENC_OVER_MASK    = 0x01000000;  /*!< Encoder bit overflow mask */
const AmpIO_UInt32 ENC_VEL_MASK_16  = 0x0000ffff;  /*!< Mask for encoder velocity (period) bits, Firmware Version <= 5 (16 bits) */
const AmpIO_UInt32 ENC_VEL_MASK_22  = 0x003fffff;  /*!< Mask for encoder velocity (period) bits, Firmware Version == 6 (22 bits) */
const AmpIO_UInt32 ENC_VEL_MASK_26  = 0x03ffffff;  /*!< Mask for encoder velocity (period) bits, Firmware Version >= 7 (26 bits) */

// The following masks read the most recent quarter-cycle period and the previous one of the same type, which are used
// for estimating acceleration in Firmware Rev 6+.
// Following are for Firmware Rev 6, which split the bits among different fields due to packet size limitations.
const AmpIO_UInt32 ENC_ACC_REC_MS_MASK   = 0xfff00000;   /*!< Mask (into encoder freq/acc) for upper 12 bits of most recent quarter-cycle period */
const AmpIO_UInt32 ENC_ACC_REC_LS_MASK   = 0x3fc00000; /*!< Mask (into encoder period) for lower 8 bits of most recent quarter-cycle period */
const AmpIO_UInt32 ENC_ACC_PREV_MASK     = 0x000fffff; /*!< Mask (into encoder period) for all 20 bits of previous quarter-cycle period */
// Following are for Firmware Rev 7+, which increased the block read packet size, allowing all bits to be grouped together
const AmpIO_UInt32 ENC_VEL_QTR_MASK   = 0x03ffffff;   /*!< Mask (into encoder QTR1/QTR5) for all 26 bits of quarter cycle period */

// Following offsets are for FPGA Firmware Version 6 (22 bits) and 7+ (26 bits)
// (Note that older versions of software assumed that Firmware Version 6 would have different bit assignments)
const AmpIO_UInt32 ENC_VEL_OVER_MASK   = 0x80000000;  /*!< Mask for encoder velocity (period) overflow bit */
const AmpIO_UInt32 ENC_DIR_MASK        = 0x40000000;  /*!< Mask for encoder velocity (period) direction bit */
const AmpIO_UInt32 ENC_DIR_CHANGE_MASK = 0x20000000;  /*!< Mask for encoder velocity (period) direction change (V7+) */

const double FPGA_sysclk_MHz        = 49.152;         /* FPGA sysclk in MHz (from FireWire) */
const double VEL_PERD               = 1.0/49152000;   /* Clock period for velocity measurements (Rev 7+ firmware) */
const double VEL_PERD_REV6          = 1.0/3072000;    /* Slower clock for velocity measurements (Rev 6 firmware) */
const double VEL_PERD_OLD           = 1.0/768000;     /* Slower clock for velocity measurements (prior to Rev 6 firmware) */

const double WDOG_ClockPeriod       = 256.0/(FPGA_sysclk_MHz*1e6);   /* Watchdog clock period, in seconds */

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

void AmpIO::EncoderVelocityData::Init()
{
    clkPeriod = 1.0;
    velPeriod = 0;
    velOverflow = false;
    velPeriodMax = 0;
    velDir = false;
    dirChange = false;
    encError = false;
    partialCycle = true;
    qtr1Period = 0;
    qtr1Overflow = false;
    qtr1Dir = false;
    qtr1Edges = 0;
    qtr5Period = 0;
    qtr5Overflow = false;
    qtr5Dir = false;
    qtr5Edges = 0;
    qtrPeriodMax = 0;
    runPeriod = 0;
    runOverflow = false;
}

// TODO: hack
uint16_t ampio_pi_fixed_from_float(float val) {
    int result = int(val * 4096);
    if (result < 0) result = 0;
    if (result > 0xffff) result = 0xffff;
    return result;
}

AmpIO::AmpIO(AmpIO_UInt8 board_id, unsigned int numAxes) : BoardIO(board_id), NumAxes(numAxes),
                                                           firmwareTime(0.0), collect_state(false), collect_cb(0)
{
    memset(ReadBuffer, 0, sizeof(ReadBuffer));
    InitWriteBuffer();
    for (size_t i = 0; i < NUM_CHANNELS; i++) {
        encVelData[i].Init();
        encErrorCount[i] = 0;
    }
}

AmpIO::~AmpIO()
{
    if (port) {
        std::cerr << "Warning: AmpIO being destroyed while still in use by "
                  << BasePort::PortTypeString(port->GetPortType()) <<" Port" << std::endl;
        port->RemoveBoard(this);
    }
}

unsigned int AmpIO::GetReadNumBytes() const
{
    auto v = GetFirmwareVersion();
    if (v < 7) {
        return (ReadBufSize_Old*sizeof(quadlet_t));
    } else if (v == 7) {
        return (ReadBufSize*sizeof(quadlet_t));
    } else {
        return ReadBufSize_v8*sizeof(quadlet_t); // TODO
    } 
}

void AmpIO::SetReadData(const quadlet_t *buf)
{
    unsigned int numQuads = GetReadNumBytes() / sizeof(quadlet_t);
    size_t i;
    for (i = 0; i < numQuads; i++)
        ReadBuffer[i] = bswap_32(buf[i]);
    for (i = 0; i < NUM_ENCODERS; i++)
        SetEncoderVelocityData(i);
    // Add 1 to timestamp because block read clears counter, rather than incrementing
    firmwareTime += (GetTimestamp()+1)*GetFPGAClockPeriod();
}

void AmpIO::InitWriteBuffer(void)
{
    if (GetFirmwareVersion() < 8) {
        quadlet_t data = (BoardId & 0x0F) << 24;
        for (size_t i = 0; i < NUM_CHANNELS; i++)
            WriteBuffer[WB_CURR_OFFSET+i] = data;
        WriteBuffer[WB_CTRL_OFFSET] = 0;
    } else {
        std::fill(std::begin(WriteBuffer), std::end(WriteBuffer), 0);
        WriteBuffer[WB_HEADER_OFFSET] = (BoardId & 0x0F) << 8 | ((WriteBufSize_v8) & 0x0F);
    }
}

bool AmpIO::GetWriteData(quadlet_t *buf, unsigned int offset, unsigned int numQuads, bool doSwap) const
{

    for (size_t i = 0; i < numQuads; i++)
        buf[i] = doSwap ? bswap_32(WriteBuffer[offset+i]) : WriteBuffer[offset+i];
    return true;
}

bool AmpIO::WriteBufferResetsWatchdog(void) const
{
    return
        (WriteBuffer[WB_CURR_OFFSET + 0] & VALID_BIT)
        | (WriteBuffer[WB_CURR_OFFSET + 1] & VALID_BIT)
        | (WriteBuffer[WB_CURR_OFFSET + 2] & VALID_BIT)
        | (WriteBuffer[WB_CURR_OFFSET + 3] & VALID_BIT);
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
    out << std::hex << ReadBuffer[0] << std::endl;
    out << std::hex << ReadBuffer[1] << std::endl;
    // next two quadlets are digital I/O and amplifier temperature
    out << std::hex << ReadBuffer[2] << std::endl;
    out << std::hex << ReadBuffer[3] << std::endl;

    // remaining quadlets are in 4 groups of NUM_CHANNELS as follows:
    //   - motor current and analog pot per channel
    //   - encoder position per channel
    //   - encoder velocity per channel
    //   - encoder acceleration data (depends on firmware version)
    for (unsigned int i=4; i < GetReadNumBytes()/sizeof(quadlet_t); i++) {
        out << std::hex << ReadBuffer[i] << " ";
        if (!((i-1)%NUM_CHANNELS)) out << std::endl;
    }
    out << std::dec;
}

double AmpIO::GetFPGAClockPeriod(void) const
{
    return (1.0e-6/FPGA_sysclk_MHz);
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
    return ReadBuffer[STATUS_OFFSET];
}

AmpIO_UInt32 AmpIO::GetTimestamp(void) const
{
    return ReadBuffer[TIMESTAMP_OFFSET];
}

double AmpIO::GetTimestampSeconds(void) const
{
    return GetTimestamp()*GetFPGAClockPeriod();
}

AmpIO_UInt32 AmpIO::GetDigitalInput(void) const
{
    return ReadBuffer[DIGIO_OFFSET];
}

AmpIO_UInt8 AmpIO::GetDigitalOutput(void) const
{
    // Starting with Version 1.3.0 of this library, the digital outputs are inverted
    // before being returned to the caller because they are inverted in hardware and/or firmware.
    // This way, the digital output state matches the hardware state (i.e., 0 means digital output
    // is at 0V).
    AmpIO_UInt8 dout = static_cast<AmpIO_UInt8>((~(ReadBuffer[DIGIO_OFFSET]>>12))&0x000f);
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

AmpIO_UInt8 AmpIO::GetAmpTemperature(unsigned int index) const
{
    AmpIO_UInt8 temp = 0;
    if (index == 0)
        temp = (ReadBuffer[TEMP_OFFSET]>>8) & 0x000000ff;
    else if (index == 1)
        temp = ReadBuffer[TEMP_OFFSET] & 0x000000ff;
    return temp;
}

AmpIO_UInt32 AmpIO::GetMotorCurrent(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = ReadBuffer[index+MOTOR_CURR_OFFSET];
    buff &= MOTOR_CURR_MASK;       // mask for applicable bits

    return static_cast<AmpIO_UInt32>(buff) & ADC_MASK;
}

double AmpIO::GetMotorVoltageRatio(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0.0;

    quadlet_t buff;
    buff = ReadBuffer[index+MOTOR_STATUS_OFFSET];
    int16_t raw = buff & 0xffff;
    return (raw >> 5) / 1023.0l;
}

AmpIO_UInt32 AmpIO::GetAnalogInput(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = ReadBuffer[index+ANALOG_POS_OFFSET];
    buff &= ANALOG_POS_MASK;       // mask for applicable bits
    buff >>= 16;                   // shift to lsb alignment

    return static_cast<AmpIO_UInt32>(buff) & ADC_MASK;
}

AmpIO_Int32 AmpIO::GetEncoderPosition(unsigned int index) const
{
    if (index < NUM_CHANNELS) {
        return static_cast<AmpIO_Int32>(ReadBuffer[index + ENC_POS_OFFSET] & ENC_POS_MASK) - ENC_MIDRANGE;
    }
    return 0;
}

bool AmpIO::GetEncoderOverflow(unsigned int index) const
{
    if (index < NUM_CHANNELS) {
        return ReadBuffer[index+ENC_POS_OFFSET] & ENC_OVER_MASK;
    }
    else {
        std::cerr << "AmpIO::GetEncoderOverflow: index out of range " << index
                  << ", nb channels is " << NUM_CHANNELS << std::endl;
    }
    return true; // send error "code"
}

double AmpIO::GetEncoderClockPeriod(void) const
{
    // Could instead return encVelData[0].clkPeriod
    AmpIO_UInt32 fver = GetFirmwareVersion();
    if (fver < 6)
        return VEL_PERD_OLD;
    else if (fver == 6)
        return VEL_PERD_REV6;
    return VEL_PERD;
}

// Returns encoder velocity in counts/sec -> 4/period
double AmpIO::GetEncoderVelocity(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    double clkPeriod = encVelData[index].clkPeriod;
    AmpIO_UInt32 velPeriod = encVelData[index].velPeriod;

    // Avoid divide by 0 (should never happen)
    if (velPeriod == 0) velPeriod = 1;

    double vel = 0.0;
    if (!encVelData[index].velOverflow && !encVelData[index].dirChange) {
        vel = 4.0/(velPeriod*clkPeriod);
        if (!encVelData[index].velDir)
            vel = -vel;
    }

    return vel;
}

// Returns predicted encoder velocity in counts/sec, taking into account
// acceleration and running counter.
double AmpIO::GetEncoderVelocityPredicted(unsigned int index, double percent_threshold) const
{
    double encVel = GetEncoderVelocity(index);
    double encAcc = GetEncoderAcceleration(index, percent_threshold);
    // The encoder measurement delay is half the measured period, based on the assumption that measuring the
    // period over a full cycle (4 quadrature counts) estimates the velocity in the middle of that cycle.
    double encDelay = encVelData[index].velPeriod*encVelData[index].clkPeriod/2.0;
    double encRun = GetEncoderRunningCounterSeconds(index);
    double deltaVel = encAcc*(encDelay+encRun);
    double predVel = encVel+deltaVel;
    if (encVel < 0) {
        // Do not change velocity direction
        if (predVel > 0.0)
            predVel = 0.0;
        // Maximum velocity limited by 1 count (i.e., we know
        // that a count has not happened for encRun seconds)
        if (predVel*encRun < -1.0)
            predVel = -1.0/encRun;
    }
    else if (encVel > 0.0) {
        // Do not change velocity direction
        if (predVel < 0.0)
            predVel = 0.0;
        // Maximum velocity limited by 1 count (i.e., we know
        // that a count has not happened for encRun seconds)
        if (predVel*encRun > 1.0)
            predVel = 1.0/encRun;
    }
    else {
        // If not moving, do not attempt to predict
        predVel = 0.0;
    }
    return predVel;
}

// Estimate acceleration from two quarters of the same type; units are counts/second**2
// Valid for firmware version 6+.
double AmpIO::GetEncoderAcceleration(unsigned int index, double percent_threshold) const
{

    if (index >= NUM_CHANNELS)
        return 0.0;

    if (encVelData[index].velOverflow)
        return 0.0;

    double clkPeriod = encVelData[index].clkPeriod;
    AmpIO_UInt32 qtr1Period = encVelData[index].qtr1Period;               // Current quarter-cycle period
    AmpIO_UInt32 qtr5Period = encVelData[index].qtr5Period;               // Previous quarter-cycle period of same type
    AmpIO_UInt32 velPeriod = encVelData[index].velPeriod;                 // Current full-cycle period
    AmpIO_UInt32 velPeriodPrev = velPeriod - qtr1Period + qtr5Period;     // Previous full-cycle period
    if (encVelData[index].qtr5Overflow)
        velPeriodPrev = encVelData[index].velPeriodMax;

    // Should never happen
    if ((qtr1Period == 0) || (qtr5Period == 0) || (velPeriod == 0) || (velPeriodPrev == 0))
        return 0.0;

    if (encVelData[index].qtr1Edges != encVelData[index].qtr5Edges)
        return 0.0;

    if ((encVelData[index].qtr1Dir != encVelData[index].qtr5Dir) || (encVelData[index].qtr1Dir != encVelData[index].velDir))
        return 0.0;

    double acc = 0.0;
    if (1.0/qtr1Period <= percent_threshold) {
        double qtrDiff = static_cast<double>(qtr5Period) - static_cast<double>(qtr1Period);
        double qtrSum = static_cast<double>(qtr5Period + qtr1Period);
        double velProd = static_cast<double>(velPeriod)*static_cast<double>(velPeriodPrev)*clkPeriod*clkPeriod;
        acc = (8.0*qtrDiff)/(velProd*qtrSum);
        if (!encVelData[index].velDir)
            acc = -acc;
    }
    return acc;
}

// Raw velocity field; includes period of velocity and other data, depending on firmware version.
// For firmware version 6, includes part of AccRec. For later firmware versions, no AccRec
AmpIO_UInt32 AmpIO::GetEncoderVelocityRaw(unsigned int index) const
{
    return ReadBuffer[index+ENC_VEL_OFFSET];
}

// Raw acceleration field; for firmware prior to Version 6, this was actually the encoder "frequency"
// (i.e., number of pulses in specified time period, which can be used to estimate velocity), but was never used.
// For Firmware Version 6, it was reused to return some data that can be used to estimate the acceleration.
// For Firmware Version 7+, it returns QTR1; note that this method is equivalent to GetEncoderQtr1Raw because
// ENC_FRQ_OFFSET == ENC_QTR1_OFFSET. For testing only.
AmpIO_UInt32 AmpIO::GetEncoderAccelerationRaw(unsigned int index) const
{
    return ReadBuffer[index+ENC_FRQ_OFFSET];
}

// Get the most recent encoder quarter cycle period for internal use and testing (Rev 7+).
// Note that this is equivalent to GetEncoderAccelerationRaw because ENC_FRQ_OFFSET == ENC_QTR1_OFFSET
AmpIO_UInt32 AmpIO::GetEncoderQtr1Raw(unsigned int index) const
{
    return ReadBuffer[index+ENC_QTR1_OFFSET];
}

// Get the encoder quarter cycle period from 5 cycles ago (i.e., 4 cycles prior to the one returned
// by GetEncoderQtr1) for internal use and testing (Rev 7+).
AmpIO_UInt32 AmpIO::GetEncoderQtr5Raw(unsigned int index) const
{
    return ReadBuffer[index+ENC_QTR5_OFFSET];
}

// Get the encoder running counter, which measures the elasped time since the last encoder edge;
// for internal use and testing (Rev 7+).
AmpIO_UInt32 AmpIO::GetEncoderRunningCounterRaw(unsigned int index) const
{
    return ReadBuffer[index+ENC_RUN_OFFSET];
}

// Get the encoder running counter, in seconds. This is primarily used for Firmware Rev 7+, but
// also supports the running counter in Firmware Rev 4-5.
double AmpIO::GetEncoderRunningCounterSeconds(unsigned int index) const
{
    return (encVelData[index].runPeriod)*(encVelData[index].clkPeriod);
}

AmpIO_Int32 AmpIO::GetEncoderMidRange(void)
{
    return ENC_MIDRANGE;
}

bool AmpIO::SetEncoderVelocityData(unsigned int index)
{
    if (index >= NUM_CHANNELS)
        return false;

    encVelData[index].Init();  // Set default values
    AmpIO_UInt32 fver = GetFirmwareVersion();

    if (fver < 6) {
        // Prior to Firmware Version 6, the latched counter value is returned
        // as the lower 16 bits. Starting with Firmware Version 4, the upper 16 bits are
        // the free-running counter, which was not used.
        // Note that the counter values are signed, so we convert to unsigned and set a direction bit
        // to be consistent with later versions of firmware.
        encVelData[index].clkPeriod = VEL_PERD_OLD;
        encVelData[index].velPeriodMax = ENC_VEL_MASK_16;
        AmpIO_UInt16 velPeriod = static_cast<AmpIO_UInt16>(ReadBuffer[ENC_VEL_OFFSET+index] & ENC_VEL_MASK_16);
        // Convert from signed count to unsigned count and direction
        if (velPeriod == 0x8000) { // if overflow
            encVelData[index].velPeriod = 0x00007fff;
            encVelData[index].velOverflow = true;
            // Firmware also sets velPeriod to overflow when direction change occurred, so could potentially
            // set encVelData[index].dirChange = true.
        }
        else if (velPeriod & 0x8000) {  // if negative
            velPeriod = ~velPeriod;     // ones complement (16-bits)
            encVelData[index].velPeriod = velPeriod + 1;  // twos complement (32-bits)
            encVelData[index].velDir = false;
        }
        else {
            encVelData[index].velPeriod = velPeriod;
            encVelData[index].velDir = true;
        }
        if (fver >= 4) {
            AmpIO_UInt16 runCtr = static_cast<AmpIO_UInt16>((ReadBuffer[ENC_VEL_OFFSET+index]>>16) & ENC_VEL_MASK_16);
            // Convert from signed count to unsigned count and direction
            if (runCtr == 0x8000) {  // if overflow
                encVelData[index].runOverflow = true;
                runCtr = 0x7fff;
            }
            else if (runCtr & 0x8000) {  // if negative
                runCtr = ~runCtr;        // ones complement (16-bits)
                runCtr += 1;             // twos complement (16-bits)
            }
            encVelData[index].runPeriod = static_cast<AmpIO_UInt32>(runCtr);
        }
    }
    else if (fver == 6) {
        encVelData[index].clkPeriod = VEL_PERD_REV6;
        encVelData[index].velPeriodMax = ENC_VEL_MASK_22;
        encVelData[index].qtrPeriodMax = ENC_ACC_PREV_MASK;  // 20 bits
        // Firmware 6 has bits stuffed in different places:
        //   Q1: lower 8 bits in velPeriod[29:22] and upper 12 bits in accQtr1[31:20]
        //   Q5: all 20 bits in accQtr1[19:0]
        encVelData[index].velPeriod = ReadBuffer[ENC_VEL_OFFSET+index] & ENC_VEL_MASK_22;
        encVelData[index].qtr1Period = (((ReadBuffer[ENC_QTR1_OFFSET+index] & ENC_ACC_REC_MS_MASK)>>12) |
                          ((ReadBuffer[ENC_VEL_OFFSET+index] & ENC_ACC_REC_LS_MASK) >> 22)) & ENC_ACC_PREV_MASK;
        encVelData[index].qtr5Period = ReadBuffer[ENC_QTR1_OFFSET+index] & ENC_ACC_PREV_MASK;
        encVelData[index].velOverflow = ReadBuffer[ENC_VEL_OFFSET+index] & ENC_VEL_OVER_MASK;
        encVelData[index].velDir = ReadBuffer[ENC_VEL_OFFSET+index] & ENC_DIR_MASK;
        // Qtr1 and Qtr5 overflow at 0x000fffff (no overflow bit is set by firmware)
        if (encVelData[index].qtr1Period == encVelData[index].qtrPeriodMax)
            encVelData[index].qtr1Overflow = true;
        if (encVelData[index].qtr5Period == encVelData[index].qtrPeriodMax)
            encVelData[index].qtr5Overflow = true;
        // Qtr1 and Qtr5 direction are not recorded, so set them same as velDir;
        // i.e., assume that there hasn't been a direction change.
        encVelData[index].qtr1Dir = encVelData[index].velDir;
        encVelData[index].qtr5Dir = encVelData[index].velDir;
    }
    else {  // V7+
        encVelData[index].clkPeriod = VEL_PERD;
        encVelData[index].velPeriodMax = ENC_VEL_MASK_26;
        encVelData[index].qtrPeriodMax = ENC_VEL_QTR_MASK;  // 26 bits
        encVelData[index].velPeriod = ReadBuffer[ENC_VEL_OFFSET+index] & ENC_VEL_MASK_26;
        encVelData[index].velOverflow = ReadBuffer[ENC_VEL_OFFSET+index] & ENC_VEL_OVER_MASK;
        encVelData[index].velDir = ReadBuffer[ENC_VEL_OFFSET+index] & ENC_DIR_MASK;
        encVelData[index].dirChange = ReadBuffer[ENC_VEL_OFFSET+index] & 0x20000000;
        encVelData[index].encError = ReadBuffer[ENC_VEL_OFFSET+index] & 0x10000000;
        encVelData[index].partialCycle = ReadBuffer[ENC_VEL_OFFSET+index] & 0x08000000;
        encVelData[index].qtr1Period = ReadBuffer[ENC_QTR1_OFFSET+index] & ENC_VEL_QTR_MASK;
        encVelData[index].qtr1Overflow = ReadBuffer[ENC_QTR1_OFFSET+index] & ENC_VEL_OVER_MASK;
        encVelData[index].qtr1Dir = ReadBuffer[ENC_QTR1_OFFSET+index] & ENC_DIR_MASK;
        encVelData[index].qtr1Edges = (ReadBuffer[ENC_QTR1_OFFSET+index]>>26)&0x0f;
        encVelData[index].qtr5Period = ReadBuffer[ENC_QTR5_OFFSET+index] & ENC_VEL_QTR_MASK;
        encVelData[index].qtr5Overflow = ReadBuffer[ENC_QTR5_OFFSET+index] & ENC_VEL_OVER_MASK;
        encVelData[index].qtr5Dir = ReadBuffer[ENC_QTR5_OFFSET+index] & ENC_DIR_MASK;
        encVelData[index].qtr5Edges = (ReadBuffer[ENC_QTR5_OFFSET+index]>>26)&0x0f;
        encVelData[index].runPeriod = ReadBuffer[ENC_RUN_OFFSET+index] & ENC_VEL_QTR_MASK;
        encVelData[index].runOverflow = ReadBuffer[ENC_RUN_OFFSET+index] & ENC_VEL_OVER_MASK;
    }

    // Increment error counter if necessary
    if (encVelData[index].encError)
        encErrorCount[index]++;

    return true;
}

bool AmpIO::GetEncoderVelocityData(unsigned int index, EncoderVelocityData &data) const
{
    if (index >= NUM_CHANNELS)
        return false;
    data = encVelData[index];
    return true;
}

unsigned int AmpIO::GetEncoderErrorCount(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0;
    return encErrorCount[index];
}

bool AmpIO::ClearEncoderErrorCount(unsigned int index)
{
    if (index > NUM_CHANNELS)
        return false;

    if (index == NUM_CHANNELS) {
        // Clear all counters
        for (unsigned int i = 0; i < NUM_CHANNELS; i++)
            encErrorCount[i] = 0;
    }
    else {
        encErrorCount[index] = 0;
    }

    return true;
}

bool AmpIO::GetPowerEnable(void) const
{
    // Bit 18
    return (GetStatus()&0x00040000);
}

bool AmpIO::GetPowerStatus(void) const
{
    // Bit 19: MV_GOOD
    return (GetStatus()&0x00080000);
}

bool AmpIO::GetSafetyRelay(void) const
{
    // Bit 16
    return (GetStatus()&0x00010000);
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
    if (GetFirmwareVersion() < 8) {
        if (index >= NUM_CHANNELS)
            return false;
        AmpIO_UInt32 mask = (0x00000001 << index);
        return GetStatus()&mask;
    } else {
        if (index >= NUM_MOTORS)
            return false;
        return ReadBuffer[MOTOR_STATUS_OFFSET + index] & (1 << 29);
    }
}

AmpIO_UInt8 AmpIO::GetAmpEnableMask(void) const
{
    return GetStatus()&0x0000000f;
}

bool AmpIO::GetAmpStatus(unsigned int index) const
{
    if (GetFirmwareVersion() < 8) {
        if (index >= NUM_CHANNELS)
            return false;
        AmpIO_UInt32 mask = (0x00000100 << index);
        return GetStatus()&mask;
    } else {
        if (index >= NUM_MOTORS)
            return false;
        return !(ReadBuffer[MOTOR_STATUS_OFFSET + index] & (0xf0000));
    }
}

AmpIO_UInt32 AmpIO::GetSafetyAmpDisable(void) const
{
    AmpIO_UInt32 mask = 0x000000F0;
    return (GetStatus() & mask) >> 4;
}

AmpIO_UInt32 AmpIO::GetAmpFaultCode(unsigned int index) const
{
    return (ReadBuffer[MOTOR_STATUS_OFFSET + index] & (0xf0000)) >> 16;
}

/*******************************************************************************
 * Set commands
 */

void AmpIO::SetPowerEnable(bool state)
{
    AmpIO_UInt32 enable_mask = 0x00080000;
    AmpIO_UInt32 state_mask  = 0x00040000;
    WriteBuffer[WB_CTRL_OFFSET] |=  enable_mask;
    if (state)
        WriteBuffer[WB_CTRL_OFFSET] |=  state_mask;
    else
        WriteBuffer[WB_CTRL_OFFSET] &= ~state_mask;
}

bool AmpIO::SetAmpEnable(unsigned int index, bool state)
{
    if (GetFirmwareVersion() < 8) {
        if (index < NUM_CHANNELS) {
            AmpIO_UInt32 enable_mask = 0x00000100 << index;
            AmpIO_UInt32 state_mask  = 0x00000001 << index;
            WriteBuffer[WB_CTRL_OFFSET] |=  enable_mask;
            if (state)
                WriteBuffer[WB_CTRL_OFFSET] |=  state_mask;
            else
                WriteBuffer[WB_CTRL_OFFSET] &= ~state_mask;
            return true;
        }
        return false;
    } else {
        if (index < NUM_MOTORS) {
            AmpIO_UInt32 enable_mask = 1 << 28;
            AmpIO_UInt32 state_mask  = 1 << 29;
            WriteBuffer[WB_CURR_OFFSET + index] |=  enable_mask;
            if (state)
                WriteBuffer[WB_CURR_OFFSET + index] |=  state_mask;
            else
                WriteBuffer[WB_CURR_OFFSET + index] &= ~state_mask;
            return true;
        }
        return false;        
    }
}

bool AmpIO::SetAmpEnableMask(AmpIO_UInt32 mask, AmpIO_UInt32 state)
{
    if (GetFirmwareVersion() < 8) { 
        AmpIO_UInt32 enable_mask = static_cast<AmpIO_UInt32>(mask) << 8;
        AmpIO_UInt32 state_clr_mask = static_cast<AmpIO_UInt32>(mask);
        AmpIO_UInt32 state_set_mask = static_cast<AmpIO_UInt32>(state);
        // Following will correctly handle case where SetAmpEnable/SetAmpEnableMask is called multiple times,
        // with different masks
        WriteBuffer[WB_CTRL_OFFSET] = (WriteBuffer[WB_CTRL_OFFSET]&(~state_clr_mask)) | enable_mask | state_set_mask;
        return true;
    } else {
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (mask >> i & 1) SetAmpEnable(i, state >> i & 1);
        }
        return true;
    }
}

void AmpIO::SetSafetyRelay(bool state)
{
    AmpIO_UInt32 enable_mask = 0x00020000;
    AmpIO_UInt32 state_mask  = 0x00010000;
    WriteBuffer[WB_CTRL_OFFSET] |=  enable_mask;
    if (state)
        WriteBuffer[WB_CTRL_OFFSET] |=  state_mask;
    else
        WriteBuffer[WB_CTRL_OFFSET] &= ~state_mask;
}

bool AmpIO::SetMotorCurrent(unsigned int index, AmpIO_UInt32 sdata)
{
    if (GetFirmwareVersion() < 8) { 
        quadlet_t data = VALID_BIT | ((BoardId & 0x0F) << 24) | (sdata & DAC_MASK);
        if (collect_state && (collect_chan == (index+1)))
            data |= COLLECT_BIT;

        if (index < NUM_CHANNELS) {
            WriteBuffer[index+WB_CURR_OFFSET] = data;
            return true;
        }
        else
            return false;
    } else {
        if (index < NUM_MOTORS) {
            if (collect_state && (collect_chan == (index+1)))
                WriteBuffer[index+WB_CURR_OFFSET] |= COLLECT_BIT;
            WriteBuffer[index+WB_CURR_OFFSET] |= VALID_BIT;
            WriteBuffer[index+WB_CURR_OFFSET] &= ~0x0FFFFFFF;
            WriteBuffer[index+WB_CURR_OFFSET] |= sdata & 0XFFFF;
        return true;
        } else return false;
    }
}

bool AmpIO::SetMotorVoltageRatio(unsigned int index, double ratio)
{
    if (GetFirmwareVersion() < 8) { 
        return false;
    } else {
        if (index < NUM_MOTORS) {
            if (collect_state && (collect_chan == (index+1)))
                WriteBuffer[index+WB_CURR_OFFSET] |= COLLECT_BIT;
            WriteBuffer[index+WB_CURR_OFFSET] |= VALID_BIT;
            WriteBuffer[index+WB_CURR_OFFSET] &= ~0x0FFFFFFF;
            WriteBuffer[index+WB_CURR_OFFSET] |= 1 << 24; // select voltage mode
            WriteBuffer[index+WB_CURR_OFFSET] |= ((int)(ratio * 1023) & 0b11111111111) << 13;
        return true;
        } else return false;
    }
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

bool AmpIO::ReadBlock(nodeaddr_t addr, quadlet_t *rdata, unsigned int nbytes)
{
    if (!port) {
        return false;
    }
    return port->ReadBlock(BoardId, addr, rdata, nbytes);
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

bool AmpIO::IsEncoderPreloadMidrange(unsigned int index, bool & isMidrange) const
{
    AmpIO_Int32 encoderPreload;
    bool ret = ReadEncoderPreload(index, encoderPreload);
    if (ret) {
        isMidrange = (encoderPreload == ENC_MIDRANGE);
    }
    return ret;
}

AmpIO_Int32 AmpIO::ReadWatchdogPeriod(bool applyMask) const
{
    AmpIO_UInt32 counts = 0;
    if (port) {
        port->ReadQuadlet(BoardId, 3, counts);
    }
    if (applyMask)
        counts &= 0x0000ffff;
    return counts;
}

double AmpIO::ReadWatchdogPeriodInSeconds(void) const
{
    return ReadWatchdogPeriod()*WDOG_ClockPeriod;
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

bool AmpIO::ReadWaveformStatus(bool &active, AmpIO_UInt32 &tableIndex)
{
    if (GetFirmwareVersion() < 7) return false;
    AmpIO_UInt32 read_data = 0;
    if (!port) return false;
    bool ret = port->ReadQuadlet(BoardId, 6, read_data);
    if (ret) {
        active = read_data&VALID_BIT;
        tableIndex = (read_data>>16)&0x000003ff;
    }
    return ret;
}

AmpIO_UInt32 AmpIO::ReadIPv4Address(void) const
{
    if (GetFirmwareVersion() < 7) {
        std::cerr << "AmpIO::ReadIPv4Address: requires firmware 7 or above" << std::endl;
        return 0;
    }
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, 11, read_data);
    return read_data;
}

/*******************************************************************************
 * Write commands
 */

bool AmpIO::WriteReboot(void)
{
    if (GetFirmwareVersion() < 7) {
        // Could instead allow this to be used with older firmware, where the "reboot"
        // flag was instead used as a "soft reset".
        std::cerr << "AmpIO::WriteReboot: requires firmware 7 or above" << std::endl;
        return false;
    }
    AmpIO_UInt32 write_data = REBOOT_FPGA;
    return (port ? port->WriteQuadlet(BoardId, 0, write_data) : false);
}

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

    if ((sdata >= ENC_MIDRANGE) || (sdata < -ENC_MIDRANGE)) {
        std::cerr << "AmpIO::WriteEncoderPreload, preload out of range " << sdata << std::endl;
        return false;
    }
    bool ret = false;
    if (port && (index < NUM_CHANNELS)) {
        ret = port->WriteQuadlet(BoardId, channel | ENC_LOAD_OFFSET,
                                 static_cast<AmpIO_UInt32>(sdata + ENC_MIDRANGE));
    }
    return ret;
}

bool AmpIO::WriteDoutConfigReset(void)
{
    return (port && (GetFirmwareVersion() >= 7)) ? port->WriteQuadlet(BoardId, 0, DOUT_CFG_RESET) : false;
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

bool AmpIO::WriteWaveformControl(AmpIO_UInt8 mask, AmpIO_UInt8 bits)
{
    quadlet_t write_data = (mask << 8) | ((~bits)&0x0f);
    if (mask != 0)
        write_data |= VALID_BIT;  // Same valid bit as motor current
    return port->WriteQuadlet(BoardId, 6, write_data);
}

bool AmpIO::WriteWatchdogPeriod(AmpIO_UInt32 counts)
{
    //TODO: hack
    if (port->GetHardwareVersion(BoardId) == dRA1_String) {
        for (int axis = 1; axis < 11; axis ++) {
            WriteCurrentKpRaw(axis - 1, ampio_pi_fixed_from_float(0.05));
            WriteCurrentKiRaw(axis - 1, ampio_pi_fixed_from_float(0.01));
            WriteCurrentITermLimitRaw(axis - 1, 600);
            WriteDutyCycleLimit(axis - 1, 1000);
        }
    }
    
    // period = counts(16 bits) * 5.208333 us (0 = no timeout)
    return port->WriteQuadlet(BoardId, 3, counts);
}

bool AmpIO::WriteWatchdogPeriodInSeconds(const double seconds, bool ledDisplay)
{
    AmpIO_UInt32 counts;
    if (seconds == 0.0) {
        // Disable watchdog
        counts = 0;
    } else {
        // Use at least one tick just to make sure we don't accidentaly disable;
        // the truth is that the count will be so low that watchdog will
        // continuously trigger.
        counts = static_cast<AmpIO_UInt32>(seconds/WDOG_ClockPeriod);
        counts = std::max(counts, static_cast<AmpIO_UInt32>(1));
    }
    if (ledDisplay)
        counts |= 0x80000000;
    return WriteWatchdogPeriod(counts);
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

bool AmpIO::WriteIPv4Address(AmpIO_UInt32 IPaddr)
{
    if (GetFirmwareVersion() < 7) {
        std::cerr << "AmpIO::WriteIPv4Address: requires firmware 7 or above" << std::endl;
        return false;
    }
    return (port ? port->WriteQuadlet(BoardId, 11, IPaddr) : false);
}

AmpIO_UInt32 AmpIO::GetDoutCounts(double time) const
{
    return static_cast<AmpIO_UInt32>((FPGA_sysclk_MHz*1e6)*time + 0.5);
}

/*******************************************************************************
 * Static Write methods (for broadcast)
 *
 * These methods duplicate the board-specific methods (WriteReboot, WritePowerEnable,
 * WriteAmpEnable, ...), but are sent to the broadcast address (FW_NODE_BROADCAST).
 */

bool AmpIO::WriteRebootAll(BasePort *port)
{
    // Note that Firmware V7+ supports the reboot command; earlier versions of
    // firmware will instead perform a limited reset.
    AmpIO_UInt32 write_data = REBOOT_FPGA;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, 0, write_data) : false);
}

bool AmpIO::WritePowerEnableAll(BasePort *port, bool state)
{
    AmpIO_UInt32 write_data = state ? PWR_ENABLE : PWR_DISABLE;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, 0, write_data) : false);
}

bool AmpIO::WriteAmpEnableAll(BasePort *port, AmpIO_UInt8 mask, AmpIO_UInt8 state)
{
    quadlet_t write_data = (mask << 8) | state;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, 0, write_data) : false);
}

bool AmpIO::WriteSafetyRelayAll(BasePort *port, bool state)
{
    AmpIO_UInt32 write_data = state ? RELAY_ON : RELAY_OFF;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, 0, write_data) : false);
}

bool AmpIO::WriteEncoderPreloadAll(BasePort *port, unsigned int index, AmpIO_Int32 sdata)
{
    unsigned int channel = (index+1) << 4;

    if ((sdata >= ENC_MIDRANGE) || (sdata < -ENC_MIDRANGE)) {
        std::cerr << "AmpIO::WriteEncoderPreloadAll, preload out of range " << sdata << std::endl;
        return false;
    }
    bool ret = false;
    if (port && (index < NUM_CHANNELS)) {
        ret = port->WriteQuadlet(FW_NODE_BROADCAST, channel | ENC_LOAD_OFFSET,
                                 static_cast<AmpIO_UInt32>(sdata + ENC_MIDRANGE));
    }
    return ret;
}

bool AmpIO::ResetKSZ8851All(BasePort *port) {
    quadlet_t write_data = RESET_KSZ8851;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, 12, write_data) : false);
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
    if (!port->WriteBlock(BoardId, 0x3100, data, nquads*sizeof(quadlet_t)))
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
        // Done when in idle state. The following test works for both Firmware Rev 7
        // (state is bits 7:4) and Firmware Rev 8 (state is bits 8:4 and busy flag is bit 13)
        if ((status&0x000020F0) == 0)
            break;
    }
    //std::cerr << "Wait time = " << i << " milliseconds" << std::endl;
    return (i < 500);
}

bool AmpIO::DallasReadMemory(unsigned short addr, unsigned char *data, unsigned int nbytes, bool useDS2480B)
{
    if (GetFirmwareVersion() < 7) return false;
    AmpIO_UInt32 status = ReadStatus();
    // Check whether bi-directional I/O is available
    if ((status & 0x00300000) != 0x00300000) return false;
    AmpIO_UInt32 ctrl = (addr<<16)|2;
    if (useDS2480B) ctrl |= 4;
    if (!DallasWriteControl(ctrl)) return false;
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
    quadlet_t write_data = RESET_KSZ8851;
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

bool AmpIO::ReadEthernetData(quadlet_t *buffer, unsigned int offset, unsigned int nquads)
{
    if (GetFirmwareVersion() < 5) return false;
    // Firmware currently cannot read more than 64 quadlets
    if (nquads > 64) return false;
    nodeaddr_t address = 0x4000 + offset;   // ADDR_ETH = 0x4000
    bool ret = port->ReadBlock(BoardId, address, buffer, nquads*sizeof(quadlet_t));
    if (ret) {
        for (unsigned int i = 0; i < nquads; i++)
            buffer[i] = bswap_32(buffer[i]);
    }
    return ret;
}

// Following not yet fully implemented in firmware
bool AmpIO::ReadFirewireData(quadlet_t *buffer, unsigned int offset, unsigned int nquads)
{
    if (GetFirmwareVersion() < 5) return false;
    // Firmware currently cannot read more than 64 quadlets
    if (nquads > 64) return false;
    nodeaddr_t address = 0x5000 + offset;   // ADDR_FW = 0x5000
    bool ret = port->ReadBlock(BoardId, address, buffer, nquads*sizeof(quadlet_t));
    if (ret) {
        for (unsigned int i = 0; i < nquads; i++)
            buffer[i] = bswap_32(buffer[i]);
    }
    return ret;
}

bool AmpIO::ReadWaveformTable(quadlet_t *buffer, unsigned short offset, unsigned short nquads)
{
    if (GetFirmwareVersion() < 7) return false;
    if (nquads == 0) return true;
    if (nquads > (port->GetMaxReadDataSize()/sizeof(quadlet_t))) return false;
    if (offset > 1023) return false;
    nodeaddr_t address = 0x8000 + offset;   // ADDR_WAVEFORM = 0x8000
    bool ret = port->ReadBlock(BoardId, address, buffer, nquads*sizeof(quadlet_t));
    if (ret) {
        // Byteswap and invert digital output bits (see WriteDigitalOutput and GetDigitalOutput)
        for (unsigned short i = 0; i < nquads; i++)
            buffer[i] = bswap_32(buffer[i])^0x0000000f;
    }
    return ret;
}

bool AmpIO::WriteWaveformTable(const quadlet_t *buffer, unsigned short offset, unsigned short nquads)
{
    if (GetFirmwareVersion() < 7) return false;
    if (nquads == 0) return true;
    if (nquads > (port->GetMaxWriteDataSize()/sizeof(quadlet_t))) return false;
    if (offset > 1023) return false;
    static quadlet_t localBuffer[MAX_POSSIBLE_DATA_SIZE/sizeof(quadlet_t)];
    nodeaddr_t address = 0x8000 + offset;   // ADDR_WAVEFORM = 0x8000
    // Byteswap and invert digital output bits (see WriteDigitalOutput and GetDigitalOutput)
    for (unsigned short i = 0; i < nquads; i++)
        localBuffer[i] = bswap_32(buffer[i]^0x0000000f);
    return port->WriteBlock(BoardId, address, localBuffer, nquads*sizeof(quadlet_t));
}

bool AmpIO::DataCollectionStart(unsigned char chan, CollectCallback collectCB)
{
    if (GetFirmwareVersion() < 7) return false;
    if ((chan < 1) || (chan > NUM_CHANNELS)) return false;
    collect_state = true;
    collect_chan = chan;
    collect_rindex = 0;
    collect_cb = collectCB;
    return true;
}

void AmpIO::DataCollectionStop()
{
    collect_state = false;
}

bool AmpIO::IsCollecting() const
{
    // Collection active on both host and FPGA
    return (collect_state&&(ReadBuffer[TEMP_OFFSET]&0x80000000));
}

bool AmpIO::GetCollectionStatus(bool &collecting, unsigned char &chan, unsigned short &writeAddr) const
{
    if (GetFirmwareVersion() < 7) return false;
    collecting = (ReadBuffer[TEMP_OFFSET]&0x80000000);
    chan = (ReadBuffer[TEMP_OFFSET]&0x3c000000)>>26;
    writeAddr = (ReadBuffer[TEMP_OFFSET]&0x03ff0000)>>16;;
    return true;
}

bool AmpIO::ReadCollectionStatus(bool &collecting, unsigned char &chan, unsigned short &writeAddr) const
{
    if (GetFirmwareVersion() < 7) return false;
    quadlet_t read_data;
    bool ret = port->ReadQuadlet(BoardId, 0x7800, read_data);
    if (ret) {
        collecting = (read_data&0x00008000);
        chan = (read_data&0x00003c00)>>10;
        writeAddr = (read_data&0x000003ff);
    }
    return ret;
}

bool AmpIO::ReadCollectedData(quadlet_t *buffer, unsigned short offset, unsigned short nquads)
{
    if (GetFirmwareVersion() < 7) return false;
    if (nquads > (port->GetMaxReadDataSize()/sizeof(quadlet_t))) return false;
    if (offset > 1023) return false;
    nodeaddr_t address = 0x7000 + offset;   // ADDR_DATA_BUF = 0x7000
    bool ret = port->ReadBlock(BoardId, address, buffer, nquads*sizeof(quadlet_t));
    if (ret) {
        for (unsigned short i = 0; i < nquads; i++)
            buffer[i] = bswap_32(buffer[i]);
    }
    return ret;
}

void AmpIO::CheckCollectCallback()
{
    if (collect_cb == 0) return;
    bool fpgaCollecting;
    unsigned char fpgaChan;
    unsigned short collect_windex;
    if (!GetCollectionStatus(fpgaCollecting, fpgaChan, collect_windex)) {
        std::cerr << "CheckCollectCallback: failed to get collection status" << std::endl;
        return;
    }
    // Wait to make sure FPGA is collecting data
    if (collect_state && !fpgaCollecting)
        return;
    if (fpgaChan != collect_chan) {
        std::cerr << "CheckCollectCallback: channel mismatch: "
                  << static_cast<unsigned int>(fpgaChan) << ", "
                  << static_cast<unsigned int>(collect_chan) << std::endl;
        // Continue processing
    }
    // Figure out how much data is available
    // (this implementation works correctly for unsigned integers)
    unsigned short numAvail = (collect_windex >= collect_rindex) ? (collect_windex-collect_rindex)
                                                                 : (COLLECT_BUFSIZE +collect_windex-collect_rindex);
    if (numAvail > 0) {
        if (numAvail > (port->GetMaxReadDataSize()/sizeof(quadlet_t)))
            numAvail = port->GetMaxReadDataSize()/sizeof(quadlet_t);
        if (ReadCollectedData(collect_data, collect_rindex, numAvail)) {
            collect_rindex += numAvail;
            if (collect_rindex >= COLLECT_BUFSIZE)
                collect_rindex -= COLLECT_BUFSIZE;
        }
        else
            numAvail = 0;
    }
    if (!(*collect_cb)(collect_data, numAvail))
        DataCollectionStop();
}

bool AmpIO::WriteMotorControlMode(unsigned int index, AmpIO_UInt16 mode) {
    return (port ? port->WriteQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_MOTOR_CONTROL_MODE, mode) : false);
}

bool AmpIO::WriteCurrentKpRaw(unsigned int index, AmpIO_UInt16 val) {
    return (port ? port->WriteQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_CURRENT_KP, val) : false);
}

bool AmpIO::WriteCurrentKiRaw(unsigned int index, AmpIO_UInt16 val) {
    return (port ? port->WriteQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_CURRENT_KI, val) : false);
}

bool AmpIO::WriteCurrentITermLimitRaw(unsigned int index, AmpIO_UInt16 val) {
    return (port ? port->WriteQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_CURRENT_I_TERM_LIMIT, val) : false);
}

bool AmpIO::WriteDutyCycleLimit(unsigned int index, AmpIO_UInt16 val) {
    return (port ? port->WriteQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_DUTY_CYCLE_LIMIT, val) : false);
}

AmpIO_UInt16 AmpIO::ReadMotorControlMode(unsigned int index) const
{
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_MOTOR_CONTROL_MODE, read_data);
    return read_data;
}

AmpIO_UInt16 AmpIO::ReadCurrentKpRaw(unsigned int index) const
{
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_CURRENT_KP, read_data);
    return read_data;
}

AmpIO_UInt16 AmpIO::ReadCurrentKiRaw(unsigned int index) const
{
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_CURRENT_KI, read_data);
    return read_data;
}

AmpIO_UInt16 AmpIO::ReadCurrentITermLimitRaw(unsigned int index) const
{
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_CURRENT_I_TERM_LIMIT, read_data);
    return read_data;
}

AmpIO_UInt16 AmpIO::ReadDutyCycleLimit(unsigned int index) const
{
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_DUTY_CYCLE_LIMIT, read_data);
    return read_data;
}

AmpIO_Int16 AmpIO::ReadDutyCycle(unsigned int index) const
{
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_DUTY_CYCLE, read_data);
    return static_cast<AmpIO_Int16>(read_data);
}

AmpIO_Int16 AmpIO::ReadCurrentITerm(unsigned int index) const
{
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_CURRENT_I_TERM, read_data);
    return static_cast<AmpIO_Int16>(read_data);
}

AmpIO_Int16 AmpIO::ReadFault(unsigned int index) const
{
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_FAULT, read_data);
    return static_cast<AmpIO_Int16>(read_data);
}
