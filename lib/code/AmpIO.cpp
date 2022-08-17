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

#include <iostream>
#include <sstream>
#include <algorithm>   // for std::max

#include "AmpIO.h"
#include "BasePort.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"

const AmpIO_UInt32 VALID_BIT         = 0x80000000;  /*!< High bit of 32-bit word */
const AmpIO_UInt32 COLLECT_BIT       = 0x40000000;  /*!< Enable data collection on FPGA */
const AmpIO_UInt32 MOTOR_ENABLE_MASK = 0x20000000;  /*!< Mask for enable bit (Firmware Rev 8+) */
const AmpIO_UInt32 MOTOR_ENABLE_BIT  = 0x10000000;  /*!< Enable amplifier for motor (Firmware Rev 8+) */

const AmpIO_UInt32 MIDRANGE_ADC     = 0x00008000;  /*!< Midrange value of ADC bits */
const AmpIO_UInt32 ENC_PRELOAD      = 0x007fffff;  /*!< Encoder position preload value */
const AmpIO_Int32  ENC_MIDRANGE     = 0x00800000;  /*!< Encoder position midrange value */

const AmpIO_UInt32 DOUT_CFG_RESET   = 0x01000000;  /*!< Reset DOUT config (Rev 7+) */
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

const double FPGA_sysclk_MHz        = 49.152;         /* FPGA sysclk in MHz (from FireWire) */
const double VEL_PERD               = 1.0/49152000;   /* Clock period for velocity measurements (Rev 7+ firmware) */
const double VEL_PERD_REV6          = 1.0/3072000;    /* Slower clock for velocity measurements (Rev 6 firmware) */
const double VEL_PERD_OLD           = 1.0/768000;     /* Slower clock for velocity measurements (prior to Rev 6 firmware) */

const double WDOG_ClockPeriod       = 256.0/(FPGA_sysclk_MHz*1e6);   /* Watchdog clock period, in seconds */

AmpIO_UInt8 BitReverse4[16] = { 0x0, 0x8, 0x4, 0xC,         // 0000, 0001, 0010, 0011
                                0x2, 0xA, 0x6, 0xE,         // 0100, 0101, 0110, 0111
                                0x1, 0x9, 0x5, 0xD,         // 1000, 1001, 1010, 1011
                                0x3, 0xB, 0x7, 0xF };       // 1100, 1101, 1110, 1111

AmpIO::AmpIO(AmpIO_UInt8 board_id) : FpgaIO(board_id), NumMotors(0), NumEncoders(0), NumDouts(0),
                                     dallasState(ST_DALLAS_START), dallasTimeoutSec(10.0), collect_state(false), collect_cb(0)
{
    memset(ReadBuffer, 0, sizeof(ReadBuffer));
    memset(WriteBuffer, 0, sizeof(WriteBuffer));
    // Calling InitBoard here will initialize as a 4-axis QLA.
    // InitBoard should be called again from BasePort::AddBoard
    InitBoard();
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
    uint32_t fver = GetFirmwareVersion();
    unsigned int numQuads;
    if (fver < 7) {
        numQuads = 4 + 4*NumEncoders;
    } else if (fver == 7) {
        numQuads = 4 + 6*NumEncoders;
    }
    else {
        numQuads = 4 + 2*NumMotors + 5*NumEncoders;
    }
    return numQuads * sizeof(quadlet_t);
}

void AmpIO::SetReadData(const quadlet_t *buf)
{
    unsigned int numQuads = GetReadNumBytes() / sizeof(quadlet_t);
    unsigned int i;
    for (i = 0; i < numQuads; i++) {
        ReadBuffer[i] = bswap_32(buf[i]);
    }
    for (i = 0; i < NumEncoders; i++) {
        SetEncoderVelocityData(i);
    }
    // Add 1 to timestamp because block read clears counter, rather than incrementing
    firmwareTime += (GetTimestamp()+1)*GetFPGAClockPeriod();
}

void AmpIO::InitBoard(void)
{
    // This method should be called when the port is valid, so that GetHardwareVersion
    // and GetFirmwareVersion return the correct values.
    // If the port is not yet valid, these methods will both return 0, so board will be
    // initialized as a 4-axis QLA and assume firmware version < 8.
    if (GetHardwareVersion() == dRA1_String) {
        NumMotors = 10;
        NumEncoders = 7;
        NumDouts = 0;
    } else {
        NumMotors = 4;
        NumEncoders = 4;
        NumDouts = 4;
    }

    // Check whether buffers are too small (should never happen, but if it does, would
    // be better to exit rather than just print a message). The alternative is to use
    // dynamic memory allocation.
    if ((GetReadNumBytes()/sizeof(quadlet_t)) > ReadBufSize_Max) {
        std::cerr << "AmpIO: Read buffer size too large: " << GetReadNumBytes()/sizeof(quadlet_t)
                  << " (max = " << ReadBufSize_Max << ") quadlets" << std::endl;
    }
    if ((GetWriteNumBytes()/sizeof(quadlet_t)) > WriteBufSize_Max) {
        std::cerr << "AmpIO: Write buffer size too large: " << GetWriteNumBytes()/sizeof(quadlet_t)
                  << " (max = " << WriteBufSize_Max << ") quadlets" << std::endl;
    }

    ENC_POS_OFFSET      = MOTOR_CURR_OFFSET + NumMotors;
    ENC_VEL_OFFSET      = ENC_POS_OFFSET    + NumEncoders;
    ENC_FRQ_OFFSET      = ENC_VEL_OFFSET    + NumEncoders;
    ENC_QTR1_OFFSET     = ENC_FRQ_OFFSET;
    ENC_QTR5_OFFSET     = ENC_QTR1_OFFSET   + NumEncoders;
    ENC_RUN_OFFSET      = ENC_QTR5_OFFSET   + NumEncoders;
    MOTOR_STATUS_OFFSET = ENC_RUN_OFFSET    + NumEncoders;

    WB_HEADER_OFFSET = 0;   // only used for Firmware Rev 8+
    WB_CURR_OFFSET = (GetFirmwareVersion() < 8) ? 0 : 1;
    WB_CTRL_OFFSET = WB_CURR_OFFSET + NumMotors;

    for (size_t i = 0; i < NumEncoders; i++) {
        encVelData[i].Init();
        encErrorCount[i] = 0;
    }
    InitWriteBuffer();
}

void AmpIO::InitWriteBuffer(void)
{
    if (GetFirmwareVersion() < 8) {
        WB_CURR_OFFSET = 0;
        WB_CTRL_OFFSET = WB_CURR_OFFSET + NumMotors;
        quadlet_t data = (BoardId & 0x0F) << 24;
        for (size_t i = 0; i < NumMotors; i++) {
            WriteBuffer[WB_CURR_OFFSET+i] = data;
        }
        WriteBuffer[WB_CTRL_OFFSET] = 0;
    } else {
        WB_CURR_OFFSET = 1;
        WB_CTRL_OFFSET = WB_CURR_OFFSET + NumMotors;
        //std::fill(std::begin(WriteBuffer), std::end(WriteBuffer), 0);
        memset(WriteBuffer, 0, sizeof(WriteBuffer));
        // Write buffer size = NumMotors+2 (quadlets)
        WriteBuffer[WB_HEADER_OFFSET] = (BoardId & 0x0F) << 8 | ((NumMotors+2) & 0xFF);
    }
}

unsigned int AmpIO::GetWriteNumBytes(void) const
{
    unsigned int numQuads;
    if (GetFirmwareVersion() < 8) {
        numQuads = NumMotors + 1;
    } else {
        numQuads = NumMotors + 2;
    }
    return numQuads * sizeof(quadlet_t);
}

bool AmpIO::GetWriteData(quadlet_t *buf, unsigned int offset, unsigned int numQuads, bool doSwap) const
{
    unsigned int WriteBufSize = GetWriteNumBytes() / sizeof(quadlet_t);
    if ((offset+numQuads) > WriteBufSize) {
        std::cerr << "AmpIO:GetWriteData: invalid args: " << offset << ", " << numQuads << std::endl;
        return false;
    }

    for (size_t i = 0; i < numQuads; i++)
        buf[i] = doSwap ? bswap_32(WriteBuffer[offset+i]) : WriteBuffer[offset+i];
    return true;
}

bool AmpIO::WriteBufferResetsWatchdog(void) const
{
    // TODO: Update this
    return
        (WriteBuffer[WB_CURR_OFFSET + 0] & VALID_BIT)
        | (WriteBuffer[WB_CURR_OFFSET + 1] & VALID_BIT)
        | (WriteBuffer[WB_CURR_OFFSET + 2] & VALID_BIT)
        | (WriteBuffer[WB_CURR_OFFSET + 3] & VALID_BIT);
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
    out << std::hex << ReadBuffer[TIMESTAMP_OFFSET] << std::endl;
    out << std::hex << ReadBuffer[STATUS_OFFSET] << std::endl;
    // next two quadlets are digital I/O and amplifier temperature
    out << std::hex << ReadBuffer[DIGIO_OFFSET] << std::endl;
    out << std::hex << ReadBuffer[TEMP_OFFSET] << std::endl;

    // remaining quadlets are in groups of NumMotors/NumEncoders as follows:
    //   - motor current and analog pot per channel
    //   - encoder position per channel
    //   - encoder velocity per channel
    //   - encoder acceleration data (depends on firmware version)
    unsigned int i;
    for (i = MOTOR_CURR_OFFSET; i < ENC_POS_OFFSET; i++)
        out << std::hex << ReadBuffer[i] << " ";
    out << std::endl;
    for (i = ENC_POS_OFFSET; i < ENC_VEL_OFFSET; i++)
        out << std::hex << ReadBuffer[i] << " ";
    out << std::endl;
    for (i = ENC_VEL_OFFSET; i < ENC_QTR1_OFFSET; i++)
        out << std::hex << ReadBuffer[i] << " ";
    out << std::endl;
    for (i = ENC_QTR1_OFFSET; i < ENC_QTR5_OFFSET; i++)
        out << std::hex << ReadBuffer[i] << " ";
    out << std::endl;
    for (i = ENC_QTR5_OFFSET; i < ENC_RUN_OFFSET; i++)
        out << std::hex << ReadBuffer[i] << " ";
    out << std::endl;
    for (i = ENC_RUN_OFFSET; i < MOTOR_STATUS_OFFSET; i++)
        out << std::hex << ReadBuffer[i] << " ";
    out << std::endl;
    if (GetFirmwareVersion() >= 8) {
        for (i = MOTOR_STATUS_OFFSET; i < GetReadNumBytes()/sizeof(quadlet_t); i++)
            out << std::hex << ReadBuffer[i] << " ";
        out << std::endl;
    }
    out << std::dec;
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
    if (index >= NumMotors)
        return 0L;

    quadlet_t buff;
    buff = ReadBuffer[index+MOTOR_CURR_OFFSET];
    buff &= MOTOR_CURR_MASK;       // mask for applicable bits

    return static_cast<AmpIO_UInt32>(buff) & ADC_MASK;
}

AmpIO_UInt32 AmpIO::GetMotorStatus(unsigned int index) const
{
    if (index >= NumMotors)
        return 0L;

    if (GetFirmwareVersion() < 8)
        return 0L;

    return static_cast<AmpIO_UInt32>(ReadBuffer[index+MOTOR_STATUS_OFFSET]);
}

// For dRA1 only
double AmpIO::GetMotorVoltageRatio(unsigned int index) const
{
    if (GetHardwareVersion() != dRA1_String)
        return 0.0;

    if (index >= NumMotors) {
        return 0.0;
    }
    quadlet_t buff;
    buff = ReadBuffer[index+MOTOR_STATUS_OFFSET];
    int16_t raw = buff & 0xffff;
    return (raw >> 5) / 1023.0l;
}

AmpIO_UInt32 AmpIO::GetAnalogInput(unsigned int index) const
{
    if (index >= NumMotors)
        return 0L;

    quadlet_t buff;
    buff = ReadBuffer[index+ANALOG_POS_OFFSET];
    buff &= ANALOG_POS_MASK;       // mask for applicable bits
    buff >>= 16;                   // shift to lsb alignment

    return static_cast<AmpIO_UInt32>(buff) & ADC_MASK;
}

AmpIO_Int32 AmpIO::GetEncoderPosition(unsigned int index) const
{
    if (index < NumEncoders) {
        return static_cast<AmpIO_Int32>(ReadBuffer[index + ENC_POS_OFFSET] & ENC_POS_MASK) - ENC_MIDRANGE;
    }
    return 0;
}

bool AmpIO::GetEncoderOverflow(unsigned int index) const
{
    if (GetHardwareVersion() == dRA1_String)
        return false;

    if (index < NumEncoders) {
        return ReadBuffer[index+ENC_POS_OFFSET] & ENC_OVER_MASK;
    } else {
        std::cerr << "AmpIO::GetEncoderOverflow: index out of range " << index
                  << ", nb channels is " << NumEncoders << std::endl;
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
    if (index >= NumEncoders)
        return 0L;
    return encVelData[index].GetEncoderVelocity();
}

// Returns predicted encoder velocity in counts/sec, taking into account
// acceleration and running counter.
double AmpIO::GetEncoderVelocityPredicted(unsigned int index, double percent_threshold) const
{
    if (index >= NumEncoders)
        return 0.0;
    return encVelData[index].GetEncoderVelocityPredicted(percent_threshold);
}

// Estimate acceleration from two quarters of the same type; units are counts/second**2
// Valid for firmware version 6+.
double AmpIO::GetEncoderAcceleration(unsigned int index, double percent_threshold) const
{
    if (index >= NumEncoders)
        return 0.0;
    return encVelData[index].GetEncoderAcceleration(percent_threshold);
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
    if (index >= NumEncoders)
        return 0.0;
    return encVelData[index].GetEncoderRunningCounterSeconds();
}

AmpIO_Int32 AmpIO::GetEncoderMidRange(void)
{
    return ENC_MIDRANGE;
}

bool AmpIO::SetEncoderVelocityData(unsigned int index)
{
    if (index >= NumEncoders)
        return false;

    AmpIO_UInt32 fver = GetFirmwareVersion();
    if (fver < 6) {
        encVelData[index].SetDataOld(ReadBuffer[ENC_VEL_OFFSET+index], (fver >= 4));
    }
    else if (fver == 6) {
        encVelData[index].SetDataRev6(ReadBuffer[ENC_VEL_OFFSET+index], ReadBuffer[ENC_QTR1_OFFSET+index]);
    }
    else {  // V7+
        encVelData[index].SetData(ReadBuffer[ENC_VEL_OFFSET+index], ReadBuffer[ENC_QTR1_OFFSET+index],
                                  ReadBuffer[ENC_QTR5_OFFSET+index], ReadBuffer[ENC_RUN_OFFSET+index]);
    }
    // Increment error counter if necessary
    if (encVelData[index].IsEncoderError())
        encErrorCount[index]++;

    return true;
}

bool AmpIO::GetEncoderVelocityData(unsigned int index, EncoderVelocity &data) const
{
    if (index >= NumEncoders)
        return false;
    data = encVelData[index];
    return true;
}

unsigned int AmpIO::GetEncoderErrorCount(unsigned int index) const
{
    if (index >= NumEncoders)
        return 0;
    return encErrorCount[index];
}

bool AmpIO::ClearEncoderErrorCount(unsigned int index)
{
    bool ret = true;
    if (index == MAX_CHANNELS) {
        // Clear all counters
        for (unsigned int i = 0; i < NumEncoders; i++)
            encErrorCount[i] = 0;
    }
    else if (index < NumEncoders) {
        encErrorCount[index] = 0;
    }
    else {
        ret = false;
    }
    return ret;
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

bool AmpIO::GetPowerFault(void) const
{
    // Bit 15: motor power fault
    if (GetHardwareVersion() == dRA1_String)
        return false;

    return (GetStatus()&0x00008000);
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
    if (index >= NumMotors)
        return false;

    bool ret;
    if (GetFirmwareVersion() < 8) {
        AmpIO_UInt32 mask = (0x00000001 << index);
        ret = GetStatus()&mask;
    }
    else {
        ret = ReadBuffer[MOTOR_STATUS_OFFSET + index] & MOTOR_ENABLE_BIT;
    }
    return ret;
}

AmpIO_UInt8 AmpIO::GetAmpEnableMask(void) const
{
    return GetStatus()&0x0000000f;
}

bool AmpIO::GetAmpStatus(unsigned int index) const
{
    if (index >= NumMotors)
        return false;

    bool ret;
    if (GetFirmwareVersion() < 8) {
        AmpIO_UInt32 mask = (0x00000100 << index);
        ret = GetStatus()&mask;
    }
    else {
        // TODO: check 0xf0000
        ret = !(ReadBuffer[MOTOR_STATUS_OFFSET + index] & (0xf0000)) && GetAmpEnable(index);
    }
    return ret;
}

AmpIO_UInt32 AmpIO::GetSafetyAmpDisable(void) const
{
    if (GetHardwareVersion() == dRA1_String)
        return 0;

    AmpIO_UInt32 mask = 0x000000F0;
    return (GetStatus() & mask) >> 4;
}

AmpIO_UInt32 AmpIO::GetAmpFaultCode(unsigned int index) const
{
    if (GetHardwareVersion() != dRA1_String)
        return 0;
    return (ReadBuffer[MOTOR_STATUS_OFFSET + index] & (0xf0000)) >> 16;
}

bool AmpIO::IsQLAExpanded() const
{
    return (GetStatus() & 0x00001000);
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
    if (index >= NumMotors)
        return false;

    if (GetFirmwareVersion() < 8) {
        AmpIO_UInt32 enable_mask = 0x00000100 << index;
        AmpIO_UInt32 state_mask  = 0x00000001 << index;
        WriteBuffer[WB_CTRL_OFFSET] |=  enable_mask;
        if (state)
            WriteBuffer[WB_CTRL_OFFSET] |=  state_mask;
        else
            WriteBuffer[WB_CTRL_OFFSET] &= ~state_mask;
    }
    else {
        WriteBuffer[WB_CURR_OFFSET + index] |=  MOTOR_ENABLE_MASK;
        if (state)
            WriteBuffer[WB_CURR_OFFSET + index] |=  MOTOR_ENABLE_BIT;
        else
            WriteBuffer[WB_CURR_OFFSET + index] &= ~MOTOR_ENABLE_BIT;
    }
    return true;
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
    }
    else {
        for (unsigned int i = 0; i < NumMotors; i++) {
            if ((mask >> i) & 1)
                SetAmpEnable(i, (state >> i) & 1);
        }
    }
    return true;
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
    if (index >= NumMotors)
        return false;

    quadlet_t data = VALID_BIT | (sdata & DAC_MASK);
    if (collect_state && (collect_chan == (index+1)))
        data |= COLLECT_BIT;
    if (GetFirmwareVersion() < 8) {
        data |= ((BoardId & 0x0F) << 24);
    }
    else {
        // Preserve any motor enable bits set by SetAmpEnable
        data |= WriteBuffer[WB_CURR_OFFSET+index]&(MOTOR_ENABLE_MASK|MOTOR_ENABLE_BIT);
    }
    WriteBuffer[WB_CURR_OFFSET+index] = data;
    return true;
}

bool AmpIO::SetMotorVoltage(unsigned int index, AmpIO_UInt32 mvolt)
{
    if (index >= NumMotors)
        return false;
    if (GetFirmwareVersion() < 8)
        return false;
    if (!IsQLAExpanded())  // check for QLA 1.5+
        return false;

    quadlet_t data = VALID_BIT  | (1 << 24) | (mvolt & DAC_MASK);
    if (collect_state && (collect_chan == (index+1)))
        data |= COLLECT_BIT;
     // Preserve any motor enable bits set by SetAmpEnable
    data |= WriteBuffer[WB_CURR_OFFSET+index]&(MOTOR_ENABLE_MASK|MOTOR_ENABLE_BIT);
    WriteBuffer[WB_CURR_OFFSET+index] = data;
    return true;
}

bool AmpIO::SetMotorVoltageRatio(unsigned int index, double ratio)
{
    if (GetHardwareVersion() != dRA1_String)
        return false;

    if (index < NumMotors) {
        if (collect_state && (collect_chan == (index+1))) {
            WriteBuffer[index+WB_CURR_OFFSET] |= COLLECT_BIT;
        }
        WriteBuffer[index+WB_CURR_OFFSET] |= VALID_BIT;
        WriteBuffer[index+WB_CURR_OFFSET] &= ~0x0FFFFFFF;
        WriteBuffer[index+WB_CURR_OFFSET] |= 1 << 24; // select voltage mode
        WriteBuffer[index+WB_CURR_OFFSET] |= ((int)(ratio * 1023) & 0b11111111111) << 13;
        return true;
    } else {
        return false;
    }
}

/*******************************************************************************
 * Read commands
 */

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
    // This field has been present in the Status register since Firmware Rev 3
    // (Firmware Rev 2 used register 11 and Firmware Rev 1 did not have this field)
    return (ReadStatus()&0x000000F0) >> 4;
}

bool AmpIO::ReadEncoderPreload(unsigned int index, AmpIO_Int32 &sdata) const
{
    bool ret = false;
    if (port && (index < NumEncoders)) {
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

// TODO: Is this only for QLA1?
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
    if (port && (index < NumDouts)) {
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

bool AmpIO::ReadIOExpander(AmpIO_UInt32 &resp)
{
    return port->ReadQuadlet(BoardId, 14, resp);
}

/*******************************************************************************
 * Write commands
 */

bool AmpIO::WritePowerEnable(bool state)
{
    AmpIO_UInt32 write_data = state ? PWR_ENABLE : PWR_DISABLE;
    return (port ? port->WriteQuadlet(BoardId, BoardIO::BOARD_STATUS, write_data) : false);
}

bool AmpIO::WriteAmpEnable(AmpIO_UInt32 mask, AmpIO_UInt32 state)
{
    // Following still works for Firmware Rev 8 (at least for QLA)
    quadlet_t write_data = (mask << 8) | state;
    return (port ? port->WriteQuadlet(BoardId, BoardIO::BOARD_STATUS, write_data) : false);
}

bool AmpIO::WriteSafetyRelay(bool state)
{
    AmpIO_UInt32 write_data = state ? RELAY_ON : RELAY_OFF;
    return (port ? port->WriteQuadlet(BoardId, BoardIO::BOARD_STATUS, write_data) : false);
}

bool AmpIO::WriteEncoderPreload(unsigned int index, AmpIO_Int32 sdata)
{
    unsigned int channel = (index+1) << 4;

    if ((sdata >= ENC_MIDRANGE) || (sdata < -ENC_MIDRANGE)) {
        std::cerr << "AmpIO::WriteEncoderPreload, preload out of range " << sdata << std::endl;
        return false;
    }
    bool ret = false;
    if (port && (index < NumEncoders)) {
        ret = port->WriteQuadlet(BoardId, channel | ENC_LOAD_OFFSET,
                                 static_cast<AmpIO_UInt32>(sdata + ENC_MIDRANGE));
    }
    return ret;
}

bool AmpIO::WriteDoutConfigReset(void)
{
    return (port && (GetFirmwareVersion() >= 7)) ? port->WriteQuadlet(BoardId, BoardIO::BOARD_STATUS, DOUT_CFG_RESET) : false;
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
    if (GetHardwareVersion() == dRA1_String) {
        for (int axis = 1; axis < 11; axis ++) {
            WriteCurrentKpRaw(axis - 1, 2000);
            WriteCurrentKiRaw(axis - 1, 200);
            WriteCurrentITermLimitRaw(axis - 1, 1000);
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
    if (port && (index < NumDouts)) {
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

bool AmpIO::WriteIOExpander(AmpIO_UInt32 cmd)
{
    return port->WriteQuadlet(BoardId, 14, cmd);
}

/*******************************************************************************
 * Static Write methods (for broadcast)
 *
 * These methods duplicate the board-specific methods (WriteReboot, WritePowerEnable,
 * WriteAmpEnable, ...), but are sent to the broadcast address (FW_NODE_BROADCAST).
 */

bool AmpIO::WritePowerEnableAll(BasePort *port, bool state)
{
    AmpIO_UInt32 write_data = state ? PWR_ENABLE : PWR_DISABLE;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, BoardIO::BOARD_STATUS, write_data) : false);
}

bool AmpIO::WriteAmpEnableAll(BasePort *port, AmpIO_UInt32 mask, AmpIO_UInt32 state)
{
    // TODO: Following should still work for Firmware Rev 8
    quadlet_t write_data = (mask << 8) | state;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, BoardIO::BOARD_STATUS, write_data) : false);
}

bool AmpIO::WriteSafetyRelayAll(BasePort *port, bool state)
{
    AmpIO_UInt32 write_data = state ? RELAY_ON : RELAY_OFF;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, BoardIO::BOARD_STATUS, write_data) : false);
}

bool AmpIO::WriteEncoderPreloadAll(BasePort *port, unsigned int index, AmpIO_Int32 sdata)
{
    unsigned int channel = (index+1) << 4;

    if ((sdata >= ENC_MIDRANGE) || (sdata < -ENC_MIDRANGE)) {
        std::cerr << "AmpIO::WriteEncoderPreloadAll, preload out of range " << sdata << std::endl;
        return false;
    }
    bool ret = false;
    // Cannot use NumEncoders below because this is a static method
    if (port && (index < MAX_CHANNELS)) {
        ret = port->WriteQuadlet(FW_NODE_BROADCAST, channel | ENC_LOAD_OFFSET,
                                 static_cast<AmpIO_UInt32>(sdata + ENC_MIDRANGE));
    }
    return ret;
}

// ********************** Dallas DS2505 (1-wire / DS2480B) Reading Methods ****************************************
AmpIO::DallasStatus AmpIO::DallasReadTool(AmpIO_UInt32 &model, AmpIO_UInt8 &version, std::string &name,
                                          double timeoutSec)
{
    if (GetHardwareVersion() == dRA1_String) {
        if (!port->ReadQuadlet(BoardId, 0xb012, model))
            return DALLAS_IO_ERROR;
        model = bswap_32(model);
        AmpIO_UInt32 ver;
        if (!port->ReadQuadlet(BoardId, 0xb013, ver))
            return DALLAS_IO_ERROR;
        version = ver & 0x000000ff;
        name = "";
        if (version != 255) {
            return DALLAS_OK;
        }
        return DALLAS_WAIT;
    }
    else if (GetHardwareVersion() == QLA1_String) {
        if (GetFirmwareVersion() < 7)
            return DALLAS_NONE;

        AmpIO_UInt32 status;
        AmpIO_UInt32 ctrl;
        char buffer[256];

        dallasTimeoutSec = timeoutSec;
        DallasStatus ret = DALLAS_WAIT;

        switch (dallasState) {

        case ST_DALLAS_START:
            // Start reading at address DALLAS_START_READ
            ctrl = (DALLAS_START_READ<<16)|2;
            if (DallasWriteControl(ctrl)) {
                dallasState = ST_DALLAS_WAIT;
                dallasStateNext = ST_DALLAS_READ;
                dallasWaitStart = Amp1394_GetTime();
            }
            else {
                ret = DALLAS_IO_ERROR;
            }
            break;

        case ST_DALLAS_WAIT:
            if (DallasReadStatus(status)) {
                // Done when in idle state. The following test works for both Firmware Rev 7
                // (state is bits 7:4) and Firmware Rev 8 (state is bits 8:4 and busy flag is bit 13)
                if ((status&0x000020F0) == 0) {
                    dallasState = dallasStateNext;
                    // Automatically detect interface in use
                    dallasUseDS2480B = (status & 0x00008000) == 0x00008000;
                }
                else if (Amp1394_GetTime() > dallasWaitStart + dallasTimeoutSec) {
                    ret = DALLAS_TIMEOUT;
                }
            }
            else {
                ret = DALLAS_IO_ERROR;
            }
            break;

        case ST_DALLAS_READ:
            if (DallasReadBlock(reinterpret_cast<unsigned char *>(buffer), sizeof(buffer))) {
                // make sure we read the 997 from company statement
                if (strncmp(buffer, "997", 3) == 0) {
                    // get model and name of tool to create unique string identifier
                    // model number uses only 3 bytes, set first one to zero just in case
                    buffer[DALLAS_MODEL_OFFSET] = 0;
                    model = *(reinterpret_cast<AmpIO_UInt32 *>(buffer + (DALLAS_MODEL_OFFSET - DALLAS_START_READ)));
                    model = bswap_32(model);
                    // version number
                    version = static_cast<AmpIO_UInt8>(buffer[DALLAS_VERSION_OFFSET - DALLAS_START_READ]);
                    // name
                    buffer[DALLAS_NAME_END - DALLAS_START_READ] = '\0';
                    name = buffer + (DALLAS_NAME_OFFSET - DALLAS_START_READ);
                    ret = DALLAS_OK;
                    dallasState = ST_DALLAS_START;  // Nominal; could be updated below for DS2480B
                }
                else {
                    ret = DALLAS_DATA_ERROR;
                }
            }
            else {
                ret = DALLAS_IO_ERROR;
            }
            if (dallasUseDS2480B) {
                // 0x09 indicates reg_wdata[3] == 1 && reg_wdata[1:0] == 01 in firmware DS2505.v
                // If the write fails, we ignore it and return to ST_DALLAS_START, rather than
                // return DALLAS_IO_ERROR, because this is after the read.
                if (DallasWriteControl(0x09)){
                    dallasState = ST_DALLAS_WAIT;
                    dallasStateNext = ST_DALLAS_START;
                    dallasWaitStart = Amp1394_GetTime();
                }
            }
            break;
        }

        if ((ret == DALLAS_IO_ERROR) || (ret == DALLAS_TIMEOUT) || (ret == DALLAS_DATA_ERROR)) {
            dallasState = ST_DALLAS_START;
        }
        return ret;
    }

    return DALLAS_NONE;   // Not QLA or dRAC
}

bool AmpIO::DallasWriteControl(AmpIO_UInt32 ctrl)
{
    if (GetFirmwareVersion() < 7) return false;
    if (GetHardwareVersion() != QLA1_String) return false;
    return port->WriteQuadlet(BoardId, 13, ctrl);
}


bool AmpIO::DallasReadStatus(AmpIO_UInt32 &status)
{
    status = 0;
    if (GetFirmwareVersion() < 7) return false;
    if (GetHardwareVersion() != QLA1_String) return false;
    return port->ReadQuadlet(BoardId, 13, status);
}

bool AmpIO::DallasWaitIdle()
{
    int i;
    AmpIO_UInt32 status;
    // For each block (256 bytes), wait up to 2000 msec. The actual processing time varies according to system setting.
    // Based on measurements, direct 1-wire interface costs about 250-300 msec to read entire 2048 bytes.
    // For DS2480B serial method, the approximate read time is 5 sec.
    for (i = 0; i < 2000; i++) {
        // Wait 1 msec
        Amp1394_Sleep(0.001);
        if (!DallasReadStatus(status)) return false;
        // Done when in idle state. The following test works for both Firmware Rev 7
        // (state is bits 7:4) and Firmware Rev 8 (state is bits 8:4 and busy flag is bit 13)
        if ((status&0x000020F0) == 0)
            break;
    }
    //std::cerr << "Wait time = " << i << " milliseconds" << std::endl;
    return (i < 2000);
}

bool AmpIO::DallasReadBlock(unsigned char *data, unsigned int nbytes) const
{
    if (GetHardwareVersion() != QLA1_String)
        return false;

    nodeaddr_t address = 0x6000;
    if (nbytes > 256)
        return false;
    if (port)
        return port->ReadBlock(BoardId, address, reinterpret_cast<quadlet_t *>(data), nbytes);
    return false;
}

bool AmpIO::DallasReadMemory(unsigned short addr, unsigned char *data, unsigned int nbytes)
{
    if (GetFirmwareVersion() < 7) return false;
    if (GetHardwareVersion() != QLA1_String) return false;

    AmpIO_UInt32 status;
    AmpIO_UInt32 ctrl = (addr<<16)|2;
    if (!DallasWriteControl(ctrl)) return false;
    if (!DallasWaitIdle()) return false;
    if (!DallasReadStatus(status)) return false;

    // Automatically detect interface in use
    bool useDS2480B = (status & 0x00008000) == 0x00008000;

    unsigned char *ptr = data;
    // Read first block of data (up to 256 bytes)
    unsigned int nb = (nbytes>256) ? 256 : nbytes;
    if (!DallasReadBlock(ptr, nb)) return false;
    ptr += nb;
    nbytes -= nb;
    // Read additional blocks of data if necessary
    while (nbytes > 0) {
        // 0x03 indicates reg_wdata[1:0] == 11 in firmware DS2505.v
        if (!DallasWriteControl(0x03)) return false;
        if (!DallasWaitIdle()) return false;
        nb = (nbytes>256) ? 256 : nbytes;
        if (!DallasReadBlock(ptr, nb)) return false;
        ptr += nb;
        nbytes -= nb;
    }
    // End all blocks reading for DS2480B interface
    if (nbytes <= 0 && useDS2480B) {
        // 0x09 indicates reg_wdata[3] == 1 && reg_wdata[1:0] == 01 in firmware DS2505.v
        if (!DallasWriteControl(0x09)) return false;
        if (!DallasWaitIdle()) return false;
    }
    return true;
}

AmpIO_UInt32 AmpIO::SPSMReadToolModel(void) const
{
    uint32_t instrument_id = 0;
    if (GetHardwareVersion() == dRA1_String) {
        port->ReadQuadlet(BoardId, 0xb012, instrument_id);
        return bswap_32(instrument_id);
    }
    return 0;
}

AmpIO_UInt8 AmpIO::SPSMReadToolVersion(void) const
{
    uint32_t q = 0;
    if (GetHardwareVersion() == dRA1_String) {
        port->ReadQuadlet(BoardId, 0xb013, q);
        return q & 0xFF;
    }
    return q;
}

// ************************************* Waveform methods ****************************************

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

// ********************************** Data collection methods ****************************************

bool AmpIO::DataCollectionStart(unsigned char chan, CollectCallback collectCB)
{
    if (GetFirmwareVersion() < 7) return false;
    if ((chan < 1) || (chan > NumMotors)) return false;
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
            numAvail = static_cast<unsigned short>(port->GetMaxReadDataSize()/sizeof(quadlet_t));
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

//******* Following methods are for dRA1 ********

bool AmpIO::WriteMotorControlMode(unsigned int index, AmpIO_UInt16 mode) {
    return (port ? port->WriteQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_MOTOR_CONTROL_MODE, mode) : false);
}

bool AmpIO::WriteCurrentKpRaw(unsigned int index, AmpIO_UInt32 val) {
    return (port ? port->WriteQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_CURRENT_KP, val) : false);
}

bool AmpIO::WriteCurrentKiRaw(unsigned int index, AmpIO_UInt32 val) {
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

AmpIO_UInt32 AmpIO::ReadCurrentKpRaw(unsigned int index) const
{
    AmpIO_UInt32 read_data = 0;
    if (port)
        port->ReadQuadlet(BoardId, ADDR_MOTOR_CONTROL << 12 | (index + 1) << 4 | OFF_CURRENT_KP, read_data);
    return read_data;
}

AmpIO_UInt32 AmpIO::ReadCurrentKiRaw(unsigned int index) const
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
