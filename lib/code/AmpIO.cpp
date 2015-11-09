/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <byteswap.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

#include "AmpIO.h"
#include "FirewirePort.h"

const AmpIO_UInt32 VALID_BIT        = 0x80000000;  /*!< High bit of 32-bit word */
const AmpIO_UInt32 MIDRANGE_ADC     = 0x00008000;  /*!< Midrange value of ADC bits */
const AmpIO_UInt32 MIDRANGE_VEL     = 0x00008000;  /*!< Midrange value of encoder velocity */
const AmpIO_UInt32 MIDRANGE_FRQ     = 0x00008000;  /*!< Midrange value of encoder frequency */
const AmpIO_UInt32 MIDRANGE_ACC     = 0x00008000;  /*!< Midrange value of encoder acc */
const AmpIO_UInt32 ENC_PRELOAD      = 0x007fffff;  /*!< Encoder preload value */
const AmpIO_Int32  ENC_MIDRANGE     = 0x00800000;

const AmpIO_UInt32 PWR_ENABLE       = 0x000c0000;  /*!< Turn pwr_en on             */
const AmpIO_UInt32 PWR_DISABLE      = 0x00080000;  /*!< Turn pwr_en off            */
const AmpIO_UInt32 RELAY_ON         = 0x00030000;  /*!< Turn safety relay on       */
const AmpIO_UInt32 RELAY_OFF        = 0x00020000;  /*!< Turn safety relay off      */
const AmpIO_UInt32 ENABLE_MASK      = 0x0000ffff;  /*!< Mask for power enable bits */
const AmpIO_UInt32 MOTOR_CURR_MASK  = 0x0000ffff;  /*!< Mask for motor current adc bits */
const AmpIO_UInt32 ANALOG_POS_MASK  = 0xffff0000;  /*!< Mask for analog pot ADC bits */
const AmpIO_UInt32 ADC_MASK         = 0x0000ffff;  /*!< Mask for right aligned ADC bits */
const AmpIO_UInt32 DAC_MASK         = 0x0000ffff;  /*!< Mask for 16-bit DAC values */
const AmpIO_UInt32 ENC_POS_MASK     = 0x00ffffff;  /*!< Encoder position mask */
const AmpIO_UInt32 ENC_OVER_MASK    = 0x01000000;  /*!< Encoder bit overflow mask */
const AmpIO_UInt32 ENC_VEL_MASK     = 0x0000ffff;  /*!< Mask for encoder velocity bits */
const AmpIO_UInt32 ENC_FRQ_MASK     = 0x0000ffff;  /*!< Mask for encoder frequency bits */

const AmpIO_UInt32 DAC_WR_A         = 0x00300000;  /*!< Command to write DAC channel A */

const double FPGA_sysclk_MHz        = 49.152;      /* FPGA sysclk in MHz (from FireWire) */


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
    else { std::cout << MSG.str() << std::endl; }


AmpIO_UInt8 BitReverse4[16] = { 0x0, 0x8, 0x4, 0xC,         // 0000, 0001, 0010, 0011
                                0x2, 0xA, 0x6, 0xE,         // 0100, 0101, 0110, 0111
                                0x1, 0x9, 0x5, 0xD,         // 1000, 1001, 1010, 1011
                                0x3, 0xB, 0x7, 0xF };       // 1100, 1101, 1110, 1111

AmpIO::AmpIO(AmpIO_UInt8 board_id, unsigned int numAxes) : BoardIO(board_id), NumAxes(numAxes)
{
    memset(read_buffer, 0, sizeof(read_buffer));
    memset(write_buffer, 0, sizeof(write_buffer));
    // Set members in base class (in the future, some of these may be set from
    // the FirewirePort class, when the board is added)
    ReadBufferSize = sizeof(read_buffer);
    ReadBuffer = read_buffer;
    WriteBufferSize = sizeof(write_buffer);
    WriteBuffer = write_buffer;
}

AmpIO::~AmpIO()
{
    if (port) {
        std::cerr << "Warning: AmpIO being destroyed while still in use by FirewirePort" << std::endl;
        port->RemoveBoard(this);
    }
}

AmpIO_UInt32 AmpIO::GetFirmwareVersion(void) const
{
    return (port ? port->GetFirmwareVersion(BoardId) : 0);
}

std::string AmpIO::GetFPGASerialNumber(void)
{
    // Format: FPGA 1234-56 (12 bytes)
    AmpIO_UInt32 address = 0x001FFF00;
    AmpIO_UInt8 data[20];
    std::string sn; sn.clear();
    const size_t FPGASNSize = 12;

    PromReadData(address, data, FPGASNSize);
    for (size_t i = 0; i < FPGASNSize; i++) {
        sn.push_back(data[i]);
    }

    if (sn.substr(0,5) == "FPGA ")
        sn.erase(0,5);
    else {
        std::cerr << "Invalid FPGA Serial Number: " << sn << std::endl;
        sn.clear();
    }
    return sn;
}

std::string AmpIO::GetQLASerialNumber(void)
{
    // Format: QLA 1234-56
    std::string sn; sn.clear();
    AmpIO_UInt16 address = 0x0000;
    AmpIO_UInt8 data;
    const size_t QLASNSize = 11;
    for (size_t i = 0; i < QLASNSize; i++) {
        if (!PromReadByte25AA128(address, data))
            std::cerr << "Failed to get QLA Serial Number" << std::endl;
        else {
            sn.push_back(data);
            address += 1;
        }
    }
    if (sn.substr(0,4) == "QLA ")
        sn.erase(0,4);
    else {
        std::cerr << "Invalid QLA Serial Number: " << sn << std::endl;
        sn.clear();
    }
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
    AmpIO_UInt8 dout = static_cast<AmpIO_UInt8>((bswap_32(read_buffer[DIGIO_OFFSET])>>12)&0x000f);
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
        std::cerr << "Warning: GetEncoderOverflow, index out of range " << index << std::endl;
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

// temp current the enc period velocity is unsigned 16 bits
// for low level function the + MIDRANGE_VEL
AmpIO_UInt32 AmpIO::GetEncoderVelocity(unsigned int index, const bool islatch) const
{
    // buff = [cnter_now, cnter_latch]
    // cnter_latch: tick latched velcotity data
    // cnter_now  : ongoing counting cnter data
    // both are signed 16-bit data
    // Clock = 768 kHz
    // stored in a 32 bit unsiged int

    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+ENC_VEL_OFFSET]);

    AmpIO_UInt32 cnter, cnter_latch;
    cnter_latch = buff & ENC_VEL_MASK;
    cnter = ((buff & 0xFFFF0000) >> 16);

    if (islatch) return cnter_latch;
    else return cnter;
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
    AmpIO_UInt32 enable_mask = bswap_32(0x00080000);
    write_buffer[WB_CTRL_OFFSET] |=  enable_mask;
    AmpIO_UInt32 state_mask  = bswap_32(0x00040000);
    if (state)
        write_buffer[WB_CTRL_OFFSET] |=  state_mask;
    else
        write_buffer[WB_CTRL_OFFSET] &= ~state_mask;
}

bool AmpIO::SetAmpEnable(unsigned int index, bool state)
{
    if (index < NUM_CHANNELS) {
        AmpIO_UInt32 enable_mask = bswap_32(0x00000100 << index);
        write_buffer[WB_CTRL_OFFSET] |=  enable_mask;
        AmpIO_UInt32 state_mask  = bswap_32(0x00000001 << index);
        if (state)
            write_buffer[WB_CTRL_OFFSET] |=  state_mask;
        else
            write_buffer[WB_CTRL_OFFSET] &= ~state_mask;
        return true;
    }
    return false;
}

void AmpIO::SetSafetyRelay(bool state)
{
    AmpIO_UInt32 enable_mask = bswap_32(0x00020000);
    write_buffer[WB_CTRL_OFFSET] |=  enable_mask;
    AmpIO_UInt32 state_mask  = bswap_32(0x00010000);
    if (state)
        write_buffer[WB_CTRL_OFFSET] |=  state_mask;
    else
        write_buffer[WB_CTRL_OFFSET] &= ~state_mask;
}

bool AmpIO::SetMotorCurrent(unsigned int index, AmpIO_UInt32 sdata)
{
    quadlet_t data = 0x00;
    data = VALID_BIT | ((BoardId & 0x0F) << 24) | DAC_WR_A | (sdata & DAC_MASK);

    if (index < NUM_CHANNELS) {
        write_buffer[index+WB_CURR_OFFSET] = bswap_32(data);
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
    return bswap_32(read_data);
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
    return bswap_32(read_data) & 0x0000000F;
}

bool AmpIO::ReadDoutControl(unsigned int index, AmpIO_UInt16 &countsHigh, AmpIO_UInt16 &countsLow)
{
    countsHigh = 0;
    countsLow = 0;
    if (GetFirmwareVersion() < 5) return false;

    AmpIO_UInt32 read_data;
    unsigned int channel = (index+1) << 4;
    if (port && (index < NUM_CHANNELS)) {
        if (port->ReadQuadlet(BoardId, channel | DOUT_CTRL_OFFSET, read_data)) {
            read_data = bswap_32(read_data);
            countsHigh = static_cast<AmpIO_UInt16>(read_data >> 16);
            countsLow  = static_cast<AmpIO_UInt16>(read_data);
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
    return (port ? port->WriteQuadlet(BoardId, 0, bswap_32(write_data)) : false);
}

bool AmpIO::WriteAmpEnable(AmpIO_UInt8 mask, AmpIO_UInt8 state)
{
    quadlet_t write_data = (mask << 8) | state;
    return (port ? port->WriteQuadlet(BoardId, 0, bswap_32(write_data)) : false);
}

bool AmpIO::WriteSafetyRelay(bool state)
{
    AmpIO_UInt32 write_data = state ? RELAY_ON : RELAY_OFF;
    return (port ? port->WriteQuadlet(BoardId, 0, bswap_32(write_data)) : false);
}

bool AmpIO::WriteEncoderPreload(unsigned int index, AmpIO_Int32 sdata)
{
    unsigned int channel = (index+1) << 4;

    if ((sdata >= ENC_MIDRANGE)
            || (sdata < -ENC_MIDRANGE)) {
        std::cerr << "Error: WriteEncoderPreload, preload out of range" << std::endl;
        return false;
    }
    if (port && (index < NUM_CHANNELS)) {
        return port->WriteQuadlet(BoardId, channel | ENC_LOAD_OFFSET, bswap_32(static_cast<AmpIO_UInt32>(sdata + ENC_MIDRANGE)));
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
    quadlet_t write_data = (mask << 8) | bits;
    return port->WriteQuadlet(BoardId, 6, bswap_32(write_data));
}

bool AmpIO::WriteWatchdogPeriod(AmpIO_UInt32 counts)
{
    // period = counts(16 bits) * 5.208333 us (default = 0 = no timeout)
    return port->WriteQuadlet(BoardId, 3, bswap_32(counts));
}

bool AmpIO::WriteDoutControl(unsigned int index, AmpIO_UInt16 countsHigh, AmpIO_UInt16 countsLow)
{
    if (GetFirmwareVersion() < 5) return false;
 
    // Counter frequency = 49.152 MHz --> 1 count is about 0.02 uS
    //    Max high/low time = (2^16-1)/49.152 usec = 1333.3 usec = 1.33 msec
    //    The max PWM period with full adjustment of duty cycle (1-65535) is (2^16-1+1)/49.152 usec = 1.33 msec
    unsigned int channel = (index+1) << 4;
    if (port && (index < NUM_CHANNELS)) {
        AmpIO_UInt32 counts = (static_cast<AmpIO_UInt32>(countsHigh) << 16) | countsLow;
        return port->WriteQuadlet(BoardId, channel | DOUT_CTRL_OFFSET, bswap_32(counts));
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
    if (port->WriteQuadlet(BoardId, 0x08, bswap_32(data))) {
        // Should be ready by now...
        id = PromGetResult();
    }
    return id;
}

AmpIO_UInt32 AmpIO::PromGetStatus(PromType type)
{
    AmpIO_UInt32 status = 0x80000000;
    quadlet_t data = 0x05000000;
    nodeaddr_t address = GetPromAddress(type, true);

    if (port->WriteQuadlet(BoardId, address, bswap_32(data))) {
        // Should be ready by now...
        status = PromGetResult(type);
    }
    return status;
}

AmpIO_UInt32 AmpIO::PromGetResult(PromType type)
{
    AmpIO_UInt32 result = 0xffffffff;
    quadlet_t data;
    nodeaddr_t address = GetPromAddress(type, false);

    if (port->ReadQuadlet(BoardId, address, data))
        result = static_cast<AmpIO_UInt32>(bswap_32(data));
    return result;
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
        unsigned int bytesToRead = std::min(nbytes - page, 256u);
        if (!port->WriteQuadlet(BoardId, 0x08, bswap_32(write_data)))
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
            usleep(10);
            if (!port->ReadQuadlet(BoardId, 0x08, read_data)) return false;
            read_data = bswap_32(read_data)&0x000f;
        }
        if (i == MAX_LOOP_CNT) {
            std::cout << "PromReadData: command failed to finish, status = "
                      << std::hex << read_data << std::dec << std::endl;
            return false;
        }
        // Now, read result. This should be the number of quadlets written.
        AmpIO_UInt32 nRead = 4*PromGetResult();
        if (nRead != 256) { // should never happen
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
    return port->WriteQuadlet(BoardId, address, bswap_32(write_data));
}

bool AmpIO::PromWriteDisable(PromType type)
{
    quadlet_t write_data = 0x04000000;
    nodeaddr_t address = GetPromAddress(type, true);
    return port->WriteQuadlet(BoardId, address, bswap_32(write_data));
}

bool AmpIO::PromSectorErase(AmpIO_UInt32 addr, const ProgressCallback cb)
{
    PromWriteEnable();
    quadlet_t write_data = 0xd8000000 | (addr&0x00ffffff);
    if (!port->WriteQuadlet(BoardId, 0x08, bswap_32(write_data)))
        return false;
    // Wait for erase to finish
    while (PromGetStatus())
        PROGRESS_CALLBACK(cb, false);
    return true;
}

int AmpIO::PromProgramPage(AmpIO_UInt32 addr, const AmpIO_UInt8 *bytes,
                           unsigned int nbytes, const ProgressCallback cb)
{
    const unsigned int MAX_PAGE = 256;  // 64 quadlets
    if (nbytes > MAX_PAGE) {
        std::ostringstream msg;
        msg << "PromProgramPage: error, nbytes = " << nbytes
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
    if (!port->WriteBlock(BoardId, address, data_ptr, nbytes+sizeof(quadlet_t)))
        return -1;
    // Read FPGA status register; if 4 LSB are 0, command has finished
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 0x08, read_data)) return -1;
    read_data = bswap_32(read_data);
    while (read_data&0x000f) {
        PROGRESS_CALLBACK(cb, -1);
        if (!port->ReadQuadlet(BoardId, 0x08, read_data)) return false;
        read_data = bswap_32(read_data);
    }
    if (read_data & 0xff000000) { // shouldn't happen
        std::ostringstream msg;
        msg << "PromProgramPage: FPGA error = " << read_data;
        ERROR_CALLBACK(cb, msg);
    }
    // Now, read result. This should be the number of quadlets written.
    AmpIO_UInt32 nWritten = 4*(PromGetResult()-1);
    if (nWritten != nbytes) {
        std::ostringstream msg;
        msg << "PromProgramPage: wrote " << nWritten << " of "
            << nbytes << "bytes";
        ERROR_CALLBACK(cb, msg);
    }
    // Wait for "Write in Progress" bit to be cleared
    while (PromGetStatus()&MASK_WIP)
        PROGRESS_CALLBACK(cb, 0);
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
        std::cerr << "Error: unsupported PROM type" << std::endl;

    return 0x00;
}


// ********************** QLA PROM ONLY Methods ***********************************
bool AmpIO::PromReadByte25AA128(AmpIO_UInt16 addr, AmpIO_UInt8 &data)
{
    // 8-bit cmd + 16-bit addr (2 MSBs ignored)
    AmpIO_UInt32 result = 0x00000000;
    quadlet_t write_data = 0x03000000|(addr << 8);
    nodeaddr_t address = GetPromAddress(PROM_25AA128, true);

    if (port->WriteQuadlet(BoardId, address, bswap_32(write_data))) {
        // Should be ready by now...
        result = PromGetResult(PROM_25AA128);

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
    usleep(100);

    // 8-bit cmd + 16-bit addr + 8-bit data
    quadlet_t write_data = 0x02000000|(addr << 8)|data;
    nodeaddr_t address = GetPromAddress(PROM_25AA128, true);
    if (port->WriteQuadlet(BoardId, address, bswap_32(write_data))) {
        // wait 5ms for the PROM to be ready to take new commands
        usleep(5000);
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
    if (!port->WriteQuadlet(BoardId, address, bswap_32(write_data)))
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
    return port->WriteQuadlet(BoardId, address, bswap_32(write_data));
}

// ********************** KSZ8851 Ethernet Controller Methods ***********************************

bool AmpIO::ResetKSZ8851()
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x04000000;
    return port->WriteQuadlet(BoardId, 12, bswap_32(write_data));
}

bool AmpIO::WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt8 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x02000000 | (static_cast<quadlet_t>(addr) << 16) | data;
    return port->WriteQuadlet(BoardId, 12, bswap_32(write_data));
}

bool AmpIO::WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt16 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x03000000 | (static_cast<quadlet_t>(addr) << 16) | data;
    return port->WriteQuadlet(BoardId, 12, bswap_32(write_data));
}

bool AmpIO::ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt8 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = (static_cast<quadlet_t>(addr) << 16) | data;
    if (!port->WriteQuadlet(BoardId, 12, bswap_32(write_data)))
        return false;
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 12, read_data))
        return false;
    read_data = bswap_32(read_data);
    // Bit 31 indicates whether Ethernet is present
    if (!(read_data&0x80000000)) return false;
    // Bit 30 indicates whether last command had an error
    if (read_data&0x40000000) return false;
    data = static_cast<AmpIO_UInt8>(read_data);
    return true;
}

bool AmpIO::ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt16 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x01000000 | (static_cast<quadlet_t>(addr) << 16) | data;
    if (!port->WriteQuadlet(BoardId, 12, bswap_32(write_data)))
        return false;
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 12, read_data))
        return false;
    read_data = bswap_32(read_data);
    // Bit 31 indicates whether Ethernet is present
    if (!(read_data&0x80000000)) return false;
    // Bit 30 indicates whether last command had an error
    if (read_data&0x40000000) return false;
    data = static_cast<AmpIO_UInt16>(read_data);
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
    read_data = bswap_32(read_data);
    return static_cast<AmpIO_UInt16>(read_data>>16);
}
