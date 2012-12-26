/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <byteswap.h>
#include <iostream>

#include "AmpIO.h"
#include "FirewirePort.h"

const unsigned long VALID_BIT        = 0x80000000;  /*!< High bit of 32-bit word */
const unsigned long MIDRANGE_ADC     = 0x00008000;  /*!< Midrange value of ADC bits */
const unsigned long MIDRANGE_VEL     = 0x00008000;  /*!< Midrange value of encoder velocity */
const unsigned long MIDRANGE_FRQ     = 0x00008000;  /*!< Midrange value of encoder frequency */
const unsigned long MIDRANGE_ACC     = 0x00008000;  /*!< Midrange value of encoder acc */
const unsigned long ENC_PRELOAD      = 0x007fffff;  /*!< Encoder preload value */

const unsigned long ENABLE_MASK      = 0x0000ffff;  /*!< Mask for power enable bits */
const unsigned long MOTOR_CURR_MASK  = 0x0000ffff;  /*!< Mask for motor current adc bits */
const unsigned long ANALOG_POS_MASK  = 0xffff0000;  /*!< Mask for analog pot ADC bits */
const unsigned long ADC_MASK         = 0x0000ffff;  /*!< Mask for right aligned ADC bits */
const unsigned long DAC_MASK         = 0x0000ffff;  /*!< Mask for 16-bit DAC values */
const unsigned long ENC_POS_MASK     = 0x01ffffff;  /*!< Mask for quad encoder bits */
const unsigned long ENC_VEL_MASK     = 0x0000ffff;  /*!< Mask for encoder velocity bits */
const unsigned long ENC_FRQ_MASK     = 0x0000ffff;  /*!< Mask for encoder frequency bits */

const unsigned long DAC_WR_A         = 0x00300000;  /*!< Command to write DAC channel A */


AmpIO::AmpIO(unsigned char board_id, unsigned int numAxes) : BoardIO(board_id), NumAxes(numAxes)
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

void AmpIO::DisplayReadBuffer() const
{
    // first two quadlets are timestamp and status, resp.
    std::cout << std::hex << bswap_32(read_buffer[0]) << std::endl;
    std::cout << std::hex << bswap_32(read_buffer[1]) << std::endl;

    // remaining quadlets are in 4 groups of NUM_CHANNELS as follows:
    //   - motor current and analog pot per channel
    //   - encoder position per channel
    //   - encoder velocity per channel
    //   - encoder frequency per channel
    for (int i=2; i<ReadBufSize; i++) {
        std::cout << std::hex << bswap_32(read_buffer[i]) << " ";
        if (!((i-1)%NUM_CHANNELS)) std::cout << std::endl;
    }
}

unsigned long AmpIO::GetStatus() const
{
    return bswap_32(read_buffer[STATUS_OFFSET]);
}

unsigned long AmpIO::GetTimestamp() const
{
    return bswap_32(read_buffer[TIMESTAMP_OFFSET]);
}

unsigned long AmpIO::GetMotorCurrent(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+MOTOR_CURR_OFFSET]);
    buff &= MOTOR_CURR_MASK;       // mask for applicable bits

    return static_cast<unsigned long>(buff) & ADC_MASK;
}

unsigned long AmpIO::GetAnalogInput(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+ANALOG_POS_OFFSET]);
    buff &= ANALOG_POS_MASK;       // mask for applicable bits
    buff >>= 16;                   // shift to lsb alignment

    return static_cast<unsigned long>(buff) & ADC_MASK;
}

unsigned long AmpIO::GetEncoderPosition(unsigned int index) const
{
    if (index < NUM_CHANNELS)
        return bswap_32(read_buffer[index+ENC_POS_OFFSET]) & ENC_POS_MASK;
    else
        return 0;
}

// temp current the enc period velocity is unsigned 16 bits
// for low level function the + MIDRANGE_VEL
unsigned long AmpIO::GetEncoderVelocity(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+ENC_VEL_OFFSET]);
    buff &= ENC_VEL_MASK;          // mask for applicable bits
    buff += MIDRANGE_VEL;          // convert to unsigned by recentering

    return static_cast<unsigned long>(buff) & ENC_VEL_MASK;
}

unsigned long AmpIO::GetEncoderFrequency(unsigned int index) const
{
    if (index >= NUM_CHANNELS)
        return 0L;

    quadlet_t buff;
    buff = bswap_32(read_buffer[index+ENC_FRQ_OFFSET]);
    buff &= ENC_FRQ_MASK;          // mask for applicable bits
    buff += MIDRANGE_FRQ;          // convert to unsigned by recentering

    return static_cast<unsigned long>(buff) & ENC_FRQ_MASK;
}


/*******************************************************************************
 * Set commands
 */

bool AmpIO::SetPowerControl(unsigned long sdata)
{
    quadlet_t write_data = bswap_32(sdata & ENABLE_MASK);
    return (port ? port->WriteQuadlet(BoardId, 0, write_data) : false);
}

bool AmpIO::SetMotorCurrent(unsigned int index, unsigned long sdata)
{
    quadlet_t data = VALID_BIT | DAC_WR_A | (sdata & DAC_MASK);

    if (index < NUM_CHANNELS) {
        write_buffer[index+WB_CURR_OFFSET] = bswap_32(data);
        return true;
    }
    else
        return false;
}

bool AmpIO::SetEncoderPreload(unsigned int index, unsigned long sdata)
{
    unsigned int channel = (index+1) << 4;

    if (port && (index < NUM_CHANNELS))
        return port->WriteQuadlet(BoardId, channel | ENC_LOAD_OFFSET, bswap_32(sdata));
    else
        return false;
}

bool AmpIO::SetDigitalOutput(unsigned long bits)
{
    quadlet_t write_data = bswap_32(bits & 0x000F);
    return port->WriteQuadlet(BoardId, 6, write_data);
}

unsigned long AmpIO::GetDigitalOutput() const
{
    quadlet_t read_data = 0;
    if (port->ReadQuadlet(BoardId, 6, read_data))
        read_data = bswap_32(read_data) & 0x000F;
    return static_cast<unsigned long>(read_data);
}

unsigned long AmpIO::GetDigitalInput() const
{
    quadlet_t read_data = 0;
    if (port->ReadQuadlet(BoardId, 9, read_data))
        read_data = bswap_32(read_data) & 0x0FFF;
    return static_cast<unsigned long>(read_data);
}
