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
    if (port->ReadQuadlet(BoardId, 10, read_data))
        read_data = bswap_32(read_data) & 0x0FFF;
    return static_cast<unsigned long>(read_data);
}

unsigned long AmpIO::PromGetId()
{
    unsigned long id = 0;
    quadlet_t data = 0x9f000000;
    if (port->WriteQuadlet(BoardId, 8, bswap_32(data))) {
        // Should be ready by now...
        id = PromGetResult();
    }
    return id;
}

unsigned long AmpIO::PromGetStatus()
{
    unsigned long status = 0x80000000;
    quadlet_t data = 0x05000000;
    if (port->WriteQuadlet(BoardId, 8, bswap_32(data))) {
        // Should be ready by now...
        status = PromGetResult();
    }
    return status;
}

unsigned long AmpIO::PromGetResult()
{
    unsigned long result = 0xffffffff;
    quadlet_t data;
    if (port->ReadQuadlet(BoardId, 9, data))
        result = static_cast<unsigned long>(bswap_32(data));
    return result;
}

bool AmpIO::PromReadData(unsigned long addr, unsigned char *data,
                         unsigned int nbytes)
{
    unsigned long addr24 = addr&0x00ffffff;
    if (addr24+nbytes > 0x00ffffff)
        return false;
    quadlet_t *qptr = (quadlet_t *)data;
    quadlet_t write_data = 0x03000000|addr24;
    int i;
    for (i = 0; i < nbytes/4; i++, write_data += 4) {
        if (!port->WriteQuadlet(BoardId, 8, bswap_32(write_data)))
            return false;
        // Should be ready by now...
        if (!port->ReadQuadlet(BoardId, 9, qptr[i]))
            return false;
    }
    // Get any left-over bytes
    int extra = nbytes%4;
    if (extra) {
        if (!port->WriteQuadlet(BoardId, 8, write_data))
            return false;
        union {
            quadlet_t q;
            unsigned char b[4];
        } read_data;
        if (!port->ReadQuadlet(BoardId, 9, read_data.q))
            return false;
        for (i = 0; i < extra; i++)
            data[nbytes-extra+i] = read_data.b[i];
    }
    return true;
}

bool AmpIO::PromWriteEnable()
{
    quadlet_t write_data = 0x06000000;
    return port->WriteQuadlet(BoardId, 8, bswap_32(write_data));
}

bool AmpIO::PromWriteDisable()
{
    quadlet_t write_data = 0x04000000;
    return port->WriteQuadlet(BoardId, 8, bswap_32(write_data));
}

bool AmpIO::PromSectorErase(unsigned long addr)
{
    std::cout << "Erasing sector " << std::hex << addr << std::dec;
    PromWriteEnable();
    quadlet_t write_data = 0xd8000000 | (addr&0x00ffffff);
    if (!port->WriteQuadlet(BoardId, 8, bswap_32(write_data)))
        return false;
    // Wait for erase to finish
    unsigned char status;
    while (PromGetStatus())
        std::cout << ".";
    std::cout << std::endl;
    return true;
}

bool AmpIO::PromProgramPage(unsigned long addr, const unsigned char *bytes,
                            unsigned int nbytes)
{
    const unsigned int MAX_PAGE = 256;
    if (nbytes > MAX_PAGE) {
        std::cout << "PromProgramPage: error, nbytes = " << nbytes
                  << " (max = " << MAX_PAGE << ")" << std::endl;
        return false;
    }
    std::cout << "Programming page " << std::hex << addr << std::dec;
    PromWriteEnable();
    // Block write of the data
    quadlet_t write_data = 0x02000000 | (addr & 0x00ffffff);
    unsigned char page_data[MAX_PAGE+sizeof(quadlet_t)];
    quadlet_t *data_ptr = reinterpret_cast<quadlet_t *>(page_data);
    data_ptr[0] = bswap_32(write_data);   // Page program
    memcpy(page_data+sizeof(quadlet_t), bytes, nbytes);
    if (!port->WriteBlock(BoardId, 0xc0, data_ptr, nbytes+sizeof(quadlet_t)))
        return false;
    // Read prom status register; if 4 LSB are 0, command has finished
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 8, read_data)) return false;
    int cnt = 0;
    while (read_data&0x000f) {
        std::cout << ".";
        if (!port->ReadQuadlet(BoardId, 8, read_data)) return false;
        if (cnt++ > 100) {
            std::cout << "timeout" << std::endl;
            break;
        }
    }
    // Now, read result. This should be the number of quadlets written.
    std::cout << ", result = " << std::hex << PromGetResult() << std::dec << std::endl;
    // Wait for "Write in Progress" bit to be cleared
    while (PromGetStatus()&MASK_WIP)
        std::cout << ".";
    return true;
}
