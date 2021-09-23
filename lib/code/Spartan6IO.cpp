/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides

  (C) Copyright 2011-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>
#include <sstream>

#include "Spartan6IO.h"
#include "BasePort.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"

const AmpIO_UInt32 REBOOT_FPGA      = 0x00300000;  /*!< Reboot FPGA (Rev 7+)       */
const AmpIO_UInt32 LED_ON           = 0x000c0000;  /*!< Turn LED on                */
const AmpIO_UInt32 LED_OFF          = 0x00080000;  /*!< Turn LED off               */
const AmpIO_UInt32 RESET_KSZ8851    = 0x04000000;  /*!< Mask to reset KSZ8851 Ethernet chip */

const double FPGA_sysclk_MHz        = 49.152;         /* FPGA sysclk in MHz (from FireWire) */

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


Spartan6IO::Spartan6IO(AmpIO_UInt8 board_id) : BoardIO(board_id), firmwareTime(0.0)
{
}

Spartan6IO::~Spartan6IO()
{
#if 0
    // Do not need this because derived class (e.g., AmpIO) also performs this check
    if (port) {
        std::cerr << "Warning: Spartan6IO being destroyed while still in use by "
                  << BasePort::PortTypeString(port->GetPortType()) <<" Port" << std::endl;
        port->RemoveBoard(this);
    }
#endif
}

std::string Spartan6IO::GetFPGASerialNumber(void)
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
        std::cerr << "Spartan6IO::GetFPGASerialNumber: failed to read FPGA Serial Number" << std::endl;
    return sn;
}

double Spartan6IO::GetFPGAClockPeriod(void) const
{
    return (1.0e-6/FPGA_sysclk_MHz);
}

bool Spartan6IO::HasEthernet(void) const
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 12, read_data))
        return false;
    // Bit 31 indicates whether Ethernet is present
    return (read_data&0x80000000);
}

/*******************************************************************************
 * Write commands
 */

bool Spartan6IO::WriteReboot(void)
{
    if (GetFirmwareVersion() < 7) {
        // Could instead allow this to be used with older firmware, where the "reboot"
        // flag was instead used as a "soft reset".
        std::cerr << "Spartan6IO::WriteReboot: requires firmware 7 or above" << std::endl;
        return false;
    }
    AmpIO_UInt32 write_data = REBOOT_FPGA;
    return (port ? port->WriteQuadlet(BoardId, BoardIO::BOARD_STATUS, write_data) : false);
}

// Note that with the QLA, this also enables board power
bool Spartan6IO::WriteLED(bool status)
{
    AmpIO_UInt32 write_data = status ? LED_ON : LED_OFF;
    return (port ? port->WriteQuadlet(BoardId, BoardIO::BOARD_STATUS, write_data) : false);
}

/*******************************************************************************
 * Static Write methods (for broadcast)
 *
 * These methods duplicate the board-specific methods (WriteReboot, ...),
 * but are sent to the broadcast address (FW_NODE_BROADCAST).
 */

bool Spartan6IO::WriteRebootAll(BasePort *port)
{
    // Note that Firmware V7+ supports the reboot command; earlier versions of
    // firmware will instead perform a limited reset.
    AmpIO_UInt32 write_data = REBOOT_FPGA;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, BoardIO::BOARD_STATUS, write_data) : false);
}

bool Spartan6IO::ResetKSZ8851All(BasePort *port) {
    quadlet_t write_data = RESET_KSZ8851;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, 12, write_data) : false);
}

/*******************************************************************************
 * PROM commands (M25P16)
 *    data e.g. 0x9f000000 the higher 8-bit is M25P16 cmd
 *    see DataSheet Table 5: Command Set Codes
 */

AmpIO_UInt32 Spartan6IO::PromGetId(void)
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

bool Spartan6IO::PromGetStatus(AmpIO_UInt32 &status, PromType type)
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

bool Spartan6IO::PromGetResult(AmpIO_UInt32 &result, PromType type)
{
    quadlet_t read_data;
    nodeaddr_t address = GetPromAddress(type, false);

    bool ret = port->ReadQuadlet(BoardId, address, read_data);
    if (ret)
        result = static_cast<AmpIO_UInt32>(read_data);
    return ret;
}

bool Spartan6IO::PromReadData(AmpIO_UInt32 addr, AmpIO_UInt8 *data,
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

bool Spartan6IO::PromWriteEnable(PromType type)
{
    quadlet_t write_data = 0x06000000;
    nodeaddr_t address = GetPromAddress(type, true);
    return port->WriteQuadlet(BoardId, address, write_data);
}

bool Spartan6IO::PromWriteDisable(PromType type)
{
    quadlet_t write_data = 0x04000000;
    nodeaddr_t address = GetPromAddress(type, true);
    return port->WriteQuadlet(BoardId, address, write_data);
}

bool Spartan6IO::PromSectorErase(AmpIO_UInt32 addr, const ProgressCallback cb)
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

int Spartan6IO::PromProgramPage(AmpIO_UInt32 addr, const AmpIO_UInt8 *bytes,
                           unsigned int nbytes, const ProgressCallback cb)
{
    const unsigned int MAX_PAGE = 256;  // 64 quadlets
    if (nbytes > MAX_PAGE) {
        std::ostringstream msg;
        msg << "Spartan6IO::PromProgramPage: error, nbytes = " << nbytes
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
        msg << "Spartan6IO::PromProgramPage: failed to write block, nbytes = " << nbytes;
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
        msg << "Spartan6IO::PromProgramPage: FPGA error = " << read_data;
        ERROR_CALLBACK(cb, msg);
    }
    // Now, read result. This should be the number of quadlets written.
    AmpIO_UInt32 nWritten;
    if (!PromGetResult(nWritten)) {
        std::ostringstream msg;
        msg << "Spartan6IO::PromProgramPage: could not get PROM result";
        ERROR_CALLBACK(cb, msg);
    }
    if (nWritten > 0)
        nWritten = 4*(nWritten-1);  // convert from quadlets to bytes
    if (nWritten != nbytes) {
        std::ostringstream msg;
        msg << "Spartan6IO::PromProgramPage: wrote " << nWritten << " of "
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
            msg << "Spartan6IO::PromProgramPage: could not get PROM status" << std::endl;
            ERROR_CALLBACK(cb, msg);
        }
        ret = PromGetStatus(status);
    }
    return nWritten;
}


nodeaddr_t Spartan6IO::GetPromAddress(PromType type, bool isWrite)
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
        std::cerr << "Spartan6IO::GetPromAddress: unsupported PROM type " << type << std::endl;

    return 0x00;
}


// ********************** HW PROM ONLY Methods ***********************************
bool Spartan6IO::PromReadByte25AA128(AmpIO_UInt16 addr, AmpIO_UInt8 &data)
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

bool Spartan6IO::PromWriteByte25AA128(AmpIO_UInt16 addr, const AmpIO_UInt8 &data)
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
bool Spartan6IO::PromReadBlock25AA128(AmpIO_UInt16 addr, quadlet_t *data, unsigned int nquads)
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
bool Spartan6IO::PromWriteBlock25AA128(AmpIO_UInt16 addr, quadlet_t *data, unsigned int nquads)
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

// ********************** KSZ8851 Ethernet Controller Methods ***********************************

bool Spartan6IO::ResetKSZ8851()
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = RESET_KSZ8851;
    return port->WriteQuadlet(BoardId, 12, write_data);
}

bool Spartan6IO::WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt8 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x02000000 | (static_cast<quadlet_t>(addr) << 16) | data;
    return port->WriteQuadlet(BoardId, 12, write_data);
}

bool Spartan6IO::WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt16 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x03000000 | (static_cast<quadlet_t>(addr) << 16) | data;
    return port->WriteQuadlet(BoardId, 12, write_data);
}

bool Spartan6IO::ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt8 &rdata)
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

bool Spartan6IO::ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt16 &rdata)
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

bool Spartan6IO::WriteKSZ8851DMA(const AmpIO_UInt16 &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = 0x0B000000 | data;
    return port->WriteQuadlet(BoardId, 12, write_data);
}

bool Spartan6IO::ReadKSZ8851DMA(AmpIO_UInt16 &rdata)
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

AmpIO_UInt16 Spartan6IO::ReadKSZ8851ChipID()
{
    AmpIO_UInt16 data;
    if (ReadKSZ8851Reg(0xC0, data))
        return data;
    else
        return 0;
}

AmpIO_UInt16 Spartan6IO::ReadKSZ8851Status()
{
    if (GetFirmwareVersion() < 5) return 0;
    quadlet_t read_data;
    if (!port->ReadQuadlet(BoardId, 12, read_data))
        return 0;
    return static_cast<AmpIO_UInt16>(read_data>>16);
}

bool Spartan6IO::ReadEthernetData(quadlet_t *buffer, unsigned int offset, unsigned int nquads)
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
bool Spartan6IO::ReadFirewireData(quadlet_t *buffer, unsigned int offset, unsigned int nquads)
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
