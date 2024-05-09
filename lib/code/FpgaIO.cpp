/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides

  (C) Copyright 2011-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>
#include <sstream>

#include "FpgaIO.h"
#include "BasePort.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"

const uint32_t REBOOT_FPGA      = 0x00300000;  /*!< Reboot FPGA (Rev 7+)       */
const uint32_t LED_ON           = 0x000c0000;  /*!< Turn LED on                */
const uint32_t LED_OFF          = 0x00080000;  /*!< Turn LED off               */

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


FpgaIO::FpgaIO(uint8_t board_id) : BoardIO(board_id), firmwareTime(0.0)
{
}

FpgaIO::~FpgaIO()
{
#if 0
    // Do not need this because derived class (e.g., AmpIO) also performs this check
    if (port) {
        std::cerr << "Warning: FpgaIO being destroyed while still in use by "
                  << BasePort::PortTypeString(port->GetPortType()) <<" Port" << std::endl;
        port->RemoveBoard(this);
    }
#endif
}

std::string FpgaIO::GetFPGASerialNumber(void)
{
    // Format: FPGA 1234-56 (12 bytes) or FPGA 1234-567 (13 bytes).
    // Note that on PROM, the string is terminated by 0xff because the sector
    // is first erased (all bytes set to 0xff) before the string is written.
    uint32_t address = 0x001FFF00;
    char data[20];
    std::string sn;
    const size_t FPGASNSize = 13;
    const size_t bytesToRead = (FPGASNSize+3)&0xFC;  // must be multiple of 4

    data[FPGASNSize] = 0;    // Make sure null-terminated
    if (PromReadData(address, (uint8_t *)data, bytesToRead)) {
        if (strncmp(data, "FPGA ", 5) == 0) {
            char *p = strchr(data+5, 0xff);
            if (p) *p = 0;      // Null terminate at first 0xff
            sn.assign(data+5);
        }
    }
    else
        std::cerr << "FpgaIO::GetFPGASerialNumber: failed to read FPGA Serial Number" << std::endl;
    return sn;
}

double FpgaIO::GetFPGAClockPeriod(void) const
{
    return (1.0e-6/FPGA_sysclk_MHz);
}

bool FpgaIO::HasEthernet(void) const
{
    return (GetFpgaVersionMajor() > 1);
}

/*******************************************************************************
 * Write commands
 */

bool FpgaIO::WriteReboot(void)
{
    if (GetFirmwareVersion() < 7) {
        // Could instead allow this to be used with older firmware, where the "reboot"
        // flag was instead used as a "soft reset".
        std::cerr << "FpgaIO::WriteReboot: requires firmware 7 or above" << std::endl;
        return false;
    }
    uint32_t write_data = REBOOT_FPGA;
    return (port ? port->WriteQuadlet(BoardId, BoardIO::BOARD_STATUS, write_data) : false);
}

// Note that with the QLA, this also enables board power
bool FpgaIO::WriteLED(bool status)
{
    uint32_t write_data = status ? LED_ON : LED_OFF;
    return (port ? port->WriteQuadlet(BoardId, BoardIO::BOARD_STATUS, write_data) : false);
}

/*******************************************************************************
 * Static Write methods (for broadcast)
 *
 * These methods duplicate the board-specific methods (WriteReboot, ...),
 * but are sent to the broadcast address (FW_NODE_BROADCAST).
 */

bool FpgaIO::WriteRebootAll(BasePort *port)
{
    // Note that Firmware V7+ supports the reboot command; earlier versions of
    // firmware will instead perform a limited reset.
    uint32_t write_data = REBOOT_FPGA;
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, BoardIO::BOARD_STATUS, write_data) : false);
}

bool FpgaIO::ResetEthernetAll(BasePort *port) {
    // For FPGA V2, it is enough to specify ETH_CTRL_RESET_PHY
    // For FPGA V3, we specify two additional bits to indicate which PHYs are being reset
    quadlet_t write_data = ETH_CTRL_RESET_PHY | ETH_PORT_CTRL_RESET_PHY | (ETH_PORT_CTRL_RESET_PHY << 8);
    return (port ? port->WriteQuadlet(FW_NODE_BROADCAST, BoardIO::ETH_STATUS, write_data) : false);
}

/*******************************************************************************
 * PROM commands (M25P16)
 *    data e.g. 0x9f000000 the higher 8-bit is M25P16 cmd
 *    see DataSheet Table 5: Command Set Codes
 */

uint32_t FpgaIO::PromGetId(void)
{
    uint32_t id = 0;
    quadlet_t data = 0x9f000000;
    if (port->WriteQuadlet(BoardId, 0x08, data)) {
        port->PromDelay();
        // Should be ready by now...
        PromGetResult(id);
    }
    return id;
}

bool FpgaIO::PromGetStatus(uint32_t &status, PromType type)
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

bool FpgaIO::PromGetResult(uint32_t &result, PromType type)
{
    quadlet_t read_data;
    nodeaddr_t address = GetPromAddress(type, false);

    bool ret = port->ReadQuadlet(BoardId, address, read_data);
    if (ret)
        result = static_cast<uint32_t>(read_data);
    return ret;
}

bool FpgaIO::PromReadData(uint32_t addr, uint8_t *data,
                         unsigned int nbytes)
{
    uint32_t addr24 = addr&0x00ffffff;
    if (addr24+nbytes > 0x00ffffff)
        return false;
    quadlet_t write_data = 0x03000000|addr24;  // 03h = Read Data Bytes
    uint32_t page = 0;
    while (page < nbytes) {
        const unsigned int maxReadSize = 64u;
        unsigned int bytesToRead = ((nbytes-page)<maxReadSize) ? (nbytes-page) : maxReadSize;
        if (!port->WriteQuadlet(BoardId, 0x08, write_data))
            return false;
        // Read FPGA status register; if 3 LSB are 0, command has finished.
        // The IEEE-1394 clock is 24.576 MHz, so it should take
        // about 256*8*(1/24.576) = 83.3 microseconds to read 256 bytes.
        // Experimentally, 1 iteration of the loop below is sufficient
        // most of the time, with 2 iterations required occasionally.
        quadlet_t read_data = 0x0007;
        int i;
        const int MAX_LOOP_CNT = 8;
        for (i = 0; (i < MAX_LOOP_CNT) && read_data; i++) {
            Amp1394_Sleep(0.00001);   // 10 usec
            if (!port->ReadQuadlet(BoardId, 0x08, read_data)) return false;
            read_data = read_data&0x0007;
        }
        if (i == MAX_LOOP_CNT) {
            std::cout << "PromReadData: command failed to finish, status = "
                      << std::hex << read_data << std::dec << std::endl;
            return false;
        }
        // Now, read result. This should be the number of quadlets read.
        uint32_t nRead;
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

        uint32_t fver = GetFirmwareVersion();
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

bool FpgaIO::PromWriteEnable(PromType type)
{
    quadlet_t write_data = 0x06000000;
    nodeaddr_t address = GetPromAddress(type, true);
    return port->WriteQuadlet(BoardId, address, write_data);
}

bool FpgaIO::PromWriteDisable(PromType type)
{
    quadlet_t write_data = 0x04000000;
    nodeaddr_t address = GetPromAddress(type, true);
    return port->WriteQuadlet(BoardId, address, write_data);
}

bool FpgaIO::PromSectorErase(uint32_t addr, const ProgressCallback cb)
{
    PromWriteEnable();
    quadlet_t write_data = 0xd8000000 | (addr&0x00ffffff);
    if (!port->WriteQuadlet(BoardId, 0x08, write_data))
        return false;
    // Wait for erase to finish
    uint32_t status;
    if (!PromGetStatus(status))
        return false;
    while (status) {
        PROGRESS_CALLBACK(cb, false);
        if (!PromGetStatus(status))
            return false;
    }
    return true;
}

int FpgaIO::PromProgramPage(uint32_t addr, const uint8_t *bytes,
                           unsigned int nbytes, const ProgressCallback cb)
{
    const unsigned int MAX_PAGE = 256;  // 64 quadlets
    if (nbytes > MAX_PAGE) {
        std::ostringstream msg;
        msg << "FpgaIO::PromProgramPage: error, nbytes = " << nbytes
            << " (max = " << MAX_PAGE << ")";
        ERROR_CALLBACK(cb, msg);
        return -1;
    }
    PromWriteEnable();
    // Block write of the data (+1 quad for command)
    uint8_t page_data[MAX_PAGE+sizeof(quadlet_t)];
    quadlet_t *data_ptr = reinterpret_cast<quadlet_t *>(page_data);
    // First quadlet is the "page program" instruction (0x02)
    data_ptr[0] = bswap_32(0x02000000 | (addr & 0x00ffffff));
    // Remaining quadlets are the data to be programmed. These do not
    // need to be byte-swapped.
    memcpy(page_data+sizeof(quadlet_t), bytes, nbytes);

    // select address based on firmware version number
    uint32_t fver = GetFirmwareVersion();
    nodeaddr_t address;
    if (fver >= 4) {address = 0x2000;}
    else {address = 0xc0;}
    if (!port->WriteBlock(BoardId, address, data_ptr, nbytes+sizeof(quadlet_t))) {
        std::ostringstream msg;
        msg << "FpgaIO::PromProgramPage: failed to write block, nbytes = " << nbytes;
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
        msg << "FpgaIO::PromProgramPage: FPGA error = " << read_data;
        ERROR_CALLBACK(cb, msg);
    }
    // Now, read result. This should be the number of quadlets written.
    uint32_t nWritten;
    if (!PromGetResult(nWritten)) {
        std::ostringstream msg;
        msg << "FpgaIO::PromProgramPage: could not get PROM result";
        ERROR_CALLBACK(cb, msg);
    }
    if (nWritten > 0)
        nWritten = 4*(nWritten-1);  // convert from quadlets to bytes
    if (nWritten != nbytes) {
        std::ostringstream msg;
        msg << "FpgaIO::PromProgramPage: wrote " << nWritten << " of "
            << nbytes << " bytes";
        ERROR_CALLBACK(cb, msg);
    }
    // Wait for "Write in Progress" bit to be cleared
    uint32_t status;
    bool ret = PromGetStatus(status);
    while (status&MASK_WIP) {
        if (ret) {
            PROGRESS_CALLBACK(cb, 0);
        }
        else {
            std::ostringstream msg;
            msg << "FpgaIO::PromProgramPage: could not get PROM status" << std::endl;
            ERROR_CALLBACK(cb, msg);
        }
        ret = PromGetStatus(status);
    }
    return nWritten;
}


nodeaddr_t FpgaIO::GetPromAddress(PromType type, bool isWrite)
{
    if (type == PROM_M25P16 && isWrite)
        return 0x0008;
    else if (type == PROM_M25P16 && !isWrite)
        return 0x0009;
    else if (type == PROM_25AA128 && isWrite)
        return 0x3000;
    else if (type == PROM_25AA128 && !isWrite)
        return 0x3002;
    else if (type == PROM_25AA128_1 && isWrite)
        return 0x3010;
    else if (type == PROM_25AA128_1 && !isWrite)
        return 0x3012;
    else if (type == PROM_25AA128_2 && isWrite)
        return 0x3020;
    else if (type == PROM_25AA128_2 && !isWrite)
        return 0x3022;
    else
        std::cerr << "FpgaIO::GetPromAddress: unsupported PROM type " << type << std::endl;

    return 0x00;
}


// ********************** HW PROM ONLY Methods ***********************************
bool FpgaIO::PromReadByte25AA128(uint16_t addr, uint8_t &data, unsigned char chan)
{
    PromType prom_type = (chan == 1) ? PROM_25AA128_1 : (chan == 2) ? PROM_25AA128_2 : PROM_25AA128;

    // 8-bit cmd + 16-bit addr (2 MSBs ignored)
    uint32_t result = 0x00000000;
    quadlet_t write_data = 0x03000000|(addr << 8);
    nodeaddr_t address = GetPromAddress(prom_type, true);
    if (port->WriteQuadlet(BoardId, address, write_data)) {
        port->PromDelay();
        // Should be ready by now...
        if (!PromGetResult(result, prom_type))
            return false;
        // Get the last 8-bit of result
        data = result & 0xFF;
        return true;
    } else {
        data = 0x00;
        return false;
    }
}

bool FpgaIO::PromWriteByte25AA128(uint16_t addr, const uint8_t &data, unsigned char chan)
{
    PromType prom_type = (chan == 1) ? PROM_25AA128_1 : (chan == 2) ? PROM_25AA128_2 : PROM_25AA128;

    // enable write
    PromWriteEnable(prom_type);
    Amp1394_Sleep(0.0001);   // 100 usec

    // 8-bit cmd + 16-bit addr + 8-bit data
    quadlet_t write_data = 0x02000000|(addr << 8)|data;
    nodeaddr_t address = GetPromAddress(prom_type, true);
    if (port->WriteQuadlet(BoardId, address, write_data)) {
        // wait 5ms for the PROM to be ready to take new commands
        Amp1394_Sleep(0.005);
        return true;
    }
    else
        return false;
}


// Read block data (quadlet)
bool FpgaIO::PromReadBlock25AA128(uint16_t addr, quadlet_t *data, unsigned int nquads, unsigned char chan)
{
    // nquads sanity check
    if (nquads == 0 || nquads > 16) {
        std::cout << "invalid number of quadlets" << std::endl;
        return false;
    }

    PromType prom_type = (chan == 1) ? PROM_25AA128_1 : (chan == 2) ? PROM_25AA128_2 : PROM_25AA128;

    // trigger read
    quadlet_t write_data = 0xFE000000|(addr << 8)|(nquads-1);
    nodeaddr_t address = GetPromAddress(prom_type, true);
    if (!port->WriteQuadlet(BoardId, address, write_data))
        return false;

    // get result
    if (!port->ReadBlock(BoardId, (address|0x0100), data, nquads * 4))
        return false;
    else
        return true;
}


// Write block data (quadlet)
bool FpgaIO::PromWriteBlock25AA128(uint16_t addr, quadlet_t *data, unsigned int nquads,
                                   unsigned char chan)
{
    // address sanity check
    if (nquads == 0 || nquads > 16) {
        std::cout << "invalid number of quadlets" << std::endl;
        return false;
    }

    PromType prom_type = (chan == 1) ? PROM_25AA128_1 : (chan == 2) ? PROM_25AA128_2 : PROM_25AA128;
    nodeaddr_t address = GetPromAddress(prom_type, true);

    // block write data to buffer
    if (!port->WriteBlock(BoardId, (address|0x0100), data, nquads*sizeof(quadlet_t)))
        return false;

    // enable write
    PromWriteEnable(prom_type);

    // trigger write
    quadlet_t write_data = 0xFF000000|(addr << 8)|(nquads-1);
    return port->WriteQuadlet(BoardId, address, write_data);
}

// ********************** KSZ8851 Ethernet Controller Methods ***********************************

bool FpgaIO::WriteKSZ8851Reg(uint8_t addr, const uint8_t &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = ETH_CTRL_WR_V2 | (static_cast<quadlet_t>(addr) << 16) | data;
    return WriteEthernetControl(write_data);
}

bool FpgaIO::WriteKSZ8851Reg(uint8_t addr, const uint16_t &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = ETH_CTRL_WR_V2 | ETH_CTRL_WB_V2 | (static_cast<quadlet_t>(addr) << 16) | data;
    return WriteEthernetControl(write_data);
}

bool FpgaIO::ReadKSZ8851Reg(uint8_t addr, uint8_t &rdata)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = (static_cast<quadlet_t>(addr) << 16) | rdata;
    if (!WriteEthernetControl(write_data))
        return false;
    quadlet_t read_data;
    if (!ReadEthernetStatus(read_data))
        return false;
    // Check if Ethernet is present
    if (!(read_data&ETH_STAT_PRESENT_V2)) return false;
    // Check if last command had an error
    if (read_data&ETH_STAT_REQ_ERR_V2) return false;
    rdata = static_cast<uint8_t>(read_data);
    return true;
}

bool FpgaIO::ReadKSZ8851Reg(uint8_t addr, uint16_t &rdata)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = ETH_CTRL_WB_V2 | (static_cast<quadlet_t>(addr) << 16) | rdata;
    if (!WriteEthernetControl(write_data)) {
        std::cout << "WriteQuadlet failed" << std::endl;
        return false;
    }
    quadlet_t read_data;
    if (!ReadEthernetStatus(read_data))
        return false;
    // Check if Ethernet is present
    if (!(read_data&ETH_STAT_PRESENT_V2)) return false;
    // Check if last command had an error
    if (read_data&ETH_STAT_REQ_ERR_V2) return false;
    rdata = static_cast<uint16_t>(read_data);
    return true;
}

// DMA access (no address specified)
// This assumes that the chip has already been placed in DMA mode
// (e.g., by writing to register 0x82).

bool FpgaIO::WriteKSZ8851DMA(const uint16_t &data)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = ETH_CTRL_DMA_V2 | ETH_CTRL_WR_V2 | ETH_CTRL_WB_V2 | data;
    return WriteEthernetControl(write_data);
}

bool FpgaIO::ReadKSZ8851DMA(uint16_t &rdata)
{
    if (GetFirmwareVersion() < 5) return false;
    quadlet_t write_data = ETH_CTRL_DMA_V2 | ETH_CTRL_WB_V2 | rdata;
    if (!WriteEthernetControl(write_data))
        return false;
    quadlet_t read_data;
    if (!ReadEthernetStatus(read_data))
        return false;
    // Check if Ethernet is present
    if (!(read_data&ETH_STAT_PRESENT_V2)) return false;
    // Check if last command had an error
    if (read_data&ETH_STAT_REQ_ERR_V2) return false;
    rdata = static_cast<uint16_t>(read_data);
    return true;
}

uint16_t FpgaIO::ReadKSZ8851ChipID()
{
    uint16_t data;
    if (ReadKSZ8851Reg(0xC0, data))
        return data;
    else
        return 0;
}

uint16_t FpgaIO::ReadKSZ8851Status()
{
    uint16_t status = 0;
    quadlet_t status32;
    if (ReadEthernetStatus(status32))
        status = static_cast<uint16_t>(status32>>16);
    return status;
}

// ************************** RTL8211F Ethernet PHY Methods *************************************

// Wait for FPGA mdio interface to return to idle state
bool FpgaIO::WaitRTL8211F_Idle(const char *callerName, unsigned int chan, unsigned int loop_cnt)
{
    // Should have firmware/hardware checks
    // First, wait 1 millisecond
    Amp1394_Sleep(0.001);
    // Now, check up to loop_cnt iterations
    nodeaddr_t address = 0x40a0 | (chan << 8);
    quadlet_t read_data;
    bool isIdle = false;
    unsigned int i = 0;
    while (!isIdle && (i++ < loop_cnt)) {
        if (port->ReadQuadlet(BoardId, address, read_data))
            isIdle = ((read_data&0x07000000) == 0);
    }
    if (!isIdle) {
        std::cout << callerName << " timeout waiting for idle, cnt " << i << ", read " << std::hex
                  << read_data << std::dec << std::endl;
    }
    return isIdle;
}

bool FpgaIO::ReadRTL8211F_Register(unsigned int chan, unsigned int phyAddr, unsigned int regAddr, uint16_t &data)
{
    // Should have firmware/hardware checks
    // Wait for FPGA mdio interface to be idle
    if (!WaitRTL8211F_Idle("ReadRTL8211F_Register", chan, 20))
        return false;
    nodeaddr_t address = 0x40a0 | (chan << 8);
    // Format: 0110 PPPP PRRR RRXX X(16), where P indicates phyAddr, R indicates regAddr, X is don't care (0)
    uint32_t write_data = 0x60000000 | ((phyAddr&0x1f) << 23) | ((regAddr&0x1f) << 18);
    if (!port->WriteQuadlet(BoardId, address, write_data))
        return false;
    // Read data is not valid unless FPGA mdio interface is in idle state. For most interfaces (e.g., Firewire,
    // Ethernet), this will already be the case, but the Zynq EMIO mmap interface is fast so some waiting is needed.
    // First, wait 1 millisecond
    Amp1394_Sleep(0.001);
    // Now, loop up to 20 times until mdio interface is idle and register address matches
    quadlet_t read_data;
    unsigned int regAddrRead;
    bool isIdle = false;
    bool regAddrMatch = false;
    unsigned int i = 0;
    while (!(isIdle && regAddrMatch) && (i++ < 20)) {
        if (port->ReadQuadlet(BoardId, address, read_data)) {
            isIdle = ((read_data&0x07000000) == 0);
            if (isIdle) {
                regAddrRead = (read_data & 0x001f0000)>>16;
                regAddrMatch = (regAddrRead == regAddr);
            }
        }
    }
    if (!isIdle) {
        std::cout << "ReadRTL8211F_Register timeout waiting for data, read " << std::hex
                  << read_data << std::dec << std::endl;
    }
    else if (!regAddrMatch) {
        std::cout << "ReadRTL8211F_Register regAddr mismatch, expected " << std::hex
                  << regAddr << ", received " << regAddrRead << std::dec << std::endl;
    }
    // Set data even if error, since it could be correct and caller may not check return value
    data = static_cast<uint16_t>(read_data & 0x0000ffff);
    return (isIdle & regAddrMatch);
}

bool FpgaIO::WriteRTL8211F_Register(unsigned int chan, unsigned int phyAddr, unsigned int regAddr, uint16_t data)
{
    // Should have firmware/hardware checks
    // Wait for FPGA mdio interface to be idle
    if (!WaitRTL8211F_Idle("WriteRTL8211F_Register", chan, 20))
        return false;
    nodeaddr_t address = 0x40a0 | (chan << 8);
    // Format: 0101 PPPP PRRR RR10 D(16), where P indicates phyAddr, R indicates regAddr, D indicates data
    uint32_t write_data = 0x50020000 | ((phyAddr&0x1f) << 23) | ((regAddr&0x1f) << 18) | data;
    return port->WriteQuadlet(BoardId, address, write_data);
}

// ***************************** Methods shared by V2/V3 ****************************************

bool FpgaIO::ReadEthernetStatus(uint32_t &status)
{
    status = 0;
    if (GetFirmwareVersion() < 5) return false;
    return port->ReadQuadlet(BoardId, BoardIO::ETH_STATUS, status);
}

uint8_t FpgaIO::GetEthernetPortStatusV3(uint32_t status, unsigned int chan)
{
    uint8_t pstat = 0;
    if (GetFpgaVersionMajorFromStatus(status) == 3) {
        if (chan == 1)
            pstat = static_cast<uint8_t>(status&ETH_STAT_PORT1_MASK_V3);
        else if (chan == 2)
            pstat = static_cast<uint8_t>((status&ETH_STAT_PORT2_MASK_V3)>>8);
    }
    return pstat;
}

bool FpgaIO::WriteEthernetControl(uint32_t write_data)
{
    if (GetFirmwareVersion() < 5) return false;
    return port->WriteQuadlet(BoardId, BoardIO::ETH_STATUS, write_data);
}

bool FpgaIO::WriteEthernetPhyReset(unsigned int chan)
{
    quadlet_t write_data = ETH_CTRL_RESET_PHY;
    if (chan&1)
        write_data |= ETH_PORT_CTRL_RESET_PHY;
    if (chan&2)
        write_data |= (ETH_PORT_CTRL_RESET_PHY<<8);
    return WriteEthernetControl(write_data);
}

bool FpgaIO::WriteEthernetClearErrors()
{
    quadlet_t write_data = ETH_CTRL_CLR_ERR_NET;
    unsigned int fpga_ver = GetFpgaVersionMajor();
    if (fpga_ver== 2) {
        write_data |= ETH_CTRL_CLR_ERR_LINK_V2;
    }
    else if ((fpga_ver == 3) && (GetFirmwareVersion() > 8)) {
        write_data |= ETH_CTRL_CLR_ERR_SW_V3;
    }
    return WriteEthernetControl(write_data);
}

bool FpgaIO::ReadEthernetData(quadlet_t *buffer, unsigned int offset, unsigned int nquads)
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
bool FpgaIO::ReadFirewireData(quadlet_t *buffer, unsigned int offset, unsigned int nquads)
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

bool FpgaIO::ReadFirewirePhy(unsigned char addr, unsigned char &data)
{
    if (addr > 15) {
        std::cout << "ReadFirewirePhy: invalid addr "
                  << static_cast<unsigned int>(addr) << std::endl;
        return false;
    }
    quadlet_t write_data = addr;
    quadlet_t read_data = 0;
    if (!port->WriteQuadlet(BoardId, BoardIO::FW_PHY_REQ, write_data))
        return false;
    if (!port->ReadQuadlet(BoardId, BoardIO::FW_PHY_RESP, read_data))
        return false;
    // Register address in read response is bits 11-8
    unsigned char read_addr = static_cast<unsigned char>(read_data>>8) & 0x0f;
    // Check that it matches the request address
    if (read_addr != addr) {
        std::cout << "ReadFirewirePhy: read addr " << static_cast<unsigned int>(read_addr)
                  << ", expected " << static_cast<unsigned int>(addr) << std::endl;
        return false;
    }
    data = static_cast<unsigned char>(read_data);
    return true;
}

bool FpgaIO::WriteFirewirePhy(unsigned char addr, unsigned char data)
{
    if (addr > 15) {
        std::cout << "WriteFirewirePhy: invalid addr "
                  << static_cast<unsigned int>(addr) << std::endl;
        return false;
    }
    // Set bit 12 to indicate write; addr in bits 11-8, data in bits 7-0
    quadlet_t write_data = 0x00001000 | static_cast<quadlet_t>(addr) << 8 | static_cast<quadlet_t>(data);
    return port->WriteQuadlet(BoardId, BoardIO::FW_PHY_REQ, write_data);
}
