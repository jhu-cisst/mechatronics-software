/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Jie Ying Wu

  (C) Copyright 2011-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __Fpga_IO_H__
#define __Fpga_IO_H__

#include <Amp1394/AmpIORevision.h>

#include "BoardIO.h"

// Methods specific to FPGA board (Rev 1.x, Rev 2.x or Rev 3.x).
// Rev 1.x and 2.x use a Spartan6 FPGA, whereas Rev 3.x uses a Zynq FPGA.
//
// Supports:
//   - Read/Write FPGA PROM (M25P16) to reprogram firmware or FPGA serial number (Rev 1.x, 2.x)
//   - Read/Write external PROM (25AA128); although this PROM is not on the FPGA board,
//     it is suggested that all companion boards (e.g., QLA) include a 25AA128
//     (or compatible) PROM connected to pins IO1-1 through IO1-4.
//   - Reboot FPGA via software command (Firmware V7+)
//   - Turn on/off LED on FPGA board; note that for some hardware (e.g., QLA),
//     this would also enable board power
//   - Query whether FPGA board has Ethernet interface
//   - Query FPGA clock period
//   - Read Ethernet/Firewire buffers from FPGA board (for debugging)
//   - Read/Write Firewire PHY register
//
// This class also contains a firmwareTime member (and accessor methods), which is
// updated by the derived class to keep a running timer based on the FPGA clock.

class FpgaIO : public BoardIO
{
public:

    FpgaIO(uint8_t board_id);
    ~FpgaIO();

    // Return FPGA serial number (empty string if not found)
    // This method actually performs a read (should have been called ReadFPGASerialNumber)
    std::string GetFPGASerialNumber(void);

    // Returns FPGA clock period in seconds
    double GetFPGAClockPeriod(void) const;

    // Returns true if FPGA has Ethernet (Rev 2.0+)
    bool HasEthernet(void) const;

    // Get elapsed time, in seconds, based on FPGA clock. This is computed by accumulating
    // the timestamp values in the real-time read packet; thus, it is only accurate when
    // there are periodic calls to ReadAllBoards or ReadAllBoardsBroadcast.
    double GetFirmwareTime(void) const { return firmwareTime; }

    // Set firmware time
    void SetFirmwareTime(double newTime = 0.0) { firmwareTime = newTime; }

    // ********************** WRITE Methods **********************************

    // Reboot FPGA
    bool WriteReboot(void);

    // Turn FPGA LED on/off (for QLA, same bit as pwr_enable)
    bool WriteLED(bool status);

    // **************** Static WRITE Methods (for broadcast) ********************

    static bool WriteRebootAll(BasePort *port);

    static bool ResetEthernetAll(BasePort *port);

    // ********************** PROM Methods ***********************************
    // Methods for reading or programming
    //   1 - the FPGA configuration PROM (M25P16)
    //   2 - Hardware (QLA) PROM (25AA128)
    //       25AA128_1 and 25AA128_2 are used to support multiple PROMs (DQLA)
    // NOTE:
    //   - M25P16 & 25AA128 Have exact same command table, except M25P16 has
    //     a few extra commands (e.g. ReadID, DeepSleep)
    //   - address length
    //     - M25P16: 24-bit
    //     - 25AA128: 16-bit (2 MSB are ignored)
    //     - 25AA128_1: same as 25AA128
    //     - 25AA128_2: same as 25AA128
    enum PromType{
        PROM_M25P16 = 1,
        PROM_25AA128 = 2,
        PROM_25AA128_1 = 3,
        PROM_25AA128_2 = 4
    };

    /*!
     \brief Get PROM read/write address (1394 space) based on prom type

     \param type   PROM type e.g. M25P16/25AA128
     \param isWrite  write address or read address
     \return nodeaddr_t  return address in FPGA 1394 address space
    */
    nodeaddr_t GetPromAddress(PromType type = PROM_M25P16, bool isWrite = true);

    // User-supplied callback, called when software needs to wait for an
    // action, or when an error occurs. In the latter case, the error message
    // is passed as a parameter (NULL if no error). If the callback returns
    // false, the current wait operation is aborted.
    typedef bool (*ProgressCallback)(const char *msg);

    // Returns the PROM ID (M25P16 ONLY)
    // should be 0x00202015 for the M25P16
    // returns 0 on error
    uint32_t PromGetId(void);

    // PromGetStatus (General)
    // returns the status register from the M25P16 PROM
    // (this is different than the interface status read from address offset 8)
    // The following masks are for the useful status register bits.
    enum { MASK_WIP = 1, MASK_WEL = 2 };   // status register bit masks
    bool PromGetStatus(uint32_t &status, PromType type = PROM_M25P16);

    // PromGetResult (General)
    // returns the result (if any) from the last command sent
    bool PromGetResult(uint32_t &result, PromType type = PROM_M25P16);

    // Returns nbytes data read from the specified address
    bool PromReadData(uint32_t addr, uint8_t *data,
                      unsigned int nbytes);

    // Enable programming commands (erase and program page) (General)
    // This sets the WEL bit in the status register.
    // This mode is automatically cleared after a programming command is executed.
    bool PromWriteEnable(PromType type = PROM_M25P16);

    // Disable programming commands (General)
    // This clears the WEL bit in the status register.
    bool PromWriteDisable(PromType type = PROM_M25P16);

    // Erase a sector (64K) at the specified address. (M25P16 ONLY)
    // This command calls PromWriteEnable.
    // If non-zero, the callback (cb) is called while the software is waiting
    // for the PROM to be erased, or if there is an error.
    bool PromSectorErase(uint32_t addr, const ProgressCallback cb = 0);

    // Program a page (up to 256 bytes) at the specified address.
    // This command calls PromWriteEnable.
    // nbytes must be a multiple of 4.
    // If non-zero, the callback (cb) is called while the software is waiting
    // for the PROM to be programmed, or if there is an error.
    // Returns the number of bytes programmed (-1 if error).
    int PromProgramPage(uint32_t addr, const uint8_t *bytes,
                        unsigned int nbytes, const ProgressCallback cb = 0);


    // ******************* Hardware (QLA) PROM ONLY Methods ***************************
    // Parameter "chan" is used to distinguish between multiple PROMs. Set to 0 for QLA
    // and set to 1 or 2 for DQLA. These get converted to PROM_25AA128, PROM_25AA128_1
    // and PROM_25AA128_2 internally.
    bool PromReadByte25AA128(uint16_t addr, uint8_t &data,
                             unsigned char chan = 0);
    bool PromWriteByte25AA128(uint16_t addr, const uint8_t &data,
                              unsigned char chan = 0);
    bool PromReadBlock25AA128(uint16_t addr, quadlet_t* data, unsigned int nquads,
                              unsigned char chan = 0);
    bool PromWriteBlock25AA128(uint16_t addr, quadlet_t* data, unsigned int nquads,
                               unsigned char chan = 0);

    // ********************** KSZ8851 Ethernet MAC/PHY Methods ************************
    // Following functions enable access to the KSZ8851 Ethernet controller on the
    // FPGA V2 board via FireWire. They are provided for testing/debugging.
    // Note that both 8-bit and 16-bit transactions are supported.
    bool WriteKSZ8851Reg(uint8_t addr, const uint8_t &data);
    bool WriteKSZ8851Reg(uint8_t addr, const uint16_t &data);
    bool ReadKSZ8851Reg(uint8_t addr, uint8_t &rdata);
    bool ReadKSZ8851Reg(uint8_t addr, uint16_t &rdata);
    // Following are for DMA access (assumes chip has been placed in DMA mode)
    bool WriteKSZ8851DMA(const uint16_t &data);
    bool ReadKSZ8851DMA(uint16_t &rdata);
    // Read Chip ID from register 0xC0
    uint16_t ReadKSZ8851ChipID();
    // Read KSZ8851 status; format is:  VALID(1) ERROR(1) initOK(1) ...
    //    VALID=1 indicates that Ethernet is present
    //    ERROR=1 indicates that last command had an error (i.e., state machine was not idle)
    //    InitOK=1 indicates that the KSZ8851 has been initialized
    //    For other bits, see EthBasePort::PrintStatus
    // Returns 0 on error (i.e., if Ethernet not present, or read fails)
    uint16_t ReadKSZ8851Status();

    // *********************** RTL8211F Ethernet PHY Methods **************************
    // The following functions enable access to both of the RTL8211F Ethernet PHYs on
    // the FPGA V3 board via FireWire. They are provided for testing/debugging.

    // The RTL8211F registers are provided via multiple pages. Use the RTL8211F_PAGSR
    // (page select register) to select the desired page. The default page is 0xa42.
    // Version 1.4 of the datasheet specifies the page for each register, though it
    // appears that many of the registers are accessible on multiple pages (clearly PAGSR
    // must be accessible on all pages). The datasheet indicates that registers 0-15
    // (IEEE Standard Registers) can be accessed from page 0 or 0xa42.
    // It appears that in some cases, it may be possible to read a register on multiple
    // pages, but only possible to write to the register from a specific page.
    // The following enum defines some of the more useful registers, and indicates the
    // corresponding page that is specified in the Version 1.4 datasheet.
    enum RTL8211F_Pages {
        RTL8211F_PAGE_IEEE = 0,          // IEEE Standard Registers
        RTL8211F_PAGE_DEFAULT = 0x0a42,  // Default page (includes IEEE registers)
        RTL8211F_PAGE_LED = 0x0d04       // Page for LED configuration
    };

    enum RTL8211F_Regs {
        RTL8211F_BMCR = 0,               // Basic Mode Control Register, page 0
        RTL8211F_BMSR = 1,               // Basic Mode Status Register, page 0
        RTL8211F_PHYID1 = 2,             // PHY Identifier Register 1, page 0
        RTL8211F_PHYID2 = 3,             // PHY Identifier Register 2, page 0
        RTL8211F_INER = 18,              // Interrupt Enable Register, page 0xa42
        RTL8211F_PHYSR = 26,             // PHY Specific Status Register, page 0xa43
        RTL8211F_INSR = 29,              // Interrupt Status Register, page 0xa43
        RTL8211F_PAGSR = 31              // Page Select Register, 0xa43
    };

    // Some useful Ethernet PHY addresses:
    //    FPGAV3 is designed so that the RTL8211F PHY has a default address of 1.
    //    The GMII to RGMII core uses the default PHY address of 8.
    enum PHY_ADDR { PHY_BROADCAST = 0, PHY_RTL8211F = 1, PHY_GMII_CORE = 8 };

    // Wait for FPGA mdio interface to return to idle state, using specified number of loops
    // (loop_cnt) as timeout. This method is necessary for the Zynq EMIO mmap interface,
    // which is faster than the FPGA mdio state machine.
    //    callerName  name of calling routine (used if printing timeout message)
    //    chan        1 or 2 for PHY1 or PHY2
    //    loop_cnt    number of read loops for checking mdio state
    bool WaitRTL8211F_Idle(const char *callerName, unsigned int chan, unsigned int loop_cnt);

    // Read PHY register
    //    chan    1 or 2 for PHY1 or PHY2
    //    phyAddr PHY address (see PHY_ADDR)
    //    regAddr PHY register to read (see RTL8211F_Regs)
    //    data    data read from PHY register
    // Returns true if read was successful
    bool ReadRTL8211F_Register(unsigned int chan, unsigned int phyAddr, unsigned int regAddr, uint16_t &data);

    // Write PHY register
    //    chan    1 or 2 for PHY1 or PHY2
    //    phyAddr PHY address (see PHY_ADDR)
    //    regAddr PHY register to write (see RTL8211F_Regs)
    //    data    data to write to PHY register
    // Returns true if write was successful
    bool WriteRTL8211F_Register(unsigned int chan, unsigned int phyAddr, unsigned int regAddr, uint16_t data);

    // ************************ Ethernet Methods *************************************

    // Read Ethernet status
    // Reads the version-specific Ethernet status register. The first two most significant
    // bits indicate the FPGA version:
    //    00  FPGA V1 (no Ethernet)
    //    1x  FPGA V2 (one Ethernet port, with KSZ8851 PHY)
    //    01  FPGA V3 (two Ethernet ports, with RTL8211F PHYs)
    // The following enums define the bits (see also EthBasePort::PrintStatus)
    enum EthStatus {
        ETH_STAT_PRESENT_V2    = 0x80000000,   // Indicates FPGA V2
        ETH_STAT_REQ_ERR_V2    = 0x40000000,   // V2: Request error (V3: 1)
        ETH_STAT_INIT_OK_V2    = 0x20000000,   // V2: Init OK
        ETH_STAT_FRAME_ERR     = 0x10000000,   // Unsupported Ethernet frame (not Raw, IPv4, or ARP)
        ETH_STAT_IPV4_ERR      = 0x08000000,   // IPv4 header error (e.g., not UDP or ICMP)
        ETH_STAT_UDP_ERR       = 0x04000000,   // Unsupported UDP port (not 1394)
        ETH_STAT_DEST_ERR      = 0x02000000,   // Incorrect Firewire packet destination address
        ETH_STAT_ACCESS_ERR    = 0x01000000,   // Internal bus access error (0 for Rev 9+)
        ETH_STAT_STATE_ERR_V2  = 0x00800000,   // V2: KSZ8851 state machine error
        ETH_STAT_CLK125_OK_V3  = 0x00800000,   // V3: 125 MHz clock ok
        ETH_STAT_ETHST_ERR     = 0x00400000,   // EthernetIO state machine error
        ETH_STAT_CLK200_OK_V3  = 0x00200000,   // V3: 200 MHz clock ok (V2: 0)
        ETH_STAT_UDP           = 0x00100000,   // 1 -> UDP mode, 0 -> Raw Ethernet
        ETH_STAT_LINK_STAT_V2  = 0x00080000,   // V2: link status (1 -> On)
        ETH_STAT_IDLE_V2       = 0x00040000,   // V2: KSZ8851 state machine is idle
        ETH_STAT_WAIT_MASK_V2  = 0x00030000,   // V2: Mask for waitInfo
        ETH_STAT_PSETH_EN_V3   = 0x00010000,   // V3: PS Ethernet enabled (Rev 9+)
        ETH_STAT_DATA_MASK_V2  = 0x0000ffff,   // V2: Mask for register read result
        ETH_STAT_PORT2_MASK_V3 = 0x0000ff00,   // V3: Mask for Eth2 status (see EthPortV3Status)
        ETH_STAT_PORT1_MASK_V3 = 0x000000ff    // V3: Mask for Eth1 status (see EthPortV3Status)
    };
    bool ReadEthernetStatus(uint32_t &status);

    // Get Ethernet port status (for FPGA V3) from specified status value
    //    status  Status value from ReadEthernetStatus
    //    chan    Ethernet port number (1 or 2)
    // Returns the 8-bit port status
    enum EthPortV3Status {
        ETH_PORT_STAT_INIT_OK    = 0x80,
        ETH_PORT_STAT_HAS_IRQ    = 0x40,
        ETH_PORT_STAT_LINK_STAT  = 0x20,
        ETH_PORT_STAT_SPEED_MASK = 0x18,
        ETH_PORT_STAT_RECV_ERR   = 0x04,     // Rev 8 only
        ETH_PORT_STAT_SEND_OVF   = 0x02,     // Rev 8 only
        ETH_PORT_STAT_PS_ETH     = 0x01      // Rev 8 only
    };
    static uint8_t GetEthernetPortStatusV3(uint32_t status, unsigned int chan);

    // Write Ethernet control register
    enum EthControl {
        ETH_CTRL_CLR_ERR_SW_V3 = 0x40000000,    // V3: Clear switch errors
        ETH_CTRL_CLR_ERR_NET  = 0x20000000,     // Clear network layer errors
        ETH_CTRL_CLR_ERR_LINK_V2 = 0x10000000,  // V2: Clear link layer errors
        ETH_CTRL_IRQ_GEN_V3   = 0x10000000,     // V3: Mask for generating software IRQ (MaskL)
        ETH_CTRL_DMA_V2       = 0x08000000,     // V2: DMA register access
        ETH_CTRL_IRQ_DIS_V3   = 0x08000000,     // V3: Mask for disabling interrupts
        ETH_CTRL_RESET_PHY    = 0x04000000,     // Reset PHY
        ETH_CTRL_WR_V2        = 0x02000000,     // V2: Register read(0) or write(1)
        ETH_CTRL_EN_PS_ETH_V3 = 0x02000000,     // V3: Mask for enabling PS ethernet
        ETH_CTRL_WB_V2        = 0x01000000,     // V2: Byte(0) or word(1) access
        ETH_CTRL_ADDR_MASK_V2 = 0x00ff0000,     // V2: Register address (8 bits)
        ETH_CTRL_PS_ETH_V3    = 0x00010000,     // V3: Enable PS ethernet
        ETH_CTRL_DATA_MASK_V2 = 0x0000ffff      // V2: Register data (16 bits)
    };
    // Offsets for Ethernet port control (FPGA V3)
    //   For Eth1, use these bit masks
    //   For Eth2, shift left by 8
    enum EthPortV3Control {
        ETH_PORT_CTRL_RESET_PHY = 0x80,     // Reset RTL8211F PHY (if ETH_CTRL_RESET_PHY)
        ETH_PORT_CTRL_IRQ_DIS   = 0x40,     // Disable interrupts
        ETH_PORT_CTRL_IRQ_GEN   = 0x20      // Generate interrupt from software (if ETH_CTRL_IRQ_GEN_V3)
    };
    bool WriteEthernetControl(uint32_t write_data);

    // Reset the Ethernet PHY chip (KSZ8851 for FPGA V2, RTL8211F for FPGA V3)
    //    chan    FPGA V2: 0 (requires ~60 msec)
    //            FPGA V3: 1 -> PHY1, 2 -> PHY2, 3-> both
    bool WriteEthernetPhyReset(unsigned int chan = 0);

    // Clear Ethernet errors
    //   For V2, clears EthernetIO and KSZ8851 errors
    //   For V3, clears EthernetIO and EthSwitch errors
    bool WriteEthernetClearErrors();

    // Read Ethernet data
    //    buffer  buffer for storing data
    //    offset  address offset (in quadlets)
    //    nquads  number of quadlets to read (not more than 64)
    bool ReadEthernetData(quadlet_t *buffer, unsigned int offset, unsigned int nquads);

    // ************************ FireWire Methods *************************************
    // Read FireWire data
    //    buffer  buffer for storing data
    //    offset  address offset (in quadlets)
    //    nquads  number of quadlets to read (not more than 64)
    bool ReadFirewireData(quadlet_t *buffer, unsigned int offset, unsigned int nquads);

    // Read FireWire PHY register
    //    addr    FireWire PHY register number (0-15)
    //    data    Data read from register
    bool ReadFirewirePhy(unsigned char addr, unsigned char &data);

    // Write FireWire PHY register
    //    addr    FireWire PHY register number (0-15)
    //    data    Data to write to register
    bool WriteFirewirePhy(unsigned char addr, unsigned char data);

protected:

    // Accumulated firmware time
    double firmwareTime;

};

#endif // __FpgaIO_H__
