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

#ifndef __Spartan6IO_H__
#define __Spartan6IO_H__

#include <Amp1394/AmpIORevision.h>

#include "BoardIO.h"

// Methods specific to Spartan6 FPGA board (Rev 1.x or Rev 2.x)
//
// Supports:
//   - Read/Write FPGA PROM (M25P16) to reprogram firmware or FPGA serial number
//   - Read/Write external PROM (25AA128); although this PROM is not on the FPGA board,
//     it is suggested that all companion boards (e.g., QLA) include a 25AA128
//     (or compatible) PROM connected to pins IO1-1 through IO1-4.
//   - Reboot FPGA via software command (Firmware V7+)
//   - Turn on/off LED on FPGA board; note that for some hardware (e.g., QLA),
//     this would also enable board power
//   - Query whether FPGA board has Ethernet interface
//   - Query FPGA clock period
//   - Read Ethernet/Firewire buffers from FPGA board (for debugging)
//
// This class also contains a firmwareTime member (and accessor methods), which is
// updated by the derived class to keep a running timer based on the FPGA clock.

class Spartan6IO : public BoardIO
{
public:

    Spartan6IO(AmpIO_UInt8 board_id);
    ~Spartan6IO();

    // Return FPGA serial number (empty string if not found)
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

    static bool ResetKSZ8851All(BasePort *port);

    // ********************** PROM Methods ***********************************
    // Methods for reading or programming
    //   1 - the FPGA configuration PROM (M25P16)
    //   2 - Hardware (QLA) PROM (25AA128)
    // NOTE:
    //   - M25P16 & 25AA128 Have exact same command table, except M25P16 has
    //     a few extra commands (e.g. ReadID, DeepSleep)
    //   - address length
    //     - M25P16: 24-bit
    //     - 25AA128: 16-bit (2 MSB are ignored)
    enum PromType{
        PROM_M25P16 = 1,
        PROM_25AA128 = 2
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

    // Returns the PROM ID (M25P15 ONLY)
    // should be 0x00202015 for the M25P16
    // returns 0 on error
    AmpIO_UInt32 PromGetId(void);

    // PromGetStatus (General)
    // returns the status register from the M25P16 PROM
    // (this is different than the interface status read from address offset 8)
    // The following masks are for the useful status register bits.
    enum { MASK_WIP = 1, MASK_WEL = 2 };   // status register bit masks
    bool PromGetStatus(AmpIO_UInt32 &status, PromType type = PROM_M25P16);

    // PromGetResult (General)
    // returns the result (if any) from the last command sent
    bool PromGetResult(AmpIO_UInt32 &result, PromType type = PROM_M25P16);

    // Returns nbytes data read from the specified address
    bool PromReadData(AmpIO_UInt32 addr, AmpIO_UInt8 *data,
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
    bool PromSectorErase(AmpIO_UInt32 addr, const ProgressCallback cb = 0);

    // Program a page (up to 256 bytes) at the specified address.
    // This command calls PromWriteEnable.
    // nbytes must be a multiple of 4.
    // If non-zero, the callback (cb) is called while the software is waiting
    // for the PROM to be programmed, or if there is an error.
    // Returns the number of bytes programmed (-1 if error).
    int PromProgramPage(AmpIO_UInt32 addr, const AmpIO_UInt8 *bytes,
                        unsigned int nbytes, const ProgressCallback cb = 0);


    // ******************* Hardware (QLA) PROM ONLY Methods ***************************
    bool PromReadByte25AA128(AmpIO_UInt16 addr, AmpIO_UInt8 &data);
    bool PromWriteByte25AA128(AmpIO_UInt16 addr, const AmpIO_UInt8 &data);
    bool PromReadBlock25AA128(AmpIO_UInt16 addr, quadlet_t* data, unsigned int nquads);
    bool PromWriteBlock25AA128(AmpIO_UInt16 addr, quadlet_t* data, unsigned int nquads);

    // ********************** KSZ8851 Ethernet MAC/PHY Methods ************************
    // Following functions enable access to the KSZ8851 Ethernet controller on the
    // FPGA V2 board via FireWire. They are provided for testing/debugging.
    // Note that both 8-bit and 16-bit transactions are supported.
    bool ResetKSZ8851();   // Reset the chip (requires ~60 msec)
    bool WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt8 &data);
    bool WriteKSZ8851Reg(AmpIO_UInt8 addr, const AmpIO_UInt16 &data);
    bool ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt8 &rdata);
    bool ReadKSZ8851Reg(AmpIO_UInt8 addr, AmpIO_UInt16 &rdata);
    // Following are for DMA access (assumes chip has been placed in DMA mode)
    bool WriteKSZ8851DMA(const AmpIO_UInt16 &data);
    bool ReadKSZ8851DMA(AmpIO_UInt16 &rdata);
    // Read Chip ID from register 0xC0
    AmpIO_UInt16 ReadKSZ8851ChipID();
    // Get KSZ8851 status; format is:  VALID(1) 0(6) ERROR(1) PME(1) IRQ(1) STATE(4)
    //    VALID=1 indicates that Ethernet is present
    //    ERROR=1 indicates that last command had an error (i.e., state machine was not idle)
    //    PME is the state of the Power Management Event pin
    //    IRQ is the state of the Interrupt Request pin (active low)
    //    STATE is a 4-bit value that encodes the FPGA state machine (0=IDLE)
    // Returns 0 on error (i.e., if Ethernet not present, or read fails)
    AmpIO_UInt16 ReadKSZ8851Status();

    // *********************** RTL8211F Ethernet PHY Methods **************************
    // The following functions enable access to both of the RTL8211F Ethernet PHYs on
    // the FPGA V3 board via FireWire. They are provided for testing/debugging.

    // Read PHY register
    //    chan    1 or 2 for PHY1 or PHY2
    //    regNum  PHY register to read
    //    data    data read from PHY register
    // Returns true if read was successful
    bool ReadRTL8211F_Register(unsigned int chan, unsigned int regNum, AmpIO_UInt16 &data);

    // ************************ Ethernet Methods *************************************
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

protected:

    // Accumulated firmware time
    double firmwareTime;

};

#endif // __Spartan6IO_H__
