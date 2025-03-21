/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen

  (C) Copyright 2011-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __BOARDIO_H__
#define __BOARDIO_H__

// Base class for custom boards with IEEE-1394 (Firewire) and Ethernet interface.

#include <string.h>  // for memset
#include <string>
#include "Amp1394Types.h"

typedef uint32_t quadlet_t;
typedef uint64_t nodeaddr_t;
typedef uint16_t nodeid_t;

class BasePort;
class FirewirePort;
class EthBasePort;
class EthRawPort;
class EthUdpPort;

class BoardIO
{
protected:
    // Prevent copies
    BoardIO(const BoardIO &);
    BoardIO& operator=(const BoardIO&);

    unsigned char BoardId;
    BasePort *port;

    bool readValid;
    bool writeValid;

    unsigned int numReadErrors;
    unsigned int numWriteErrors;

    friend class BasePort;
    friend class FirewirePort;
    friend class EthBasePort;
    friend class EthRawPort;
    friend class EthUdpPort;

    // InitBoard sets the number of motors and encoders, based on the hardware
    // (e.g., QLA or dRA1) and firmware version. It assumes that the port member
    // data has already been set.
    virtual void InitBoard(void) = 0;

    // For real-time block reads and writes, the board class (i.e., derived classes from BoardIO)
    // determines the data size (NumBytes), but the port classes (i.e., derived classes from BasePort)
    // allocate the memory.

    // Following methods are for real-time block reads
    void SetReadValid(bool flag)
    { readValid = flag; if (!readValid) numReadErrors++; }
    virtual unsigned int GetReadNumBytes() const = 0;
    virtual void SetReadData(const quadlet_t *buf) = 0;

    // Following methods are for real-time block writes
    void SetWriteValid(bool flag)
    { writeValid = flag; if (!writeValid) numWriteErrors++; }
    virtual unsigned int GetWriteNumBytes() const = 0;
    virtual bool GetWriteData(quadlet_t *buf, unsigned int offset, unsigned int numQuads, bool doSwap = true) const = 0;
    virtual void InitWriteBuffer(void) = 0;

    virtual bool WriteBufferResetsWatchdog(void) const = 0;
    virtual void CheckCollectCallback() = 0;

public:
    enum {MAX_BOARDS = 16};   // Maximum number of boards

    // The following registers are required to be supported on all boards because they are used by
    // the Port classes when scanning/configuring the bus.
    //
    //   BOARD_STATUS:     Bits 27-24 contain the board number (e.g., rotary switch)
    //   FW_PHY_REQ:       Write 0 to this register to initiate a read of PHY register 0 (self-id);
    //                     needed for an unmanaged Firewire bus (e.g., PC not connected to Firewire)
    //   FW_PHY_RESP:      PHY response; not used by Port classes
    //   HARDWARE_VERSION: Indicates type of connected board (e.g., QLA, dRAC)
    //   FIRMWARE_VERSION: Version of FPGA firmware
    //   IP_ADDR:          Ethernet IP address (written during Ethernet configuration)
    //   ETH_STATUS:       Ethernet status register (used to distinguish FPGA version)
    //   GIT_DESC:         Git description
    //
    enum Registers {
        BOARD_STATUS = 0,      // RW: Board status/control register
        FW_PHY_REQ = 1,        // WO: Firewire PHY register read/write request
        FW_PHY_RESP = 2,       // RO: Firewire PHY register read response
        HARDWARE_VERSION = 4,  // RO: Companion board type (e.g., "QLA1")
        FIRMWARE_VERSION = 7,  // RO: Firmware version number
        IP_ADDR = 11,          // RW: Ethernet IP address (Firmware V7+)
        ETH_STATUS = 12,       // RW: Ethernet status register (Firmware V5+)
        GIT_DESC = 15          // RO: Git description (after Firmware V8 release)
    };

    BoardIO(unsigned char board_id) : BoardId(board_id), port(0), readValid(false), writeValid(false),
                                      numReadErrors(0), numWriteErrors(0) {}
    virtual ~BoardIO() {}

    inline unsigned char GetBoardId() const { return BoardId; }

    // Returns true if a valid board
    inline bool IsValid() const { return (BoardId < MAX_BOARDS); }

    inline bool ValidRead() const { return readValid; }
    inline bool ValidWrite() const { return writeValid; }

    inline unsigned int GetReadErrors() const { return numReadErrors; }
    inline unsigned int GetWriteErrors() const { return numWriteErrors; }

    inline void ClearReadErrors() { numReadErrors = 0; }
    inline void ClearWriteErrors() { numWriteErrors = 0; }

    uint32_t GetFirmwareVersion(void) const;

    // Return FPGA major version number (1, 2, 3)
    // Returns 0 if unknown
    unsigned int GetFpgaVersionMajor(void) const;

    /*! Get FPGA major version from Ethernet status register
        \param status  Ethernet status register (input)
        \return unsigned int: FPGA major version (1, 2 or 3)
    */
    static unsigned int GetFpgaVersionMajorFromStatus(uint32_t status);

    uint32_t GetHardwareVersion(void) const;
    std::string GetHardwareVersionString(void) const;

    // Returns FPGA clock period in seconds
    virtual double GetFPGAClockPeriod(void) const = 0;

    // ********************** READ Methods ***********************************
    // The ReadXXX methods below read data directly from the boards via the
    // bus using quadlet reads.

    uint32_t ReadStatus(void) const;

    /*! Read IPv4 address (only relevant for FPGA boards with Ethernet support)
        \returns IPv4 address (uint32) or 0 if error
     */
    uint32_t ReadIPv4Address(void) const;

    // ********************** WRITE Methods **********************************

    /*! Write IPv4 address.
        \param IP address to write, as uint32
        \returns true if successful; false otherwise
    */
    bool WriteIPv4Address(uint32_t IPaddr);
};

#endif // __BOARDIO_H__
