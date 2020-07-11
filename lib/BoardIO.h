/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen

  (C) Copyright 2011-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
#ifdef _MSC_VER
typedef unsigned __int8  uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
#else
#include <stdint.h>
#endif

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

    friend class BasePort;
    friend class FirewirePort;
    friend class EthBasePort;
    friend class EthRawPort;
    friend class EthUdpPort;

    // DESIGN APPROACH (IN PROCESS):
    // For real-time block reads and writes, the board class (i.e., derived classes from BoardIO)
    // determines the data size (NumBytes), but the port classes (i.e., derived classes from BasePort)
    // allocate the memory and call SetReadBuffer/SetWriteBuffer.

    // Following methods are for real-time block reads
    void SetReadValid(bool flag) { readValid = flag; }
    virtual unsigned int GetReadNumBytes() const = 0;
    virtual quadlet_t *GetReadBuffer() const = 0;
    virtual void SetReadBuffer(quadlet_t *buf) = 0;

    // Following methods are for real-time block writes
    // TODO: Consolidate Buffer and BufferData
    void SetWriteValid(bool flag)
    { writeValid = flag; if (writeValid) memset(GetWriteBufferData(), 0, GetWriteNumBytes()); }
    virtual unsigned int GetWriteNumBytes() const = 0;
    virtual quadlet_t *GetWriteBuffer() const = 0;
    virtual quadlet_t *GetWriteBufferData() const = 0;
    virtual void SetWriteBuffer(quadlet_t *buf, size_t data_offset) = 0;

    virtual bool WriteBufferResetsWatchdog(void) const = 0;
    virtual void CheckCollectCallback() = 0;

public:
    enum {MAX_BOARDS = 16};   // Maximum number of boards

    BoardIO(unsigned char board_id) : BoardId(board_id), port(0), readValid(false), writeValid(false) {}
    virtual ~BoardIO() {}

    inline unsigned char GetBoardId() const { return BoardId; }

    // Returns true if a valid board
    inline bool IsValid() const { return (BoardId < MAX_BOARDS); }

    inline bool ValidRead() const { return readValid; }
    inline bool ValidWrite() const { return writeValid; }
};

#endif // __BOARDIO_H__
