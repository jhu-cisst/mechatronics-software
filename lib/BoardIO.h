/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Zihan Chen

  (C) Copyright 2011-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
class Eth1394Port;

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

    // Following set in derived classes (in the future, by FirewirePort)
    unsigned int ReadBufferSize;
    quadlet_t *ReadBuffer;     // buffer for real-time reads
    unsigned int WriteBufferSize;
    quadlet_t *WriteBuffer;     // buffer for real-time writes
    quadlet_t *WriteBufferData; // buffer for data part

    friend class BasePort;
    friend class FirewirePort;
    friend class Eth1394Port;

    void SetReadValid(bool flag) { readValid = flag; }
    unsigned int GetReadNumBytes() const { return ReadBufferSize; }
    quadlet_t *GetReadBuffer() { return ReadBuffer; }

    void SetWriteValid(bool flag)
    { writeValid = flag; if (writeValid) memset(WriteBufferData, 0, WriteBufferSize); }
    unsigned int GetWriteNumBytes() const { return WriteBufferSize; }
    quadlet_t *GetWriteBuffer() { return WriteBuffer; }
    quadlet_t *GetWriteBufferData() { return WriteBufferData; }
    virtual void InitWriteBuffer(quadlet_t *buf, size_t data_offset) = 0;
    virtual bool WriteBufferResetsWatchdog(void) const = 0;

public:
    enum {MAX_BOARDS = 16};   // Maximum number of boards

    BoardIO(unsigned char board_id) : BoardId(board_id), port(0), readValid(false), writeValid(false),
                                      ReadBufferSize(0), ReadBuffer(0), WriteBufferSize(0), WriteBuffer(0) {}
    virtual ~BoardIO() {}

    inline unsigned char GetBoardId() const { return BoardId; }

    // Returns true if a valid board
    inline bool IsValid() const { return (BoardId < MAX_BOARDS); }

    inline bool ValidRead() const { return readValid; }
    inline bool ValidWrite() const { return writeValid; }
};

#endif // __BOARDIO_H__
