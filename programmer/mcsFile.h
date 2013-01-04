/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _MCS_H
#define _MCS_H

// mcsFile
//
// This class reads an Intel MCS-86 format file, which is produced by the Xilinx
// ISE software. It is not a complete implementation, and probably will not work
// for MCS-86 files produced by other software packages.

#include <string>
#include <fstream>

class mcsFile {
    std::ifstream file;
    int line_num;                    // current line number
    unsigned long startAddr;         // start address
    unsigned long numBytes;          // number of bytes in sector
    unsigned char curSector[65536];  // current sector

    struct RecInfo {
        enum { DATA_MAX = 16 };
        unsigned char ndata;
        unsigned short addr;
        unsigned char type;
        unsigned char data[DATA_MAX];
    };

    bool ProcessNextLine(RecInfo &rec);
    bool toHex(const char *p2, unsigned char &result) const;

    enum RecordTypes {
        RECORD_DATA = 0,
        RECORD_EOF = 1,
        RECORD_EXT_SEGMENT = 2,  // not supported here
        RECORD_EXT_LINEAR = 4
    };

public:
    mcsFile();
    ~mcsFile();
    // Open file
    bool OpenFile(const std::string &fileName);
    // Read next sector
    bool ReadNextSector();
    unsigned long GetSectorAddress() const { return startAddr; }
    unsigned long GetSectorNumBytes() const { return numBytes; }
    const unsigned char *GetSectorData() const { return curSector; }
    // Compare current sector to specified data
    bool VerifySector(const unsigned char *data, unsigned long len) const;
    // Seek back to beginning of file
    void Rewind();
    void CloseFile();
};

#endif // _MCS_H
