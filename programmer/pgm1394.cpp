/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#ifdef _MSC_VER
#include <windows.h>  // for Sleep
#else
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#endif
#include "mcsFile.h"

#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "Eth1394Port.h"
#endif
#include "AmpIO.h"

int GetMenuChoice(AmpIO &Board, const std::string &mcsName)
{
#ifndef _MSC_VER
    struct termios savedTerm, newTerm;
    tcgetattr(STDIN_FILENO, &savedTerm);
    newTerm = savedTerm;
    newTerm.c_lflag &= ~ICANON;  // turn off line buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newTerm);  // change terminal settings
#endif

    int c = 0;
    while ((c < '0') || (c > '7')) {
        if (c)
            std::cout << std::endl << "Invalid option -- try again" << std::endl;
        std::cout << std::endl
                  << "Board: " << (unsigned int)Board.GetBoardId() << std::endl
                  << "MCS file: " << mcsName << std::endl;
        std::cout << "PROM Id: " << std::hex << Board.PromGetId() << std::dec
                  << std::endl << std::endl;

        std::cout << "0) Exit programmer" << std::endl
                  << "1) Program PROM" << std::endl
                  << "2) Verify PROM" << std::endl
                  << "3) Read PROM data" << std::endl
                  << "4) Program FPGA SN" << std::endl
                  << "5) Program QLA SN" << std::endl
                  << "6) Read FPGA SN" << std::endl
                  << "7) Read QLA SN" << std::endl  << std::endl;

        std::cout << "Select option: ";
        c = getchar();
        std::cout << std::endl;
    }

#ifndef _MSC_VER
    tcsetattr(STDIN_FILENO, TCSANOW, &savedTerm);  // restore terminal settings
#endif
    return (c-'0');
}

// Returns time in seconds
double get_time()
{
#ifdef _MSC_VER
    return 0.0;
#else
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (1e-6*tv.tv_usec + tv.tv_sec);
#endif
}

static double Callback_StartTime = 0.0;

bool PromProgramCallback(const char *msg)
{
    if (msg) std::cout << std::endl << msg << std::endl;
    else {
        double t = get_time();
        if ((t - Callback_StartTime) > 0.1) {
            std::cout << "." << std::flush;
            Callback_StartTime = t;
        }
    }        
    return true;   // continue
}


bool PromProgram(AmpIO &Board, mcsFile &promFile)
{
    double startTime = get_time();
    promFile.Rewind();
    while (promFile.ReadNextSector()) {
        unsigned long addr = promFile.GetSectorAddress();
        std::cout << "Erasing sector " << std::hex << addr << std::dec << std::flush;
        Callback_StartTime = get_time();
        if (!Board.PromSectorErase(addr, PromProgramCallback)) {
            std::cout << "Failed to erase sector " << addr << std::endl;
            return false;
        }
        std::cout << std::endl << "Programming sector " << std::hex << addr
                  << std::dec << std::flush;
        const unsigned char *sectorData = promFile.GetSectorData();
        unsigned long numBytes = promFile.GetSectorNumBytes();
        unsigned long page = 0;
        Callback_StartTime = get_time();
        while (page < numBytes) {
            unsigned int bytesToProgram = std::min(numBytes-page, 256UL);
            if (!Board.PromProgramPage(addr+page, sectorData+page, bytesToProgram,
                                       PromProgramCallback)) {
                std::cout << "Failed to program page " << addr << std::endl;
                return false;
            }
            page += bytesToProgram;
        }
        std::cout << std::endl;
    }
    std::cout << "PROM programming time = " << get_time() - startTime << " seconds"
              << std::endl;
    return true;
}


bool PromVerify(AmpIO &Board, mcsFile &promFile)
{
    double startTime = get_time();
    unsigned char DownloadedSector[65536];
    promFile.Rewind();
    while (promFile.ReadNextSector()) {
        unsigned long addr = promFile.GetSectorAddress();
        unsigned long numBytes = promFile.GetSectorNumBytes();
        std::cout << "Verifying sector " << std::hex << addr << std::flush;
        if (numBytes > sizeof(DownloadedSector)) {
            std::cout << "Error: sector too large = " << numBytes << std::endl;
            return false;
        }
        if (!Board.PromReadData(addr, DownloadedSector, numBytes)) {
            std::cout << "Error reading PROM data" << std::endl;
            return false;
        }
        if (!promFile.VerifySector(DownloadedSector, numBytes))
            return false;
        std::cout << std::endl;
    }
    std::cout << std::dec;
    std::cout << "PROM verification time = " << get_time() - startTime << " seconds"
              << std::endl;
    return true;
}

bool PromDisplayPage(AmpIO &Board, unsigned long addr)
{
    unsigned char bytes[256];
    if (!Board.PromReadData(addr, bytes, sizeof(bytes)))
        return false;
    std::cout << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < sizeof(bytes); i += 16) {
        std::cout << std::setw(4) << addr+i << ": ";
        for (int j = 0; j < 16; j++)
            std::cout << std::setw(2) << (unsigned int) bytes[i+j] << "  ";
        std::cout << std::endl;
    }
    //std::cout.unsetf(std::ios_base::setfill | std::ios_base::setw);
    std::cout << std::dec;
    return true;
}

/*!
 \brief Prom FPGA serial and Revision Number

 \param[in] Board FPGA Board
 \return[out] bool true on success, false otherwise
*/
bool PromFPGASerialNumberProgram(AmpIO &Board)
{
    std::stringstream ss;
    std::string BoardType;
    std::string str;
    std::string BoardSNRead;
    bool success = true;

    AmpIO_UInt32 fver = Board.GetFirmwareVersion();
    if (fver < 4) {
        std::cout << "Firmware not supported, current version = " << fver << "\n"
                  << "Please upgrade your firmware" << std::endl;
        return false;
    }

    // ==== FPGA Serial ===
    // FPGA 1234-56
    // - FPGA: board type
    // - 1234-56: serial number
    BoardType = "FPGA";
    std::string FPGASN = "0000-00";
    std::cout << "Please Enter FPGA Serial Number: " << std::endl;
    std::cin >> FPGASN;
    std::cin.ignore(20,'\n');
    ss << BoardType << " " << FPGASN;
    str = ss.str();
    char buffer[20];
    for (size_t i = 0; i < str.length(); i++) {
        buffer[i] = str.at(i);
    }

    int ret;
    Callback_StartTime = get_time();
    Board.PromSectorErase(0x1F0000, PromProgramCallback);
    ret = Board.PromProgramPage(0x1FFF00, (AmpIO_UInt8*)&buffer, static_cast<unsigned int>(str.length()));
    if (ret < 0) {
        std::cerr << "Can't program FPGA Serial Number";
        return false;
    }
#ifdef _MSC_VER
    Sleep(1);
#else
    usleep(5000);
#endif

    BoardSNRead.clear();
    BoardSNRead = Board.GetFPGASerialNumber();

    if (FPGASN == BoardSNRead) {
        std::cout << "Programmed " << FPGASN << " Serial Number" << std::endl;
    } else {
        std::cerr << "Failed to program" << std::endl;
        std::cerr << "Board SN = " << FPGASN << "\n"
                  << "Read  SN = " << BoardSNRead << std::endl;
        success = false;
    }
    return success;
}


/*!
 \brief Prom QLA serial and Revision Number

 \param[in] Board FPGA Board
 \return[out] bool true on success, false otherwise
*/
bool PromQLASerialNumberProgram(AmpIO &Board)
{
    std::stringstream ss;
    std::string BoardType;
    std::string str;
    std::string BoardSNRead;
    bool success = true;

    AmpIO_UInt32 fver = Board.GetFirmwareVersion();
    if (fver < 4) {
        std::cout << "Firmware not supported, current version = " << fver << "\n"
                  << "Please upgrade your firmware" << std::endl;
        return false;
    }

    // ==== QLA Serial ===
    // QLA 9876-54
    // - QLA: board type
    // - 9876-54: serial number
    BoardType = "QLA";
    std::string BoardSN = "0000-00";
    AmpIO_UInt8 wbyte;
    AmpIO_UInt16 address;

    // get s/n from user
    std::cout << "Please Enter QLA Serial Number: " << std::endl;
    std::cin >> BoardSN;
    std::cin.ignore(20,'\n');
    BoardSN = BoardSN.substr(0, BoardSN.length()); // remove '\n' at the end
    ss << BoardType << " " << BoardSN;
    str = ss.str();

//    std::cout << "DATA = " << ss.str().c_str() << std::endl;

    // S1: program to QLA PROM
    address = 0x0000;
    for (size_t i = 0; i < str.length(); i++) {
        wbyte = str.at(i);
        if (!Board.PromWriteByte25AA128(address, wbyte)) {
            std::cerr << "Failed to write byte " << i << std::endl;
            return false;
        }
        address += 1;  // inc to next byte
    }

    // S2: read back and verify
    BoardSNRead.clear();
    BoardSNRead = Board.GetQLASerialNumber();
//    std::cout << "SN = " << BoardSNRead << " length = " << BoardSNRead.length() << std::endl;
//    std::cout << "st = " << str << " length = " << str.length() << std::endl;

    if (BoardSN == BoardSNRead) {
        std::cout << "Programmed QLA " << BoardSN << " Serial Number" << std::endl;
    } else {
        std::cerr << "Failed to program" << std::endl;
        std::cerr << "Board SN = " << BoardSN << "\n"
                  << "Read  SN = " << BoardSNRead << std::endl;
        success = false;
    }

    return success;
}

int main(int argc, char** argv)
{
    int i;
#if Amp1394_HAS_RAW1394
    bool useFireWire = true;
#else
    bool useFireWire = false;
#endif
    int port = 0;
    int board = BoardIO::MAX_BOARDS;
    std::string mcsName;
    std::string sn;

    int args_found = 0;
    for (i = 1; i < argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
                std::cerr << "Selecting port " << port << std::endl;
            // -p option can be -pN, -pfwN, or -pethN, where N
            // is the port number. -pN is equivalent to -pfwN
            // for backward compatibility.
            if (strncmp(argv[i]+2, "fw", 2) == 0)
                port = atoi(argv[i]+4);
            else if (strncmp(argv[i]+2, "eth", 3) == 0) {
                useFireWire = false;
                port = atoi(argv[i]+5);
            }
            else
                port = atoi(argv[i]+2);
            if (useFireWire)
                std::cerr << "Selecting FireWire port " << port << std::endl;
            else
                std::cerr << "Selecting Ethernet port " << port << std::endl;
        }
        else {
            if (args_found == 0)
                board = atoi(argv[i]);
            else if (args_found == 1)
                mcsName = std::string(argv[i]);
            args_found++;
        }
    }
    if (args_found < 1) {
        std::cerr << "Usage: pgm1394 <board-num> [<mcs-file>] [-pP]" << std::endl
                  << "       where P = port number (default 0)" << std::endl
                  << "       can also specify -pfwP or -pethP" << std::endl;
        return 0;
    }

    BasePort *Port;
    if (useFireWire) {
#if Amp1394_HAS_RAW1394
        Port = new FirewirePort(port, std::cerr);
        if (!Port->IsOK()) {
            std::cerr << "Failed to initialize firewire port " << port << std::endl;
            return -1;
        }
#else
        std::cerr << "FireWire not available (set Amp1394_HAS_RAW1394 in CMake)" << std::endl;
        return -1;
#endif
    }
    else {
#if Amp1394_HAS_PCAP
        Port = new Eth1394Port(port, std::cerr);
        if (!Port->IsOK()) {
            std::cerr << "Failed to initialize ethernet port " << port << std::endl;
            return -1;
        }
        Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW);  // PK TEMP
#else
        std::cerr << "Ethernet not available (set Amp1394_HAS_PCAP in CMake)" << std::endl;
        return -1;
#endif
    }
    AmpIO Board(board);
    Port->AddBoard(&Board);

    if (mcsName.empty()) {
        if (Board.HasEthernet())
            mcsName = std::string("FPGA1394Eth-QLA.mcs");
        else
            mcsName = std::string("FPGA1394-QLA.mcs");
    }
    mcsFile promFile;
    if (!promFile.OpenFile(mcsName)) {
        std::cerr << "Failed to open PROM file: " << mcsName << std::endl;
        return -1;
    }

    unsigned long addr;

    while (int c = GetMenuChoice(Board, mcsName)) {
        switch (c) {
        case 1:
            PromProgram(Board, promFile);
            break;
        case 2:
            PromVerify(Board, promFile);
            break;
        case 3:
            std::cout << "Enter address (hex): ";
            std::cin >> std::hex >> addr;
            std::cin.ignore(10,'\n');
            PromDisplayPage(Board, addr);
            break;
        case 4:
            PromFPGASerialNumberProgram(Board);
            break;
        case 5:
            PromQLASerialNumberProgram(Board);
            break;
        case 6:
            sn = Board.GetFPGASerialNumber();
            if (!sn.empty())
                std::cout << "FPGA serial number: " << sn << std::endl;
            break;
        case 7:
            sn = Board.GetQLASerialNumber();
            if (!sn.empty())
                std::cout << "QLA serial number: " << sn << std::endl;
            break;
        default:
            std::cout << "Not yet implemented" << std::endl;
        }
    }

    promFile.CloseFile();
    Port->RemoveBoard(board);
}
