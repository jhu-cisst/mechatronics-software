/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <sys/time.h>
#include "mcsFile.h"
#include "FirewirePort.h"
#include "AmpIO.h"

int GetMenuChoice(AmpIO &Board, const std::string &mcsName)
{
    struct termios savedTerm, newTerm;
    tcgetattr(STDIN_FILENO, &savedTerm);
    newTerm = savedTerm;
    newTerm.c_lflag &= ~ICANON;  // turn off line buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newTerm);  // change terminal settings

    int c = 0;
    while ((c < '0') || (c > '3')) {
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
                  << "3) Read and display first 256 bytes" << std::endl << std::endl;

        std::cout << "Select option: ";
        c = getchar();
        std::cout << std::endl;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &savedTerm);  // restore terminal settings
    return (c-'0');
}

double gettime_us()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (double)tv.tv_usec + (double)1e6 * (double)tv.tv_sec;
}

bool PromProgramCallback(const char *msg)
{
    if (msg) std::cout << std::endl << msg << std::endl;
    else {
        static time_t t1 = time(NULL);
        time_t t2 = time(NULL);
        if (difftime(t2, t1) > 0.1) {
            std::cout << "." << std::flush;
            t1 = t2;
        }
    }        
    return true;   // continue
}


bool PromProgram(AmpIO &Board, mcsFile &promFile)
{
    promFile.Rewind();
    while (promFile.ReadNextSector()) {
        unsigned long addr = promFile.GetSectorAddress();
        std::cout << "Erasing sector " << std::hex << addr << std::dec << std::flush;
        if (!Board.PromSectorErase(addr, PromProgramCallback)) {
            std::cout << "Failed to erase sector " << addr << std::endl;
            return false;
        }
        std::cout << std::endl << "Programming sector " << std::hex << addr
                  << std::dec << std::flush;
        const unsigned char *sectorData = promFile.GetSectorData();
        unsigned long numBytes = promFile.GetSectorNumBytes();
        unsigned long page = 0;
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
    return true;
}


bool PromVerify(AmpIO &Board, mcsFile &promFile)
{
    unsigned char DownloadedSector[65536];
    promFile.Rewind();
    while (promFile.ReadNextSector()) {
        unsigned long addr = promFile.GetSectorAddress();
        unsigned long numBytes = promFile.GetSectorNumBytes();
        std::cout << "Verifying sector " << std::hex << addr << "..." << std::flush;
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
    return true;
}

bool PromDisplayPage(AmpIO &Board, unsigned long addr)
{
    unsigned char bytes[256];
    if (!Board.PromReadData(addr, bytes, sizeof(bytes)))
        return false;
    std::cout << std::hex << std::setfill('0');
    for (int i = 0; i < sizeof(bytes); i += 16) {
        std::cout << std::setw(4) << i << ": ";
        for (int j = 0; j < 16; j++)
            std::cout << std::setw(2) << (unsigned int) bytes[i+j] << "  ";
        std::cout << std::endl;
    }
    //std::cout.unsetf(std::ios_base::setfill | std::ios_base::setw);
    std::cout << std::dec;
    return true;
}


int main(int argc, char** argv)
{
    int i, j;
    int port = 0;
    int board = BoardIO::MAX_BOARDS;
    std::string mcsName("FPGA1394qla.mcs");

    int args_found = 0;
    for (i = 1; i < argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            port = atoi(argv[i]+2);
            std::cerr << "Selecting port " << port << std::endl;
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
                  << "       where P = port number (default 0)" << std::endl;
        return 0;
    }

    FirewirePort Port(port);
    if (!Port.IsOK()) {
        std::cerr << "Failed to initialize firewire port " << port << std::endl;
        return -1;
    }
    AmpIO Board(board);
    Port.AddBoard(&Board);

    mcsFile promFile;
    if (!promFile.OpenFile(mcsName)) {
        std::cerr << "Failed to open PROM file: " << mcsName << std::endl;
        return -1;
    }

    

    while (int c = GetMenuChoice(Board, mcsName)) {
        switch (c) {
        case 1:
            PromProgram(Board, promFile);
            break;
        case 2:
            PromVerify(Board, promFile);
            break;
        case 3:
            PromDisplayPage(Board, 0L);
            break;
        default:
            std::cout << "Not yet implemented" << std::endl;
        }
    }

    promFile.CloseFile();
    Port.RemoveBoard(board);
}
