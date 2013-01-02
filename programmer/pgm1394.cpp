/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <iostream>
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
                  << "2) Verify PROM" << std::endl << std::endl;

        std::cout << "Select option: ";
        c = getchar();
        std::cout << std::endl;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &savedTerm);  // restore terminal settings
    return (c-'0');
}


bool PromProgram(AmpIO &Board, mcsFile &promFile)
{
    promFile.Rewind();
    while (promFile.ReadNextSector()) {
        unsigned long addr = promFile.GetSectorAddress();
#if 1
        if (!Board.PromSectorErase(addr)) {
            std::cout << "Failed to erase sector " << addr << std::endl;
            return false;
        }
#endif
        if (!Board.PromProgramPage(addr, promFile.GetSectorData(), 256)) {
            std::cout << "Failed to program page " << addr << std::endl;
            return false;
        }
        // For now, erase Sector 0 and program page 0
        break;
    }
    return true;
}


bool PromVerify(AmpIO &Board, mcsFile &promFile)
{
    unsigned char DownloadedSector[65536];
    promFile.Rewind();
    while (promFile.ReadNextSector()) {
        std::cout << "Read sector, address = " << std::hex 
                  << promFile.GetSectorAddress() << std::endl;
        std::cout << "Verifying ...";
        if (!Board.PromReadData(promFile.GetSectorAddress(),
                                DownloadedSector, 
                                sizeof(DownloadedSector))) {
            std::cout << "Error reading PROM data" << std::endl;
            return false;
        }
        if (!promFile.VerifySector(DownloadedSector,
                                   sizeof(DownloadedSector)))
            return false;
        std::cout << std::endl;
    }
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
        default:
            std::cout << "Not yet implemented" << std::endl;
        }
    }

    promFile.CloseFile();
    Port.RemoveBoard(board);
}
