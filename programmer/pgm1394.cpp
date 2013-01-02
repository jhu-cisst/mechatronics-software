/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <stdlib.h>
#include <iostream>
#include "mcsFile.h"
#include "FirewirePort.h"
#include "AmpIO.h"

int main(int argc, char** argv)
{
    int i, j;
    int port = 0;
    int board = BoardIO::MAX_BOARDS;

    int args_found = 0;
    for (i = 1; i < argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            port = atoi(argv[i]+2);
            std::cerr << "Selecting port " << port << std::endl;
        }
        else {
            if (args_found == 0) {
                board = atoi(argv[i]);
                std::cerr << "Selecting board " << board << std::endl;
            }
            args_found++;
        }
    }
    if (args_found < 1) {
        std::cerr << "Usage: pgm1394 <board-num> [-pP]" << std::endl
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
    unsigned char DownloadedSector[65536];
    if (promFile.OpenFile("FPGA1394qla.mcs")) {
        while (promFile.ReadNextSector()) {
            std::cout << "Read sector, address = " << std::hex 
                      << promFile.GetSectorAddress() << std::endl;
            std::cout << "Verifying ...";
            if (!Board.GetPromData(promFile.GetSectorAddress(),
                                   DownloadedSector, 
                                   sizeof(DownloadedSector))) {
                std::cout << "Error reading PROM data" << std::endl;
                break;
            }
            if (!promFile.VerifySector(DownloadedSector,
                                       sizeof(DownloadedSector)))
                break;
            std::cout << std::endl;
        }
        promFile.CloseFile();
    }
    Port.RemoveBoard(board);
}
