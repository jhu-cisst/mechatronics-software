/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */


#include <stdlib.h>
#include <unistd.h>
#include <curses.h>
#include <iostream>
#include <sstream>
#include <getopt.h>
#include <byteswap.h>

#include "FirewirePort.h"
#include "AmpIO.h"

// ZC: maybe make it a unit testing ??


// This program tests FirewirePort API
void ShowUsage(void)
{
    std::cout << "fwPortTest: this program tests the FirewirePort class\n"
              << "   -h Display this usage information\n"
              << "   -p "
              << std::endl;
}


int main(int argc, char** argv)
{
    // declare variables and default values
    int port = 0;
    const size_t numBoards = 2;
    int board_id[numBoards] = {BoardIO::MAX_BOARDS, BoardIO::MAX_BOARDS};
    std::vector<AmpIO*> BoardList;
    board_id[0] = 2;
    board_id[1] = 3;


    // command line option setup
    const char short_options[] = "hp:";
    int next_option;
    do {
        next_option = getopt(argc, argv, short_options);
        switch (next_option)
        {
        case 'h':
            ShowUsage();
            break;
        case 'p':
            port = atoi(optarg);
            std::cout << "using port number" << std::endl;
            break;
        case -1:  // parsing done
            break;
        default:  // should NEVER happen
            abort();
        }
    }
    while (next_option != -1);

    // Firewire port
    // ZC: really this port number should be interactive
    std::stringstream debugStrem(std::stringstream::out|std::stringstream::in);
    FirewirePort fwport(port, std::cout);
    if (!fwport.IsOK()) {
        std::cerr << "Failed to initialize firewire port" << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "Open port on " << port << std::endl;
    }

    // Create AmpIO board 1 and 2
    for (size_t i = 0; i < 2; i++) {
        BoardList.push_back(new AmpIO(board_id[i]));
        fwport.AddBoard(BoardList[i]);
        std::cout << "new board with node_id = "
                  << fwport.GetNodeId(board_id[i]) << std::endl;
    }

    // ----- ---------------------------------
    // Quadlet broadcast
    // ---------------------------------------
    bool rc;
    nodeaddr_t nodeaddress_wd = 0xffffff000003;
    rc = fwport.WriteQuadletBroadcast(nodeaddress_wd, bswap_32(0x3599));

    // now read back and verify
    quadlet_t dataQuadlet = 0x0000;
    for (size_t i = 0; i < BoardList.size(); i++) {
        fwport.ReadQuadlet(BoardList[i]->GetBoardId(), 0x03, dataQuadlet);
        std::cout << std::dec << "Board " << (int) BoardList[i]->GetBoardId()
                  << " watchdog = " << std::hex << bswap_32(dataQuadlet) << std::endl;
        dataQuadlet = 0x0000; // clear value
    }


    // ----- ---------------------------------
    // Block broadcast
    // ---------------------------------------

    // set write buffer
    //     NOTE: here set 1 board's write buffer
    //           this buffer value will be broadcasted to all boards on the bus

    AmpIO* board = BoardList[0];
    for (size_t i = 0; i < 4; i++) {
        board->SetMotorCurrent(i, 0x8000 + 0x200 * i, true);
    }
    fwport.WriteBlockBroadcast(0xffffff000000,
                               board->GetWriteBuffer(),
                               board->GetWriteNumBytes() - 4);

    std::cout << "board id = " << (int) board->BoardId << std::endl
              << std::hex << bswap_32(board->GetWriteBuffer()[0]) << std::endl;

    std::cout << "numBytes = " << std::dec << board->GetWriteNumBytes() << std::endl;


    // ----- ---------------------------------
    // WriteAllBoardsBroadcast()
    // ---------------------------------------
    for (size_t i = 0; i < numBoards; i++) {
        board = BoardList[i];
        // set current for all boards
        for (size_t j = 0; j < 4; j++) {
            board->SetMotorCurrent(j, 0x8000 + 0x200 * j, true);
        }
    }

    fwport.WriteAllBoardsBroadcast();


    // Shut down boards and remove from port
    for (size_t i = 0; i < BoardList.size(); i++) {
        BoardList[i]->WriteSafetyRelay(false);
        BoardList[i]->WriteAmpEnable(0x0f, 0);   // Turn power off
        BoardList[i]->WritePowerEnable(false);   // Turn power off
        fwport.RemoveBoard(BoardList[i]->GetBoardId());
    }

    return EXIT_SUCCESS;
}

