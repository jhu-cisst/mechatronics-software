/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */


#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <getopt.h>
#include <byteswap.h>
#include <sys/time.h>
#include <unistd.h>  // for usleep

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
    const size_t numBoards = 4;
    int board_id[numBoards] = {BoardIO::MAX_BOARDS, BoardIO::MAX_BOARDS};
    std::vector<AmpIO*> BoardList;

    // set board id
    board_id[0] = 2;
    board_id[1] = 3;
    board_id[2] = 6;
    board_id[3] = 7;

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

    // ---------------------------------------------------
    // Firewire port
    // ZC: really this port number should be interactive
    // ---------------------------------------------------
    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);
    FirewirePort fwport(port, std::cout);
    if (!fwport.IsOK()) {
        std::cerr << "Failed to initialize firewire port" << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "Open port on " << port << std::endl;
    }

    // Create AmpIO board 1 and 2
    for (size_t i = 0; i < numBoards; i++) {
        BoardList.push_back(new AmpIO(board_id[i]));
        fwport.AddBoard(BoardList[i]);
        std::cout << "new board with node_id = " << std::dec
                  << fwport.GetNodeId(board_id[i]) << std::endl;
    }

    // ----- ---------------------------------
    // Quadlet broadcast
    // ---------------------------------------
    bool rc;
    nodeaddr_t nodeaddress_wd = 0xffffff000003;
    rc = fwport.WriteQuadletBroadcast(nodeaddress_wd, bswap_32(0x3888));
    if (!rc) {
        std::cerr << "Quadlet broadcast error" << std::endl;
    }

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
    // --------------------------------------
    for (size_t i = 0; i < BoardList.size(); i++) {
        BoardList[i]->WriteSafetyRelay(true);
        BoardList[i]->WritePowerEnable(true);
        usleep(40000); // sleep 40 ms
        BoardList[i]->WriteAmpEnable(0x0f, 0x0f);
    }

    AmpIO* board;
    for (size_t i = 0; i < numBoards; i++) {
        board = BoardList[i];
        // set current for all boards
        for (size_t j = 0; j < 4; j++) {
            board->SetMotorCurrent(j, 0x8000 + 0x200 * j);
        }
    }

	
    // ----- ---------------------------------
    // Broadcast Write/Read boards
    // ---------------------------------------

    int counter = 0;
    int csize = 100;
    timeval start, prev;
    timeval stop[csize];

    gettimeofday(&start, NULL);
    while(counter < csize){
        counter++;
        fwport.ReadAllBoardsBroadcast();
        fwport.WriteAllBoardsBroadcast();
        gettimeofday(&stop[counter-1], NULL);
    }

    prev = start;
    for (int i = 0; i < csize; i++) {
        double mtime;
        mtime = (stop[i].tv_sec - prev.tv_sec) * 1000000.0 +
                (stop[i].tv_usec - prev.tv_usec) + 0.5;
        prev = stop[i];

        std::cout << "time " << i << " = " << std::dec << mtime << std::endl;
    }


    // Shut down boards and remove from port
    for (size_t i = 0; i < BoardList.size(); i++) {
        BoardList[i]->WriteSafetyRelay(false);
        BoardList[i]->WriteAmpEnable(0x0f, 0);   // Turn power off
        BoardList[i]->WritePowerEnable(false);   // Turn power off
        fwport.RemoveBoard(BoardList[i]->GetBoardId());
    }

    return EXIT_SUCCESS;
}
