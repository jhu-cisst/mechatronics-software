/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015

  (C) Copyright 2015-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <stdlib.h> // for atoi
#include <unistd.h> // for usleep
#include <iostream>

#include <FirewirePort.h>
#include <AmpIO.h>

int main(int argc, char** argv)
{
    unsigned int i, j;
    int port = 0;
    bool closing = true;

    for (i = 1; i < (unsigned int)argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            port = atoi(argv[i]+2);
        }
        if ((argv[i][0] == '-') && (argv[i][1] == 'o')) {
            closing = false;
        }
    }

    std::cout << "Using firewire port " << port << " (to change port, use -p)" << std::endl;
    if (closing) {
        std::cout << "Closing all relays (to open all, use -o)" << std::endl;
    } else {
        std::cout << "Opening all relays" << std::endl;
    }

    FirewirePort Port(port, std::cout);
    if (!Port.IsOK()) {
        std::cerr << "Failed to initialize firewire port " << port << std::endl;
        return -1;
    }

    std::vector<AmpIO *> BoardList;
    for (i = 0; i < Port.GetNumOfNodes(); i++) {
        int boardId = Port.GetBoardId(i);
        if (boardId != BoardIO::MAX_BOARDS) {
            std::cout << "Adding firewire Node " << i << ", BoardID " << boardId << std::endl;
            AmpIO *board = new AmpIO(boardId);
            BoardList.push_back(board);
            Port.AddBoard(board);
        }
    }

    for (j = 0; j < BoardList.size(); j++) {
        BoardList[j]->WriteSafetyRelay(closing);
        usleep(40000); // sleep 40 ms
    }

    if (closing) {
        std::cout << "Safety relays closed" << std::endl;
    } else {
        std::cout << "Safety relays opened" << std::endl;
    }
    return 0;
}
