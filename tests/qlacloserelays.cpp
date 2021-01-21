/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015

  (C) Copyright 2015-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <stdlib.h> // for atoi
#include <unistd.h> // for usleep
#include <iostream>
#include <vector>

#include <PortFactory.h>
#include <AmpIO.h>

int main(int argc, char** argv)
{
    std::cerr << "qlacloserelays will soon be deprecated, use `qlacommand` instead.  Examples:" << std::endl
              << "> qlacommand -c close-relays" << std::endl
              << "> qlacommand -pfw:0 -c close-relays" << std::endl
              << "> qlacommand -c open-relays" << std::endl;

    unsigned int i, j;
    BasePort * port = 0;
    std::string portArgs;
    bool closing = true;

    for (i = 1; i < (unsigned int)argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            portArgs = argv[i]+2;
        }
        if ((argv[i][0] == '-') && (argv[i][1] == 'o')) {
            closing = false;
        }
    }

    // if port is not specified
    port = PortFactory(portArgs.c_str());
    if (!port) {
        std::cerr << "Failed to create port using: " << portArgs << std::endl;
        return -1;
    }

    if (closing) {
        std::cout << "Closing all relays (to open all, use -o)" << std::endl;
    } else {
        std::cout << "Opening all relays" << std::endl;
    }

    std::vector<AmpIO *> boardList;
    for (i = 0; i < port->GetNumOfNodes(); i++) {
        int boardId = port->GetBoardId(i);
        if (boardId != BoardIO::MAX_BOARDS) {
            std::cout << "Adding firewire Node " << i << ", BoardID " << boardId << std::endl;
            AmpIO *board = new AmpIO(boardId);
            boardList.push_back(board);
            port->AddBoard(board);
        }
    }

    for (j = 0; j < boardList.size(); j++) {
        boardList[j]->WriteSafetyRelay(closing);
        usleep(40000); // sleep 40 ms
    }

    if (closing) {
        std::cout << "Safety relays closed" << std::endl;
    } else {
        std::cout << "Safety relays opened" << std::endl;
    }

    delete port;

    return 0;
}
