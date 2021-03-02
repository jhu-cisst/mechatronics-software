/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2015 (from qlacloserelays)

  (C) Copyright 2015-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <stdlib.h> // for atoi
#include <iostream>
#include <vector>
#include <string>

#include <PortFactory.h>
#include <AmpIO.h>
#include "Amp1394Time.h"

int main(int argc, char** argv)
{
    unsigned int i, j;
    unsigned int nbArgs = static_cast<unsigned int>(argc);
    BasePort * port = 0;
    std::string portArgs;
    std::string command;

    for (i = 1; i < nbArgs; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            portArgs = argv[i] + 2;
        }
        if ((argv[i][0] == '-') && (argv[i][1] == 'c')) {
            // make sure we have an extra argument
            if ((i + 1) >= nbArgs) {
                std::cerr << "Missing argument after -c" << std::endl;
                return -1;
            }
            command = argv[i + 1];
        }
    }

    // check that command is supported before creating anything
    if (! ((command == "open-relays")
           || (command == "close-relays")
           || (command == "reboot")
           || (command == "reset-eth")
           || (command == "reset-encoder-preload")
           )) {
        std::cerr << "Invalid command \"" << command
                  << "\".  Supported commands: `qlacommand -c {open-relays,close-relays,reboot,reset-eth,reset-encoder-preload}`" << std::endl
                  << "Port can be specified using `-p`: -pupd, -pupd:X.X.X.X, -pfw, -pfw:X" << std::endl;
        return -1;
    }

    // if port is not specified
    port = PortFactory(portArgs.c_str());
    if (!port) {
        std::cerr << "Failed to create port using: " << portArgs << std::endl;
        return -1;
    }

    std::vector<AmpIO *> boardList;
    for (i = 0; i < port->GetNumOfNodes(); i++) {
        int boardId = port->GetBoardId(i);
        if (boardId != BoardIO::MAX_BOARDS) {
            std::cout << "Adding firewire node " << i << ", BoardID " << boardId << std::endl;
            AmpIO *board = new AmpIO(boardId);
            boardList.push_back(board);
            port->AddBoard(board);
        }
    }

    // actual command
    for (j = 0; j < boardList.size(); j++) {
        std::cout << "Executing command \"" << command << "\" on node " << j << std::endl;
        if (command == "open-relays") {
            boardList[j]->WriteSafetyRelay(false);
        } else if (command == "close-relays") {
            boardList[j]->WriteSafetyRelay(true);
        } else if (command == "reboot") {
            boardList[j]->WriteReboot();
        } else if (command == "reset-eth") {
            boardList[j]->ResetKSZ8851();
        } else if (command == "reset-encoder-preload") {
            for (i = 0; i < 4; i++) {
                boardList[j]->WriteEncoderPreload(i, 0);
            }
        }
    }

    delete port;

    return 0;
}
