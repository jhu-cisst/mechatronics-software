/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015

  (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

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
    int port = 0;;

    for (i = 1; i < (unsigned int)argc; i++) {
        if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
            port = atoi(argv[i]+2);
        }
    }

    std::cout << "Using firewire port " << port << " (to change port, use -p)" << std::endl;

    FirewirePort Port(port, std::cout);
    if (!Port.IsOK()) {
        std::cerr << "Failed to initialize firewire port " << port << std::endl;
        return -1;
    }

    std::vector<AmpIO *> BoardList;
    unsigned char * node2board = Port.GetNode2Board();
    for (i = 0; i < Port.GetNumOfNodes(); i++) {
        std::cout << "Adding firewire Node " << i << ", BoardID " << static_cast<int>(node2board[i]) << std::endl;
        BoardList.push_back(new AmpIO(node2board[i]));
        Port.AddBoard(BoardList[i]);
    }

    for (j = 0; j < BoardList.size(); j++) {
        BoardList[j]->WriteSafetyRelay(true);
        usleep(40000); // sleep 40 ms
    }

    std::cout << "Safety relays closed" << std::endl;
    return 0;
}
