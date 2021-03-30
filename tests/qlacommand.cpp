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

// Following constants are from AmpIO
const quadlet_t REBOOT_FPGA   = 0x00300000;
const quadlet_t RELAY_ON      = 0x00030000;
const quadlet_t RELAY_OFF     = 0x00020000;
const quadlet_t RESET_KSZ8851 = 0x04000000;
const nodeaddr_t ENC_LOAD_OFFSET = 4;

int main(int argc, char** argv)
{
    unsigned int i;
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

    std::cout << "Executing command \"" << command << "\" on " << port->GetNumOfNodes() << " nodes" << std::endl;
    if (command == "open-relays") {
        port->WriteQuadlet(FW_NODE_BROADCAST, 0, RELAY_OFF);
    } else if (command == "close-relays") {
        port->WriteQuadlet(FW_NODE_BROADCAST, 0, RELAY_ON);
    } else if (command == "reboot") {
        port->WriteQuadlet(FW_NODE_BROADCAST, 0, REBOOT_FPGA);
    } else if (command == "reset-eth") {
        port->WriteQuadlet(FW_NODE_BROADCAST, 12, RESET_KSZ8851);
    } else if (command == "reset-encoder-preload") {
        for (i = 0; i < 4; i++) {
            unsigned int channel = (i+1) << 4;
            port->WriteQuadlet(FW_NODE_BROADCAST, channel | ENC_LOAD_OFFSET, AmpIO::GetEncoderMidRange());
        }
    }

    delete port;

    return 0;
}
