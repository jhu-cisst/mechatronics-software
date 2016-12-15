/******************************************************************************
 *
 *
 ******************************************************************************/

#include <sstream>
#include <iostream>
#include <vector>
#include <byteswap.h>
#include <stdlib.h>
#include <stdio.h>

#include <Eth1394Port.h>
#include <AmpIO.h>

void PrintDebugStream(std::stringstream &debugStream)
{
    char line[80];
    while (!debugStream.eof()) {
        debugStream.getline(line, sizeof(line));
        std::cerr << line << std::endl;
    }
    debugStream.clear();
    debugStream.str("");
}

int main(int argc, char** argv)
{
    int args_found = 0;
    nodeaddr_t addr = 0x0;
    int size = 1;
    quadlet_t data1;
    quadlet_t *data = &data1;
    bool isQuad1394 = (strstr(argv[0], "quadeth1394") != 0);

    int i,j;
    int bid = BoardIO::MAX_BOARDS;
    bool verbose = false;

    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'b') {
                bid = atoi(argv[i]+2);
                std::cout << "Selecting board " << bid << "\n";
            }
            else if (argv[i][1] == 'v') {
                verbose = true;
            }
        }
        else {
            if (args_found == 0)
                addr = strtoull(argv[i], 0, 16);
            else if ((args_found == 1) && (isQuad1394))
                data1 = strtoul(argv[i], 0, 16);
            else if ((args_found == 1) && (!isQuad1394)) {
                std::cout << "" << std::endl;
                size = strtoul(argv[i], 0, 10);
                /* Allocate data array, initializing contents to 0 */
                data = (quadlet_t *) calloc(sizeof(quadlet_t), size);
                if (!data) {
                    std::cerr << "Failed to allocate memory for " << size  << " quadlets\n";
                    exit(-1);
                }
            }
            else if (!isQuad1394 && (j < size))
                data[j++] = bswap_32(strtoul(argv[i], 0, 16));
            else
                std::cerr << "Warning: extra parameter: " << argv[i] << "\n";

            args_found++;
        }
    }

    if (args_found < 1) {
        if (isQuad1394)
            printf("Usage: %s [-pP] [-nN] [-v] <address in hex> [value to write in hex]\n", argv[0]);
        else
            printf("Usage: %s [-pP] [-nN] [-v] <address in hex> <size in quadlets> [write data quadlets in hex]\n", argv[0]);
        printf("       where P = port number, N = node number\n");
        exit(0);
    }


    BasePort* Port = NULL;
    int port = 0;
    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);

    Port = new Eth1394Port(port, debugStream);
    if (!Port->IsOK()) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to initialize ethernet port " << port << std::endl;
        return -1;
    }
    else if (verbose) {
        PrintDebugStream(debugStream);
    }
    Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW);  // PK TEMP

    std::vector<AmpIO*> BoardList;
    BoardList.push_back(new AmpIO(bid));
    Port->AddBoard(BoardList[0]);


    // Quadlet R/W
    if (isQuad1394 && (args_found == 1))
    {
        if (Port->ReadQuadlet(bid, addr, (*data)))
            std::cout << "0x" << std::hex << data[0] << "\n";
        else
            std::cerr << "ReadQuadlet Failed \n";
    }
    else if (isQuad1394 && (args_found == 2))
    {
        std::cout << "WriteQuadlet\n";
        if (!Port->WriteQuadlet(bid, addr, (*data) )) {
            std::cerr << "WriteQuadlet Failed\n";
        }
    }

    // Block R/W
    else if (!isQuad1394 && (args_found <= 2))
    {
        if (Port->ReadBlock(bid, addr, data, size * 4)) {
            for (j=0; j<size; j++)
                printf("0x%08X\n", bswap_32(data[j]));
        }
        else
            std::cerr << "ReadBlock Failed \n";
    }
    else
    {
        if (!Port->WriteBlock(bid, addr, data, size * 4)) {
            std::cerr << "WriteBlock Failed \n";
        }
    }

    delete Port;
    return 0;
}

