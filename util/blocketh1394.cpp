/******************************************************************************
 *
 *
 ******************************************************************************/

#include <sstream>
#include <iostream>
#include <vector>
#include <byteswap.h>
#include <stdlib.h>

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
    nodeaddr_t addr;
    int size;
    quadlet_t data1;
    quadlet_t *data = &data1;
    bool isQuad1394 = true;
    int i,j;
    int bid = BoardIO::MAX_BOARDS;

    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            if (argv[i][1] == 'b') {
                bid = atoi(argv[i]+2);
                std::cout << "Selecting board " << bid << "\n";
            }
        }
        else {
            if (args_found == 0)
                addr = strtoull(argv[i], 0, 16);
            else if ((args_found == 1) && (isQuad1394))
                data1 = strtoul(argv[i], 0, 16);
            else if ((args_found == 1) && (!isQuad1394)) {
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


    BasePort* Port = NULL;
    int port = 0;
    std::stringstream debugStream(std::stringstream::out|std::stringstream::in);

    Port = new Eth1394Port(port, debugStream);
    if (!Port->IsOK()) {
        PrintDebugStream(debugStream);
        std::cerr << "Failed to initialize ethernet port " << port << std::endl;
        return -1;
    }
    Port->SetProtocol(BasePort::PROTOCOL_SEQ_RW);  // PK TEMP

    std::vector<AmpIO*> BoardList;
    BoardList.push_back(new AmpIO(bid));
    Port->AddBoard(BoardList[0]);


    if (args_found == 1)
    {
        if (Port->ReadQuadlet(bid, addr, (*data)))
            std::cout << "0x" << std::hex << data[0] << "\n";
        else
            std::cerr << "ReadQuadlet Failed \n";
    }
    else if (args_found == 2)
    {
        Port->WriteQuadlet(bid, addr, (*data) );
    }

    delete Port;
    return 0;
}
