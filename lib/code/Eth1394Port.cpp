/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Zihan Chen

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "Eth1394Port.h"
#include <byteswap.h>

const unsigned long QLA1_String = 0x514C4131;
const unsigned long BOARD_ID_MASK    = 0x0f000000;  /*!< Mask for board_id */


bool Eth1394Port::Init()
{
    // Put Ethernet initialization here

    //setting the device
    dev = pcap_lookupdev(errbuf);
    if (dev == NULL)
        outStr <<"ERROR: Couldn't find dfault device: "<<errbuf<<std::endl;
    else
        outStr << "Device: " << dev << std::endl;

    handle = NULL;
    handle = pcap_open_live(dev, BUFSIZ, 1, 100, errbuf);
    if(handle == NULL)
    {
        outStr <<"ERROR: Couldn't open device: "<<dev<<std::endl;
        return false;
    }

    // initialize ethernet header
    byte_t eth_dst[6] = {0x50,0x43,0x3e,0x48,0x55,0x42};
    byte_t eth_src[6] = {0x4c,0x43,0x53,0x52,0x00,0x00};
    memcpy(frame_hdr, eth_dst, 6);
    memcpy(&frame_hdr[3], eth_src, 6);
    frame_hdr[6] = bswap_16(0x0801);

    return this->ScanNodes();
}

bool Eth1394Port::ScanNodes()
{
    // get number of boards

#if 0
    EthPcapNode node;
    quadlet_t readBuffer[32];
    node.ethpcap_write(3, 0x03, 4, readBuffer);

    int num_node = 0;//1~15
    for(int i=0;i<15;i++)
    {
        if(node.ethpcap_read(i, 0x00, 4, readBuffer) == 1)
        {
            num_node ++;
        }
    }
    std::cout<<"Num of nodes: "<<num_node<<std::endl;
    node.ethpcap_write_nodenum(num_node);
#endif

    // read each boards
    //  - read firmware version
    //  - set boardid (list or something)

    NumOfNodes_ = 0;
    for (size_t node = 0; node < Eth1394Port::MAX_NODES; node++)
    {
        quadlet_t data;
        if (eth1394_read(node, 0x04, 4, &data))
            continue;   // continue on error

        // check hardware version
        if (data != QLA1_String) {
            outStr << "Node " << node << " is not a QLA board" << std::endl;
            continue;
        }

        // read firmware version
        unsigned long fver = 0;
        if (eth1394_read(node, 0x07, 4, &data)) {
            outStr << "ScanNodes: unable to read firmware version from node "
                   << node << std::endl;
            return false;
        }
        fver = data;

        // verify boardid
        unsigned long board = 0;
        if (eth1394_read(node, 0x00, 4, &data)) {
            outStr << "ScanNodes: unable to read status from node "
                   << node << std::endl;
            return false;
        }
        // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
        board = (data & BOARD_ID_MASK) >> 24;
        outStr << "  BoardId = " << board
               << ", Firmware Version = " << fver << std::endl;

        //        Node2Board[node] = board;
        //        FirmwareVersion[board] = fver;
        NumOfNodes_++;
    }

    if (NumOfNodes_ == 0) {
        outStr << "No FPGA boards connected" << std::endl;
        return false;
    }

    // wirte num of nodes to eth1394 FPGA
    outStr <<"Num of nodes: "<< NumOfNodes_ <<std::endl;
    eth1394_write_nodenum(NumOfNodes_);

    return true;
}



bool Eth1394Port::ReadQuadlet(unsigned char boardId,
                              nodeaddr_t addr, quadlet_t &data)
{
    if (boardId < MAX_NODES)
        return true;
    else
        return false;
}


bool Eth1394Port::WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data)
{
    if (boardId < MAX_NODES)
        return true;
    else
        return false;
}
