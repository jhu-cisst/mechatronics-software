/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2011-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <algorithm>
#include <string.h>
#include <byteswap.h>
#include <sys/select.h>
#include <errno.h>   // errno

// firewire
#include "FirewirePort.h"
#include "Amp1394Time.h"
#include <libraw1394/raw1394.h>
#include <libraw1394/csr.h>

FirewirePort::PortListType FirewirePort::PortList;

FirewirePort::FirewirePort(int portNum, std::ostream &debugStream):
    BasePort(portNum, debugStream)
{
    Init();
}

bool FirewirePort::Init(void)
{
    // create firewire port handle
    handle = raw1394_new_handle();
    if (!handle) {
        outStr << "FirewirePort::Init: error, could not create handle" << std::endl;
        return false;
    }

    PortListType::const_iterator it;
    for (it = PortList.begin(); it != PortList.end(); it++) {
        if ((*it)->PortNum == PortNum)
            outStr << "FirewirePort::Init: warning, port " << PortNum
                   << " is already used (be careful about thread safety)" << std::endl;
        if ((*it)->handle == handle) // should never happen
            outStr << "FirewirePort::Init: warning, handle is already used" << std::endl;
    }
    PortList.push_back(this);

    memset(Node2Board, BoardIO::MAX_BOARDS, sizeof(Node2Board));

    // set the bus reset handler
    old_reset_handler = raw1394_set_bus_reset_handler(handle, reset_handler);

    // get number of ports
    int nports = raw1394_get_port_info(handle, NULL, 0);
    outStr << "FirewirePort::Init: number of ports = " << nports << std::endl;
    if (nports > 0) {
        struct raw1394_portinfo *pinfo = new raw1394_portinfo[nports];
        raw1394_get_port_info(handle, pinfo, nports);
        for (int i = 0; i < nports; i++)
            outStr << "  Port " << i << ": " << pinfo[i].name << ", " << pinfo[i].nodes << " nodes" << std::endl;
        delete [] pinfo;
    }
    if (nports < PortNum) {
        outStr << "FirewirePort::Init: port " << PortNum << " does not exist" << std::endl;
        raw1394_destroy_handle(handle);
        handle = NULL;
        return false;
    }

    if (raw1394_set_port(handle, PortNum)) {
        outStr << "FirewirePort::Init: error setting port to " << PortNum << std::endl;
        raw1394_destroy_handle(handle);
        handle = NULL;
        return false;
    }
    outStr << "FirewirePort::Init: successfully initialized port " << PortNum << std::endl;
    outStr << "Using libraw1394 version " << raw1394_get_libversion() << std::endl;

    // stop cycle start packet
    StopCycleStartPacket();

    bool ret = ScanNodes();
    if (ret)
        SetDefaultProtocol();
    return ret;
}

void FirewirePort::StopCycleStartPacket(void)
{
    // IMPORTANT: Disable Cycle Start Packet, no isochronous
    int rc = 0;  // return code
    quadlet_t data_stop_cmc = bswap_32(0x100);
    rc = raw1394_write(handle,
                       raw1394_get_local_id(handle),
                       CSR_REGISTER_BASE + CSR_STATE_CLEAR,
                       4,
                       &data_stop_cmc);
    if (rc) {
        outStr << "FirewirePort::Init: error, can NOT disable cycle start packet" << std::endl;
    } else {
        outStr << "FirewirePort::Init: successfully disabled cycle start packet" << std::endl;
    }
}


FirewirePort::~FirewirePort()
{
    Cleanup();
}

int FirewirePort::NumberOfUsers(void)
{
    // try to count of many users on the handle
    char command[256];
    sprintf(command, "lsof -t /dev/fw%d 2> /dev/null", PortNum);
    FILE * fp;
    int status;
    char path[512];
    fp = popen(command, "r");
    if (fp == NULL) {
        outStr << "FirewirePort::NumberOfUsers: unable to start lsof to count number of users on port " << PortNum << std::endl;
    }
    std::string result;
    int numberOfLines = 0;
    while (fgets(path, 512, fp) != NULL) {
        result.append(path);
        numberOfLines++;
    }
    status = pclose(fp);

    if (status == -1) {
        outStr << "FirewirePort::NumberOfUsers: error in pclose after lsof call" << std::endl;
    }
    int numberOfUsers = numberOfLines - 1; // lsof itself
    if (numberOfUsers > 1) {
        sprintf(command, "lsof -R /dev/fw%d 2> /dev/null", PortNum);
        fp = popen(command, "r");
        if (fp == NULL) {
            outStr << "FirewirePort::NumberOfUsers: unable to start lsof to count number of users on port " << PortNum << std::endl;
        }
        result = "";   // clear result
        while (fgets(path, 512, fp) != NULL) {
            result.append(path);
        }
        status = pclose(fp);

        outStr << "FirewirePort::NumberOfUsers: found " << numberOfUsers << " users" << std::endl
               << "Full lsof result:" << std::endl
               << result << std::endl;
    }
    return numberOfUsers;
}

void FirewirePort::Cleanup(void)
{
    PortListType::iterator it = std::find(PortList.begin(), PortList.end(), this);
    if (it == PortList.end())
        outStr << "FirewirePort::Cleanup: could not find entry for port " << PortNum << std::endl;
    else
        PortList.erase(it);
    raw1394_destroy_handle(handle);
    handle = NULL;
}

unsigned int FirewirePort::GetBusGeneration(void) const
{
    if (FwBusGeneration != raw1394_get_generation(handle))
        outStr << "FirewirePort::GetBusGeneration mismatch: " << FwBusGeneration
               << ", " << raw1394_get_generation(handle) << std::endl;
    return FwBusGeneration;
}

void FirewirePort::UpdateBusGeneration(unsigned int gen)
{
    raw1394_update_generation(handle, gen);
    FwBusGeneration = gen;
}

int FirewirePort::reset_handler(raw1394handle_t hdl, uint gen)
{
    int ret = 0;
    PortListType::iterator it;
    for (it = PortList.begin(); it != PortList.end(); it++) {
        if ((*it)->handle == hdl) {
            (*it)->outStr << "FirewirePort::reset_handler: generation = " << gen << std::endl;
            // Call default handler, which calls raw1394_update_generation
            // ret = (*it)->old_reset_handler(hdl, gen);
            (*it)->newFwBusGeneration = gen;
            break;
        }
    }
    if (it == PortList.end())
        std::cerr << "FirewirePort::reset_handler: could not find port" << std::endl;
    return ret;
}

// This function checks if there is a pending read on the IEEE-1394 bus,
// which could include a bus reset
void FirewirePort::PollEvents(void)
{
    fd_set fds;
    struct timeval timeout = {0, 0};

    int fd = raw1394_get_fd(handle);
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    int ret = select(fd+1, &fds, 0, 0, &timeout);
    if (ret > 0)
        raw1394_loop_iterate(handle);
}

nodeid_t FirewirePort::InitNodes(void)
{
    // Get base node id (zero out 6 lsb)
    baseNodeId = raw1394_get_local_id(handle) & 0xFFC0;
    outStr << "FirewirePort::InitNodes: base node id = " << std::hex << baseNodeId << std::dec << std::endl;

    // Get Firewire bus generation
    FwBusGeneration = raw1394_get_generation(handle);
    newFwBusGeneration = FwBusGeneration;

    // Get total number of nodes on bus
    int numNodes = raw1394_get_nodecount(handle);
    // Subtract 1 for PC which, as bus master, is always the highest number
    return (numNodes-1);
}


bool FirewirePort::AddBoard(BoardIO *board)
{
    bool ret = BasePort::AddBoard(board);
    if (ret) {
        unsigned int id = board->BoardId;
        HubBoard = id;  // last added board would be hub board
    }
    return ret;
}

bool FirewirePort::RemoveBoard(unsigned char boardId)
{
    return BasePort::RemoveBoard(boardId);
}

bool FirewirePort::WriteBroadcastOutput(quadlet_t *buffer, unsigned int size)
{
#if 1
    return WriteBlockNode(0, 0xffffffff0000, buffer, size);
#else
    return WriteBlockNode(FW_NODE_BROADCAST, 0xffffff000000,  // now the address is hardcoded
                          buffer, size);
#endif
}

bool FirewirePort::WriteBroadcastReadRequest(unsigned int seq)
{
    quadlet_t bcReqData = (seq << 16);
    if (IsAllBoardsRev4_5_ || IsBroadcastShorterWait())
        bcReqData += BoardInUseMask_;
    else
        // For a mixed set of boards, disable the shorter wait capability by "tricking" the system into thinking
        // that all nodes are used in this configuration. Otherwise, boards with the shorter wait capability may
        // attempt to transmit at the same time as boards that do not have this capability. The FireWire bus
        // arbitration protocol should prevent disaster, but this could lead to lower efficiency.
        bcReqData += ((1 << NumOfNodes_)-1);

    nodeaddr_t bcReqAddr;
    if (IsAllBoardsRev4_6_)
        bcReqAddr = 0xffffffff000f;      // special address to trigger broadcast read
    else
        bcReqAddr = 0x1800;              // special address to trigger broadcast read

#if 1
    WriteQuadletNode(0, bcReqAddr, bcReqData);
    // WriteQuadletNode returns false because the "fake" broadcast packet to node 0 does not
    // return an ACK.
    return true;
#else
    return WriteQuadlet(FW_NODE_BROADCAST, bcReqAddr, bcReqData);
#endif
}

void FirewirePort::WaitBroadcastRead(void)
{
    // Wait for all boards to respond with data
    // Shorter wait: 10 + 5 * Nb us, where Nb is number of boards used in this configuration
    // Standard wait: 5 + 5 * Nn us, where Nn is the total number of nodes on the FireWire bus
    double waitTime_uS = IsBroadcastShorterWait() ? (10.0 + 5.0*NumOfBoards_) : (5.0 + 5.0*NumOfNodes_);
    Amp1394_Sleep(waitTime_uS*1e-6);
}

void FirewirePort::OnNoneRead(void)
{
    PollEvents();
}

void FirewirePort::OnNoneWritten(void)
{
    PollEvents();
}

bool FirewirePort::ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char)
{
    if (!CheckFwBusGeneration("FirewirePort::ReadQuadlet"))
        return false;

    bool ret = !raw1394_read(handle, baseNodeId+node, addr, 4, &data);
    if (ret)
        data = bswap_32(data);
    return ret;
}

bool FirewirePort::WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char)
{
    if (!CheckFwBusGeneration("FirewirePort::WriteQuadlet"))
        return false;

    data = bswap_32(data);
    bool ret = !raw1394_write(handle, baseNodeId+node, addr, 4, &data);

    // Workaround for Firewire broadcast, which is not currently working due to an issue with
    // either libraw1394 or the Firewire JuJu driver.
    if (!ret && (node == FW_NODE_BROADCAST)) {
        ret = true;
        for (nodeid_t curNode = 0; curNode < NumOfNodes_; curNode++) {
            if (raw1394_write(handle, baseNodeId+curNode, addr, 4, &data))
                ret = false;
        }
    }

    return ret;
}

bool FirewirePort::ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata,
                                 unsigned int nbytes, unsigned char)
{
    if (!CheckFwBusGeneration("FirewirePort::ReadBlock"))
        return false;

    rtRead = true;   // for debugging
    return !raw1394_read(handle, baseNodeId+node, addr, nbytes, rdata);
}

bool FirewirePort::WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata,
                                  unsigned int nbytes, unsigned char)
{
    if (!CheckFwBusGeneration("FirewirePort::WriteBlock"))
        return false;

    rtWrite = true;   // for debugging
    return !raw1394_write(handle, baseNodeId+node, addr, nbytes, wdata);
}
