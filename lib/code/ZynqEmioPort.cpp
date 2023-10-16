/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides

  (C) Copyright 2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "ZynqEmioPort.h"
#include "fpgav3_emio.h"

ZynqEmioPort::ZynqEmioPort(int portNum, std::ostream &debugStream):
    BasePort(portNum, debugStream), emio(0)
{
    Init();
}

bool ZynqEmioPort::Init(void)
{
    memset(Node2Board, BoardIO::MAX_BOARDS, sizeof(Node2Board));

    // Initialize EMIO interface to FPGA
    emio = EMIO_Init();

    // Turn off verbose output
    // EMIO_SetVerbose(emio, false);

    bool ret = ScanNodes();
    if (ret)
        SetDefaultProtocol();
    return ret;
}

ZynqEmioPort::~ZynqEmioPort()
{
    Cleanup();
}

void ZynqEmioPort::Cleanup(void)
{
    EMIO_Release(emio);
    emio = 0;
}

unsigned int ZynqEmioPort::GetBusGeneration(void) const
{
    return FwBusGeneration;
}

void ZynqEmioPort::UpdateBusGeneration(unsigned int gen)
{
    FwBusGeneration = gen;
}

nodeid_t ZynqEmioPort::InitNodes(void)
{
    return 1;
}

bool ZynqEmioPort::AddBoard(BoardIO *board)
{
    bool ret = BasePort::AddBoard(board);
    if (ret) {
        unsigned int id = board->GetBoardId();
        HubBoard = id;  // last added board would be hub board
    }
    return ret;
}

bool ZynqEmioPort::RemoveBoard(unsigned char boardId)
{
    return BasePort::RemoveBoard(boardId);
}

bool ZynqEmioPort::WriteBroadcastOutput(quadlet_t *buffer, unsigned int size)
{
    return WriteBlockNode(FW_NODE_BROADCAST, 0, buffer, size);
}

bool ZynqEmioPort::WriteBroadcastReadRequest(unsigned int seq)
{
    quadlet_t bcReqData = (seq << 16) | BoardInUseMask_;
    return WriteQuadlet(FW_NODE_BROADCAST, 0x1800, bcReqData);
}

// Node number should always be 0 (or broadcast) for EMIO
bool ZynqEmioPort::CheckNodeId(nodeid_t node) const
{
    return ((node == 0) || (node == FW_NODE_BROADCAST));
}

bool ZynqEmioPort::ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char)
{
    if (!CheckNodeId(node)) {
        outStr << "ReadQuadletNode: invalid node " << node << std::endl;
        return false;
    }

    return EMIO_ReadQuadlet(emio, addr, &data);
}

bool ZynqEmioPort::WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char)
{
    if (!CheckNodeId(node)) {
        outStr << "WriteQuadletNode: invalid node " << node << std::endl;
        return false;
    }

    return EMIO_WriteQuadlet(emio, addr, data);
}

bool ZynqEmioPort::ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata,
                                 unsigned int nbytes, unsigned char)
{
    if (!CheckNodeId(node)) {
        outStr << "ReadBlockNode: invalid node " << node << std::endl;
        return false;
    }

    rtRead = true;   // for debugging
    return EMIO_ReadBlock(emio, addr, rdata, nbytes);
}

bool ZynqEmioPort::WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata,
                                  unsigned int nbytes, unsigned char)
{
    if (!CheckNodeId(node)) {
        outStr << "WriteBlockNode: invalid node " << node << std::endl;
        return false;
    }

    rtWrite = true;   // for debugging
    return EMIO_WriteBlock(emio, addr, wdata, nbytes);
}
