/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides

  (C) Copyright 2023-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __ZynqEmioPort_H__
#define __ZynqEmioPort_H__

#include <ostream>
#include "BoardIO.h"
#include "BasePort.h"
#include "fpgav3_emio.h"

class ZynqEmioPort : public BasePort {
public:
protected:

    EMIO_Interface *emio;

    // Internal method to check whether node id is valid
    // (should be 0 or FW_NODE_BROADCAST)
    bool CheckNodeId(nodeid_t node) const;

    //! Read quadlet from node (internal method called by ReadQuadlet).
    //  Parameter "flags" is not used for EMIO
    bool ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags = 0);

    //! Write quadlet to node (internal method called by WriteQuadlet)
    //  Parameter "flags" is not used for EMIO
    bool WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags = 0);

    // Initialize EMIO port
    bool Init(void);

    // Cleanup EMIO port
    void Cleanup(void);

    // Initialize nodes on the bus; called by ScanNodes
    // \return Maximum number of nodes on bus (0 if error)
    nodeid_t InitNodes(void);

    // Write the broadcast packet containing the DAC values and power control
    bool WriteBroadcastOutput(quadlet_t *buffer, unsigned int size);

    // Write a block to the specified node. Internal method called by WriteBlock and
    // WriteAllBoardsBroadcast.
    // Parameter "flags" is not used for EMIO.
    bool WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata,
                        unsigned int nbytes, unsigned char flags = 0);

    // Read a block from the specified node. Internal method called by ReadBlock.
    // Parameter "flags" is not used for EMIO.
    bool ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata,
                       unsigned int nbytes, unsigned char flags = 0);

public:
    // Initialize Zynq EMIO port
    //
    // There are two available interfaces, one using mmap for direct register access,
    // and one using the gpiod driver. The mmap interface is much faster and therefore
    // is the default. To use the gpiod interface, set portNum to 1.
    ZynqEmioPort(int portNum = 0, std::ostream &debugStream = std::cerr);
    ~ZynqEmioPort();

    //****************** BasePort pure virtual methods ***********************

    PortType GetPortType(void) const { return PORT_ZYNQ_EMIO; }

    int NumberOfUsers(void) { return 1; }

    bool IsOK(void) { return emio->IsOK(); }

    unsigned int GetBusGeneration(void) const;

    void UpdateBusGeneration(unsigned int gen);

    unsigned int GetPrefixOffset(MsgType) const   { return 0; }
    unsigned int GetWritePostfixSize(void) const  { return 0; }
    unsigned int GetReadPrefixSize(void) const    { return 0; }
    unsigned int GetReadPostfixSize(void) const   { return 0; }

    unsigned int GetWriteQuadAlign(void) const    { return 0; }
    unsigned int GetReadQuadAlign(void) const     { return 0; }

    // Get the maximum number of data bytes that can be read
    // (via ReadBlock) or written (via WriteBlock).
    unsigned int GetMaxReadDataSize(void) const  { return MAX_POSSIBLE_DATA_SIZE; }
    unsigned int GetMaxWriteDataSize(void) const { return MAX_POSSIBLE_DATA_SIZE; }

    // Adds board(s)
    bool AddBoard(BoardIO *board);

    // Removes board
    bool RemoveBoard(unsigned char boardId);

    /*!
     \brief Write the broadcast read request
    */
    bool WriteBroadcastReadRequest(unsigned int seq);

    /*!
     \brief Wait for broadcast read data to be available
    */
    void WaitBroadcastRead(void) {}

    /*!
     \brief Add delay (if needed) for PROM I/O operations
    */
    void PromDelay(void) const;

};

#endif // __ZynqEmioPort_H__
