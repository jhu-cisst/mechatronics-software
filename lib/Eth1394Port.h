/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen

  (C) Copyright 2014-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef __Eth1394Port_H__
#define __Eth1394Port_H__

#include <iostream>
#include "BasePort.h"

// Forward declaration
struct pcap;
typedef struct pcap pcap_t;

class Eth1394Port : public BasePort
{
public:
    //enum { MAX_NODES = 15 };  // max number of boards, limited by rotary switch

protected:

    //! FireWire tcode
    enum TCODE {
        QWRITE = 0,
        BWRITE = 1,
        QREAD = 4,
        BREAD = 5,
        QRESPONSE = 6,
        BRESPONSE = 7
    };

    pcap_t *handle;
    uint16_t frame_hdr[7];

    int NumOfNodes_;    // number of nodes exist
    int NumOfNodesInUse_;   //number of nodes under control
    bool BoardExistMask_[BoardIO::MAX_BOARDS];  // Boards connected in the network
    bool BoardInUseMask_[BoardIO::MAX_BOARDS];  // Boards under control of PC
    uint32_t BoardInUseInteger_;

    // read FPGA node

    /**
     * @brief
     *
     * @param node
     * @param addr
     * @param length
     * @param buffer
     *
     * @return int 0 on success, -1 on failure
     */
    int eth1394_read(nodeid_t node, nodeaddr_t addr, size_t length, quadlet_t* buffer);

    /**
     * @brief
     *
     * @param node
     * @param addr
     * @param length
     * @param buffer
     *
     * @return int 0 on success, -1 on failure
     */
    int eth1394_write(nodeid_t node, nodeaddr_t addr, size_t length, quadlet_t* buffer);

    /*!
     \brief write number of FPGA boards on the bus

     \return int return 0 on success, -1 on failure
    */
    int eth1394_write_nodenum(void);


    /*!
     \brief
     \param mode  1: firewire mode, 0: eth1394 mode
     \return int  return 0 on success, -1 otherwise
    */
    int eth1394_write_nodeidmode(int mode);


    // Initialize Firewire port
    bool Init(void);

    // Look for nodes on the bus
    bool ScanNodes(void);

public:
    Eth1394Port(int portNum, std::ostream &debugStream = std::cerr);

    void BoardInUseIntegerUpdate(void);

    bool IsOK(void);

    void Reset(void);

    /*!
     \brief This is a dummy function, returning boardID directly in
            Eth1394 mode.

     \param boardId
     \return int
    */
    int GetNodeId(unsigned char boardId) const;


    unsigned long GetFirmwareVersion(unsigned char boardId) const;

    // Adds board(s)
    bool AddBoard(BoardIO *board);

    /*!
     \brief Remove board

     \param boardId Board ID (rotary switch value)
     \return bool true: success false: fail
    */
    bool RemoveBoard(unsigned char boardId);

    // Set UseBroadcast_
    void SetUseBroadcastFlag(bool bc = false);

    // Read all boards
    bool ReadAllBoards(void);

    // Read all boards broadcasting
    virtual bool ReadAllBoardsBroadcast(void);

    // Write to all boards
    bool WriteAllBoards(void);

    // Write to all boards using broadcasting
    virtual bool WriteAllBoardsBroadcast(void);

    // Read a quadlet from the specified board
    bool ReadQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t &data);

    // Write a quadlet to the specified board
    bool WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data);

    //
    /*!
     \brief Write a quadlet to all boards using broadcasting

     \param addr  The register address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param data  The quadlet data to be broadcasted
     \return bool  True on success or False on failure
    */
    bool WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data);

    // Read a block from the specified board
    bool ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data,
                   unsigned int nbytes);

    // Write a block to the specified board
    bool WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data,
                    unsigned int nbytes);

    /*!
     \brief Write a block of data using asynchronous broadcast

     \param addr  The starting target address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param data  The pointer to write buffer data
     \param nbytes  Number of bytes to be broadcasted
     \return bool  True on success or False on failure
    */
    bool WriteBlockBroadcast(nodeaddr_t addr, quadlet_t *data, unsigned int nbytes);
};

#endif  // __Eth1394Port_H__
