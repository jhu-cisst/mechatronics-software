/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2016 Johns Hopkins University (JHU), All Rights Reserved.

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
    typedef bool (*Eth1394CallbackType)(Eth1394Port &port, unsigned char boardId, std::ostream &debugStream);

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
    uint8_t frame_hdr[14];  // dest addr (6), src addr (6), length (2)
    uint8_t BidBridge_; // bridge board ID

    uint8_t fw_tl;          // FireWire transaction label (6 bits)

    unsigned char Node2Board[BoardIO::MAX_BOARDS];
    unsigned char Board2Node[BoardIO::MAX_BOARDS];
    unsigned long FirmwareVersion[BoardIO::MAX_BOARDS];

    int NumOfNodes_;    // number of nodes exist
    int NumOfNodesInUse_;   //number of nodes under control
    bool BoardExistMask_[BoardIO::MAX_BOARDS];  // Boards connected in the network
    bool BoardInUseMask_[BoardIO::MAX_BOARDS];  // Boards under control of PC
    uint32_t BoardInUseInteger_;

    Eth1394CallbackType eth1394_read_callback;

    bool headercheck(uint8_t* header, bool isHUBtoPC) const;
    bool checkCRC(const unsigned char *packet) const;

    /**
     * @brief send async read request to a node and wait for response.
     *
     * @param node: target node ID
     * @param addr: address to read from
     * @param length: amount of bytes of data to read
     * @param buffer: pointer to buffer where data will be saved
     *
     * @return int 0 on success, -1 on failure
     */
    int eth1394_read(nodeid_t node, nodeaddr_t addr, size_t length, quadlet_t* buffer);

    /**
     * \brief Write FireWire packet via Ethernet
     *
     * \param node        destination board number
     * \param buffer      address of pre-allocated buffer (includes space for Ethernet header)
     * \param length_fw   length of FireWire packet, in quadlets
     *
     * \return true on success; false otherwise
     */
    bool eth1394_write(nodeid_t node, quadlet_t* buffer, size_t length_fw);

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


    //! Initialize Eth1394 port
    bool Init(void);

    //! Look for nodes on the bus
    bool ScanNodes(void);

public:
    Eth1394Port(int portNum, std::ostream &debugStream = std::cerr, Eth1394CallbackType cb = 0);

    ~Eth1394Port();

    // Returns the destination MAC address (6 bytes)
    // The first 3 characters are FA:61:OE, which is the CID assigned to LCSR by IEEE
    // The next 2 characters are 13:94
    // The last character is 0 (set it to the board address)
    static void GetDestMacAddr(unsigned char *macAddr);

    // Returns the destination multicast MAC address (6 bytes)
    // The first 3 characters are FB:61:OE, which is the CID assigned to LCSR by IEEE, with the multicast bit set
    // The last 3 characters are 13:94:FF
    static void GetDestMulticastMacAddr(unsigned char *macAddr);

    static void PrintDebug(std::ostream &debugStream, unsigned short status);

    // For now, always returns 1
    int NumberOfUsers(void) { return 1; }

    void BoardInUseIntegerUpdate(void);

    void SetEth1394Callback(Eth1394CallbackType callback)
    { eth1394_read_callback = callback; }

    bool IsOK(void);

    void Reset(void);

    /*!
     \brief This is a dummy function, returning boardID directly in
            Eth1394 mode.

     \param boardId
     \return int
    */
    int GetNodeId(unsigned char boardId) const;


    /*!
     \brief Get board firmware version

     \param boardId: board ID (rotary switch value)
     \return unsigned long: firmware version number
    */
    unsigned long GetFirmwareVersion(unsigned char boardId) const;

    /*!
     \brief Add board to Eth1394 port

     \param board: Board ID (rotary switch value)
     \return bool true on success or false on failure
    */
    bool AddBoard(BoardIO *board);

    /*!
     \brief Remove board

     \param boardId Board ID (rotary switch value)
     \return bool true: success false: fail
    */
    bool RemoveBoard(unsigned char boardId);

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
    bool ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata,
                   unsigned int nbytes);

    // Write a block to the specified board
    bool WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata,
                    unsigned int nbytes);

    /*!
     \brief Write a block of data using asynchronous broadcast

     \param addr  The starting target address, should larger than CSR_REG_BASE + CSR_CONFIG_END
     \param wdata  The pointer to write buffer data
     \param nbytes  Number of bytes to be broadcasted
     \return bool  True on success or False on failure
    */
    bool WriteBlockBroadcast(nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes);

    /*!
     \brief Add delay (if needed) for PROM I/O operations
     The delay is non-zero for Ethernet.
    */
    void PromDelay(void) const;

    /*!
     \brief helper function to create 1394 packet header

     \param packet: firewire packet
     \param node: target node ID
     \param addr: address to R/W to
     \param tcode: 1394 transaction code
     \param tl: transaction label
    */
    static void make_1394_header(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tcode,
                                 unsigned int tl);
};

#endif  // __Eth1394Port_H__
