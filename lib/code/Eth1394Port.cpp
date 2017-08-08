/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "Eth1394Port.h"
#include "Amp1394Time.h"

#include <pcap.h>
#include <iomanip>
#include <sstream>

#ifndef PCAP_NETMASK_UNKNOWN
// Some versions of pcap.h do not define this
#define PCAP_NETMASK_UNKNOWN    0xffffffff
#endif

#ifdef _MSC_VER
#include <memory.h>   // for memcpy
#include <stdlib.h>   // for byteswap functions
#include <Packet32.h> // winpcap include for PacketRequest
#include <ntddndis.h> // for OID_802_3_CURRENT_ADDRESS
inline uint16_t bswap_16(uint16_t data) { return _byteswap_ushort(data); }
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <byteswap.h>
#endif

const unsigned int ETH_ALIGN32 = 2;          // Number of extra bytes for 32-bit alignment
const unsigned int ETH_HEADER_LEN = 14;      // Number of bytes in Ethernet header
const unsigned int FW_QWRITE_HEADER = 3*4;   // Number of bytes in Firewire quadlet write header
const unsigned int FW_BWRITE_HEADER = 5*4;   // Number of bytes in Firewire block write header
const unsigned int FW_CRC = 4;               // Number of bytes in Firewire CRC
const unsigned long QLA1_String = 0x514C4131;
const unsigned long BOARD_ID_MASK    = 0x0f000000;  /*!< Mask for board_id */

#define DEBUG 0

// crc related
uint32_t BitReverse32(uint32_t input);
uint32_t crc32(uint32_t crc, const void *buf, size_t size);

/*!
 \brief Validate ethernet packet header

 \param header Ethernet packet header (14 bytes)
 \param isHUBtoPC  true: check HUB to PC, false: check PC to HUB
 \return bool true if valid
*/
void print_frame(unsigned char* buffer, int length);
void print_mac(std::ostream &outStr, const char* name, const uint8_t *addr);


Eth1394Port::Eth1394Port(int portNum, std::ostream &debugStream, Eth1394CallbackType cb):
    BasePort(portNum, debugStream),
    BidBridge_(0xFF),
    fw_tl(0),
    NumOfNodes_(0),
    NumOfNodesInUse_(0),
    eth1394_read_callback(cb)
{
    if (Init())
//        eth1394_write_nodeidmode(1);
//        eth1394_write_nodeidmode(0);
        outStr << "Initialization done" << std::endl;
    else
        outStr << "Initialization failed" << std::endl;
}

Eth1394Port::~Eth1394Port()
{
}

void Eth1394Port::GetDestMacAddr(unsigned char *macAddr)
{
    // CID,0x1394,boardid(0)
    macAddr[0] = 0xFA;
    macAddr[1] = 0x61;
    macAddr[2] = 0x0E;
    macAddr[3] = 0x13;
    macAddr[4] = 0x94;
    macAddr[5] = 0x00;
}

void Eth1394Port::GetDestMulticastMacAddr(unsigned char *macAddr)
{
    // Same as GetDestMacAddr, except with multicast bit set and
    // last byte set to 0xFF.
    GetDestMacAddr(macAddr);
    macAddr[0] |= 0x01;
    macAddr[5] = 0xFF;
}

void Eth1394Port::PrintDebug(std::ostream &debugStream, unsigned short status)
{
    debugStream << "Status: ";
    if (status&0x4000) debugStream << "error ";
    if (status&0x2000) debugStream << "initOK ";
    if (status&0x1000) debugStream << "initReq ";
    if (status&0x0800) debugStream << "ethIoErr ";
    if (status&0x0400) debugStream << "PacketErr ";
    if (status&0x0200) debugStream << "DestErr ";
    //if (status&0x0400) debugStream << "cmdReq ";
    //if (status&0x0200) debugStream << "cmdAck ";
    //if (status&0x0400) debugStream << "local ";
    //if (status&0x0200) debugStream << "remote ";
    if (status&0x0100) debugStream << "qRead ";
    if (status&0x0080) debugStream << "qWrite ";
    if (status&0x0040) debugStream << "bRead ";
    if (status&0x0020) debugStream << "bWrite ";
    //if (status&0x0020) debugStream << "PME ";
    //if (!(status&0x0010)) debugStream << "IRQ ";
    if ((status&0x0010)) debugStream << "multicast ";
    if (status&0x0008) debugStream << "KSZ-idle ";
    if (status&0x0004) debugStream << "ETH-idle ";
    int waitInfo = status&0x0003;
    if (waitInfo == 0) debugStream << "wait-none";
    else if (waitInfo == 1) debugStream << "wait-ack";
    else if (waitInfo == 2) debugStream << "wait-ack-clear";
    else debugStream << "wait-flush";
    debugStream << std::endl;
}

void Eth1394Port::BoardInUseIntegerUpdate(void)
{
    uint32_t tempBoardInUse = 0;
    for (int i = 0; i < 16; i++)
    {
        if(BoardInUseMask_[i])
            tempBoardInUse = tempBoardInUse | (0x1 << i);
    }
    BoardInUseInteger_ = tempBoardInUse;
}

bool Eth1394Port::Init(void)
{
    for (size_t i = 0; i < BoardIO::MAX_BOARDS; i++) {
        BoardExistMask_[i] = false;
        BoardInUseMask_[i] = false;
    }

    if (PortNum < 0) {
        outStr << "Invalid Port Number" << std::endl;
        return false;
    }

    // Put Ethernet initialization here
    pcap_if_t* alldevs;
    char errbuf[PCAP_ERRBUF_SIZE];
    if ((pcap_findalldevs(&alldevs, errbuf)) == -1) {
        outStr << "ERROR: " << errbuf <<std::endl;
        return false;
    }
    if (alldevs == NULL) {
        outStr << "No devices -- perhaps need root privileges?" << std::endl;
        return false;
    }

    pcap_if_t* dev = alldevs;
    for (int i = 0; i < PortNum; i++) {
        dev = dev->next;
        if (dev == NULL) break;
    }

    if (dev == NULL) {
        outStr << "Invalid Port Number, device does not exist" << std::endl;

        // Print all devices
        size_t i;
        for(dev= alldevs, i = 0; dev != NULL; dev= dev->next, i++)
        {
            outStr << i << " " << dev->name << std::endl;
        }
        return false;
    }

    // Open pcap handle
    handle = NULL;
    handle = pcap_open_live(dev->name,
                            BUFSIZ,  // data buffer size
                            0,       // turn off promisc mode
                            1,       // read timeout 1 ms
                            errbuf); // error buffer
    if(handle == NULL)
    {
        outStr << "ERROR: Couldn't open device: "<< dev->name <<std::endl;
        return false;
    }

    // initialize ethernet header (FA-61-OE is CID assigned to LCSR by IEEE)
    u_int8_t eth_dst[6];
    GetDestMacAddr(eth_dst);
    u_int8_t eth_src[6];   // Ethernet source address (local MAC address, see below)

    // Get local MAC address. There doesn't seem to be a better (portable) way to do this.
#ifdef _MSC_VER
    // On Windows, we can use the PacketRequest function provided by winpcap. This requires
    // inclusion of Packet32.h and linking with Packet.lib.
    // Other alternatives are SendARP and GetAdaptersAddresses. The latter is a replacement
    // for the deprecated GetAdaptersInfo.
    LPADAPTER adapter = PacketOpenAdapter(dev->name);
    if (adapter) {
        char data[sizeof(PACKET_OID_DATA)+5];
        memset(data, 0, sizeof(data));
        PPACKET_OID_DATA pOidData = (PPACKET_OID_DATA) data;
        pOidData->Oid = OID_802_3_CURRENT_ADDRESS;
        pOidData->Length = 6;
        bool ret = PacketRequest(adapter, FALSE, pOidData);
        PacketCloseAdapter(adapter);
        if (ret) {
            memcpy(eth_src, pOidData->Data, 6);
        }
        else {
            outStr << "ERROR: could not get local MAC address for " << dev->name << std::endl;
            return false;
        }
    }
    else {
        outStr << "ERROR: could not open adapter for " << dev->name << std::endl;
        return false;
    }
#else
    // On Linux, use the following socket/ioctl calls.
    struct ifreq s;
    strcpy(s.ifr_name, dev->name);

    // free alldevs
    pcap_freealldevs(alldevs);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        outStr << "ERROR: could not create socket for local MAC address" << std::endl;
        return false;
    }
    else {
        if (ioctl(fd, SIOCGIFHWADDR, &s) == 0) {
           memcpy(eth_src, s.ifr_addr.sa_data, 6);
        }
        else {
            outStr << "ERROR: could not get local MAC address for " << dev->name << std::endl;
            close(fd);
            return false;
        }
        close(fd);
    }
#endif

    print_mac(outStr, "Local MAC address", eth_src);

    // Following filters out any packets from local ethernet interface,
    // most of which are multicast packets and are not of interest
    struct bpf_program fp;		            // The compiled filter expression
    std::stringstream ss_filter;
    ss_filter << "not ether src " << std::hex
       << (int)eth_src[0] << ":" << (int)eth_src[1] << ":" << (int)eth_src[2] << ":"
       << (int)eth_src[3] << ":" << (int)eth_src[4] << ":" << (int)eth_src[5];

    std::cout << "pcap filter = " << ss_filter.str() << "\n";

    if (pcap_compile(handle, &fp, ss_filter.str().c_str(), 0, PCAP_NETMASK_UNKNOWN) == -1) {
        outStr << "ERROR: could not parse filter " << ss_filter.str() << "\n";
        return false;
    }

    if (pcap_setfilter(handle, &fp) == -1) {
        outStr << "ERROR: could not install fitler " << ss_filter.str() << "\n";
        return false;
    }

    memcpy(frame_hdr, eth_dst, 6);
    memcpy(frame_hdr+6, eth_src, 6);
    frame_hdr[12] = 0;   // length field
    frame_hdr[13] = 0;   // length field

    bool ret = this->ScanNodes();
    if (!ret) {
        pcap_close(handle);
        handle = 0;
    }
    return ret;
}

bool Eth1394Port::ScanNodes(void)
{
    // read each boards
    //  1. Read bridge board id using Ethernet broadcast
    //      eth1394_read(0xff, 0x00, 4, &data)
    //  2. Now loop through node id (0x0-0xF, max 16 board) to build a Node2Board map
    //  3. Build Board2Node map

    int node, board;  // loop counters

    // Clear Node2Board & Board2Node maps
    memset(Node2Board, BoardIO::MAX_BOARDS, sizeof(Node2Board));
    memset(Board2Node, BoardIO::MAX_BOARDS, sizeof(Board2Node));

    IsAllBoardsBroadcastCapable_ = true;
    IsAllBoardsBroadcastShorterWait_ = false;    // Not yet supported for Ethernet interface
    IsNoBoardsBroadcastShorterWait_ = true;      // Not yet supported for Ethernet interface

    NumOfNodes_ = 0;
    quadlet_t data = 0x0;   // initialize data to 0

    // Find board id for first board (i.e., one connected by Ethernet)
    int rc = eth1394_read(0xff, 0x00, 4, &data);
    // One retry
    if (rc) {
        outStr << "ScanNodes: multicast failed, retrying" << std::endl;
        rc = eth1394_read(0xff, 0x00, 4, &data);
    }
    if (rc) {
        outStr << "ScanNodes: no response via multicast" << std::endl;
        return false;
    }

    // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
    data = bswap_32(data);
    std::cout << "data = 0x" << std::hex << data << "\n";

    BidBridge_ = (data & BOARD_ID_MASK) >> 24;
    outStr << "BidBridge_ = " << (int)BidBridge_ << "\n";

    // Now scan all nodes
    for (node = 0; node < BoardIO::MAX_BOARDS; node++)
    {
        // check hardware version
        if (eth1394_read(node, 0x04, 4, &data)) {
            outStr << "ScanNodes: unable to read hardware version from node "
                   << node << std::endl;
            break;
        }
        data = bswap_32(data);
        if (data != QLA1_String) {
            outStr << "Node " << node << " is not a QLA board    "
                   << "data = " << std::hex << data << std::endl;
            continue;
        }

        // read firmware version
        unsigned long fver = 0;
        if (eth1394_read(node, 0x07, 4, &data)) {
            outStr << "ScanNodes: unable to read firmware version from node "
                   << node << std::endl;
            return false;
        }
        data = bswap_32(data);
        fver = data;

        // read board id
        if (eth1394_read(node, 0x00, 4, &data)) {
            outStr << "ScanNodes: unable to read status from node " << node << std::endl;
            return false;
        }
        data = bswap_32(data);
        // board_id is bits 27-24, BOARD_ID_MASK = 0x0F000000
        board = (data & BOARD_ID_MASK) >> 24;
        outStr << "  Node " << node << ", BoardId = " << board
               << ", Firmware Version = " << fver << std::endl;
        if (Node2Board[node] < BoardIO::MAX_BOARDS) {
            outStr << "    Duplicate entry, previous value = "
                   << static_cast<int>(Node2Board[node]) << std::endl;
        }
        Node2Board[node] = board;
        FirmwareVersion[board] = fver;
        BoardExistMask_[board] = true;

        // check firmware version
        // FirmwareVersion >= 4, broadcast capable
        if (fver < 4) IsAllBoardsBroadcastCapable_ = false;
        NumOfNodes_++;
    }

    // Use broadcast by default if all firmware are bc capable
    if (IsAllBoardsBroadcastCapable_) {
        Protocol_ = BasePort::PROTOCOL_SEQ_R_BC_W;
        outStr << "ScanNodes: all nodes broadcast capable" << std::endl;
    }

    // update Board2Node
    for (board = 0; board < BoardIO::MAX_BOARDS; board++) {
        Board2Node[board] = BoardIO::MAX_BOARDS;
        for (node = 0; node < NumOfNodes_; node++) {
            if (Node2Board[node] == board) {
                if (Board2Node[board] < BoardIO::MAX_BOARDS)
                    outStr << "Warning: GetNodeId detected duplicate board id for " << board << std::endl;
                Board2Node[board] = node;
            }
        }
    }

    // write num of nodes to eth1394 FPGA
    outStr << "Num of nodes: " << NumOfNodes_ << std::endl;

    return true;
}

bool Eth1394Port::IsOK(void)
{
    return (handle != NULL);
}

void Eth1394Port::Reset(void)
{
    return;
}

int Eth1394Port::GetNodeId(unsigned char boardId) const
{
//    return boardId;
    if (boardId < BoardIO::MAX_BOARDS)
        return Board2Node[boardId];
    else
        return BoardIO::MAX_BOARDS;  // ZC: maybe return -1
}

//! \todo MAXBOARDS?
unsigned long Eth1394Port::GetFirmwareVersion(unsigned char boardId) const
{
    if (boardId < BoardIO::MAX_BOARDS)
        return FirmwareVersion[boardId];
    else
        return 0;
}

bool Eth1394Port::AddBoard(BoardIO *board)
{
    unsigned char bid = board->BoardId;
    if(bid >= BoardIO::MAX_BOARDS)
        return false;

    if (!BoardExistMask_[bid])
        return false;

    if (!BoardInUseMask_[bid]){
        BoardInUseMask_[bid] = true;
        NumOfNodesInUse_++;
        BoardList[bid] = board;
        BoardInUseIntegerUpdate();
#if 0 // PK TEMP
        eth1394_write_nodenum();
        // write the GLOBAL_BOARD register
        WriteQuadletBroadcast(0xC,bswap_32(BoardInUseInteger_));
#endif
        board->port = this;
        // Allocate a buffer that is big enough for Ethernet and FireWire headers as well
        // as the data to be sent.
        size_t block_write_len = (ETH_ALIGN32 + ETH_HEADER_LEN + FW_BWRITE_HEADER +
                                  board->GetWriteNumBytes() + FW_CRC)/sizeof(quadlet_t);
        quadlet_t * buf = new quadlet_t[block_write_len];
        // Offset into the data part of the buffer
        size_t offset = (ETH_ALIGN32 + ETH_HEADER_LEN + FW_BWRITE_HEADER)/sizeof(quadlet_t);
        board->InitWriteBuffer(buf, offset);
    }

    return true;
}

bool Eth1394Port::RemoveBoard(unsigned char boardId)
{
    if(boardId >= BoardIO::MAX_BOARDS)
        return false;
    if(BoardInUseMask_[boardId]){
        BoardInUseMask_[boardId] = false;
        NumOfNodesInUse_--;
        // Free up the memory that was allocated in AddBoard
        delete [] BoardList[boardId]->GetWriteBuffer();
        BoardList[boardId]->InitWriteBuffer(0, 0);
        BoardList[boardId]->port = 0;
        BoardList[boardId] = 0;        
        BoardInUseIntegerUpdate();
#if 0 // PK TEMP
        eth1394_write_nodenum();
        // write the GLOBAL_BOARD register
        WriteQuadletBroadcast(0xC,bswap_32(BoardInUseInteger_));
#endif
    }
    return true;
}

bool Eth1394Port::ReadAllBoards(void)
{       
    if (Protocol_ == BasePort::PROTOCOL_BC_QRW) {
        return ReadAllBoardsBroadcast();
    }

    if (!handle) {
        outStr << "ReadAllBoards: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }
    bool allOK = true;
    bool noneRead = true;

    for (size_t bid = 0; bid < BoardIO::MAX_BOARDS; bid++)
    {
        if (BoardInUseMask_[bid]) {
            bool ret = ReadBlock(bid, 0, BoardList[bid]->GetReadBuffer(),
                                 BoardList[bid]->GetReadNumBytes());
            if (ret) noneRead = false;
            else allOK = false;
            BoardList[bid]->SetReadValid(ret);
        }
    }
    if (noneRead) {
        outStr << "Failed to read any board, check Ethernet physical connection" << std::endl;
    }

    return allOK;
}

bool Eth1394Port::ReadAllBoardsBroadcast(void)
{
    if (!handle) {
        outStr << "ReadAllBoardsBroadcast: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }

    bool ret;
    bool allOK = true;

    // sequence number from 16 bits 0 to 65535
    ReadSequence_++;
    if (ReadSequence_ == 65536) {
        ReadSequence_ = 1;
    }
    quadlet_t bcReqData = (ReadSequence_ << 16) | BoardInUseInteger_;

    //--- send out broadcast read request -----
    const size_t length_fw = 5;
    quadlet_t packet_FW[length_fw];
    // length field (big endian 16-bit integer)
    frame_hdr[12] = 0;
    frame_hdr[13] = length_fw * 4;

    packet_FW[0] = bswap_32(0xFFC00000);
    packet_FW[1] = bswap_32(0xFFCFFFFF);
    packet_FW[2] = bswap_32(0xFFFF000F);
    packet_FW[3] = bswap_32(bcReqData);
    packet_FW[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, 16)));

    // Ethernet frame
    const int ethlength = length_fw * 4 + ETH_HEADER_LEN;
    unsigned char frame[ethlength];
    memcpy(frame, frame_hdr, ETH_HEADER_LEN);
    memcpy(frame + ETH_HEADER_LEN, packet_FW, length_fw * 4);

    if (pcap_sendpacket(handle, frame, ethlength) != 0)
    {
        outStr << "ERROR: send packet failed" << std::endl;
        return false;
    }

    // Grab a packet
    struct pcap_pkthdr header;	/* The header that pcap gives us */
    const u_char *packet;		/* The actual packet */
    packet = pcap_next(handle, &header);

    if (packet == NULL) {
        outStr << "Error: Broadcast Read Received failed" << std::endl;
        return false;
    }

    ret = true;

    // check the packet length, in bytes
    if (header.len - ETH_HEADER_LEN - NumOfNodesInUse_ * 4 * 17 != 0)
    {
        outStr << "ERROR: Broadcast read packet length error" << std::endl;
        return false;
    }

    // record the number of cycle
    int loopSeq = 0;
    for (int bid = 0; bid < BoardIO::MAX_BOARDS; bid++) {
        if (BoardList[bid]) {
            const int readSize = 17;  // 1 seq + 16 data, unit quadlet
            quadlet_t readBuffer[readSize];

            memcpy(readBuffer, packet + ETH_HEADER_LEN + loopSeq * readSize * 4, readSize * 4);

            unsigned int seq = (bswap_32(readBuffer[0]) >> 16);

            //! check boardID

            static int errorcounter = 0;
            if (ReadSequence_ != seq) {
                errorcounter++;
                outStr << "errorcounter = " << errorcounter << std::endl;
                outStr << std::hex << seq << "  " << ReadSequence_ << "  " << (int)bid << std::endl;
            }

            memcpy(BoardList[bid]->GetReadBuffer(), &(readBuffer[1]), (readSize-1) * 4);

            if (!ret) allOK = false;
            BoardList[bid]->SetReadValid(ret);
            loopSeq++;
        }
    }

    return allOK;
}

bool Eth1394Port::WriteAllBoards()
{
    if ((Protocol_ == BasePort::PROTOCOL_SEQ_R_BC_W) || (Protocol_ == BasePort::PROTOCOL_BC_QRW)) {
        return WriteAllBoardsBroadcast();
    }

    if (!handle) {
        outStr << "WriteAllBoards: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }
    bool allOK = true;
    bool noneWritten = true;

    for (size_t bid = 0; bid < BoardIO::MAX_BOARDS; bid++) {
        if(BoardInUseMask_[bid]){
            quadlet_t *buf = BoardList[bid]->GetWriteBufferData();
            unsigned int numBytes = BoardList[bid]->GetWriteNumBytes();
            unsigned int numQuads = numBytes/4;
            // Currently (Rev 1 firmware), the last quadlet (Status/Control register)
            // is done as a separate quadlet write.
            bool ret = WriteBlock(bid, 0, buf, numBytes-4);
            if (ret) noneWritten = false;
            else allOK = false;
            quadlet_t ctrl = buf[numQuads-1];  // Get last quadlet
            bool ret2 = true;
            if (ctrl) {    // if anything non-zero, write it
                ret2 = WriteQuadlet(bid, 0x00, ctrl);
                if (ret2) noneWritten = false;
                else allOK = false;
            }
            // SetWriteValid clears the buffer if the write was valid
            BoardList[bid]->SetWriteValid(ret&&ret2);
        }
    }
    if (noneWritten)
        outStr << "Fail to write any board, check Ethernet physical connection" << std::endl;
    return allOK;
}

bool Eth1394Port::WriteAllBoardsBroadcast(void)
{
    // check handle
    if (!handle) {
        outStr << "WriteAllBoardsBroadcast: handle for port " << PortNum << " is NULL" << std::endl;
        return false;
    }

    // sanity check vars
    bool allOK = true;

    // loop 1: broadcast write block

    // construct broadcast write buffer
    const int numOfChannel = 4;
    quadlet_t bcBuffer[numOfChannel * BoardIO::MAX_BOARDS];
    memset(bcBuffer, 0, sizeof(bcBuffer));
    int bcBufferOffset = 0; // the offset for new data to be stored in bcBuffer (bytes)
    int numOfBoards = 0;

    for (int bid = 0; bid < BoardIO::MAX_BOARDS; bid++) {
        if (BoardList[bid]) {
            numOfBoards++;
            quadlet_t *buf = BoardList[bid]->GetWriteBufferData();
            unsigned int numBytes = BoardList[bid]->GetWriteNumBytes();
            memcpy(bcBuffer + bcBufferOffset/4, buf, numBytes-4); // -4 for ctrl offset
            // bcBufferOffset equals total numBytes to write, when the loop ends
            bcBufferOffset = bcBufferOffset + numBytes - 4;
        }
    }

    // now broadcast out the huge packet
    bool ret = true;

    ret = WriteBlockBroadcast(0xffffff000000,  // now the address is hardcoded
                              bcBuffer,
                              bcBufferOffset);

    // loop 2: send out control quadlet if necessary
    for (int bid = 0; bid < BoardIO::MAX_BOARDS; bid++) {
        if (BoardList[bid]) {
            quadlet_t *buf = BoardList[bid]->GetWriteBufferData();
            unsigned int numBytes = BoardList[bid]->GetWriteNumBytes();
            unsigned int numQuads = numBytes/4;
            quadlet_t ctrl = buf[numQuads-1];  // Get last quedlet
            bool ret2 = true;
            if (ctrl) {  // if anything non-zero, write it
                ret2 = WriteQuadlet(bid, 0x00, ctrl);
                if (!ret2) allOK = false;
            }
            // SetWriteValid clears the buffer if the write was valid
            BoardList[bid]->SetWriteValid(ret&&ret2);
        }
    }

    // return
    return allOK;
}

bool Eth1394Port::ReadQuadlet(unsigned char boardId,
                              nodeaddr_t addr, quadlet_t &data)
{
    int node = -1;
    if (boardId != 0xff) {   // 0xff is for Ethernet multicast
        if (boardId >= BoardIO::MAX_BOARDS) {
            outStr << "Invalid board ID: " << static_cast<int>(boardId) << std::endl;
            return false;
        }
//        if (!BoardExistMask_[boardId]) {
//            outStr << "Board " << static_cast<int>(boardId) << " does not exist" << std::endl;
//            return false;
//        }
        node = GetNodeId(boardId);
    }
    else {
        node = 0xff;
    }

    bool ret = eth1394_read(node, addr, 4, &data);
    data = bswap_32(data);
    return (!ret);
}


bool Eth1394Port::WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data)
{
    int node = -1;
    if (boardId != 0xff) {   // 0xff is for Ethernet multicast
        if (boardId >= BoardIO::MAX_BOARDS) {
            outStr << "Invalid board ID: " << static_cast<int>(boardId) << std::endl;
            return false;
        }
        if (!BoardExistMask_[boardId]) {
            outStr << "Board " << static_cast<int>(boardId) << " does not exist" << std::endl;
            return false;
        }
        node = GetNodeId(boardId);
    }
    else {
        node = 0xff;   // multicast
    }

    // Create buffer that is large enough for Ethernet header and Firewire packet
    quadlet_t buffer[(ETH_ALIGN32+ETH_HEADER_LEN+FW_QWRITE_HEADER+4+FW_CRC)/sizeof(quadlet_t)];
    quadlet_t *packet_FW = buffer + (ETH_ALIGN32+ETH_HEADER_LEN)/sizeof(quadlet_t);

    // header
    fw_tl = (fw_tl+1)&0x3F;   // increment transaction label
    make_1394_header(packet_FW, node, addr, Eth1394Port::QWRITE, fw_tl);
    // quadlet data
    packet_FW[3] = bswap_32(data);
    // CRC
    packet_FW[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, FW_QWRITE_HEADER+4)));

    size_t length_fw = (FW_QWRITE_HEADER + sizeof(quadlet_t) + FW_CRC)/sizeof(quadlet_t);
    return eth1394_write(node, buffer, length_fw);
}

bool Eth1394Port::WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data)
{
    // special case of WriteBlockBroadcast
    // nbytes = 4
    data = bswap_32(data);
    return WriteBlockBroadcast(addr, &data, 4);
}

bool Eth1394Port::ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata, unsigned int nbytes)
{
    int node = -1;
    if (boardId != 0xff) {   // 0xff is for Ethernet multicast
        if (boardId >= BoardIO::MAX_BOARDS) {
            outStr << "Invalid board ID: " << static_cast<int>(boardId) << std::endl;
            return false;
        }
        if (!BoardExistMask_[boardId]) {
            outStr << "Board " << static_cast<int>(boardId) << " does not exist" << std::endl;
            return false;
        }
        node = GetNodeId(boardId);
    }
    else {
        node = 0xff;
    }
    return !eth1394_read(node, addr, nbytes, rdata);
}

bool Eth1394Port::WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes)
{
    if (nbytes == 4) {
        return WriteQuadlet(boardId, addr, *wdata);
    }
    else if ((nbytes == 0) || ((nbytes%4) != 0)) {
        outStr << "WriteBlock: illegal length in bytes = " << nbytes << std::endl;
        return false;
    }

    quadlet_t *frame = 0;     // The entire Ethernet frame (headers and data)
    // Offset into the data part of the frame
    size_t data_offset = (ETH_ALIGN32 + ETH_HEADER_LEN + FW_BWRITE_HEADER)/sizeof(quadlet_t);
    bool realTimeWrite = false;

    int node = -1;
    if (boardId != 0xff) {   // 0xff is for Ethernet multicast
        if (boardId >= BoardIO::MAX_BOARDS) {
            outStr << "Invalid board ID: " << static_cast<int>(boardId) << std::endl;
            return false;
        }
        if (!BoardExistMask_[boardId]) {
            outStr << "Board " << static_cast<int>(boardId) << " does not exist" << std::endl;
            return false;
        }
        // Check for real-time write
        if ((BoardList[boardId]->GetWriteBufferData() == wdata) &&
            (nbytes <= BoardList[boardId]->GetWriteNumBytes())) {
            realTimeWrite = true;
            frame = BoardList[boardId]->GetWriteBuffer();
        }
        node = GetNodeId(boardId);
    }
    else {
        node = 0xff;
    }

    if (!realTimeWrite) {
        size_t block_write_len = data_offset + (nbytes + FW_CRC)/sizeof(quadlet_t);
        frame = new quadlet_t[block_write_len];
    }

    quadlet_t *fw_header = frame + (ETH_ALIGN32 + ETH_HEADER_LEN)/sizeof(quadlet_t);
    // header
    fw_tl = (fw_tl+1)&0x3F;   // increment transaction label
    make_1394_header(fw_header, node, addr, Eth1394Port::BWRITE, fw_tl);
    // block length
    fw_header[3] = bswap_32(nbytes<<16);
    // header CRC
    fw_header[4] = bswap_32(BitReverse32(crc32(0U, (void*)fw_header, FW_BWRITE_HEADER-sizeof(quadlet_t))));
    // Pointer to data part of frame
    quadlet_t *fw_data = frame + data_offset;
    if (!realTimeWrite) {
        // Copy the user-supplied data into the frame
        memcpy(fw_data, wdata, nbytes);
    }
    // CRC
    quadlet_t *fw_crc = fw_data + (nbytes/sizeof(quadlet_t));
    // PK: temporarily save the last quadlet, which is the power control on a real-time write,
    //     so that it does not get overwritten by the CRC.
    quadlet_t temp_saved = *fw_crc;   // PK TEMP for real-time writes
    *fw_crc = bswap_32(BitReverse32(crc32(0U, (void*)fw_data, nbytes)));

    size_t length_fw = (FW_BWRITE_HEADER + nbytes + FW_CRC)/4;  // size in quadlets
    bool ret = eth1394_write(node, frame, length_fw);
    *fw_crc = temp_saved;   // PK TEMP for real-time writes
    if (!realTimeWrite)
        delete [] frame;
    return ret;
}

bool Eth1394Port::WriteBlockBroadcast(
        nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes)
{
#if 0 // PK TEMP
    // TO FIX: eth1394_write does not yet handle 0xffff
    return !eth1394_write(0xffff, addr, nbytes, data);;
#else
    outStr << "Read from " << addr << " for " << nbytes << " bytes\n";
    return false;
#endif
}

void Eth1394Port::PromDelay(void) const
{
    // Wait 1 msec
    Amp1394_Sleep(0.001);
}

// ---------------------------------------------------------
// Protected
// ---------------------------------------------------------

void Eth1394Port::make_1394_header(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tcode,
                                   unsigned int tl)
{
    // For now, (node == 0xff) means Ethernet multicast. We change it to node 0.
    if (node == 0xff)
        node = 0;
    // FFC0 replicates the base node ID when using FireWire on PC. This is followed by a transaction
    // label (arbitrary value that is returned by any resulting FireWire packets) and the transaction code.
    packet[0] = bswap_32((0xFFC0 | node) << 16 | (tl & 0x3F) << 10 | (tcode & 0x0F) << 4);
    // FFFF is used as source ID (most significant 16 bits); not sure if this is needed.
    // This is followed by the destination address, which is 48-bits long
    packet[1] = bswap_32(0xFFFF << 16 | ((addr & 0x0000FFFF00000000) >> 32));
    packet[2] = bswap_32(addr&0xFFFFFFFF);
}


bool Eth1394Port::checkCRC(const unsigned char *packet) const
{
    // Eliminate CRC checking of FireWire packets received via Ethernet
    // because Ethernet already includes CRC.
#if 0
    uint32_t crc_check = BitReverse32(crc32(0U,(void*)(packet+14),16));
    uint32_t crc_original = bswap_32(*((uint32_t*)(packet+30)));
    return (crc_check == crc_original);
#else
    return true;
#endif
}

int Eth1394Port::eth1394_read(nodeid_t node, nodeaddr_t addr,
                              size_t length, quadlet_t *buffer)
{
    unsigned int tcode;  // fw tcode
    size_t length_fw;    // fw length in quadlets

    if (length == 4) {
        tcode = Eth1394Port::QREAD;
        length_fw = 4;
    } else if ((length > 4) && (length%4 == 0)) {
        tcode = Eth1394Port::BREAD;
        length_fw = 5;
    }
    else
    {
        outStr << "eth1394_read: illegal length = " << length << std::endl;
        return -1;
    }

    //quadlet_t packet_FW[length_fw];
    quadlet_t packet_FW[5];

    // header
    //    make_1394_header(packet_FW, node, addr, tcode);
    fw_tl = (fw_tl+1)&0x3F;   // increment transaction label
    make_1394_header(packet_FW, node, addr, tcode, fw_tl);

    if (tcode == BREAD)
        packet_FW[3] = bswap_32((length & 0xFFFF) << 16);
    packet_FW[length_fw-1] =
    bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, length_fw * 4 - 4)));

    if (DEBUG) print_frame((unsigned char*)packet_FW, length_fw*sizeof(quadlet_t));

    // Ethernet frame
    const int ethlength = length_fw * 4 + 14;  // eth frame length in bytes
    // length field (big endian 16-bit integer)
    // Following assumes length is less than 256 bytes
    frame_hdr[12] = 0;
    frame_hdr[13] = length_fw * 4;

    //uint8_t frame[ethlength];
    uint8_t frame[5*4+14];
    memcpy(frame, frame_hdr, 14);
    if (node == 0xff) {      // multicast
        frame[0] |= 0x01;    // set multicast destination address
        frame[5] = node;     // keep multicast address
    } else {
        frame[5] = BidBridge_;   // last byte of dest address is bridge board id
    }

    memcpy(frame + 14, packet_FW, length_fw * 4);

    if (DEBUG) {
        std::cout << "------ Eth Frame ------" << std::endl;
        print_frame((unsigned char*)frame, ethlength*sizeof(uint8_t));
    }

    if (pcap_sendpacket(handle, frame, ethlength) != 0)
    {
        outStr << "ERROR: send packet failed" << std::endl;
        return -1;
    }


    double startTime = Amp1394_GetTime();

    // Invoke callback (if defined) between sending read request
    // and checking for read response. If callback returns false, we
    // skip checking for a received packet.
    if (eth1394_read_callback) {
        if (!(*eth1394_read_callback)(*this, node, outStr)) {
            outStr << "eth1394_read_callback: aborting (not reading packet)" << std::endl;
            return -1;
        }
    }

    // Read packets
    struct pcap_pkthdr header;	/* The header that pcap gives us */
    const u_char *packet;		/* The actual packet */
    unsigned int numPackets = 0;
    unsigned int numPacketsValid = 0;
    double timeDiffSec = 0.0;

    while ((numPacketsValid < 1) && (timeDiffSec < 0.5)) {
        while ((packet = pcap_next(handle, &header)) != NULL) {
            numPackets++;
            if (headercheck((unsigned char *)packet, true)) {
                if ((node != 0xff) && (packet[11] != BidBridge_)) {
                    outStr << "Packet not from node " << BidBridge_ << " (src lsb is "
                           << static_cast<unsigned int>(packet[11]) << ")" << std::endl;
                    continue;
                }
                //unsigned int packetLength = static_cast<int>(packet[13])<<8 | packet[12];
                //outStr << "Packet length = " << std::dec << packetLength << std::endl;
                numPacketsValid++;
                int tl_recv = packet[16] >> 2;
                if (tl_recv != fw_tl) {
                    outStr << "WARNING: expected tl = " << (unsigned int)fw_tl
                           << ", received tl = " << tl_recv << std::endl;
                }
                int tcode_recv = packet[17] >> 4;
                if ((tcode == QREAD) && (tcode_recv == QRESPONSE)) {
                    // check header crc
                    if (checkCRC(packet))
                        memcpy(buffer, &packet[26], 4);
                    else {
                        outStr <<"ERROR: crc check error"<<std::endl;
                        return -1;
                    }
                }
                else if ((tcode == BREAD) && (tcode_recv == BRESPONSE)) {
                    if (length == ((packet[26] << 8) | packet[27])) {
                        if (checkCRC(packet))
                            memcpy(buffer, &packet[34], length);
                        else {
                            outStr <<"ERROR: header crc check error"<<std::endl;
                            return -1;
                       }
                    }
                    else{
                        outStr << "ERROR: block read response size error" << std::endl;
                        return -1;
                    }
                }
                else {
                    outStr << "WARNING: unexpected response tcode: " << tcode_recv
                           << " (sent tcode: " << tcode << ")" << std::endl;
                    return -1;
//                    continue;
                }
            }
        }
        timeDiffSec = Amp1394_GetTime() - startTime;
    }

    if (numPacketsValid < 1) {
        outStr << "Error: Receive failed, addr = " << std::hex << addr
               << ", nbytes = " << length << ", time = " << timeDiffSec << " sec" << ", num pkt = " << numPackets << std::endl;
        return -1;
    }
#if 0
    outStr << "Processed " << numPackets << " packets, " << numPacketsValid << " valid"
           << ", time = " << timeDiffSec << " sec" << std::endl;
#endif

    return 0;
}


bool Eth1394Port::eth1394_write(nodeid_t node, quadlet_t *buffer, size_t length_fw)
{
    // Ethernet frame
    uint8_t *frame = reinterpret_cast<uint8_t *>(buffer) + ETH_ALIGN32;
    memcpy(frame, frame_hdr, 12);
    if (node == 0xff) {      // multicast
        frame[0] |= 0x01;    // set multicast destination address
        frame[5] = node;     // keep multicast address
    }
    frame[5] = BidBridge_;   // last byte of dest address is board id

    // length field (big endian 16-bit integer)
    // Following assumes length is less than 256 bytes
    frame[12] = 0;
    frame[13] = length_fw * 4;

    // Firewire packet is already created by caller
    int ethlength = length_fw*4 + ETH_HEADER_LEN;  // eth frame length in bytes

    if (DEBUG) {
        std::cout << "------ Eth Frame ------" << std::endl;
        print_frame((unsigned char*)frame, ethlength);
    }

    if (pcap_sendpacket(handle, frame, ethlength) != 0)
    {
        outStr << "ERROR: send packet failed" << std::endl;
        return false;
    }
    return true;
}


int Eth1394Port::eth1394_write_nodenum(void)
{
    frame_hdr[12] = ((NumOfNodesInUse_-1) << 4) | 0x08;
    frame_hdr[13] = 0;
    unsigned char MSG[] = "This is a num_node synchronizing frame.";
    const int length_MSG = sizeof(MSG);

    // Ethernet frame
    const int ethlength = length_MSG + 14;
    uint8_t frame[ethlength];
    memcpy(frame, frame_hdr, 14);
    memcpy(frame + 14, MSG, length_MSG);

    // print
    if (DEBUG) {
        std::cout << "------ Eth Frame ------" << std::endl;
        print_frame((unsigned char*)frame, sizeof(frame));
    }

    if (pcap_sendpacket(handle, frame, ethlength) != 0)
    {
        outStr << "ERROR: send packet failed" << std::endl;
        return -1;
    }
    return 0;
}

int Eth1394Port::eth1394_write_nodeidmode(int mode)// mode: 1: board_id, 0: fw_node_id
{
    if( mode == 0 || mode == 1)
    {
        // actually a modified qwrite
        const size_t length_fw = 5;
        quadlet_t packet_FW[length_fw];
        // length field (big endian 16-bit integer)
        frame_hdr[12] = 0;
        frame_hdr[13] = length_fw * 4;

        packet_FW[0] = bswap_32(0xFFFF0000);
        packet_FW[1] = bswap_32(0xFFFF0000);
        packet_FW[2] = bswap_32(0x00000000);
        if  (mode == 1)
            packet_FW[3] = bswap_32(0x00C00000);
        else
            packet_FW[3] = bswap_32(0x00800000);
        packet_FW[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, 16)));

        // Ethernet frame
        const int ethlength = length_fw * 4 + 14;
        u_int8_t frame[ethlength];
        memcpy(frame, frame_hdr, 14);
        memcpy(frame + 14, packet_FW, length_fw * 4);

        // print
        if (DEBUG) {
            std::cout << "------ Eth Frame ------" << std::endl;
            print_frame((unsigned char*)frame, sizeof(frame));
        }

        if (pcap_sendpacket(handle, frame, ethlength) != 0)
        {
            outStr << "ERROR: send packet failed" << std::endl;
            return -1;
        }
    }
    else{
        outStr << "ERROR: invalid mode number (0 or 1)" << std::endl;
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------
// CRC related
// ---------------------------------------------------------

bool Eth1394Port::headercheck(uint8_t* header, bool isHUBtoPC) const
{
    unsigned int i;
    const uint8_t *srcAddr;
    const uint8_t *destAddr;
    if (isHUBtoPC) {
        // the header should be "Local MAC addr" + "CID,0x1394,boardid"
        destAddr = frame_hdr+6;
        srcAddr = frame_hdr;
    }
    else {
        // the header should be "CID,0x1394,boardid" + "Local MAC addr"
        destAddr = frame_hdr;
        srcAddr = frame_hdr+6;
    }
    bool isBroadcast = true;
    for (i = 0; i < 6; i++) {
        if (header[i] != 0xff) {
            isBroadcast = false;
            break;
        }
    }
    // For now, we don't use broadcast packets
    if (isBroadcast) {
        outStr << "Header check found broadcast packet" << std::endl;
        return false;
    }
    // We also don't use multicast packets
    if (header[0]&1) {
        outStr << "Header check found multicast packet" << std::endl;
        print_mac(outStr, "Header", header);
        print_mac(outStr, "Src", header+6);
        return false;
    }
    for (i = 0; i < 5; i++) {   // don't check board id (last byte)
        if (header[i] != destAddr[i]) {
            outStr << "Header check failed for destination address" << std::endl;
            print_mac(outStr, "Header", header);
            print_mac(outStr, "DestAddr", destAddr);
            return false;
        }
        if (header[i+6] != srcAddr[i]) {
            outStr << "Header check failed for source address" << std::endl;
            print_mac(outStr, "Header", header+6);
            print_mac(outStr, "SrcAddr", srcAddr);
            return false;
        }
    }
    return true;
}

//  -----------  CRC ----------------
//source: http://www.opensource.apple.com/source/xnu/xnu-1456.1.26/bsd/libkern/crc32.c
//online check: http://www.lammertbies.nl/comm/info/crc-calculation.html
static uint32_t crc32_tab[] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3,	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de,	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,	0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5,	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,	0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940,	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,	0x76dc4190, 0x01db7106,
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};
static const unsigned char BitReverseTable[] =
{
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};


uint32_t BitReverse32(uint32_t input)
{
    unsigned char inputs[4];
    inputs[0] = BitReverseTable[input & 0x000000ff];
    inputs[1] = BitReverseTable[(input & 0x0000ff00)>>8];
    inputs[2] = BitReverseTable[(input & 0x00ff0000)>>16];
    inputs[3] = BitReverseTable[(input & 0xff000000)>>24];
    uint32_t output = 0x00000000;
    output |= (uint32_t)inputs[0] << 24;
    output |= (uint32_t)inputs[1] << 16;
    output |= (uint32_t)inputs[2] << 8;
    output |= (uint32_t)inputs[3];
    return output;
}


// The sample use of CRC
// crc = BitReverse32(crc32(0U,(void*)array_char,len_in_byte));
// It is also needed to be byteSwaped before putting into stream
uint32_t crc32(uint32_t crc, const void *buf, size_t size)
{
    const uint8_t *p;

    p = (uint8_t*)buf;
    crc = crc ^ ~0U;

    while (size--)
        crc = crc32_tab[(crc ^ BitReverseTable[*p++]) & 0xFF] ^ (crc >> 8);

  return crc ^ ~0U;
}


void print_frame(unsigned char* buffer, int length)
{
    unsigned char* currentChar = buffer;
    std::cout<<"Frame length: "<<std::dec<<length<<std::endl;
    for(int i=0;i<length;i++)
    {
        std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<(int)*currentChar;
        currentChar = currentChar + 1;
        if(i%2)
            std::cout<<" ";
        if((i+3)%8 == 0)
            std::cout<<std::endl;
    }
    std::cout << std::endl;
}

void print_mac(std::ostream &outStr, const char* name, const uint8_t *addr)
{
    outStr << name << ": " << std::hex
           << (int)addr[0] << ":" << (int)addr[1] << ":" << (int)addr[2] << ":"
           << (int)addr[3] << ":" << (int)addr[4] << ":" << (int)addr[5] << std::endl;
}

