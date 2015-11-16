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

#include "Eth1394Port.h"
#include <pcap.h>
#include <iomanip>

#ifdef _MSC_VER
#include <windows.h>  // for Sleep
#include <stdlib.h>   // for byteswap functions
inline uint16_t bswap_16(uint16_t data) { return _byteswap_ushort(data); }
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }
#else
#include <byteswap.h>
#endif

const unsigned long QLA1_String = 0x514C4131;
const unsigned long BOARD_ID_MASK    = 0x0f000000;  /*!< Mask for board_id */

#define DEBUG 0
#define TEST 1

// crc related
uint32_t BitReverse32(uint32_t input);
uint32_t crc32(uint32_t crc, const void *buf, size_t size);

/*!
 \brief Validate ethernet packet header

 \param header Ethernet packet header (14 bytes)
 \param isHUBtoPC  true: check HUB to PC, false: check PC to HUB
 \return bool true if valid
*/
bool headercheck(uint8_t* header, bool isHUBtoPC);
void print_frame(unsigned char* buffer, int length);


Eth1394Port::Eth1394Port(int portNum, std::ostream &debugStream):
    BasePort(portNum, debugStream),
    NumOfNodes_(0),
    NumOfNodesInUse_(0)
{
    if (Init())
        eth1394_write_nodeidmode(1);
    else
        outStr << "Initialization failed" << std::endl;
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

bool Eth1394Port::Init()
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
                            1,       // promisc mode
                            100,     // read timeout 100 ms
                            errbuf); // error buffer
    if(handle == NULL)
    {
        outStr <<"ERROR: Couldn't open device: "<< dev->name <<std::endl;
        return false;
    }

    // free alldevs
    pcap_freealldevs(alldevs);

    // initialize ethernet header
    u_int8_t eth_dst[6] = {0x50,0x43,0x3e,0x48,0x55,0x42};
    u_int8_t eth_src[6] = {0x4c,0x43,0x53,0x52,0x00,0x00};
    memcpy(frame_hdr, eth_dst, 6);
    memcpy(&frame_hdr[3], eth_src, 6);
    frame_hdr[6] = bswap_16(0x0801);

//    std::cerr << "Before Scannodes" << std::endl;

    return this->ScanNodes();
}

bool Eth1394Port::ScanNodes(void)
{
    // read each boards
    //  - read firmware version
    //  - set boardid (list or something)

//    eth1394_write_nodeidmode(0);
    IsAllBoardsBroadcastCapable_ = true;

    NumOfNodes_ = 0;
    for (size_t bid = 0; bid < BoardIO::MAX_BOARDS; bid++)
    {
        quadlet_t data;
        if (eth1394_read(bid, 0x04, 4, &data))
            continue;   // continue on error

        // check hardware version
        data = bswap_32(data);
        if (data != QLA1_String) {
            outStr << "Node " << bid << " is not a QLA board" << std::endl
                   << "data = " << std::hex << data << std::endl;
            continue;
        }

        // read firmware version
        unsigned long fver = 0;
        if (eth1394_read(bid, 0x07, 4, &data)) {
            outStr << "ScanNodes: unable to read firmware version from node "
                   << bid << std::endl;
            continue;
        }
        data = bswap_32(data);
        fver = data;

        // verify boardid
        unsigned long board_ret = 0;
        if (eth1394_read(bid, 0x00, 4, &data)) {
            outStr << "ScanNodes: unable to read status from node "
                   << bid << std::endl;
            continue;
        }
        data = bswap_32(data);
        // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
        board_ret = (data & BOARD_ID_MASK) >> 24;
        if (board_ret != bid) {
            outStr << "Error: board ID does not match "
                   << bid << std::endl;
            continue;
        }

        outStr << "  BoardId = " << board_ret
               << ", Firmware Version = " << fver << std::endl;

        BoardExistMask_[bid] = true;
        FirmwareVersion[bid] = fver;
        NumOfNodes_++;        

        // check firmware version
        // FirmwareVersion >= 4, broadcast capable
        if (fver < 4) IsAllBoardsBroadcastCapable_ = false;
    }

    if (NumOfNodes_ == 0) {
        outStr << "No FPGA boards connected" << std::endl;
        pcap_close(handle);
        handle = 0;
        return false;
    }

    // Use broadcast by default if all firmware are bc capable
    if (IsAllBoardsBroadcastCapable_) {
        Protocol_ = BasePort::PROTOCOL_SEQ_R_BC_W;
        outStr << "ScanNodes: all nodes broadcast capable" << std::endl;
    }

    // write num of nodes to eth1394 FPGA
    outStr <<"Num of nodes: "<< NumOfNodes_ <<std::endl;

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
    return boardId;
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
    if(board->BoardId >= BoardIO::MAX_BOARDS)
        return false;

    if (!BoardExistMask_[board->BoardId])
        return false;

    if (!BoardInUseMask_[board->BoardId]){
        BoardInUseMask_[board->BoardId] = true;
        NumOfNodesInUse_++;
        BoardList[board->BoardId] = board;
        BoardInUseIntegerUpdate();
        eth1394_write_nodenum();
        // write the GLOBAL_BOARD register
        WriteQuadletBroadcast(0xC,bswap_32(BoardInUseInteger_));
        board->port = this;
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
        BoardList[boardId]->port = 0;
        BoardList[boardId] = 0;        
        BoardInUseIntegerUpdate();
        eth1394_write_nodenum();
        // write the GLOBAL_BOARD register
        WriteQuadletBroadcast(0xC,bswap_32(BoardInUseInteger_));
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
        if(BoardInUseMask_[bid]){
//            std::cerr << "Read board = " << bid << std::endl;

            bool ret = ReadBlock(bid, 0, BoardList[bid]->GetReadBuffer(),
                                 BoardList[bid]->GetReadNumBytes());
            if (ret) noneRead = false;
            else allOK = false;
            BoardList[bid]->SetReadValid(ret);

            if (!ret) {
                outStr << "------- Oops failed --------" << std::endl;
            }
        }
    }
    if (noneRead) {
        outStr << "Fail to read any board, check Ethernet physical connection" << std::endl;
    }

//    std::cerr << "read all boards done" << std::endl;

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
    frame_hdr[5] = bswap_16(length_fw * 4);

    packet_FW[0] = bswap_32(0xFFC00000);
    packet_FW[1] = bswap_32(0xFFCFFFFF);
    packet_FW[2] = bswap_32(0xFFFF000F);
    packet_FW[3] = bswap_32(bcReqData);
    packet_FW[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, 16)));

    // Ethernet frame
    const int ethlength = length_fw * 4 + 14;
    unsigned char frame[ethlength];
    memcpy(frame, frame_hdr, 14);
    memcpy(frame + 14, packet_FW, length_fw * 4);

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
    if (header.len - 14 - NumOfNodesInUse_ * 4 * 17 != 0)
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

            memcpy(readBuffer, packet + 14 + loopSeq * readSize * 4, readSize * 4);

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
            quadlet_t *buf = BoardList[bid]->GetWriteBuffer();
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
                ret2 = WriteQuadlet(bid, 0, ctrl);
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
    // check hanle
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
            quadlet_t *buf = BoardList[bid]->GetWriteBuffer();
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
            quadlet_t *buf = BoardList[bid]->GetWriteBuffer();
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
    if (boardId >= BoardIO::MAX_BOARDS)
        return false;
    if (!BoardInUseMask_[boardId])
        return false;
    return !eth1394_read(boardId, addr, 4, &data);
}


bool Eth1394Port::WriteQuadlet(unsigned char boardId, nodeaddr_t addr, quadlet_t data)
{
//    std::cerr << "Write Quadlet called" << std::endl;
//    std::cerr << "data = " << std::hex << data << std::endl;

    if (boardId >= BoardIO::MAX_BOARDS)
        return false;
    if (!BoardInUseMask_[boardId])
        return false;    

    return !eth1394_write(boardId, addr, 4, &data);
}

bool Eth1394Port::WriteQuadletBroadcast(nodeaddr_t addr, quadlet_t data)
{
    // special case of WriteBlockBroadcast
    // nbytes = 4
    return WriteBlockBroadcast(addr, &data, 4);
}

bool Eth1394Port::ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data, unsigned int nbytes)
{
    if (boardId >= BoardIO::MAX_BOARDS) {
        outStr << "Invalid board ID" << std::endl;
        return false;
    }

    if (!BoardInUseMask_[boardId]) {
        outStr << "Board not under control" << std::endl;
        return false;
    }

//    std::cerr << "eth1394 read before" << std::endl;

    return !eth1394_read(boardId, addr, nbytes, data);
}

bool Eth1394Port::WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *data, unsigned int nbytes)
{
    if (boardId >= BoardIO::MAX_BOARDS)
        return false;
    if (!BoardInUseMask_[boardId])
        return false;
    return !eth1394_write(boardId, addr, nbytes, data);
}

bool Eth1394Port::WriteBlockBroadcast(
        nodeaddr_t addr, quadlet_t *data, unsigned int nbytes)
{
    return !eth1394_write(0xffff, addr, nbytes, data);;
}



// ---------------------------------------------------------
// Protected
// ---------------------------------------------------------

int Eth1394Port::eth1394_read(nodeid_t node, nodeaddr_t addr,
                              size_t length, quadlet_t *buffer)
{
    // sanity check
    unsigned int tcode;  // fw tcode
    size_t length_fw;   // fw length in quadlet
    if (length == 4) {
        tcode = Eth1394Port::QREAD;
        length_fw = 4;
    } else if(length >4 && length%4 ==0) {
        tcode = Eth1394Port::BREAD;
        length_fw = 5;
    }
    else
    {
        outStr <<"ERROR: illegal length"<<std::endl;
        return -1;
    }

    //quadlet_t packet_FW[length_fw];
    quadlet_t packet_FW[5];

    packet_FW[0] = bswap_32((0xFFC0 | node) << 16 | (tcode & 0xF) << 4);
    packet_FW[1] = bswap_32(0xFFFF << 16 | ((addr & 0x0000FFFF00000000) >> 32));
    packet_FW[2] = bswap_32(addr&0xFFFFFFFF);
    if (tcode == BREAD)
        packet_FW[3] = bswap_32((length & 0xFFFF) << 16);
    packet_FW[length_fw-1] =
    bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, length_fw * 4 - 4)));

    if (DEBUG) print_frame((unsigned char*)packet_FW, length_fw*sizeof(quadlet_t));

    // Ethernet frame
    const int ethlength = length_fw * 4 + 14;  // eth frame length in byte
    frame_hdr[5] = bswap_16(length_fw * 4);

    //uint8_t frame[ethlength];
    uint8_t frame[5*4+14];
    memcpy(frame, frame_hdr, 14);
    memcpy(frame + 14, packet_FW, length_fw * 4);

    if (DEBUG) {
        std::cout << "-------- frame ---------" << std::endl;
        print_frame((unsigned char*)frame, ethlength*sizeof(uint8_t));
    }

    if (pcap_sendpacket(handle, frame, ethlength) != 0)
    {
        outStr << "ERROR: send packet failed" << std::endl;
        return -1;
    }

//    ethpcap_recv_nonblocking();

    // Grab a packet
    struct pcap_pkthdr header;	/* The header that pcap gives us */
    const u_char *packet;		/* The actual packet */
    packet = pcap_next(handle, &header);

    if (packet == NULL) {
//        outStr << "Error: Received failed" << std::endl;
        return -1;
    }

    int tcode_recv = packet[17] >> 4;
    if (tcode == QREAD && tcode_recv == QRESPONSE) {
        // check header crc
        uint32_t crc_check = BitReverse32(crc32(0U,(void*)(packet+14),16));
        uint32_t crc_original = bswap_32(*((uint32_t*)(packet+30)));
        if(headercheck((unsigned char*)packet, 1)){
            if(crc_check == crc_original) {
                memcpy(buffer, &packet[26], 4);
//                buffer[0] = bswap_32(buffer[0]);
            }
            else{
                outStr <<"ERROR: crc check error"<<std::endl;
                return -1;
            }
        }
        else{
            outStr <<"ERROR: Ethernet header error"<<std::endl;
            return -1;
        }
    }
    else if (tcode == BREAD && tcode_recv == BRESPONSE) {
        if(headercheck((unsigned char*)packet, 1)){
            if (length == (packet[26] << 8 | packet[27])) {
                uint32_t crc_h_check = BitReverse32(crc32(0U,(void*)(packet+14),16));
                uint32_t crc_h_original = bswap_32(*((uint32_t*)(packet+30)));
                if(crc_h_check == crc_h_original){
// A little bug here: if length==8, then the data crc is not correct(due to QLA code error)
                    memcpy(buffer, &packet[34], length);
//                    for(int i=0;i<length/4;i++)
//                    {
//                        buffer[i] = bswap_32(buffer[i]);
//                    }
                }
                else{
                    outStr <<"ERROR: header crc check error"<<std::endl;
                    return -1;
                }
            }
            else{
                outStr << "ERROR: block read response size error" << std::endl;
                return -1;
            }
        }
        else{
            outStr <<"ERROR: Ethernet header error"<<std::endl;
            return -1;
        }

    }
    else {
        outStr << "ERROR: unknown response tcode" << std::endl;
        return -1;
    }

    if (!TEST) {
        std::cout << "-------- Read Response ---------" << std::endl;
        print_frame((unsigned char*)buffer, length);
    }
    return 0;
}


int Eth1394Port::eth1394_write(nodeid_t node, nodeaddr_t addr,
                               size_t length, quadlet_t *buffer)
{
//    std::cerr << "data = " << std::hex << buffer[0] << std::endl;

    size_t length_fw;  // in quadlet
    unsigned int tcode;

    if (length == 4) {
        tcode = Eth1394Port::QWRITE;
        length_fw = 5;
    } else if(length > 4 && length%4 == 0){
        tcode = Eth1394Port::BWRITE;
        length_fw = 5 + length/4 + 1;
    }
    else{
        outStr <<"ERROR: illegal length"<<std::endl;
        return -1;
    }

    // sanity check
    quadlet_t *packet_FW = new quadlet_t[length_fw];
    if (length == 4) tcode = Eth1394Port::QWRITE;
    else tcode = Eth1394Port::BWRITE;

    // header
    //! \todo fix packet[3] bug for block write request
    packet_FW[0] = bswap_32((0xFFC0 | node) << 16 | (tcode & 0xF) << 4);
    packet_FW[1] = bswap_32(0xFFFF << 16 | ((addr & 0x0000FFFF00000000) >> 32));
    packet_FW[2] = bswap_32(addr&0xFFFFFFFF);

    //quadlet write
    if(length == 4){
//        packet_FW[3] = bswap_32(buffer[0]);
        packet_FW[3] = buffer[0];
        packet_FW[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, 16)));
    }
    // data for block write
    if (length > 4) {
        packet_FW[3] = bswap_32(length<<16);
        packet_FW[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, 16)));
        memcpy(&(packet_FW[5]), buffer, length);
        packet_FW[length_fw-1] = bswap_32(BitReverse32(crc32(0U, (void*)buffer, length)));
    }

    // print
    if (DEBUG) {
        std::cout << "------ FW Packet ------" << std::endl;
        print_frame((unsigned char*)packet_FW, length_fw*sizeof(quadlet_t));
        std::cout << std::endl;
    }

    // Ethernet frame
    int ethlength = length_fw * 4 + 14;
    frame_hdr[5] = bswap_16(length_fw * 4);

    uint8_t *frame = new uint8_t[ethlength];
    memcpy(frame, frame_hdr, 14);
    memcpy(frame + 14, packet_FW, length_fw * 4);

    // print
    if (DEBUG) {
        std::cout << "------ Eth Frame ------" << std::endl;
        print_frame((unsigned char*)frame, ethlength*sizeof(uint8_t));
    }

    if (pcap_sendpacket(handle, frame, ethlength) != 0)
    {
        outStr << "ERROR: send packet failed" << std::endl;
        return -1;
    }
    delete [] packet_FW;
    delete [] frame;
    return 0;
}

int Eth1394Port::eth1394_write_nodenum(void)
{
    uint16_t syn_frame = 0;
    syn_frame = ((NumOfNodesInUse_ - 1) << 12) | 0x0800;
    frame_hdr[5] = bswap_16(syn_frame);
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
        frame_hdr[5] = bswap_16(length_fw * 4);

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
            std::cerr << "ERROR: send packet failed" << std::endl;
            return -1;
        }
    }
    else{
        std::cerr<<"ERROR: invalid mode number (0 or 1)"<<std::endl;
        return -1;
    }
    return 0;
}

// ---------------------------------------------------------
// CRC related
// ---------------------------------------------------------

bool headercheck(uint8_t* header, bool isHUBtoPC)
{
    // the header should be "HUB>PC" "LCSR??" + 0x0801(ethertype)
    if(isHUBtoPC && header[0] == 0x48 && header[1] == 0x55 && header[2] == 0x42 && header[3] == 0x3e &&
            header[4] == 0x50 && header[5] == 0x43 && header[6] == 0x4c && header[7] == 0x43 &&
            header[8] == 0x53 && header[9] == 0x52 && header[12] == 0x08 && header[13] == 0x01)
    {
        return true;
    }

    // the header should be "PC>HUB" "LCSR??" + 0x0801(ethertype)
    if (!isHUBtoPC && header[0] == 0x50 && header[1] == 0x43 && header[2] == 0x3e && header[3] == 0x48 &&
            header[4] == 0x55 && header[5] == 0x42 && header[6] == 0x4c && header[7] == 0x43 &&
            header[8] == 0x53 && header[9] == 0x52 && header[12] == 0x08 && header[13] == 0x01)
    {
        return true;
    }

    return false;
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
    std::cout<<std::endl;
}
