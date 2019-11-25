/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "EthRawPort.h"
#include "Amp1394Time.h"

#ifdef _MSC_VER
// Following seems to be necessary on Windows, at least with Npcap
#define PCAP_DONT_INCLUDE_PCAP_BPF_H
#endif
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
#include <unistd.h>
#include <byteswap.h>
#endif

#define DEBUG 0

// PK TEMP following CRC defs
// crc related
uint32_t BitReverse32(uint32_t input);
uint32_t crc32(uint32_t crc, const void *buf, size_t size);

const unsigned int ETH_ALIGN32 = 2;          // Number of extra bytes for 32-bit alignment
const unsigned int ETH_HEADER_LEN = 14;      // Number of bytes in Ethernet header

EthRawPort::EthRawPort(int portNum, std::ostream &debugStream, EthCallbackType cb):
    EthBasePort(portNum, debugStream, cb)
{
    if (Init())
        outStr << "Initialization done" << std::endl;
    else
        outStr << "Initialization failed" << std::endl;
}

EthRawPort::~EthRawPort()
{
    pcap_close(handle);
}

bool EthRawPort::Init(void)
{
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
    uint8_t eth_dst[6];
    GetDestMacAddr(eth_dst);
    uint8_t eth_src[6];   // Ethernet source address (local MAC address, see below)

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

    EthBasePort::PrintMAC(outStr, "Local MAC address", eth_src);

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
        outStr << "ERROR: could not install filter " << ss_filter.str() << "\n";
        return false;
    }

    memcpy(frame_hdr, eth_dst, 6);
    memcpy(frame_hdr+6, eth_src, 6);
    frame_hdr[12] = 0;   // length field
    frame_hdr[13] = 0;   // length field

    bool ret = InitNodes();
    if (ret)
        ret = ScanNodes();
    if (!ret) {
        pcap_close(handle);
        handle = 0;
    }
    return ret;
}

bool EthRawPort::InitNodes(void)
{
    // Find board id for first board (i.e., one connected by Ethernet)
    quadlet_t data = 0x0;   // initialize data to 0
    bool ret = ReadQuadletNode(FW_NODE_BROADCAST, 0, data, (FW_NODE_ETH_BROADCAST_MASK|FW_NODE_NOFORWARD_MASK));
    // One retry
    if (!ret) {
        outStr << "ScanNodes: multicast failed, retrying" << std::endl;
        ret = ReadQuadletNode(FW_NODE_BROADCAST, 0, data, (FW_NODE_ETH_BROADCAST_MASK|FW_NODE_NOFORWARD_MASK));
    }
    if (!ret) {
        outStr << "InitNodes: no response via multicast" << std::endl;
        return false;
    }

    // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
    HubBoard = (data & BOARD_ID_MASK) >> 24;
    outStr << "InitNodes: found hub board: " << static_cast<int>(HubBoard) << std::endl;
    return true;
}

bool EthRawPort::IsOK(void)
{
    return (handle != NULL);
}

void EthRawPort::Reset(void)
{
    return;
}

bool EthRawPort::AddBoard(BoardIO *board)
{
    bool ret = BasePort::AddBoard(board);
    if (ret) {
        // Allocate a buffer that is big enough for Ethernet and FireWire headers as well
        // as the data to be sent.
        size_t block_write_len = (ETH_ALIGN32 + ETH_HEADER_LEN + FW_BWRITE_HEADER_SIZE +
                                  board->GetWriteNumBytes() + FW_CRC_SIZE)/sizeof(quadlet_t);
        quadlet_t * buf = new quadlet_t[block_write_len];
        // Offset into the data part of the buffer
        size_t offset = (ETH_ALIGN32 + ETH_HEADER_LEN + FW_BWRITE_HEADER_SIZE)/sizeof(quadlet_t);
        board->InitWriteBuffer(buf, offset);
    }
    return ret;
}

bool EthRawPort::RemoveBoard(unsigned char boardId)
{
    bool ret = false;
    BoardIO *board = BoardList[boardId];
    if (board) {
        // Free up the memory that was allocated in AddBoard
        delete [] BoardList[boardId]->GetWriteBuffer();
        BoardList[boardId]->InitWriteBuffer(0, 0);
        ret = BasePort::RemoveBoard(boardId);
    }
    else
        outStr << "RemoveBoard: board " << static_cast<unsigned int>(boardId) << " not in use" << std::endl;
    return ret;
}

bool EthRawPort::ReadAllBoardsBroadcast(void)
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
    quadlet_t bcReqData = (ReadSequence_ << 16) | BoardInUseMask_;

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
        outStr << "ERROR: PCAP send packet failed" << std::endl;
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

    // check the packet length, in bytes
    if (header.len - ETH_HEADER_LEN - NumOfBoards_ * 4 * 17 != 0)
    {
        outStr << "ERROR: Broadcast read packet length error" << std::endl;
        return false;
    }

    ret = true;

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

bool EthRawPort::WriteAllBoardsBroadcast(void)
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

bool EthRawPort::ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags)
{
    bool ret = eth1394_read(node, addr, 4, &data, flags&FW_NODE_ETH_BROADCAST_MASK);
    data = bswap_32(data);
    return (!ret);
}


bool EthRawPort::WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags)
{
    // Create buffer that is large enough for Ethernet header and Firewire packet
    quadlet_t buffer[(ETH_ALIGN32+ETH_HEADER_LEN+FW_QWRITE_SIZE)/sizeof(quadlet_t)];
    quadlet_t *packet_FW = buffer + (ETH_ALIGN32+ETH_HEADER_LEN)/sizeof(quadlet_t);

    // header
    fw_tl = (fw_tl+1)&FW_TL_MASK;   // increment transaction label
    make_1394_header(packet_FW, node, addr, EthRawPort::QWRITE, fw_tl);
    // quadlet data
    packet_FW[3] = bswap_32(data);
    // CRC
    packet_FW[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, FW_QWRITE_SIZE-FW_CRC_SIZE)));

    size_t length_fw = FW_QWRITE_SIZE/sizeof(quadlet_t);
    return eth1394_write(node, buffer, length_fw, flags&FW_NODE_ETH_BROADCAST_MASK);
}

bool EthRawPort::ReadBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *rdata, unsigned int nbytes)
{
    if (nbytes == 4)
        return ReadQuadlet(boardId, addr, *rdata);
    else if ((nbytes == 0) || ((nbytes%4) != 0)) {
        outStr << "ReadBlock: illegal size (" << nbytes << "), must be multiple of 4" << std::endl;
        return false;
    }

    nodeid_t node = ConvertBoardToNode(boardId);
    if (node == MAX_NODES) {
        outStr << "ReadBlock: board " << static_cast<unsigned int>(boardId&FW_NODE_MASK) << " does not exist" << std::endl;
        return false;
    }

    return !eth1394_read(node, addr, nbytes, rdata, boardId&FW_NODE_ETH_BROADCAST_MASK);
}

bool EthRawPort::WriteBlock(unsigned char boardId, nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes)
{
    if (nbytes == 4) {
        return WriteQuadlet(boardId, addr, *wdata);
    }
    else if ((nbytes == 0) || ((nbytes%4) != 0)) {
        outStr << "WriteBlock: illegal size (" << nbytes << "), must be multiple of 4" << std::endl;
        return false;
    }

    nodeid_t node = ConvertBoardToNode(boardId);
    if (node == MAX_NODES) {
        outStr << "WriteBlock: board " << static_cast<unsigned int>(boardId&FW_NODE_MASK) << " does not exist" << std::endl;
        return false;
    }

    quadlet_t *frame = 0;     // The entire Ethernet frame (headers and data)
    // Offset into the data part of the frame
    size_t data_offset = (ETH_ALIGN32 + ETH_HEADER_LEN + FW_BWRITE_HEADER_SIZE)/sizeof(quadlet_t);


    // Check for real-time write
    bool realTimeWrite = false;
    boardId = boardId&FW_NODE_MASK;
    if (boardId < BoardIO::MAX_BOARDS) {
        if ((BoardList[boardId]->GetWriteBufferData() == wdata) &&
            (nbytes <= BoardList[boardId]->GetWriteNumBytes())) {
            realTimeWrite = true;
            frame = BoardList[boardId]->GetWriteBuffer();
        }
        node = GetNodeId(boardId);
    }

    if (!realTimeWrite) {
        size_t block_write_len = data_offset + (nbytes + FW_CRC_SIZE)/sizeof(quadlet_t);
        frame = new quadlet_t[block_write_len];
    }

    quadlet_t *fw_header = frame + (ETH_ALIGN32 + ETH_HEADER_LEN)/sizeof(quadlet_t);
    // header
    fw_tl = (fw_tl+1)&FW_TL_MASK;   // increment transaction label
    make_1394_header(fw_header, node, addr, EthRawPort::BWRITE, fw_tl);
    // block length
    fw_header[3] = bswap_32(nbytes<<16);
    // header CRC
    fw_header[4] = bswap_32(BitReverse32(crc32(0U, (void*)fw_header, FW_BWRITE_HEADER_SIZE-FW_CRC_SIZE)));
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

    size_t length_fw = (FW_BWRITE_HEADER_SIZE + nbytes + FW_CRC_SIZE)/sizeof(quadlet_t);  // size in quadlets
    bool ret = eth1394_write(node, frame, length_fw, boardId&FW_NODE_ETH_BROADCAST_MASK);
    *fw_crc = temp_saved;   // PK TEMP for real-time writes
    if (!realTimeWrite)
        delete [] frame;
    return ret;
}

bool EthRawPort::WriteBlockBroadcast(
        nodeaddr_t addr, quadlet_t *wdata, unsigned int nbytes)
{
#if 0 // PK TEMP
    // TO FIX: eth1394_write does not yet handle 0xffff
    return !eth1394_write(0xffff, addr, nbytes, data);;
#else
    outStr << "WriteBlockBroadcast not yet implemented" << std::endl;
    return false;
#endif
}

int EthRawPort::eth1394_read(nodeid_t node, nodeaddr_t addr,
                             size_t length, quadlet_t *buffer, bool useEthernetBroadcast)
{
    unsigned int tcode;  // fw tcode
    size_t length_fw;    // fw length in quadlets
    int length_fw_bytes; // fw length in bytes

    if (length == 4) {
        tcode = EthBasePort::QREAD;
        length_fw = 4;
    } else if ((length > 4) && (length%4 == 0)) {
        tcode = EthBasePort::BREAD;
        length_fw = 5;
    }
    else
    {
        outStr << "eth1394_read: illegal length = " << length << std::endl;
        return -1;
    }
    length_fw_bytes = length_fw*sizeof(quadlet_t);

    //quadlet_t packet_FW[length_fw];
    quadlet_t packet_FW[5];

    // header
    //    make_1394_header(packet_FW, node, addr, tcode);
    fw_tl = (fw_tl+1)&FW_TL_MASK;   // increment transaction label
    make_1394_header(packet_FW, node, addr, tcode, fw_tl);

    if (tcode == BREAD)
        packet_FW[3] = bswap_32((length & 0xFFFF) << 16);
    packet_FW[length_fw-1] =
    bswap_32(BitReverse32(crc32(0U, (void*)packet_FW, length_fw_bytes - FW_CRC_SIZE)));

    if (DEBUG) PrintFrame((unsigned char*)packet_FW, length_fw_bytes);

    // Ethernet frame
    const int ethlength = length_fw * 4 + 14;  // eth frame length in bytes
    // length field (big endian 16-bit integer)
    // Following assumes length is less than 256 bytes
    frame_hdr[12] = 0;
    frame_hdr[13] = length_fw * 4;

    //uint8_t frame[ethlength];
    uint8_t frame[5*4+14];
    memcpy(frame, frame_hdr, 14);
    if (useEthernetBroadcast) {      // multicast
        frame[0] |= 0x01;    // set multicast destination address
        frame[5] = 0xff;     // keep multicast address
    } else {
        frame[5] = HubBoard;   // last byte of dest address is bridge board id
    }

    memcpy(frame + 14, packet_FW, length_fw * 4);

    if (DEBUG) {
        std::cout << "------ Eth Frame ------" << std::endl;
        PrintFrame((unsigned char*)frame, ethlength*sizeof(uint8_t));
    }

    if (pcap_sendpacket(handle, frame, ethlength) != 0)
    {
        outStr << "ERROR: PCAP send packet failed" << std::endl;
        return -1;
    }

    double startTime = Amp1394_GetTime();

    // Invoke callback (if defined) between sending read request
    // and checking for read response. If callback returns false, we
    // skip checking for a received packet.
    if (eth_read_callback) {
        if (!(*eth_read_callback)(*this, node, outStr)) {
            outStr << "eth_read_callback: aborting (not reading packet)" << std::endl;
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
        while (1) {  // can probably eliminate this loop
            packet = pcap_next(handle, &header);
            if (packet == NULL)
                break;
            numPackets++;
            if (headercheck((unsigned char *)packet, true)) {
                if ((node != 0xff) && (packet[11] != HubBoard)) {
                    outStr << "Packet not from node " << static_cast<unsigned int>(HubBoard) << " (src lsb is "
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
                    if (length == static_cast<size_t>((packet[26] << 8) | packet[27])) {
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


bool EthRawPort::eth1394_write(nodeid_t node, quadlet_t *buffer, size_t length_fw, bool useEthernetBroadcast)
{
    // Firewire packet is already created by caller
    bool ret = true;
    // Ethernet frame
    uint8_t *frame = reinterpret_cast<uint8_t *>(buffer) + ETH_ALIGN32;
    memcpy(frame, frame_hdr, 12);
    if (useEthernetBroadcast) {      // multicast
        frame[0] |= 0x01;    // set multicast destination address
        frame[5] = 0xff;     // keep multicast address
    }
    frame[5] = HubBoard;     // last byte of dest address is board id

    // length field (big endian 16-bit integer)
    // Following assumes length is less than 256 bytes
    frame[12] = 0;
    frame[13] = length_fw * 4;

    int ethlength = length_fw*4 + ETH_HEADER_LEN;  // eth frame length in bytes

    if (DEBUG) {
        std::cout << "------ Eth Frame ------" << std::endl;
        PrintFrame((unsigned char*)frame, ethlength);
    }
    if (pcap_sendpacket(handle, frame, ethlength) != 0)  {
        outStr << "ERROR: PCAP send packet failed" << std::endl;
        ret = false;
    }
    return ret;
}

int EthRawPort::make_ethernet_header(unsigned char *buffer, unsigned int numBytes)
{
    memcpy(buffer, frame_hdr, 12);
    // length field (big endian 16-bit integer)
    // Following assumes length is less than 256 bytes
    buffer[12] = 0;
    buffer[13] = numBytes;
    return 14;
}

/*!
 \brief Validate ethernet packet header

 \param header Ethernet packet header (14 bytes)
 \param toPC  true: check FPGA to PC, false: check PC to FPGA
 \return bool true if valid
*/
bool EthRawPort::headercheck(uint8_t* header, bool toPC) const
{
    unsigned int i;
    const uint8_t *srcAddr;
    const uint8_t *destAddr;
    if (toPC) {
        // the header should be "Local MAC addr" + "CID,0x1394,boardid"
        destAddr = frame_hdr+6;
        srcAddr = frame_hdr;
    }
    else {
        // the header should be "CID,0x1394,boardid" + "Local MAC addr"
        destAddr = frame_hdr;
        srcAddr = frame_hdr+6;
    }
#if 0  // PK TEMP
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
#endif
    // We also don't use multicast packets
    if (header[0]&1) {
        outStr << "Header check found multicast packet" << std::endl;
        PrintMAC(outStr, "Header", header);
        PrintMAC(outStr, "Src", header+6);
        return false;
    }
    for (i = 0; i < 5; i++) {   // don't check board id (last byte)
        if (header[i] != destAddr[i]) {
            outStr << "Header check failed for destination address" << std::endl;
            PrintMAC(outStr, "Header", header);
            PrintMAC(outStr, "DestAddr", destAddr);
            return false;
        }
        if (header[i+6] != srcAddr[i]) {
            outStr << "Header check failed for source address" << std::endl;
            PrintMAC(outStr, "Header", header+6);
            PrintMAC(outStr, "SrcAddr", srcAddr);
            return false;
        }
    }
    return true;
}

void EthRawPort::PrintFrame(unsigned char* buffer, int length)
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
