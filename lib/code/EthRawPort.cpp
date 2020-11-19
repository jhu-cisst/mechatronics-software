/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
    Cleanup();
}

bool EthRawPort::Init(void)
{
    if (PortNum < 0) {
        outStr << "Invalid Port Number" << std::endl;
        return false;
    }

    // Increase ReceiveTimeout
    ReceiveTimeout = 0.1;

    // Ethernet initialization
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
    struct bpf_program fp;                  // The compiled filter expression
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

    bool ret = ScanNodes();
#if 0
    if (!ret) {
        pcap_close(handle);
        handle = 0;
    }
#endif
    return ret;
}

void EthRawPort::Cleanup(void)
{
    pcap_close(handle);
}

nodeid_t EthRawPort::InitNodes(void)
{
    quadlet_t data = 0x0;   // initialize data to 0

    // Check hardware version of hub board
    if (!ReadQuadletNode(FW_NODE_BROADCAST, 4, data, (FW_NODE_ETH_BROADCAST_MASK|FW_NODE_NOFORWARD_MASK))) {
        outStr << "InitNodes: failed to read hardware version for hub/bridge board" << std::endl;
        return 0;
    }
    if (data != QLA1_String) {
        outStr << "InitNodes: hub board is not a QLA board, data = " << std::hex << data << std::endl;
        return 0;
    }

    // ReadQuadletNode should have updated bus generation
    FwBusGeneration = newFwBusGeneration;
    outStr << "InitNodes: Firewire bus generation = " << FwBusGeneration << std::endl;

    // Broadcast a command to initiate a read of Firewire PHY Register 0. In cases where there is no
    // Firewire bus master (i.e., only FPGA/QLA boards on the Firewire bus), this allows each board
    // to obtain its Firewire node id.
    data = 0;
    if (!WriteQuadletNode(FW_NODE_BROADCAST, 1, data, FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to broadcast PHY command" << std::endl;
        return 0;
    }

    // Find board id for first board (i.e., one connected by Ethernet)
    if (!ReadQuadletNode(FW_NODE_BROADCAST, 0, data, (FW_NODE_ETH_BROADCAST_MASK|FW_NODE_NOFORWARD_MASK)))  {
        outStr << "InitNodes: failed to read board id for hub/bridge board" << std::endl;
        return 0;
    }
    // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
    HubBoard = (data & BOARD_ID_MASK) >> 24;
    outStr << "InitNodes: found hub board: " << static_cast<int>(HubBoard) << std::endl;

    // Scan for up to 16 nodes on bus
    return BoardIO::MAX_BOARDS;
}

bool EthRawPort::IsOK(void)
{
    return (handle != NULL);
}

unsigned int EthRawPort::GetPrefixOffset(MsgType msg) const
{
    switch (msg) {
        case WR_CTRL:      return ETH_FRAME_HEADER_SIZE;
        case WR_FW_HEADER: return ETH_FRAME_HEADER_SIZE+FW_CTRL_SIZE;
        case WR_FW_BDATA:  return ETH_FRAME_HEADER_SIZE+FW_CTRL_SIZE+FW_BWRITE_HEADER_SIZE;
        case RD_FW_HEADER: return ETH_FRAME_HEADER_SIZE;
        case RD_FW_BDATA:  return ETH_FRAME_HEADER_SIZE+FW_BRESPONSE_HEADER_SIZE;
    }
    outStr << "EthRawPort::GetPrefixOffset: Invalid type: " << msg << std::endl;
    return 0;
}

bool EthRawPort::PacketSend(unsigned char *packet, size_t nbytes, bool)
{
    if (pcap_sendpacket(handle, packet, nbytes) != 0)  {
        outStr << "ERROR: PCAP send packet failed" << std::endl;
        return false;
    }
    return true;
}

int EthRawPort::PacketReceive(unsigned char *recvPacket, size_t nbytes)
{
    struct pcap_pkthdr header;      /* The header that pcap gives us */
    const unsigned char *packet;    /* The actual packet */
    unsigned int numPackets = 0;
    unsigned int numPacketsValid = 0;
    double timeDiffSec = 0.0;
    unsigned int nRead = 0;

    double startTime = Amp1394_GetTime();
    while ((numPacketsValid < 1) && (timeDiffSec < ReceiveTimeout)) {
        packet = pcap_next(handle, &header);
        if (packet) {
            numPackets++;
            if (headercheck(packet, true)) {
                // Get length from Ethernet header
                nRead = bswap_16(*reinterpret_cast<const uint16_t *>(packet+ETH_FRAME_LENGTH_OFFSET));
                if (nRead < 1500) {
                    nRead += ETH_FRAME_HEADER_SIZE;
                }
                else {
                    // Shouldn't happen with raw Ethernet, but in case it does, use the capture length instead
                    nRead = header.caplen;
                }
                if (nRead == (ETH_FRAME_HEADER_SIZE + FW_EXTRA_SIZE)) {
                    outStr << "PacketReceive: only extra data" << std::endl;
                    ProcessExtraData(packet);
                    nRead = 0;
                }
                numPacketsValid++;
            }
        }
        timeDiffSec = Amp1394_GetTime() - startTime;
    }
    if (nRead > 0) {
        if (nRead > nbytes) {
            outStr << "PacketReceive: truncating packet from " << std::dec << nRead << " to "
                   << nbytes << " bytes" << std::endl;
            nRead = nbytes;
        }
        memcpy(recvPacket, packet, nRead);
    }
#if 0
    outStr << "Processed " << numPackets << " packets, " << numPacketsValid << " valid"
           << ", time = " << timeDiffSec << " sec" << std::endl;
#endif
    return static_cast<int>(nRead);
}

int EthRawPort::PacketFlushAll(void)
{
    struct pcap_pkthdr header;      /* The header that pcap gives us */

    int numFlushed = 0;
    while (pcap_next(handle, &header))
        numFlushed++;
    return numFlushed;
}

bool EthRawPort::CheckEthernetHeader(const unsigned char *packet, bool useEthernetBroadcast)
{
    if (!useEthernetBroadcast && (packet[11] != HubBoard)) {
        outStr << "WARNING: Packet not from node " << static_cast<unsigned int>(HubBoard) << " (src lsb is "
               << static_cast<unsigned int>(packet[11]) << ")" << std::endl;
        return false;
    }
    return true;
}

void EthRawPort::make_write_header(unsigned char *packet, unsigned int nBytes, unsigned char flags)
{
    make_ethernet_header(packet, nBytes, flags);
    EthBasePort::make_write_header(packet, nBytes, flags);
}

void EthRawPort::make_ethernet_header(unsigned char *packet, unsigned int numBytes, unsigned char flags)
{
    memcpy(packet, frame_hdr, ETH_FRAME_LENGTH_OFFSET);  // Copy header except length field
    if (flags&FW_NODE_ETH_BROADCAST_MASK) {      // multicast
        packet[0] |= 0x01;    // set multicast destination address
        packet[5] = 0xff;     // keep multicast address
    }
    else {
        packet[5] = HubBoard;     // last byte of dest address is board id
    }
    // length field (big endian 16-bit integer)
    *reinterpret_cast<uint16_t *>(packet+ETH_FRAME_LENGTH_OFFSET) = bswap_16(numBytes-ETH_FRAME_HEADER_SIZE);
}

/*!
 \brief Validate ethernet packet header

 \param header Ethernet packet header (14 bytes)
 \param toPC  true: check FPGA to PC, false: check PC to FPGA
 \return bool true if valid
*/
bool EthRawPort::headercheck(const unsigned char *header, bool toPC) const
{
    unsigned int i;
    const unsigned char *srcAddr;
    const unsigned char *destAddr;
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
