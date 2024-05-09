/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "EthRawPort.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"

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
#include <Packet32.h> // winpcap include for PacketRequest
#include <ntddndis.h> // for OID_802_3_CURRENT_ADDRESS
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#endif

EthRawPort::EthRawPort(int portNum, bool forceFwBridge, std::ostream &debugStream, EthCallbackType cb):
    EthBasePort(portNum, forceFwBridge, debugStream, cb)
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
    outStr << pcap_lib_version() << std::endl;
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

    // For pcap versions >= 1.5.0, need to set immediate mode;
    // otherwise, code can hang waiting for packets (e.g., when
    // trying to read from a board that does not exist on the bus).
    // In this case, we manually call pcap_create, pcap_set_snaplen,
    // pcap_set_promisc, pcap_set_timeout and pcap_activate rather
    // than calling pcap_open_live, so we can include the call to
    // pcap_set_immediate_mode.
    //
    // The PCAP_HAS_IMMEDIATE_MODE compiler definition is set by
    // CMake (check_cxx_source_compiles).

#ifdef PCAP_HAS_IMMEDIATE_MODE
    handle = pcap_create(dev->name, errbuf);
    if (handle == NULL)
    {
        outStr << "ERROR: Couldn't create device: "<< dev->name <<std::endl;
        return false;
    }
    if (pcap_set_snaplen(handle, BUFSIZ) < 0) {
        outStr << "ERRROR: Couldn't set pcap snaplen" << std::endl;
        pcap_close(handle);
        handle = NULL;
        return false;
    }
    if (pcap_set_promisc(handle, 0) < 0) {
        outStr << "ERRROR: Couldn't set pcap promisc" << std::endl;
        pcap_close(handle);
        handle = NULL;
        return false;
    }
    if (pcap_set_timeout(handle, 1) < 0) {
        outStr << "ERRROR: Couldn't set pcap timeout" << std::endl;
        pcap_close(handle);
        handle = NULL;
        return false;
    }
    if (pcap_set_immediate_mode(handle, 1) != 0) {
        outStr << "ERROR: Could not set pcap immediate mode" << std::endl;
        pcap_close(handle);
        handle = NULL;
        return false;
    }
    if (pcap_activate(handle) < 0) {
        outStr << "ERROR: Could not active pcap" << std::endl;
        pcap_close(handle);
        handle = NULL;
        return false;
    }
#else
    handle = pcap_open_live(dev->name,
                            BUFSIZ,  // data buffer size
                            0,       // turn off promisc mode
                            1,       // read timeout 1 ms
                            errbuf); // error buffer
    if (handle == NULL)
    {
        outStr << "ERROR: Couldn't open device: "<< dev->name <<std::endl;
        return false;
    }
#endif

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

    outStr << "pcap filter = " << ss_filter.str() << std::endl;

    if (pcap_compile(handle, &fp, ss_filter.str().c_str(), 0, PCAP_NETMASK_UNKNOWN) == -1) {
        outStr << "ERROR: could not parse filter " << ss_filter.str() << std::endl;
        return false;
    }

    if (pcap_setfilter(handle, &fp) == -1) {
        outStr << "ERROR: could not install filter " << ss_filter.str() << std::endl;
        return false;
    }

    memcpy(frame_hdr, eth_dst, 6);
    memcpy(frame_hdr+6, eth_src, 6);
    frame_hdr[12] = 0;   // length field
    frame_hdr[13] = 0;   // length field

    bool ret = ScanNodes();

    if (ret) {
        SetDefaultProtocol();
        if (useFwBridge)
            OptimizeFirewireGapCount();
    }

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
    if (!ReadQuadletNode(FW_NODE_BROADCAST, BoardIO::HARDWARE_VERSION, data,
                         FW_NODE_NOFORWARD_MASK | FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to read hardware version for hub/bridge board" << std::endl;
        return 0;
    }
    if (!HardwareVersionValid(data)) {
        outStr << "InitNodes: hub board is not a supported board, data = " << std::hex << data << std::endl;
        return 0;
    }

    // ReadQuadletNode should have updated bus generation
    FwBusGeneration = newFwBusGeneration;
    outStr << "InitNodes: Firewire bus generation = " << FwBusGeneration << std::endl;

    // Broadcast a command to initiate a read of Firewire PHY Register 0. In cases where there is no
    // Firewire bus master (i.e., only FPGA/QLA boards on the Firewire bus), this allows each board
    // to obtain its Firewire node id.
    data = 0;
    if (!WriteQuadletNode(FW_NODE_BROADCAST, BoardIO::FW_PHY_REQ, data, FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to broadcast PHY command" << std::endl;
        return 0;
    }
    Amp1394_Sleep(0.01);

    // Find board id for first board (i.e., one connected by Ethernet) by FireWire and Ethernet broadcast
    if (!ReadQuadletNode(FW_NODE_BROADCAST, BoardIO::BOARD_STATUS, data,
                         FW_NODE_NOFORWARD_MASK | FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to read board id for hub/bridge board" << std::endl;
        return 0;
    }
    // board_id is bits 27-24, BOARD_ID_MASK = 0x0f000000
    HubBoard = (data & BOARD_ID_MASK) >> 24;
    outStr << "InitNodes: found hub board: " << static_cast<int>(HubBoard) << std::endl;

    // read firmware version
    if (!ReadQuadletNode(FW_NODE_BROADCAST, BoardIO::FIRMWARE_VERSION, data,
                         FW_NODE_NOFORWARD_MASK | FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: unable to read firmware version from hub/bridge board" << std::endl;
        return 0;
    }
    unsigned long fver = data;
    FirmwareVersion[HubBoard] = data;

    // Get FPGA version of Hub board
    if (!ReadQuadletNode(FW_NODE_BROADCAST, BoardIO::ETH_STATUS, data,
                         FW_NODE_NOFORWARD_MASK | FW_NODE_ETH_BROADCAST_MASK)) {
        outStr << "InitNodes: failed to read Ethernet status from hub/bridge board" << std::endl;
        return 0;
    }
    unsigned long fpga_ver = BoardIO::GetFpgaVersionMajorFromStatus(data);
    FpgaVersion[HubBoard] = fpga_ver;

    if (!useFwBridge && ((fpga_ver != 3) || (fver < 9))) {
        if (fpga_ver != 3)
            outStr << "InitNodes: hub board is FPGA V" << fpga_ver;
        else if (fver < 9)
            outStr << "InitNodes: hub board has Firmware Rev " << fver;
        outStr << ", using Ethernet/Firewire bridge" << std::endl;
        useFwBridge = true;
    }
    else if (useFwBridge)
        outStr << "InitNodes: using Ethernet/Firewire bridge" << std::endl;
    else
        outStr << "InitNodes: using Ethernet network" << std::endl;

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

unsigned int EthRawPort::GetMaxReadDataSize(void) const
{
    return ETH_RAW_FRAME_MAX_SIZE - GetPrefixOffset(RD_FW_BDATA) - GetReadPostfixSize();
}

unsigned int EthRawPort::GetMaxWriteDataSize(void) const
{
    return ETH_RAW_FRAME_MAX_SIZE - GetPrefixOffset(WR_FW_BDATA) - GetWritePostfixSize();
}

bool EthRawPort::PacketSend(nodeid_t node, unsigned char *packet, size_t nbytes, bool)
{
    if (pcap_sendpacket(handle, packet, nbytes) != 0)  {
        outStr << "ERROR: PCAP send packet failed to node " << node << std::endl;
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
    if (!useEthernetBroadcast && useFwBridge && (packet[11] != HubBoard)) {
        outStr << "WARNING: Packet not from node " << static_cast<unsigned int>(HubBoard) << " (src lsb is "
               << static_cast<unsigned int>(packet[11]) << ")" << std::endl;
        return false;
    }
    return true;
}

void EthRawPort::make_ethernet_header(unsigned char *packet, unsigned int numBytes, nodeid_t node, unsigned char flags)
{
    memcpy(packet, frame_hdr, ETH_FRAME_LENGTH_OFFSET);  // Copy header except length field
    if ((flags&FW_NODE_ETH_BROADCAST_MASK) || (!useFwBridge && (node == FW_NODE_BROADCAST))) {      // multicast
        packet[0] |= 0x01;    // set multicast destination address
        packet[5] = 0xff;     // keep multicast address
    }
    else {
        // last byte of dest address is board id; when using Ethernet/Firewire bridge, this is the HubBoard;
        // otherwise it is the node (in Ethernet-only network, node-id and board-id are identical)
        packet[5] = useFwBridge ? HubBoard : node;
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
