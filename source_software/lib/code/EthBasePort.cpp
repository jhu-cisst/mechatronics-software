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

#include "EthBasePort.h"
#include "FpgaIO.h"
#include "Amp1394Time.h"
#include "Amp1394BSwap.h"
#include <iomanip>

#ifdef _MSC_VER
#include <string>
#else
#include <string.h>  // for memset
#endif

// crc related
uint32_t crc32(uint32_t crc, const void *buf, size_t size);


EthBasePort::EthBasePort(int portNum, bool forceFwBridge, std::ostream &debugStream, EthCallbackType cb):
    BasePort(portNum, debugStream),
    useFwBridge(forceFwBridge),
    fw_tl(0),
    eth_read_callback(cb),
    ReceiveTimeout(0.02)
{
}

EthBasePort::~EthBasePort()
{
}

void EthBasePort::GetDestMacAddr(unsigned char *macAddr)
{
    // CID,0x1394,boardid(0)
    macAddr[0] = 0xFA;
    macAddr[1] = 0x61;
    macAddr[2] = 0x0E;
    macAddr[3] = 0x13;
    macAddr[4] = 0x94;
    macAddr[5] = 0x00;
}

void EthBasePort::GetDestMulticastMacAddr(unsigned char *macAddr)
{
    // Same as GetDestMacAddr, except with multicast bit set and
    // last byte set to 0xFF.
    GetDestMacAddr(macAddr);
    macAddr[0] |= 0x01;
    macAddr[5] = 0xFF;
}

void EthBasePort::PrintMAC(std::ostream &outStr, const char* name, const uint8_t *addr, bool swap16)
{
    if (swap16) {
        outStr << name << ": " << std::hex << std::setw(2) << std::setfill('0')
               << (int)addr[1] << ":" << (int)addr[0] << ":" << (int)addr[3] << ":"
               << (int)addr[2] << ":" << (int)addr[5] << ":" << (int)addr[4] << std::dec << std::endl;
    }
    else {
        outStr << name << ": " << std::hex << std::setw(2) << std::setfill('0')
               << (int)addr[0] << ":" << (int)addr[1] << ":" << (int)addr[2] << ":"
               << (int)addr[3] << ":" << (int)addr[4] << ":" << (int)addr[5] << std::dec << std::endl;
    }
}

void EthBasePort::PrintIP(std::ostream &outStr, const char* name, const uint8_t *addr, bool swap16)
{
    if (swap16) {
        outStr << name << ": " << std::dec
               << (int)addr[1] << "." << (int)addr[0] << "." << (int)addr[3] << "." << (int)addr[2]
               << std::endl;
    }
    else {
        outStr << name << ": " << std::dec
               << (int)addr[0] << "." << (int)addr[1] << "." << (int)addr[2] << "." << (int)addr[3]
               << std::endl;
    }
}

void EthBasePort::ProcessExtraData(const unsigned char *packet)
{
    FpgaStatus.FwBusReset = (packet[0]&FwBusReset);
    FpgaStatus.FwPacketDropped = (packet[0]&FwPacketDropped);
    FpgaStatus.EthInternalError = (packet[0]&EthInternalError);
    FpgaStatus.EthSummaryError = (packet[0]&EthSummaryError);
    FpgaStatus.noForwardFlag = (packet[0]&noForwardFlag);
    FpgaStatus.srcPort = (packet[0]&srcPortMask)>>srcPortShift;
    FpgaStatus.numStateInvalid = packet[2];
    FpgaStatus.numPacketError = packet[3];
    unsigned int FwBusGeneration_FPGA = packet[1];

    const double FPGA_sysclk_MHz = 49.152;      /* FPGA sysclk in MHz (from AmpIO.cpp) */
    const unsigned short *packetW = reinterpret_cast<const unsigned short *>(packet);
    FPGA_RecvTime = bswap_16(packetW[2])/(FPGA_sysclk_MHz*1.0e6);
    FPGA_TotalTime = bswap_16(packetW[3])/(FPGA_sysclk_MHz*1.0e6);

    newFwBusGeneration = FwBusGeneration_FPGA;
    if ((FwBusGeneration_FPGA != FwBusGeneration) && !FpgaStatus.noForwardFlag)
        OnFwBusReset(FwBusGeneration_FPGA);
}

// Default implementation
bool EthBasePort::CheckEthernetHeader(const unsigned char *, bool)
{
    return true;
}

void EthBasePort::GetFirewireHeaderInfo(const unsigned char *packet, nodeid_t *src_node, unsigned int *tcode,
                                        unsigned int *tl)
{
    const quadlet_t *qpacket = reinterpret_cast<const quadlet_t *>(packet);
    if (tcode) *tcode = (qpacket[0]&0x000000F0)>>4;
    if (src_node) *src_node = (qpacket[1]>>16)&FW_NODE_MASK;
    if (tl) *tl = (qpacket[0]>>10)&FW_TL_MASK;
}

bool EthBasePort::CheckFirewirePacket(const unsigned char *packet, size_t length, nodeid_t node, unsigned int tcode,
                                      unsigned int tl)
{
    if (!checkCRC(packet)) {
        outStr << "CheckFirewirePacket: CRC error" << std::endl;
        return false;
    }

    nodeid_t src_node;
    unsigned int tcode_recv;
    unsigned int tl_recv;
    GetFirewireHeaderInfo(packet, &src_node, &tcode_recv, &tl_recv);

    if (tcode_recv != tcode) {
        outStr << "Unexpected tcode: received = " << tcode_recv << ", expected = " << tcode << std::endl;
        return false;
    }
    if ((node != FW_NODE_BROADCAST) && (src_node != node)) {
        outStr << "Inconsistent source node: received = " << src_node << ", expected = " << node << std::endl;
        return false;
    }
    if (tl_recv != tl) {
        outStr << "Inconsistent Firewire TL: received = " << tl_recv
               << ", expected = " << tl << std::endl;
        return false;
    }
    // TODO: could also check QRESPONSE length
    if (tcode == BRESPONSE) {
        const quadlet_t *qpacket = reinterpret_cast<const quadlet_t *>(packet);
        size_t length_recv = (qpacket[3]&0xffff0000) >> 16;
        if (length_recv != length) {
            outStr << "Inconsistent length: received = " << length_recv << ", expected = " << length << std::endl;
            return false;
        }
    }
    return true;
}

 // Assumes "normal" byte ordering
void EthBasePort::PrintFirewirePacket(std::ostream &out, const quadlet_t *packet, unsigned int max_quads)
{
    static const char *tcode_name[16] = { "qwrite", "bwrite", "wresponse", "", "qread", "bread",
                                          "qresponse", "bresponse", "cycstart", "lockreq",
                                          "stream", "lockresp", "", "", "", "" };
    unsigned char tcode = (packet[0]&0x000000F0)>>4;
    unsigned int data_length = 0;
    // No point in printing anything if less than 4
    if (max_quads < 4) {
        out << "PrintPacket: should print more than 4 quadlets (max_quads = "
            << max_quads << ")" << std::endl;
        return;
    }
    out << "Firewire Packet:" << std::endl;
    out << "  dest: " << std::hex << ((packet[0]&0xffc00000)>>20)
        << ", node: " << std::dec << ((packet[0]&0x003f0000)>>16)
        << ", tl: " << std::hex << std::setw(2) << std::setfill('0') << ((packet[0]&0x0000fc00)>>10)
        << ", rt: " << ((packet[0]&0x00000300)>>8)
        << ", tcode: " << static_cast<unsigned int>(tcode) << " (" << tcode_name[tcode] << ")"
        << ", pri: " << (packet[0]&0x0000000F) << std::endl;
    out << "  src: " << std::hex << ((packet[1]&0xffc00000)>>20)
        << ", node: " << std::dec << ((packet[1]&0x003f0000)>>16);

    if ((tcode == QRESPONSE) || (tcode == BRESPONSE)) {
        out << ", rcode: " << std::dec << ((packet[1]&0x0000f0000)>>12);
    }
    else if ((tcode == QWRITE) || (tcode == QREAD) || (tcode == BWRITE) || (tcode == BREAD)) {
        out << ", dest_off: " << std::hex << (packet[1]&0x0000ffff) << std::endl;
        out << "  dest_off: " << packet[2] << std::dec;
    }
    out << std::endl;

    if ((tcode == BWRITE) || (tcode == BRESPONSE) || (tcode == BREAD)) {
        data_length = (packet[3]&0xffff0000) >> 16;
        out << "  data_length: " << std::dec << data_length
            << ", ext_tcode: " << std::hex << (packet[3]&0x0000ffff) << std::dec << std::endl;
        if (data_length%4 != 0)
            out << "WARNING: data_length is not a multiple of 4" << std::endl;
    }
    else if ((tcode == QWRITE) || (tcode == QRESPONSE)) {
        out << "  data: " << std::hex << packet[3] << std::dec << std::endl;
    }

    if (tcode == QREAD)
        out << "  header_crc: " << std::hex << packet[3] << std::dec << std::endl;
    else if (max_quads < 5)  // Nothing else to do for short packets
        return;
    else
        out << "  header_crc: " << std::hex << packet[4] << std::dec << std::endl;

    if ((tcode == BWRITE) || (tcode == BRESPONSE)) {
        data_length /= sizeof(quadlet_t);   // data_length in quadlets
        unsigned int lim = (data_length <= max_quads-5) ? data_length : max_quads-5;
        for (unsigned int i = 0; i < lim; i += 4) {
            out << "  data[" << std::dec << std::setfill(' ') << std::setw(3) << i << ":"
                << std::setw(3) << (i+3) << "]:  ";
            for (unsigned int j = 0; (j < 4) && ((i+j) < lim); j++)
                out << std::hex << std::setw(8) << std::setfill('0') << packet[5+i+j] << std::dec << "  ";
            out << std::endl;
        }
        if ((data_length > 0) && (data_length < max_quads-5))
            out << "  data_crc: " << std::hex << packet[5+data_length] << std::dec << std::endl;
    }
}

void EthBasePort::PrintStatus(std::ostream &debugStream, uint32_t status)
{
    unsigned int ver = BoardIO::GetFpgaVersionMajorFromStatus(status);
    debugStream << "Status: ";
    if (ver == 0) {
        debugStream << "Unknown FPGA, status = " << std::hex << status << std::dec << std::endl;
    }
    else if (ver == 1) {
        // FPGA V1
        debugStream << "No Ethernet present" << std::endl;
    }
    else {
        // FPGA V2 or V3
        if (ver == 2) {
            if (status&FpgaIO::ETH_STAT_REQ_ERR_V2) debugStream << "error ";
            if (status&FpgaIO::ETH_STAT_INIT_OK_V2) debugStream << "initOK ";
        }
        if (status & FpgaIO::ETH_STAT_FRAME_ERR)    debugStream << "FrameErr ";
        if (status & FpgaIO::ETH_STAT_IPV4_ERR)     debugStream << "IPv4Err ";
        if (status & FpgaIO::ETH_STAT_UDP_ERR)      debugStream << "UDPErr ";
        if (status & FpgaIO::ETH_STAT_DEST_ERR)     debugStream << "DestErr ";
        if (status & FpgaIO::ETH_STAT_ACCESS_ERR)   debugStream << "AccessErr ";
        if ((ver == 2) && (status & FpgaIO::ETH_STAT_STATE_ERR_V2)) debugStream << "StateErr ";
        if ((ver == 3) && (status & FpgaIO::ETH_STAT_CLK125_OK_V3)) debugStream << "Clk125 ";
        if (status & FpgaIO::ETH_STAT_ETHST_ERR )   debugStream << "EthStateErr ";
        if ((ver == 3) && (status & FpgaIO::ETH_STAT_CLK200_OK_V3)) debugStream << "Clk200 ";
        if (status & FpgaIO::ETH_STAT_UDP)          debugStream << "UDP ";
        if (ver == 2) {
            if (status & FpgaIO::ETH_STAT_LINK_STAT_V2) debugStream << "Link-On ";
            else                                        debugStream << "Link-Off ";
            if (status & FpgaIO::ETH_STAT_IDLE_V2)  debugStream << "ETH-idle ";
            int waitInfo = (status & FpgaIO::ETH_STAT_WAIT_MASK_V2)>>16;
            if (waitInfo == 0) debugStream << "wait-none";
            else if (waitInfo == 1) debugStream << "wait-recv";
            else if (waitInfo == 2) debugStream << "wait-send";
            else debugStream << "wait-flush";
            debugStream << std::endl;
        }
        else {   // ver == 3
            // Following bit introduced in Firmware Rev 9, but was previously 0
            if (status & FpgaIO::ETH_STAT_PSETH_EN_V3) debugStream << "PS-Eth ";
            debugStream << std::endl;
            for (unsigned int port = 1; port <= 2; port++) {
                debugStream << "Port " << port << ": ";
                uint8_t portStatus = FpgaIO::GetEthernetPortStatusV3(status, port);
                if (portStatus & FpgaIO::ETH_PORT_STAT_INIT_OK)   debugStream << "initOK ";
                if (portStatus & FpgaIO::ETH_PORT_STAT_HAS_IRQ)   debugStream << "hasIRQ ";
                if (portStatus & FpgaIO::ETH_PORT_STAT_LINK_STAT) debugStream << "Link-On ";
                else                                              debugStream << "Link-Off ";
                uint8_t linkSpeed = (portStatus & FpgaIO::ETH_PORT_STAT_SPEED_MASK)>>3;
                if (linkSpeed == 0)                               debugStream << "10 Mbps ";
                else if (linkSpeed == 1)                          debugStream << "100 Mbps ";
                else if (linkSpeed == 2)                          debugStream << "1000 Mbps ";
                // Following are only for Firmware Rev 8
                if (portStatus & FpgaIO::ETH_PORT_STAT_RECV_ERR)  debugStream << "RecvErr ";
                if (portStatus & FpgaIO::ETH_PORT_STAT_SEND_OVF)  debugStream << "SendOvf ";
                if (portStatus & FpgaIO::ETH_PORT_STAT_PS_ETH)    debugStream << "PS-Eth ";
                debugStream << std::endl;
            }
        }
    }
}

bool EthBasePort::CheckDebugHeader(std::ostream &debugStream, const std::string &caller, const char *header,
                                   unsigned int version)
{
    if (strncmp(header, "DBG", 3) != 0) {
        debugStream << caller << ": Unexpected header string: " << header[0] << header[1]
                    << header[2] << " (should be DBG)" << std::endl;
        return false;
    }
    unsigned int debugLevel = header[3] - '0';
    if (debugLevel == 0) {
        debugStream << caller << ": No debug data available" << std::endl;
        return false;
    }
    // Introduced debugLevel 2 with Firmware V8. Use an older version of this software for
    // earlier versions of firmware.
    else if (debugLevel != version) {
        debugStream << caller << ": Unsupported debug level: " << debugLevel << " (must be "
                    << version << ")" << std::endl;
        return false;
    }
    return true;
}

// Prints the debug data from the higher-level Ethernet module (EthernetIO)
void EthBasePort::PrintDebugData(std::ostream &debugStream, const quadlet_t *data, double clockPeriod)
{
    // Following structure must match DebugData in EthernetIO.v
    struct DebugData {
        char     header[4];        // Quad 0
        uint32_t timestampBegin;   // Quad 1
        uint32_t statusbits;       // Quad 2
        uint16_t LengthFW;         // Quad 3
        uint16_t quad3_high;
        uint16_t Host_FW_Addr;     // Quad 4
        uint16_t FwCtrl;
        uint16_t quad5_low;        // Quad 5
        uint16_t quad5_high;
        uint16_t numIPv4;          // Quad 6
        uint16_t numUDP;
        uint8_t  numARP;           // Quad 7
        uint8_t  fw_bus_gen;
        uint8_t  numICMP;
        uint8_t  br_wait_cnt;
        uint8_t  numPacketError;   // Quad 8
        uint8_t  bwState;
        uint16_t bw_left;
        uint8_t  fw_wait_cnt;      // Quad 9
        uint8_t  fwState;
        uint16_t fw_left;
        uint16_t port_unknown;     // Quad 10
        uint8_t  numMulticastWrite;
        uint8_t  quad10_high;
        uint32_t unused[5];        // Quads 10-15
    };
    if (sizeof(DebugData) != 16*sizeof(quadlet_t)) {
        debugStream << "PrintDebugData: structure packing problem" << std::endl;
        return;
    }
    const DebugData *p = reinterpret_cast<const DebugData *>(data);
    if (!CheckDebugHeader(debugStream, "PrintDebugData", p->header, 2))
        return;

    debugStream << "Timestamp: " << std::hex << p->timestampBegin << std::dec << std::endl;
    if (p->statusbits & 0x80000000) debugStream << "wrReq ";
    if (p->statusbits & 0x20000000) debugStream << "bw_active ";
    if (p->statusbits & 0x10000000) debugStream << "eth_send_idle ";
    if (p->statusbits & 0x08000000) debugStream << "eth_recv_idle ";
    if (p->statusbits & 0x04000000) debugStream << "UDPError ";
    if (p->statusbits & 0x01000000) debugStream << "IPV4Error ";
    if (p->statusbits & 0x00800000) debugStream << "sendBusy ";
    if (p->statusbits & 0x00400000) debugStream << "sendRequest ";
    if (p->statusbits & 0x00200000) debugStream << "eth_send_fw_ack ";
    if (p->statusbits & 0x00100000) debugStream << "eth_send_fw_req ";
    debugStream << std::endl;
    if (p->statusbits & 0x00080000) debugStream << "recvBusy ";
    if (p->statusbits & 0x00040000) debugStream << "recvRequest ";
    if (p->statusbits & 0x00020000) debugStream << "isLocal ";
    if (p->statusbits & 0x00010000) debugStream << "isRemote ";
    if (p->statusbits & 0x00008000) debugStream << "fwPacketFresh ";
    if (p->statusbits & 0x00004000) debugStream << "isForward ";
    if (p->statusbits & 0x00002000) debugStream << "sendARP ";
    if (p->statusbits & 0x00001000) debugStream << "isUDP ";
    if (p->statusbits & 0x00000800) debugStream << "isICMP  ";
    if (p->statusbits & 0x00000400) debugStream << "isEcho ";
    if (p->statusbits & 0x00000200) debugStream << "ipv4_long ";
    if (p->statusbits & 0x00000100) debugStream << "ipv4_short ";
    if (p->statusbits & 0x00000080) debugStream << "fw_bus_reset ";
    if (p->statusbits & 0x00000040) debugStream << "ipWrite ";
    if (p->statusbits & 0x00000020) debugStream << "hubSend ";
    if (p->statusbits & 0x00000010) debugStream << "bcResp ";
    debugStream << std::endl;
    unsigned int node_id = (p->quad3_high&0xfc00) >> 10;    // node_is is upper 6 bits
    debugStream << "FireWire node_id: " << node_id << std::endl;
    debugStream << "LengthFW: " << std::dec << p->LengthFW << std::endl;
    debugStream << "MaxCountFW: " << std::dec << (p->quad3_high&0x3ff) << std::endl;
    debugStream << "FwCtrl: " << std::hex << p->FwCtrl << std::endl;
    debugStream << "Host FW Addr: " << std::hex << p->Host_FW_Addr << std::endl;
    debugStream << "Fw Bus Generation: " << std::dec << static_cast<uint16_t>(p->fw_bus_gen);
    debugStream << std::endl;
    debugStream << "sendState: "   << std::dec << (p->quad5_high>>12)
                << ", recvState: " << ((p->quad5_low&0x7000)>>12)
                << ", nextRecv: "  << ((p->quad5_low&0x0700)>>8)
                << ", recvCnt: " << (p->quad5_low&0x003f) << std::endl;
    debugStream << "numIPv4: " << std::dec << p->numIPv4 << std::endl;
    debugStream << "numUDP: " << std::dec << p->numUDP << std::endl;
    debugStream << "numARP: " << std::dec << static_cast<uint16_t>(p->numARP) << std::endl;
    debugStream << "numICMP: " << std::dec << static_cast<uint16_t>(p->numICMP) << std::endl;
    debugStream << "numMulticastWrite: " << std::dec << static_cast<uint16_t>(p->numMulticastWrite) << std::endl;
    debugStream << "br_wait: " << std::dec << static_cast<uint16_t>(p->br_wait_cnt) << std::endl;
    debugStream << "numPacketError: " << std::dec << static_cast<uint16_t>(p->numPacketError) << std::endl;
    debugStream << "bwState: " << std::dec << static_cast<uint16_t>(p->bwState & 0x07);
    if (p->bwState & 0x80) debugStream << ", bw_err";
    debugStream << std::endl << "bw_left: " << std::dec << p->bw_left << std::endl;
    debugStream << "fw_left: " << p->fw_left;
    if (p->fwState & 0x80) debugStream << ", fw_err";
    debugStream << ", fw_wait_cnt: " << static_cast<uint16_t>(p->fw_wait_cnt) << std::endl;
    if (p->port_unknown)
        debugStream << "Unsupported UDP port: " << p->port_unknown << std::endl;
}

// Prints the debug data from the lower-level Ethernet module (KSZ8851 for Ethernet V2)
void EthBasePort::PrintDebugDataKSZ(std::ostream &debugStream, const quadlet_t *data, double clockPeriod)
{
    // Following structure must match DebugData in KSZ8851.v
    struct DebugData {
        char     header[4];        // Quad 0
        uint16_t quad1_low;        // Quad 1
        uint16_t statusbits;
        uint8_t  runPC;            // Quad 2
        uint8_t  nextState;
        uint8_t  retState;
        uint8_t  state;
        uint16_t regISROther;      // Quad 3
        uint16_t respBytes;
        uint8_t  numPacketSent;    // Quad 4
        uint8_t  frameCount;
        uint16_t bw_wait;
        uint16_t rxPktWords;      // Quad 5
        uint16_t txPktWords;
        uint16_t timeReceive;     // Quad 6
        uint16_t timeSend;
        uint16_t numPacketValid;  // Quad 7
        uint8_t  numPacketInvalid;
        uint8_t  numReset;
        uint32_t unused[8];       // Unused (except if DEBOUNCE_STATES)
    };
    if (sizeof(DebugData) != 16*sizeof(quadlet_t)) {
        debugStream << "PrintDebugDataKSZ: structure packing problem" << std::endl;
        return;
    }
    const DebugData *p = reinterpret_cast<const DebugData *>(data);
    if (!CheckDebugHeader(debugStream, "PrintDebugDataKSZ", p->header, 2))
        return;

    debugStream << "*** KSZ: ";
    if (p->statusbits & 0x8000) debugStream << "isDMAWrite ";
    if (p->statusbits & 0x4000) debugStream << "sendRequest ";
    if (p->statusbits & 0x2000) debugStream << "IRQ ";
    if (p->statusbits & 0x1000) debugStream << "isInIRQ ";
    if (p->statusbits & 0x0800) debugStream << "link-on ";
    debugStream << std::endl;
    debugStream << "State: " << static_cast<uint16_t>(p->state)
                << ", nextState: " << static_cast<uint16_t>(p->nextState)
                << ", retState: " << static_cast<uint16_t> (p->retState)
                << ", PC: " << static_cast<uint16_t>(p->runPC) << std::endl;
    debugStream << "RegISROther: " << std::hex << p->regISROther << std::endl;
    debugStream << "FrameCount: " << std::dec << static_cast<uint16_t>(p->frameCount) << std::endl;
    debugStream << "numReset: " << std::dec << static_cast<uint16_t>(p->numReset) << std::endl;
    debugStream << "numPacketValid: " << std::dec << p->numPacketValid << std::endl;
    debugStream << "numPacketInvalid: " << std::dec << static_cast<uint16_t>(p->numPacketInvalid) << std::endl;
    debugStream << "numPacketSent: " << std::dec << static_cast<uint16_t>(p->numPacketSent) << std::endl;
    debugStream << "rxPktWords: " << std::dec << p->rxPktWords << std::endl;
    debugStream << "txPktWords: " << std::dec << p->txPktWords << std::endl;
    debugStream << "respBytes: " << p->respBytes << std::endl;
    const double bits2uS = clockPeriod*1e6;
    debugStream << "timeReceive (us): " << p->timeReceive*bits2uS << std::endl;
    debugStream << "timeSend (us): " << p->timeSend*bits2uS << std::endl;
    debugStream << "bw_wait: " << std::dec << p->bw_wait << " (" << p->bw_wait*bits2uS << " us)" << std::endl;
}

void EthBasePort::PrintDebugDataRTL(std::ostream &debugStream, const quadlet_t *data, const char *portName)
{
    struct DebugData {
        uint32_t mdio_result;
        uint16_t numIRQ;
        uint16_t numReset;
    };
    const DebugData *p = reinterpret_cast<const DebugData *>(data);
    debugStream << "*** " << portName << " (RTL8211F): " << std::endl;
    debugStream << "numReset: " << p->numReset << ", numIRQ: " << p->numIRQ << std::endl;
}

// Prints the debug data from the lower-level real-time Ethernet interface (EthRtInterface for FPGA V3)
void EthBasePort::PrintDebugDataRTI(std::ostream &debugStream, const quadlet_t *data, double clockPeriod)
{
    // Following structure must match DebugData in EthRtInterface.v
    struct DebugData {
        char      header[4];        // Quad 0
        uint16_t  recv_nbytes;      // Quad 1
        uint8_t   numRecv;
        uint8_t   statusbits;
        uint8_t   numSent;          // Quad 2
        uint8_t   txStateBits;
        uint16_t  respBytes;
        uint16_t  timeReceive;      // Quad 3
        uint16_t  timeSend;
        uint16_t  quad4_low;        // Quad 4
        uint16_t  bw_wait;
        uint32_t  unused[3];        // Quad 5-7
    };
    if (sizeof(DebugData) != 8*sizeof(quadlet_t)) {
        debugStream << "PrintDebugDataRTI: structure packing problem" << std::endl;
        return;
    }
    const DebugData *p = reinterpret_cast<const DebugData *>(data);
    if (!CheckDebugHeader(debugStream, "PrintDebugDataRTI", p->header, 3))
        return;

    debugStream << "*** RTI: ";
    if (p->statusbits & 0x80) debugStream << "sendBusy ";
    if (p->statusbits & 0x40) debugStream << "sendRequest ";
    if (p->statusbits & 0x08) debugStream << "responseRequired ";
    if (p->statusbits & 0x04) debugStream << "recvNotReady ";
    if (p->statusbits & 0x02) debugStream << "recvReady ";
    if (p->statusbits & 0x01) debugStream << "PortReady ";
    debugStream << std::endl << "rxState: " << static_cast<uint16_t>((p->statusbits&0x30)>>4) << std::endl;
    debugStream << "numPacketRecv: " << static_cast<uint16_t>(p->numRecv)
                << ", numBytesRecv: " << p->recv_nbytes << std::endl;
    debugStream << "numPacketSent: " << static_cast<uint16_t>(p->numSent) << std::endl;
    debugStream << "txState: " << static_cast<uint16_t>(p->txStateBits&0x07);
    if (p->txStateBits&0x10) debugStream << ", txStateError ";
    debugStream << std::endl << "respBytes: " << p->respBytes << std::endl;
    const double bits2uS = clockPeriod*1e6;
    debugStream << "timeReceive: " << (p->timeReceive*clockPeriod) << " timeSend: " << (p->timeSend*clockPeriod) << std::endl;
    debugStream << "bw_wait: " << std::dec << p->bw_wait << " (" << p->bw_wait*bits2uS << " us)" << std::endl;
}

void EthBasePort::PrintEthernetPacket(std::ostream &out, const quadlet_t *packet, unsigned int max_quads)
{
    struct FrameHeader {
        uint8_t  destMac[6];
        uint8_t  srcMac[6];
        uint16_t etherType;
    };
    struct IPv4Header {
        uint16_t word0;
        uint16_t length;
        uint16_t ident;
        uint16_t flags;
        uint8_t  protocol;
        uint8_t  ttl;
        uint16_t checksum;
        uint8_t  hostIP[4];
        uint8_t  destIP[4];
    };
    struct UDPHeader {
        uint16_t hostPort;
        uint16_t destPort;
        uint16_t length;
        uint16_t checksum;
    };
    struct ARPHeader {
        uint16_t htype;
        uint16_t ptype;
        uint8_t  plen;
        uint8_t  hlen;
        uint16_t oper;
        uint8_t  srcMac[6];
        uint8_t  srcIP[4];
        uint8_t  destMac[6];  // ignored
        uint8_t  destIP[4];
    };
    // Easier to work with 16-bit data, since that is how it is stored on FPGA.
    const uint16_t *packetw = reinterpret_cast<const uint16_t *>(packet);

    const FrameHeader *frame = reinterpret_cast<const FrameHeader *>(packetw);
    out << "Ethernet Frame:" << std::endl;
    EthBasePort::PrintMAC(out, "  Dest MAC", frame->destMac, true);
    EthBasePort::PrintMAC(out, "  Src MAC", frame->srcMac, true);
    out << "  Ethertype/Length: " << std::hex << std::setw(4) << std::setfill('0') << frame->etherType << std::dec;
    if (frame->etherType == 0x0800) out << " (IPv4)";
    else if (frame->etherType == 0x0806) out << " (ARP)";
    out << std::endl;
    if (frame->etherType == 0x0800) {
        const IPv4Header *ipv4 = reinterpret_cast<const IPv4Header *>(packetw+7);
        out << "  IPv4:" << std::endl;
        out << "    Version: " << std::dec << ((ipv4->word0&0xf000)>>12) << ", IHL: " << ((ipv4->word0&0x0f00)>>8) << std::endl;
        out << "    Length: " << std::dec << ipv4->length << std::endl;
        unsigned int flags = (ipv4->flags&0xc0)>>13;
        out << "    Flags: " << std::dec << flags;
        if (flags == 2) out << " (DF)";
        out << ", TTL: " << static_cast<unsigned int>(ipv4->ttl) << std::endl;
        out << "    Protocol: " << static_cast<unsigned int>(ipv4->protocol);
        if (ipv4->protocol == 1) out << " (ICMP)";
        else if (ipv4->protocol == 17) out << " (UDP)";
        out << std::endl;
        EthBasePort::PrintIP(out, "    Host IP", ipv4->hostIP, true);
        EthBasePort::PrintIP(out, "    Dest IP", ipv4->destIP, true);
        if (ipv4->protocol == 1) {
            const uint8_t *icmp = reinterpret_cast<const uint8_t *>(packetw+17);
            out << "    ICMP:" << std::endl;
            out << "      Type: " << static_cast<unsigned int>(icmp[1])
                << ", Code: " << static_cast<unsigned int>(icmp[0]);
            if ((icmp[1] == 8) && (icmp[0] == 0)) out << " (Echo Request)";
            out << std::endl;
        }
        else if (ipv4->protocol == 17) {
            const UDPHeader *udp = reinterpret_cast<const UDPHeader *>(packetw+17);
            out << "    UDP:" << std::endl;
            out << std::dec << "      Host Port: " << udp->hostPort << std::endl;
            out << std::dec << "      Dest Port: " << udp->destPort << std::endl;
            out << std::dec << "      Length: " << udp->length << std::endl;
        }
    }
    else if (frame->etherType == 0x0806) {
        const ARPHeader *arp = reinterpret_cast<const ARPHeader *>(packetw+17);
        out << "  ARP:" << std::endl;
        out << std::hex << "    htype:" << arp->htype << ", ptype:" << arp->ptype
            << ", hlen:" << static_cast<unsigned int>(arp->hlen)
            << ", plen: " << static_cast<unsigned int>(arp->plen)
            << ", oper:" << arp->oper << std::dec << std::endl;
        EthBasePort::PrintMAC(out, "    Src MAC", arp->srcMac, true);
        EthBasePort::PrintIP(out, "    Src IP", arp->srcIP, true);
        EthBasePort::PrintIP(out, "    Dest IP", arp->destIP, true);
    }
    else {
        out << "  Raw frame (len = " << std::dec << frame->etherType << ")" << std::endl;
    }
}

bool EthBasePort::ReadQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t &data, unsigned char flags)
{
    if ((node != FW_NODE_BROADCAST) && !CheckFwBusGeneration("ReadQuadlet"))
        return false;

    // Flush before reading
    int numFlushed = PacketFlushAll();
    if (numFlushed > 0)
        outStr << "ReadQuadlet: flushed " << numFlushed << " packets" << std::endl;

    // Increment transaction label
    fw_tl = (fw_tl+1)&FW_TL_MASK;

    SetGenericBuffer();   // Make sure buffer is allocated

    unsigned char *sendPacket = GenericBuffer+GetWriteQuadAlign();
    unsigned int sendPacketSize = GetPrefixOffset(WR_FW_HEADER)+FW_QREAD_SIZE;

    // Make Ethernet header (only needed for raw Ethernet)
    make_ethernet_header(sendPacket, sendPacketSize, node, flags);

    // Make control word
    make_write_header(sendPacket, sendPacketSize, flags);

    // Build FireWire packet
    make_qread_packet(reinterpret_cast<quadlet_t *>(sendPacket+GetPrefixOffset(WR_FW_HEADER)), node, addr, fw_tl);
    if (!PacketSend(node, sendPacket, sendPacketSize, flags&FW_NODE_ETH_BROADCAST_MASK))
        return false;

    // Invoke callback (if defined) between sending read request
    // and checking for read response. If callback returns false, we
    // skip checking for a received packet.
    if (eth_read_callback && !(*eth_read_callback)(*this, node, outStr)) {
        outStr << "ReadQuadlet: callback aborting (not reading packet)" << std::endl;
        return false;
    }

    unsigned char *recvPacket = GenericBuffer+GetReadQuadAlign();
    unsigned int recvPacketSize = GetPrefixOffset(RD_FW_HEADER)+FW_QRESPONSE_SIZE+FW_EXTRA_SIZE;
    int nRecv = PacketReceive(recvPacket, recvPacketSize);
    if (nRecv != static_cast<int>(recvPacketSize)) {
        // Only print message if Node2Board contains valid board number, to avoid unnecessary error messages during ScanNodes.
        unsigned int boardId = Node2Board[node];
        if (boardId < BoardIO::MAX_BOARDS) {
            outStr << "ReadQuadlet: failed to receive read response from board " << boardId
                   << " via UDP: return value = " << nRecv
                   << ", expected = " << recvPacketSize << std::endl;
        }
        return false;
    }

    ProcessExtraData(recvPacket+GetPrefixOffset(RD_FW_HEADER)+FW_QRESPONSE_SIZE);

    if (!CheckEthernetHeader(recvPacket, flags&FW_NODE_ETH_BROADCAST_MASK))
        return false;
    // Byteswap Firewire header (also swaps quadlet data)
    ByteswapQuadlets(recvPacket + GetPrefixOffset(RD_FW_HEADER), FW_QRESPONSE_SIZE);
    if (!CheckFirewirePacket(recvPacket+GetPrefixOffset(RD_FW_HEADER), 0, node, EthBasePort::QRESPONSE, fw_tl))
        return false;

    const quadlet_t *packet_FW = reinterpret_cast<const quadlet_t *>(recvPacket+GetPrefixOffset(RD_FW_HEADER));
    data = packet_FW[3];
    return true;
}

bool EthBasePort::WriteQuadletNode(nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned char flags)
{
    if ((node != FW_NODE_BROADCAST) && !CheckFwBusGeneration("WriteQuadlet"))
        return false;

    // Use GenericBuffer, which is much larger than needed
    SetGenericBuffer();   // Make sure buffer is allocated
    unsigned char *packet = GenericBuffer+GetWriteQuadAlign();
    unsigned int packetSize = GetPrefixOffset(WR_FW_HEADER)+FW_QWRITE_SIZE;

    // Increment transaction label
    fw_tl = (fw_tl+1)&FW_TL_MASK;

    // Make Ethernet header (only needed for raw Ethernet)
    make_ethernet_header(packet, packetSize, node, flags);

    // Make control word
    make_write_header(packet, packetSize, flags);

    // Build FireWire packet (also byteswaps data)
    make_qwrite_packet(reinterpret_cast<quadlet_t *>(packet+GetPrefixOffset(WR_FW_HEADER)), node, addr, data, fw_tl);

    return PacketSend(node, packet, packetSize, flags&FW_NODE_ETH_BROADCAST_MASK);
}

bool EthBasePort::ReadBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *rdata,
                                unsigned int nbytes, unsigned char flags)
{
    if ((node != FW_NODE_BROADCAST) && !CheckFwBusGeneration("ReadBlock"))
        return false;

    // Flush before reading
    int numFlushed = PacketFlushAll();
    if (numFlushed > 0)
        outStr << "ReadBlock: flushed " << numFlushed << " packets" << std::endl;

    // Create buffer that is large enough for Firewire packet
    SetGenericBuffer();   // Make sure buffer is allocated
    unsigned char *sendPacket = GenericBuffer+GetWriteQuadAlign();
    unsigned int sendPacketSize = GetPrefixOffset(WR_FW_HEADER)+FW_BREAD_SIZE;

    // Increment transaction label
    fw_tl = (fw_tl+1)&FW_TL_MASK;

    // Make Ethernet header (only needed for raw Ethernet)
    make_ethernet_header(sendPacket, sendPacketSize, node, flags);

    // Make control word
    make_write_header(sendPacket, sendPacketSize, flags);

    // Build FireWire packet
    make_bread_packet(reinterpret_cast<quadlet_t *>(sendPacket+GetPrefixOffset(WR_FW_HEADER)), node, addr, nbytes, fw_tl);
    if (!PacketSend(node, sendPacket, sendPacketSize, flags&FW_NODE_ETH_BROADCAST_MASK))
        return false;

    // Invoke callback (if defined) between sending read request
    // and checking for read response. If callback returns false, we
    // skip checking for a received packet.
    if (eth_read_callback && !(*eth_read_callback)(*this, node, outStr)) {
        outStr << "ReadBlock: callback aborting (not reading packet)" << std::endl;
        return false;
    }

    return ReceiveResponseNode(node, rdata, nbytes, fw_tl);
}

bool EthBasePort::ReceiveResponseNode(nodeid_t node, quadlet_t *rdata,
                                      unsigned int nbytes, uint8_t fw_tl, nodeid_t *src_node)
{
    // Packet to receive
    unsigned char *packet = GenericBuffer+GetReadQuadAlign();;
    unsigned int packetSize = GetPrefixOffset(RD_FW_BDATA) + nbytes + GetReadPostfixSize();

    // Check for real-time read
    unsigned char *rdata_base = reinterpret_cast<unsigned char *>(rdata)-GetReadQuadAlign()-GetPrefixOffset(RD_FW_BDATA);
    if (rdata_base == ReadBufferBroadcast) {
        packet = ReadBufferBroadcast;
    }

    int nRecv = PacketReceive(packet, packetSize);
    if (nRecv != static_cast<int>(packetSize)) {
        outStr << "ReadBlock: failed to receive read response";
        unsigned char boardId = Node2Board[node];
        if (boardId < BoardIO::MAX_BOARDS)
            outStr << " from board " << static_cast<unsigned int>(boardId);
        outStr << ": return value = " << nRecv
               << ", expected = " << packetSize << std::endl;
        return false;
    }

    ProcessExtraData(packet+packetSize-FW_EXTRA_SIZE);

    if (!CheckEthernetHeader(packet, false))
        return false;
    // Byteswap Firewire header
    unsigned char *packetFw = packet + GetPrefixOffset(RD_FW_HEADER);
    ByteswapQuadlets(packetFw, FW_BRESPONSE_HEADER_SIZE);
    if (!CheckFirewirePacket(packetFw, nbytes, node, EthBasePort::BRESPONSE, fw_tl))
        return false;
    if (src_node)
        GetFirewireHeaderInfo(packetFw, src_node, 0, 0);

    const quadlet_t *packet_data = reinterpret_cast<const quadlet_t *>(packet+GetPrefixOffset(RD_FW_BDATA));
    if (rdata != packet_data) {
        rtRead = false;
        memcpy(rdata, packet_data, nbytes);
    }
    return true;
}


void EthBasePort::ByteswapQuadlets(unsigned char *packet, unsigned int nbytes)
{
    quadlet_t *qp = (quadlet_t *)packet;
    unsigned int nQuads = nbytes/sizeof(quadlet_t);
    for (unsigned int i = 0; i < nQuads; i++)
        qp[i] = bswap_32(qp[i]);
}


bool EthBasePort::WriteBlockNode(nodeid_t node, nodeaddr_t addr, quadlet_t *wdata,
                                 unsigned int nbytes, unsigned char flags)
{
    if ((node != FW_NODE_BROADCAST) && !CheckFwBusGeneration("WriteBlock"))
        return false;

    // Packet to send
    SetGenericBuffer();   // Make sure buffer is allocated
    unsigned char *packet = GenericBuffer+GetWriteQuadAlign();
    size_t packetSize = GetPrefixOffset(WR_FW_BDATA) + nbytes + GetWritePostfixSize();

    // Check for real-time write
    unsigned char *wdata_base = reinterpret_cast<unsigned char *>(wdata)-GetWriteQuadAlign()-GetPrefixOffset(WR_FW_BDATA);
    if (wdata_base == WriteBufferBroadcast) {
        packet = WriteBufferBroadcast+GetWriteQuadAlign();
    }

    // Increment transaction label
    fw_tl = (fw_tl+1)&FW_TL_MASK;

    // Make Ethernet header (only needed for raw Ethernet)
    make_ethernet_header(packet, packetSize, node, flags);

    // Make control word
    make_write_header(packet, packetSize, flags);

    // Build FireWire packet
    make_bwrite_packet(reinterpret_cast<quadlet_t *>(packet+GetPrefixOffset(WR_FW_HEADER)), node, addr, wdata, nbytes, fw_tl);

    // Now, send the packet
    return PacketSend(node, packet, packetSize, flags&FW_NODE_ETH_BROADCAST_MASK);
}

void EthBasePort::OnNoneRead(void)
{
    outStr << "Failed to read any board, check Ethernet physical connection" << std::endl;
}

void EthBasePort::OnNoneWritten(void)
{
    outStr << "Failed to write any board, check Ethernet physical connection" << std::endl;
}

void EthBasePort::OnFwBusReset(unsigned int FwBusGeneration_FPGA)
{
    outStr << "Firewire bus reset, FPGA = " << std::dec << FwBusGeneration_FPGA << ", PC = " << FwBusGeneration << std::endl;
}

bool EthBasePort::CheckFwBusGeneration(const std::string &caller, bool doScan)
{
    bool ret = true;
    if (useFwBridge)
        ret = BasePort::CheckFwBusGeneration(caller, doScan);
    return ret;
}

bool EthBasePort::WriteBroadcastOutput(quadlet_t *buffer, unsigned int size)
{
    return WriteBlockNode(FW_NODE_BROADCAST, 0, buffer, size);
}

bool EthBasePort::WriteBroadcastReadRequest(unsigned int seq)
{
    if (!useFwBridge) {
        // Ethernet-only system automatically sends a response packet in response to
        // the broadcast read request, so we first flush any existing packets.
        // When using the Ethernet/Firewire bridge, the flush happens in ReadBlockNode,
        // which is called by BasePort::ReceiveBroadcastReadResponse.
        int numFlushed = PacketFlushAll();
        if (numFlushed > 0)
            outStr << "WriteBroadcastReadRequest: flushed " << numFlushed << " packets"
                   << ", seq = " << seq << std::endl;
    }
    quadlet_t bcReqData = (seq << 16) | BoardInUseMask_;
    return WriteQuadlet(FW_NODE_BROADCAST, 0x1800, bcReqData);
}

void EthBasePort::WaitBroadcastRead(void)
{
    // Wait for all boards to respond with data
    // Ethernet/Firewire bridge: 10 + 10 * (Nb-1) us, where Nb is number of boards used in this configuration
    // Ethernet-only: 3 + 38 * (Nb-1) us, based on measurements, assuming contiguous Ethernet chain
    // Note that for Ethernet-only, it is not necessary to wait because the FPGA sends the broadcast read
    // response packet when all data is ready. Thus, the only advantage to waiting here is that we avoid
    // possible timeouts when receiving the response packet.
    double waitTime_uS = useFwBridge ? (10.0 + 10.0*(NumOfBoards_-1)) : (3.0 + 38.0*(NumOfBoards_-1));
    // Check the most recent measured update time, and use that if it is longer than
    // the computed waitTime_uS.
    double bcUpdateTime_uS = (bcReadInfo.updateFinishTime-bcReadInfo.updateStartTime)*1e6;
    if (bcUpdateTime_uS > waitTime_uS)
        waitTime_uS = bcUpdateTime_uS;
    Amp1394_Sleep(waitTime_uS*1e-6);
}

bool EthBasePort::ReceiveBroadcastReadResponse(quadlet_t *rdata, unsigned int nbytes)
{
    bool ret;
    if (useFwBridge) {
        ret = BasePort::ReceiveBroadcastReadResponse(rdata, nbytes);
    }
    else {
        nodeid_t src_node;
        // Use FW_NODE_BROADCAST because we do not care which board responds
        ret = ReceiveResponseNode(FW_NODE_BROADCAST, rdata, nbytes, fw_tl, &src_node);
        if (ret) {
            unsigned char newHubBoard = Node2Board[src_node];
            if (newHubBoard >= BoardIO::MAX_BOARDS) {
                // Should not happen
                outStr << "ReceiveBroadcastReadResponse: invalid source node " << src_node << std::endl;
            }
            else if (newHubBoard != HubBoard) {
                // Not printing message because current hub board is shown in qladisp
                // outStr << "ReceiveBroadcastReadResponse: changing hub board from "
                //        << static_cast<unsigned int>(HubBoard) << " to "
                //        << static_cast<unsigned int>(newHubBoard) << std::endl;
                HubBoard = newHubBoard;
            }
        }
        else {
            outStr << "ReceiveBroadcastReadResponse failed -- reading from board" << std::endl;
            ret = BasePort::ReceiveBroadcastReadResponse(rdata, nbytes);
        }
    }
    return ret;
}

bool EthBasePort::isBroadcastReadOrdered(void) const
{
    return useFwBridge;
}

double EthBasePort::GetBroadcastReadClockPeriod(void) const
{
    return useFwBridge ? BasePort::GetBroadcastReadClockPeriod()  // 49.152 MHz for Ethernet/Firewire
                       : (1.0e-6/125.0);                          // 125 MHz for Ethernet-only
}

void EthBasePort::PromDelay(void) const
{
    // Wait 1 msec
    Amp1394_Sleep(0.001);
}

// ---------------------------------------------------------
// Protected
// ---------------------------------------------------------

void EthBasePort::make_ethernet_header(unsigned char *, unsigned int, nodeid_t, unsigned char)
{
}

void EthBasePort::make_write_header(unsigned char *packet, unsigned int, unsigned char flags)
{
    unsigned int ctrlOffset = GetPrefixOffset(WR_CTRL);
    packet[ctrlOffset] = 0;
    if ((flags&FW_NODE_NOFORWARD_MASK) || !useFwBridge) packet[ctrlOffset] |= FW_CTRL_NOFORWARD;
    packet[ctrlOffset+1] = FwBusGeneration;
}

// The first 3 quadlets in a FireWire request packet are the same, and we only need to create request packets.
// Quadlet 0:  | Destination bus (10) | Destination node (6) | TL (6) | RT (2) | TCODE (4) | PRI (4) |
// Quadlet 1:  | Source bus (10)      | Source node (6)      | Destination offset MSW (16)           |
// Quadlet 2:  | Destination offset (32)                                                             |
void EthBasePort::make_1394_header(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tcode,
                                   unsigned int tl)
{
    // FFC0 replicates the base node ID when using FireWire on PC. This is followed by a transaction
    // label (arbitrary value that is returned by any resulting FireWire packets) and the transaction code.
    unsigned char fw_pri = 0;
    packet[0] = bswap_32((0xFFC0 | (node&FW_NODE_MASK)) << 16 | (tl & 0x003F) << 10 | (tcode & 0x000F) << 4 | (fw_pri & 0x000F));
    // FFD0 is used as source ID (most significant 16 bits); this sets source node to 0x10 (16).
    // Previously, FFFF was used, which set the source node to 0x3f (63), which is the broadcast address.
    // This is followed by the destination address, which is 48-bits long
    packet[1] = bswap_32((0xFFD0 << 16) | ((addr & 0x0000FFFF00000000) >> 32));
    packet[2] = bswap_32(addr&0xFFFFFFFF);
}

// Create a quadlet read request packet.
// In addition to the header, it contains the CRC in Quadlet 3.
void EthBasePort::make_qread_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int tl)
{
    make_1394_header(packet, node, addr, EthBasePort::QREAD, tl);
    // CRC
    packet[3] = bswap_32(BitReverse32(crc32(0U, (void*)packet, FW_QREAD_SIZE-FW_CRC_SIZE)));
}

// Create a quadlet write packet.
// In addition to the header, it contains the 32-bit data (Quadlet 3) and the CRC (Quadlet 4).
void EthBasePort::make_qwrite_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, quadlet_t data, unsigned int tl)
{
    make_1394_header(packet, node, addr, EthBasePort::QWRITE, tl);
    // quadlet data
    packet[3] = bswap_32(data);
    // CRC
    packet[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet, FW_QWRITE_SIZE-FW_CRC_SIZE)));
}

// Create a block read request packet.
// In addition to the header, it contains the following:
// Quadlet 3:  | Data length (16) | Extended tcode (16) |
// Quadlet 4:  | CRC (32)                               |
void EthBasePort::make_bread_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, unsigned int nBytes, unsigned int tl)
{
    make_1394_header(packet, node, addr, EthBasePort::BREAD, tl);
    packet[3] = bswap_32((nBytes & 0x0000ffff) << 16);
    // CRC
    packet[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet, FW_BREAD_SIZE-FW_CRC_SIZE)));
}

// Create a block write packet.
// In addition to the header, it contains the following:
// Quadlet 3:  | Data length (16) | Extended tcode (16) |
// Quadlet 4:  | Header CRC (32)                        |
// Quadlet 5 to 5+N | Data block (N quadlets)           |
// Quadlet 5+N+1:   | Data CRC (32)                     |
void EthBasePort::make_bwrite_packet(quadlet_t *packet, nodeid_t node, nodeaddr_t addr, quadlet_t *data, unsigned int nBytes, unsigned int tl)
{
    make_1394_header(packet, node, addr, EthBasePort::BWRITE, tl);
    // block length
    packet[3] = bswap_32((nBytes & 0x0000ffff) << 16);
    // header CRC
    packet[4] = bswap_32(BitReverse32(crc32(0U, (void*)packet, FW_BWRITE_HEADER_SIZE-FW_CRC_SIZE)));
    // Now, copy the data. We first check if the copy is needed.
    size_t data_offset = FW_BWRITE_HEADER_SIZE/sizeof(quadlet_t);  // data_offset = 20/4 = 5
    // Only copy data if it is not already in packet (i.e., if addresses are not equal).
    if (data != &packet[data_offset]) {
        rtWrite = false;
        memcpy(&packet[data_offset], data, nBytes);
    }
    // Now, compute the data CRC (assumes nBytes is a multiple of 4 because this is checked in WriteBlock)
    size_t data_crc_offset = data_offset + nBytes/sizeof(quadlet_t);
    packet[data_crc_offset] = bswap_32(BitReverse32(crc32(0U, static_cast<void *>(packet+data_offset), nBytes)));
#if 0 // ALTERNATIVE IMPLEMENTATION
    // CRC
    quadlet_t *fw_crc = fw_data + (nbytes/sizeof(quadlet_t));
    *fw_crc = bswap_32(BitReverse32(crc32(0U, (void*)fw_data, nbytes)));
#endif
}

// Default gap_count (also the maximum)
const unsigned char gap_count_default = 63;

// Hard-coded gap_counts based on number of nodes on bus, assuming worst case (i.e., a single long chain)
static unsigned char gap_counts[25] = { 63,                 // 0 nodes (should not happen)
                                         5,  7,  8, 10, 13, 16, 18, 21,     // 1-8  nodes
                                        24, 26, 29, 32, 35, 37, 40, 43,     // 9-16 nodes
                                        46, 48, 51, 54, 57, 59, 62, 63 };   // 17-24 nodes

bool EthBasePort::OptimizeFirewireGapCount()
{
    // Broadcast to all boards to initiate read of PHY register 1
    quadlet_t data = 1;
    if (!WriteQuadletNode(FW_NODE_BROADCAST, BoardIO::FW_PHY_REQ, data)) {
        outStr << "OptimizeFirewireGapCount: failed to broadcast PHY command" << std::endl;
        return false;
    }
    // Now, read each node to get the gap count
    unsigned char gap_count_min = gap_count_default;
    unsigned char gap_count_max = 0;
    for (unsigned int node = 0; node < NumOfNodes_; node++) {
        if (!ReadQuadletNode(node, BoardIO::FW_PHY_RESP, data))
            return false;
        unsigned char gap_count = static_cast<unsigned char>(data) & 0x3f;
        if (gap_count < gap_count_min)
            gap_count_min = gap_count;
        if (gap_count > gap_count_max)
            gap_count_max = gap_count;
    }
    if (gap_count_min == gap_count_max) {
        std::cout << "OptimizeFirewireGapCount: current gap count is "
                  << static_cast<unsigned int>(gap_count_min) << std::endl;
    }
    else if (gap_count_min != gap_count_max)
        std::cout << "OptimizeFirewireGapCount: inconsistent gap counts ("
                  << static_cast<unsigned int>(gap_count_min) << "-"
                  << static_cast<unsigned int>(gap_count_max) << ")" << std::endl;
    // Update the gap count if it is inconsistent, or if the gap count is the default value of 63,
    // which would indicate that it has not yet been set (e.g., there is PC connected via Firewire).
    if ((gap_count_min != gap_count_max) || (gap_count_min == gap_count_default)) {
        unsigned char gap_count_new = gap_count_default;
        if (NumOfNodes_ < 25)
            gap_count_new = gap_counts[NumOfNodes_];
        std::cout << "OptimizeFirewireGapCount: updating gap count to "
                  << static_cast<unsigned int>(gap_count_new) << std::endl;
        // Set bit 12 to indicate write; addr (1) in bits 11-8, data (gap_count) in bits 7-0
        data = 0x00001100 | static_cast<quadlet_t>(gap_count_new);
        if (!WriteQuadletNode(FW_NODE_BROADCAST, BoardIO::FW_PHY_REQ, data)) {
            outStr << "OptimizeFirewireGapCount: failed to broadcast PHY command" << std::endl;
            return false;
        }
    }
    return true;
}

bool EthBasePort::checkCRC(const unsigned char *packet)
{
    // Eliminate CRC checking of FireWire packets received via Ethernet
    // because Ethernet already includes CRC.
#if 0
    // Note that FW_QREPONSE_SIZE == FW_BRESPONSE_HEADER_SIZE
    uint32_t crc_check = BitReverse32(crc32(0U, packet, FW_QRESPONSE_SIZE-FW_CRC_SIZE));
    uint32_t crc_original = bswap_32(*reinterpret_cast<const uint32_t *>(packet+FW_QRESPONSE_SIZE-FW_CRC_SIZE));
    return (crc_check == crc_original);
#else
    return true;
#endif
}

//  -----------  CRC ----------------
//source: http://www.opensource.apple.com/source/xnu/xnu-1456.1.26/bsd/libkern/crc32.c
//online check: http://www.lammertbies.nl/comm/info/crc-calculation.html
static uint32_t crc32_tab[] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
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


uint32_t EthBasePort::BitReverse32(uint32_t input)
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
// It is also needed to be byteSwapped before putting into stream
uint32_t crc32(uint32_t crc, const void *buf, size_t size)
{
    const uint8_t *p;

    p = (uint8_t*)buf;
    crc = crc ^ ~0U;

    while (size--)
        crc = crc32_tab[(crc ^ BitReverseTable[*p++]) & 0xFF] ^ (crc >> 8);

  return crc ^ ~0U;
}
