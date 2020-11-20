/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <stdio.h>

#include <Amp1394/AmpIORevision.h>
#ifndef _MSC_VER
#include <termios.h>
#endif
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "EthRawPort.h"
#endif
#include "EthUdpPort.h"
#include "AmpIO.h"
#include "Amp1394Time.h"

#ifdef _MSC_VER
#include <stdlib.h>   // for byteswap functions
inline uint16_t bswap_16(uint16_t data) { return _byteswap_ushort(data); }
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }
#else
#include <unistd.h>
#include <byteswap.h>
#endif

const AmpIO_UInt32 VALID_BIT        = 0x80000000;  /*!< High bit of 32-bit word */
const AmpIO_UInt32 DAC_MASK         = 0x0000ffff;  /*!< Mask for 16-bit DAC values */

AmpIO_UInt32 KSZ8851CRC(const unsigned char *data, size_t len)
{
    AmpIO_UInt32 crc = 0xffffffff;
    for (size_t i = 0; i < len; i++) {
        for (size_t j = 0; j < 8; j++) {
            if (((crc >> 31) ^ (data[i] >> j)) & 0x01)
                crc = (crc << 1) ^ 0x04c11db7;
            else
                crc = crc << 1;
        }
    }
    return crc;    
}

// Compute parameters to initialize multicast hash table
void ComputeMulticastHash(unsigned char *MulticastMAC, AmpIO_UInt8 &regAddr, AmpIO_UInt16 &regData)
{
    AmpIO_UInt32 crc = KSZ8851CRC(MulticastMAC, 6);
    int regOffset = (crc >> 29) & 0x0006;  // first 2 bits of CRC (x2)
    int regBit = (crc >> 26) & 0x00F;      // next 4 bits of CRC
    regAddr = 0xA0 + regOffset;            // 0xA0 --> MAHTR0 (MAC Address Hash Table Register 0)
    regData = (1 << regBit);
}

// Ethernet debug
void PrintEthernetDebug(AmpIO &Board)
{
    AmpIO_UInt16 status = Board.ReadKSZ8851Status();
    if (!(status&0x8000)) {
        std::cout << "   No Ethernet controller, status = " << std::hex << status << std::endl;
        return;
    }
    EthBasePort::PrintDebug(std::cout, status);
}

#if Amp1394_HAS_RAW1394
// Ethernet status, as reported by KSZ8851 on FPGA board
void PrintEthernetStatus(AmpIO &Board)
{
    AmpIO_UInt16 reg;
    Board.ReadKSZ8851Reg(0xF8, reg);
    std::cout << "Port 1 status:";
    if (reg & 0x0020) std::cout << " link-good";
    if (reg & 0x0040) std::cout << " an-done";
    if (reg & 0x0200) std::cout << " full-duplex";
    else std::cout << " half-duplex";
    if (reg & 0x0400) std::cout << " 100Mbps";
    else std::cout << " 10Mbps";
    if (reg & 0x2000) std::cout << " polarity-reversed";
    std::cout << std::endl;
}

// Check contents of KSZ8851 register
bool CheckRegister(AmpIO &Board, AmpIO_UInt8 regNum, AmpIO_UInt16 mask, AmpIO_UInt16 value)
{
    AmpIO_UInt16 reg;
    Board.ReadKSZ8851Reg(regNum, reg);
    if ((reg&mask) != value) {
        std::cout << "Register " << std::hex << (int)regNum << ": read = " << reg
                  << ", expected = " << value << " (mask = " << mask << ")" << std::endl;
        return false;
    }
    return true;
}

// Check whether Ethernet initialized correctly
bool CheckEthernet(AmpIO &Board)
{
    std::cout << "Checking --- start ---" << "\n";
    bool ret = true;
    ret &= CheckRegister(Board, 0x10, 0xfff0, 0x9400);  // MAC address low = 0x940n (n = board id)
    ret &= CheckRegister(Board, 0x12, 0xffff, 0x0E13);  // MAC address middle = 0xOE13
    ret &= CheckRegister(Board, 0x14, 0xffff, 0xFA61);  // MAC address high = 0xFA61
    ret &= CheckRegister(Board, 0x84, 0x4000, 0x4000);  // Enable QMU transmit frame data pointer auto increment
    ret &= CheckRegister(Board, 0x70, 0x01ff, 0x01EF);  // Enable QMU transmit flow control, CRC, padding, and transmit module
    ret &= CheckRegister(Board, 0x86, 0x4000, 0x4000);  // Enable QMU receive frame data pointer auto increment
    ret &= CheckRegister(Board, 0x9C, 0x00ff, 0x0001);  // Configure receive frame threshold for 1 frame
    ret &= CheckRegister(Board, 0x74, 0xffff, 0x7CE1);  // Enable checksums, MAC address filtering, and receive module
    // Check multicast hash table
    unsigned char MulticastMAC[6];
    EthBasePort::GetDestMulticastMacAddr(MulticastMAC);
    AmpIO_UInt8 HashReg;
    AmpIO_UInt16 HashValue;
    ComputeMulticastHash(MulticastMAC, HashReg, HashValue);
    ret &= CheckRegister(Board, HashReg, 0xffff, HashValue);
    ret &= CheckRegister(Board, 0x82, 0x03f7, 0x0020);  // Enable QMU frame count threshold (1), no auto-dequeue
    ret &= CheckRegister(Board, 0x90, 0xffff, 0xa000);  // Enable receive and link change interrupts
    std::cout << "Checking ---- end ----" << "\n";
    return ret;
}

bool InitEthernet(AmpIO &Board)
{
    if (Board.GetFirmwareVersion() < 5) {
        std::cout << "   No Ethernet controller, firmware version = " << Board.GetFirmwareVersion() << std::endl;
        return false;
    }

    AmpIO_UInt16 status = Board.ReadKSZ8851Status();
    if (!(status&0x8000)) {
        std::cout << "   No Ethernet controller, status = " << std::hex << status << std::endl;
        return false;
    }
    std::cout << "   Ethernet controller status = " << std::hex << status << std::endl;
    PrintEthernetDebug(Board);

    // Reset the board
    Board.ResetKSZ8851();
    // Wait 100 msec
    Amp1394_Sleep(0.1);
    // Scan Nodes

    // Read the status
    status = Board.ReadKSZ8851Status();
    std::cout << "   After reset, status = " << std::hex << status << std::endl;

    if (!(status&0x2000)) {
        std::cout << "   Ethernet failed initialization" << std::endl;
        PrintEthernetDebug(Board);
        return false;
    }

    // Read the Chip ID (16-bit read)
    AmpIO_UInt16 chipID = Board.ReadKSZ8851ChipID();
    std::cout << "   Chip ID = " << std::hex << chipID << std::endl;
    if ((chipID&0xfff0) != 0x8870)
        return false;


    // Check that KSZ8851 registers are as expected
    if (!CheckEthernet(Board)) {
        PrintEthernetDebug(Board);
        return false;
    }

    // Display the MAC address
    AmpIO_UInt16 regLow, regMid, regHigh;
    Board.ReadKSZ8851Reg(0x10, regLow);   // MAC address low = 0x94nn (nn = board id)
    Board.ReadKSZ8851Reg(0x12, regMid);   // MAC address middle = 0xOE13
    Board.ReadKSZ8851Reg(0x14, regHigh);  // MAC address high = 0xFA61
    std::cout << "   MAC address = " << std::hex << regHigh << ":" << regMid << ":" << regLow << std::endl;

    // Wait 2.5 sec
    Amp1394_Sleep(2.5);
    return true;
}
#endif

AmpIO *SelectBoard(const std::string &portName, std::vector<AmpIO *> boardList, AmpIO *curBoard)
{
    AmpIO *newBoard = curBoard;
    if (boardList.size() > 1) {
        size_t i;
        std::cout << "Select " << portName << " Board: ";
        for (i = 0; i < boardList.size(); i++)
            std::cout << static_cast<unsigned int>(boardList[i]->GetBoardId()) << " ";
        std::cout << "(any other key to keep current board " << static_cast<unsigned int>(curBoard->GetBoardId()) << ")" << std::endl;
        int num = (getchar() - '0');
        std::cout << std::endl;
        for (i = 0; i < boardList.size(); i++) {
            if (num == boardList[i]->GetBoardId()) {
                newBoard = boardList[i];
                break;
            }
        }
    }
    return newBoard;
}

void  ContinuousReadTest(BasePort *port, unsigned char boardNum)
{
    bool done = false;
    quadlet_t read_data;
    //char buf[5] = "QLA1";
    char buf[5] = "1ALQ";
    size_t success = 0;
    size_t readFailures = 0;
    size_t compareFailures = 0;
    unsigned long count = 0;
    while (!done) {
        count++;
        read_data = 0;
        if (!port->ReadQuadlet(boardNum, 4, read_data))
            readFailures++;
        else {
            if (memcmp((void *)&read_data, buf, 4) == 0)
                success++;
            else
                compareFailures++;
        }
        if (readFailures + compareFailures > 5) done = true;
        if (count >= 10000) done = true;

        // print status
        if (count % 1000 == 0) {
            std::cout << "Success = " << std::dec << success << ", read failures = " << readFailures << ", compare failures = "
                      << compareFailures << std::endl;
        }
    }
    if (count % 1000 != 0) {
        std::cout << "Success = " << std::dec << success << ", read failures = " << readFailures << ", compare failures = "
                  << compareFailures << std::endl;
    }
}

void  ContinuousWriteTest(BasePort *ethPort, unsigned char boardNum)
{
    bool done = false;
    quadlet_t read_data;
    size_t success = 0;
    size_t writeFailures = 0;
    size_t compareFailures = 0;
    quadlet_t write_data = 0x0;
    int count = 0;

    while (!done) {
        read_data = -1;
        write_data++;
        count++;
        if (!ethPort->WriteQuadlet(boardNum, 0x0F, write_data))   // 0x0F is REG_DEBUG
            writeFailures++;
        else {
            Amp1394_Sleep(50*1e-6);  // sleep 50 us
            ethPort->ReadQuadlet(boardNum, 0x0F, read_data);
            if (memcmp((void *)&read_data, (void *)&write_data, 4) == 0)
                success++;
            else {
                compareFailures++;
                std::cout << std::hex << "write_data = 0x" << write_data << "  " << " read_data = 0x" << read_data << std::endl;
            }
        }
        if (writeFailures + compareFailures > 200) done = true;

        if (count % 1000 == 0) {
            std::cout << "Success = " << std::dec << success << ", write failures = " << writeFailures << ", compare failures = "
                      << compareFailures << std::endl;
        }

        if (count >= 10000) {
            done = true;
            std::cout << "end write_data = 0x" << std::hex << write_data << "\n";
        }
    }
}


bool PrintFirewirePHY(BasePort *port, int boardNum)
{
    quadlet_t write_data = 0;
    quadlet_t read_data = 0;
    if (!port->WriteQuadlet(boardNum, 1, write_data))
        return false;
    if (!port->ReadQuadlet(boardNum, 2, read_data))
        return false;
    std::cout << "Node: " << std::dec << ((read_data >> 2) && 0x000003f);
    if (read_data & 0x02) std::cout << " (root)";
    if (read_data & 0x01) std::cout << " (power)";
    std::cout << std::endl;
    write_data = 1;
    read_data = 0;
    if (!port->WriteQuadlet(boardNum, 1, write_data))
        return false;
    if (!port->ReadQuadlet(boardNum, 2, read_data))
        return false;
    std::cout << "Gap count = " << (read_data&0x000003f) << " (default = 63)" << std::endl;
    write_data = 2;
    read_data = 0;
    if (!port->WriteQuadlet(boardNum, 1, write_data))
        return false;
    if (!port->ReadQuadlet(boardNum, 2, read_data))
        return false;
    int speed = (read_data >> 6)&0x00000003;
    std::cout << "Speed = " << speed << ", num_ports = " << (read_data&0x0000001f) << std::endl;
    return true;
}

bool RunTiming(const std::string &portName, AmpIO *boardTest, AmpIO *hubFw, const std::string &msgType, size_t numIter = 1000)
{
    size_t i;
    EthBasePort::TCODE msgCode;
    if (msgType == "quadlet read")
        msgCode = EthBasePort::QREAD;
    else if (msgType == "quadlet write")
        msgCode = EthBasePort::QWRITE;
    else {
        std::cout << "Unsupported message type: " << msgType << std::endl;
        return false;
    }
    std::cout << "Measuring " << portName << " " << msgType << " time (" << std::dec << numIter << " iterations)" << std::endl;
    double timeSum = 0.0;
    double minTime = 1.0;   // one second should be much higher than expected readings
    double maxTime = 0.0;
    unsigned short minTimeReceive = 65535;
    unsigned short maxTimeReceive = 0;
    unsigned short minTimeSend = 65535;
    unsigned short maxTimeSend = 0;
    double clockPeriod_us = 1.0e6*boardTest->GetFPGAClockPeriod();  // clock period in microseconds
    for (i = 0; i < numIter; i++) {
        double startTime, deltaTime = 0;
        AmpIO_UInt32 status;
        if (msgCode == EthBasePort::QREAD) {
            startTime = Amp1394_GetTime();
            status = boardTest->ReadStatus();
            deltaTime = Amp1394_GetTime()-startTime;
            int boardNum = (status&0x0f000000)>>24;
            if (boardNum != boardTest->GetBoardId()) {
                std::cout << "   Iteration " << i << ": board number mismatch, read=" << boardNum
                          << ", expected=" << static_cast<unsigned int>(boardTest->GetBoardId()) << std::endl;
                break;
            }
        }
        else if (msgCode == EthBasePort::QWRITE) {
            startTime = Amp1394_GetTime();
            status = boardTest->WriteWatchdogPeriod(0);
            deltaTime = Amp1394_GetTime()-startTime;
        }
        timeSum += deltaTime;
        if (deltaTime < minTime)
            minTime = deltaTime;
        if (deltaTime > maxTime)
            maxTime = deltaTime;
        if (hubFw) {
            // Read Ethernet timing from FPGA
            // TODO: formalize this, rather than hard-coding buffer[8] below
            quadlet_t buffer[16];
            if (hubFw->ReadEthernetData(buffer, 0x80, 16)) {
                unsigned short timeReceive = (buffer[8]&0x0000ffff);
                unsigned short timeSend = (buffer[8]>>16) - timeReceive;
                if (timeReceive < minTimeReceive)
                    minTimeReceive = timeReceive;
                if (timeReceive > maxTimeReceive)
                    maxTimeReceive = timeReceive;
                if (timeSend < minTimeSend)
                    minTimeSend = timeSend;
                if (timeSend > maxTimeSend)
                    maxTimeSend = timeSend;
            }
        }
    }
    if (i > 0) {
        std::cout << "   Times (us), " << i << " iterations: mean = " << std::fixed << std::setprecision(2)
                  << (1.0e6*timeSum/i) << ", min = " << (1.0e6*minTime) << ", max = " << (1.0e6*maxTime) << std::endl;
        if (hubFw) {
            std::cout << "      FPGA receive min/max (us) = " << (clockPeriod_us*minTimeReceive) << "/" << (clockPeriod_us*maxTimeReceive);
            if (msgType == "quadlet read")
                std::cout << ", send min/max (us) = " << (clockPeriod_us*minTimeSend) << "/" << (clockPeriod_us*maxTimeSend);
            std::cout << std::endl;
        }
    }
    return true;
}

static char QuadletReadCallbackBoardId = 0;

bool QuadletReadCallback(EthBasePort &, unsigned char boardId, std::ostream &debugStream)
{
    if ((QuadletReadCallbackBoardId != boardId) && (boardId != 0xff)) {
        debugStream << "Warning: QuadletReadCallback called for board " << (unsigned int) boardId
                    << ", expected board " << (unsigned int) QuadletReadCallbackBoardId << std::endl;
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    BasePort::PortType desiredPort = BasePort::PORT_ETH_UDP;
    int port = 0;
    std::string IPaddr(ETH_UDP_DEFAULT_IP);

    if (argc > 1) {
        int args_found = 0;
        for (int i = 1; i < argc; i++) {
            if ((argv[i][0] == '-') && (argv[i][1] == 'p')) {
                if (!BasePort::ParseOptions(argv[i]+2, desiredPort, port, IPaddr)) {
                    std::cerr << "Failed to parse option: " << argv[i] << std::endl;
                    return 0;
                }
                if (desiredPort == BasePort::PORT_FIREWIRE) {
                    std::cerr << "Please select Ethernet raw (-pethP) or UDP (-pudp[xx.xx.xx.xx])" << std::endl;
                    return 0;
                }
            }
            else if (args_found == 0) {
                // Could parse additional args here
            }
            else {
                    std::cerr << "Usage: eth1394test [-pP]" << std::endl
                    << "       where P = port number (default 0)" << std::endl
                    << "                 can also specify -pethP or -pudp" << std::endl;
                    return 0;
            }

        }
    }

    // Compute the hash table values used by the KSZ8851 chip to filter for multicast packets.
    // The results (RegAddr and RegData) are hard-coded in the FPGA code (EthernetIO.v).
    unsigned char MulticastMAC[6];
    EthBasePort::GetDestMulticastMacAddr(MulticastMAC);
    AmpIO_UInt8 RegAddr;
    AmpIO_UInt16 RegData;
    ComputeMulticastHash(MulticastMAC, RegAddr, RegData);
    std::cout << "Multicast hash table: register " << std::hex << (int)RegAddr << ", data = " << RegData << std::endl;

    std::vector<AmpIO *> FwBoardList;
    std::vector<AmpIO *> EthBoardList;

    AmpIO *curBoard = 0;     // Current board via Ethernet or Firewire
    AmpIO *curBoardFw = 0;   // Current board via Firewire
    AmpIO *curBoardEth = 0;  // Current board via Ethernet (sets boardNum)
    AmpIO *HubFw = 0;        // Ethernet Hub board via Firewire

#if Amp1394_HAS_RAW1394
    FirewirePort FwPort(0, std::cout);
    if (!FwPort.IsOK()) {
        std::cout << "Failed to initialize firewire port" << std::endl;
        return 0;
    }
    for (unsigned int bnum = 0; bnum < BoardIO::MAX_BOARDS; bnum++) {
        if (FwPort.GetNodeId(bnum) != BasePort::MAX_NODES) {
            std::cout << "Found Firewire board: " << bnum << std::endl;
            AmpIO *board = new AmpIO(bnum);
            FwPort.AddBoard(board);
            FwBoardList.push_back(board);
        }
    }
    if (FwBoardList.size() > 0) {
        curBoardFw = FwBoardList[0];
    }

#endif
    EthBasePort *EthPort = 0;
    if (desiredPort == BasePort::PORT_ETH_UDP) {
        std::cout << "Creating Ethernet UDP port, IP address = " << IPaddr << std::endl;
        EthPort = new EthUdpPort(port, IPaddr, std::cout);
    }
#if Amp1394_HAS_PCAP
    else if (desiredPort == BasePort::PORT_ETH_RAW) {
        std::cout << "Creating Ethernet raw (PCAP) port" << std::endl;
        EthPort = new EthRawPort(port, std::cout);
    }
#endif
    if (!EthPort) {
        std::cout << "Failed to create Ethernet port" << std::endl;
        return 0;
    }
    if (EthPort->IsOK()) {
        for (unsigned int bnum = 0; bnum < BoardIO::MAX_BOARDS; bnum++) {
            if (EthPort->GetNodeId(bnum) != BasePort::MAX_NODES) {
                std::cout << "Found Ethernet board: " << bnum << std::endl;
                AmpIO *board = new AmpIO(bnum);
                EthPort->AddBoard(board);
                EthBoardList.push_back(board);
             }
        }
        if (EthBoardList.size() > 0)
            curBoardEth = EthBoardList[0];
        // Find Firewire board corresponding to Ethernet hub
        unsigned int hub_num = EthPort->GetHubBoardId();
        for (size_t i = 0; i < FwBoardList.size(); i++) {
            if (FwBoardList[i]->GetBoardId() == hub_num) {
                HubFw = FwBoardList[i];
                break;
            }
        }
    }
    else
        std::cout << "Failed to initialize Ethernet port" << std::endl;

#ifndef _MSC_VER
    // Turn off buffered I/O for keyboard
    struct termios oldTerm, newTerm;
    tcgetattr(0, &oldTerm);
    newTerm = oldTerm;
    newTerm.c_lflag &= ~ICANON;
    newTerm.c_lflag &= ECHO;
    tcsetattr(0, TCSANOW, &newTerm);
#endif

    bool done = false;
    quadlet_t read_data;
    quadlet_t write_data = 0L;
    quadlet_t buffer[128];

    while (!done) {

        // Set curBoard to curBoardFw if it is valid; otherwise curBoardEth
        curBoard = curBoardFw ? curBoardFw : curBoardEth;

        std::cout << std::endl << "Ethernet Test Program" << std::endl;
        if (curBoardFw)
            std::cout << "  Firewire board: " << static_cast<unsigned int>(curBoardFw->GetBoardId()) << std::endl;
        if (curBoardEth)
            std::cout << "  Ethernet board: " << static_cast<unsigned int>(curBoardEth->GetBoardId()) << std::endl;
        std::cout << "  0) Quit" << std::endl;
        if (curBoardEth) {
            std::cout << "  1) Quadlet write (power/relay toggle) to board via Ethernet" << std::endl;
            std::cout << "  2) Quadlet read from board via Ethernet" << std::endl;
        }
        if (curBoardEth || curBoardFw)
            std::cout << "  3) Block read from board via Ethernet and/or Firewire" << std::endl;
        if (curBoardEth)
            std::cout << "  4) Block write to board via Ethernet" << std::endl;
        if (curBoardFw) {
            std::cout << "  5) Ethernet port status" << std::endl;
            std::cout << "  6) Initialize Ethernet port" << std::endl;
        }
        if (curBoard)
            std::cout << "  7) Ethernet debug info" << std::endl;
        std::cout << "  8) Multicast quadlet read via Ethernet" << std::endl;
        if (curBoardEth)
            std::cout << "  9) Block write test to 0x4090" << std::endl;
        if ((EthBoardList.size() > 0) || (FwBoardList.size() > 0))
            std::cout << "  b) Change Firewire/Ethernet board" << std::endl;
        if (curBoardEth) {
            std::cout << "  c) Continuous test (quadlet reads)" << std::endl;
            std::cout << "  d) Continuous write test (quadlet write)" << std::endl;
        }
        if (curBoardFw)
            std::cout << "  e) Read RXFCTR packet count" << std::endl;
        if (curBoardEth || curBoardFw)
            std::cout << "  f) Print Firewire PHY registers" << std::endl;
        if (curBoardFw) {
            std::cout << "  i) Read IPv4 address via FireWire" << std::endl;
            std::cout << "  I) Clear IPv4 address via FireWire" << std::endl;
        }
        std::cout << "  r) Check Firewire bus generation and rescan if needed" << std::endl;
        if (curBoardEth)
            std::cout << "  t) Run Ethernet timing analysis" << std::endl;
        if (curBoard)
            std::cout << "  x) Read Ethernet debug data" << std::endl;
        if (curBoardEth)
            std::cout << "  y) Read Firewire data via Ethernet" << std::endl;
        if (curBoardFw)
            std::cout << "  z) Check Ethernet initialization" << std::endl;
        std::cout << "Select option: ";
        
        int c = getchar();
        std::cout << std::endl << std::endl;

        nodeaddr_t addr;
        quadlet_t fw_block_data[28];
        quadlet_t eth_block_data[28];
        quadlet_t write_block[5] = { 0x11111111, 0x22222222, 0x33333333, 0x44444444, 0x55555555 };
        int i;
        char buf[5];

        unsigned char boardNum = 0;
        if (curBoardEth)
            boardNum = curBoardEth->GetBoardId();

        switch (c) {
        case '0':   // Quit
                done = true;
                break;

        case '1':   // Write quadlet from PC to FPGA
            if (curBoardEth) {
                if (write_data == 0x000f0000)
                    write_data = 0x000a0000;   // power/relay off
                else
                    write_data = 0x000f0000;   // power/relay on
                if (!EthPort->WriteQuadlet(boardNum, 0x0, write_data ))
                    std::cout << "Failed to write quadlet via Ethernet port" << std::endl;
                else
                    std::cout << "Write data = 0x" << std::hex << write_data << "\n";
            }
            break;

        case '2':   // Read request from PC via Ethernet (note that QuadletReadCallback is called)
            if (curBoardEth) {
                read_data = 0;
                addr = 0x04;  // Return QLA1
                if (EthPort->ReadQuadlet(boardNum, addr, read_data)) {
                    std::cout << "Read quadlet data: " << std::hex << read_data << std::endl;
                    memcpy(buf, (char *)(&read_data), 4);
                    buf[4] = 0;
                    std::cout << "  as string: " << buf << std::endl;
                    std::cout << "FPGA Recv time (us): " << EthPort->GetFpgaReceiveTime()*1.0e6
                              << ", FPGA Total time (us): " << EthPort->GetFpgaTotalTime()*1.0e6 << std::endl;
                }
                else
                    std::cout << "Failed to read quadlet via Ethernet port" << std::endl;
            }
            break;

        case '3':
            if (curBoardFw || curBoardEth) {
                memset(fw_block_data, 0, sizeof(fw_block_data));
#if Amp1394_HAS_RAW1394
                if (curBoardFw) {
                    if (!FwPort.ReadBlock(boardNum, 0, fw_block_data, sizeof(fw_block_data)))
                        std::cout << "Failed to read block data via FireWire port" << std::endl;
                }
#endif
                memset(eth_block_data, 0, sizeof(eth_block_data));
                if (curBoardEth) {
                    if (!EthPort->ReadBlock(boardNum, 0, eth_block_data, sizeof(eth_block_data)))
                        std::cout << "Failed to read block data via Ethernet port" << std::endl;
                    else
                        std::cout << "FPGA Recv time (us): " << EthPort->GetFpgaReceiveTime()*1.0e6
                                  << ", FPGA Total time (us): " << EthPort->GetFpgaTotalTime()*1.0e6 << std::endl;
                }
                std::cout << "     Firewire   Ethernet" << std::endl;
                for (i = 0; static_cast<size_t>(i) < sizeof(fw_block_data)/sizeof(quadlet_t); i++)
                    std::cout << std::setw(2) << std::setfill(' ') << std::dec << i << ":  "
                              << std::setw(8) << std::setfill('0') <<  std::hex
                              << bswap_32(fw_block_data[i]) << "   " << std::setw(8) << std::setfill('0')
                              << bswap_32(eth_block_data[i]) << std::endl;
            }
            break;

        case '4':
            if (curBoardEth) {
                // Read from DAC (quadlet reads), modify values, write them using
                // a block write, then read them again to check.
                // Note that test can be done using FireWire by changing EthPort to FwPort.
                for (i = 0; i < 4; i++) {
                    addr = 0x0001 | ((i+1) << 4);  // channel 1-4, DAC Control
                    EthPort->ReadQuadlet(boardNum, addr, write_block[i]);
                    write_block[i] &= 0x0000ffff;
                }
                std::cout << "Read from DAC: " << std::hex << write_block[0] << ", "
                          << write_block[1] << ", " << write_block[2] << ", "
                          << write_block[3] << std::endl;
                for (i = 0; i < 4; i++) {
                    write_block[i] = bswap_32(VALID_BIT | (boardNum<<24) | (write_block[i]+(i+1)*0x100));
                }
                write_block[4] = 0;
                std::cout << "Setting new values" << std::endl;
                if (!EthPort->WriteBlock(boardNum, 0, write_block, sizeof(write_block))) {
                    std::cout << "Failed to write block data via Ethernet port" << std::endl;
                    break;
                }
#if 0
                for (i = 0; i < 4; i++) {
                    addr = 0x0001 | ((i+1) << 4);  // channel 1-4, DAC Control
                    EthPort->ReadQuadlet(boardNum, addr, write_block[i]);
                }
                std::cout << "Read from DAC: " << std::hex << write_block[0] << ", "
                          << write_block[1] << ", " << write_block[2] << ", "
                          << write_block[3] << std::endl;
#endif
            }
            break;

#if Amp1394_HAS_RAW1394
        case '5':
            if (curBoardFw)
                PrintEthernetStatus(*curBoardFw);
            break;

        case '6':
            if (curBoardFw)
                InitEthernet(*curBoardFw);
            break;
#endif

        case '7':
            if (curBoard)
                PrintEthernetDebug(*curBoard);
            break;

        case '8':   // Read request via Ethernet multicast
            read_data = 0;
            addr = 0;  // Return status register
            if (EthPort->ReadQuadlet(FW_NODE_BROADCAST, addr, read_data))
                std::cout << "Read quadlet data: " << std::hex << read_data << std::endl;
            else
                std::cout << "Failed to read quadlet via Ethernet port" << std::endl;
            break;

        case '9':   // Block read test
            if (curBoardEth) {
                size_t i;
                quadlet_t testBlock[16];
                memset(testBlock, 0, sizeof(testBlock));
                if (EthPort->ReadBlock(boardNum, 0x4090, testBlock, sizeof(testBlock))) {
                   std::cout << "Current contents of block: " << std::endl;
                   for (i = 0; i < 16; i++) {
                       testBlock[i] = bswap_32(testBlock[i]);
                       std::cout << testBlock[i] << " ";
                   }
                   std::cout << std::endl;
                }
                for (i = 0; i < 16; i++) {
                    testBlock[i] += i;
                    testBlock[i] = bswap_32(testBlock[i]);
                }
                std::cout << "Writing new data" << std::endl;
                EthPort->WriteBlock(boardNum, 0x4090, testBlock, sizeof(testBlock));
            }
            break;

        case 'b':
            curBoardFw = SelectBoard("Firewire", FwBoardList, curBoardFw);
            curBoardEth = SelectBoard("Ethernet", EthBoardList, curBoardEth);
            break;

        case 'c':
            if (curBoardEth)
                ContinuousReadTest(EthPort, boardNum);
            break;

        case 'd':
            if (curBoardEth)
                ContinuousWriteTest(EthPort, boardNum);
            break;

        case 'e':
            if (curBoardFw) {
                AmpIO_UInt16 reg;
                if (curBoardFw->ReadKSZ8851Reg(0x90, reg))
                    std::cout << "IER    = 0x" << std::hex << reg << "  ";
                if (curBoardFw->ReadKSZ8851Reg(0x92, reg))
                    std::cout << "ISR    = 0x" << std::hex << reg << "  ";
                if (curBoardFw->ReadKSZ8851Reg(0x9C, reg))
                    std::cout << "RXFCTR = 0x" << std::hex << reg << "  ";
                if (curBoardFw->ReadKSZ8851Reg(0x7C, reg))
                    std::cout << "RXFHSR = 0x" << std::hex << reg << "  ";
                if (curBoardFw->ReadKSZ8851Reg(0x80, reg))
                    std::cout << "TXQCR = 0x" << std::hex << reg << std::endl;
            }
            break;

        case 'f':
#if Amp1394_HAS_RAW1394
            if (curBoardFw) {
                unsigned int fw_board = curBoardFw->GetBoardId();
                std::cout << "Firewire PHY data via FireWire (board " << fw_board << "):" << std::endl;
                PrintFirewirePHY(&FwPort, fw_board);
            }
#endif
            if (curBoardEth) {
                std::cout << "Firewire PHY data via Ethernet (board " << static_cast<unsigned int>(boardNum) << "):" << std::endl;
                PrintFirewirePHY(EthPort, boardNum);
            }
            break;

        case 'i':
            if (curBoardFw)
                std::cout << "IP Address = " << EthUdpPort::IP_String(curBoardFw->ReadIPv4Address()) << std::endl;
            break;

        case 'I':
            if (curBoardFw) {
                if (curBoardFw->WriteIPv4Address(0xffffffff))
                    std::cout << "Write IP address 255.255.255.255" << std::endl;
                else
                    std::cout << "Failed to write IP address" << std::endl;
            }
            break;

        case 'r':
            if (EthPort->IsOK())
                EthPort->CheckFwBusGeneration("EthPort", true);
#if Amp1394_HAS_RAW1394
            if (FwPort.IsOK())
                FwPort.CheckFwBusGeneration("FwPort", true);
#endif
            break;

        case 't':
            if (curBoardEth)
                RunTiming("Ethernet", curBoardEth, HubFw, "quadlet read");
            if (curBoardFw)
                RunTiming("Firewire", curBoardFw, 0, "quadlet read");
            if (curBoardEth)
                RunTiming("Ethernet", curBoardEth, HubFw, "quadlet write");
            if (curBoardFw)
                RunTiming("Firewire", curBoardFw, 0, "quadlet write");
            break;

        case 'x':
            if (curBoard) {
                if (curBoard->ReadEthernetData(buffer, 0xc0, 16))
                    EthBasePort::PrintEthernetPacket(std::cout, buffer, 16);
                if (curBoard->ReadEthernetData(buffer, 0, 64))
                    EthBasePort::PrintFirewirePacket(std::cout, buffer, 64);
                if (curBoard->ReadEthernetData(buffer, 0x80, 16))
                    EthBasePort::PrintDebugData(std::cout, buffer, curBoard->GetFPGAClockPeriod());
#if 0
                if (curBoard->ReadEthernetData(buffer, 0xa0, 32)) {
                    std::cout << "Initialization Program:" << std::endl;
                    for (int i = 0; i < 32; i++) {
                        if (i == 16)
                            std::cout << "Run Program:" << std::endl;
                        if (buffer[i] != 0) {
                           if (buffer[i]&0x02000000) std::cout << "   Write ";
                           else std::cout << "   Read  ";
                           std::cout << std::hex << std::setw(2) << std::setfill('0')
                                     << ((buffer[i]&0x00ff0000)>>16) << " ";       // address
                           std::cout << std::hex << std::setw(4) << std::setfill('0')
                                     << (buffer[i]&0x0000ffff);
                           if (buffer[i]&0x01000000) std::cout << " MOD";
                           std::cout << std::endl;
                        }
                    }
                }
                if (curBoard->ReadEthernetData(buffer, 0xe0, 21)) {
                    const uint16_t *packetw = reinterpret_cast<const uint16_t *>(buffer);
                    std::cout << "ReplyIndex: " << std::endl;
                    for (int i = 0; i < 41; i++)
                        std::cout << std::dec << i << ":  " << packetw[i] << std::endl;
                }
#endif
            }
            break;

        case 'y':
            if (curBoardEth && (curBoardEth->ReadFirewireData(buffer, 0, 64)))
                EthBasePort::PrintFirewirePacket(std::cout, buffer, 64);
            break;

#if Amp1394_HAS_RAW1394
        case 'z':
            if (curBoardFw) {
                // Check that KSZ8851 registers are as expected
                if (!CheckEthernet(*curBoardFw))
                    PrintEthernetDebug(*curBoardFw);
            }
            break;
#endif
        }
    }

#ifndef _MSC_VER
    tcsetattr(0, TCSANOW, &oldTerm);  // Restore terminal I/O settings
#endif
#if Amp1394_HAS_RAW1394
    for (unsigned int bd = 0; bd < FwBoardList.size(); bd++) {
        FwPort.RemoveBoard(FwBoardList[bd]->GetBoardId());
        delete FwBoardList[bd];
    }
#endif
    if (EthPort) {
        for (unsigned int bd = 0; bd < EthBoardList.size(); bd++) {
            EthPort->RemoveBoard(EthBoardList[bd]->GetBoardId());
            delete EthBoardList[bd];
        }
        delete EthPort;
    }
    return 0;
}
