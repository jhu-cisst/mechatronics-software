/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <iostream>
#include <iomanip>
#include <stdio.h>

#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include <termios.h>
#include "FirewirePort.h"
#endif
#include "Eth1394Port.h"
#include "AmpIO.h"

#ifdef _MSC_VER
#include <stdlib.h>   // for byteswap functions
inline uint16_t bswap_16(uint16_t data) { return _byteswap_ushort(data); }
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }
#else
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

#if Amp1394_HAS_RAW1394
// Ethernet debug
void PrintEthernetDebug(AmpIO &Board)
{
    AmpIO_UInt16 status = Board.ReadKSZ8851Status();
    if (!(status&0x8000)) {
        std::cout << "   No Ethernet controller, status = " << std::hex << status << std::endl;
        return;
    }
    Eth1394Port::PrintDebug(std::cout, status);
}

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
    ret &= CheckRegister(Board, 0x70, 0x01fe, 0x01EE);  // Enable QMU transmit flow control, CRC, and padding
    ret &= CheckRegister(Board, 0x86, 0x4000, 0x4000);  // Enable QMU receive frame data pointer auto increment
    ret &= CheckRegister(Board, 0x9C, 0x00ff, 0x0001);  // Configure receive frame threshold for 1 frame
    ret &= CheckRegister(Board, 0x74, 0xfffe, 0x7CE0);
    // Check multicast hash table
    unsigned char MulticastMAC[6];
    Eth1394Port::GetDestMulticastMacAddr(MulticastMAC);
    AmpIO_UInt8 HashReg;
    AmpIO_UInt16 HashValue;
    ComputeMulticastHash(MulticastMAC, HashReg, HashValue);
    ret &= CheckRegister(Board, HashReg, 0xffff, HashValue);
    ret &= CheckRegister(Board, 0x82, 0x03f7, 0x0020);  // Enable QMU frame count threshold (1), no auto-dequeue
//    ret &= CheckRegister(Board, 0x90, 0xffff, 0x2000);  // Enable receive interrupts (TODO: also consider link change interrupt)
    ret &= CheckRegister(Board, 0x90, 0xffff, 0xe000);  // Enable receive interrupts (TODO: also consider link change interrupt)
    ret &= CheckRegister(Board, 0x70, 0x0001, 0x0001);
    ret &= CheckRegister(Board, 0x74, 0x0001, 0x0001);
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
    usleep(100000L);
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
    usleep(2500000L);
    return true;
}

void  ContinuousReadTest(BasePort *port, unsigned char boardNum)
{
    bool done = false;
    quadlet_t read_data;
    char buf[5] = "QLA1";
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
}

void  ContinuousWriteTest(BasePort *ethPort, BasePort *fwPort, unsigned char boardNum)
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
//            usleep(50);  // sleep 1ms
//            fwPort->ReadQuadlet(boardNum, 0x0F, read_data);
//            read_data = bswap_32(read_data);
//            if (memcmp((void *)&read_data, (void *)&write_data, 4) == 0)
//                success++;
//            else {
//                compareFailures++;
//                std::cout << std::hex << "write_data = 0x" << write_data << "  " << " read_data = 0x" << read_data << std::endl;
//            }
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

static char QuadletReadCallbackBoardId = 0;

bool QuadletReadCallback(Eth1394Port &, unsigned char boardId, std::ostream &debugStream)
{
    if ((QuadletReadCallbackBoardId != boardId) && (boardId != 0xff)) {
        debugStream << "Warning: QuadletReadCallback called for board " << (unsigned int) boardId
                    << ", expected board " << (unsigned int) QuadletReadCallbackBoardId << std::endl;
        return false;
    }
    return true;
}
#endif

int main(int argc, char **argv)
{
    unsigned char boardNum = 0;
    if (argc > 1) {
        int boardNumInt;
        if (sscanf(argv[1], "%d", &boardNumInt) == 1)
            boardNum = static_cast<unsigned char>(boardNumInt);
        else
            std::cout << "Invalid board number: " << argv[1] << std::endl;
    }
    std::cout << "Testing board " << (int)boardNum << std::endl;

    // Compute the hash table values used by the KSZ8851 chip to filter for multicast packets.
    // The results (RegAddr and RegData) are hard-coded in the FPGA code (EthernetIO.v).
    unsigned char MulticastMAC[6];
    Eth1394Port::GetDestMulticastMacAddr(MulticastMAC);
    AmpIO_UInt8 RegAddr;
    AmpIO_UInt16 RegData;
    ComputeMulticastHash(MulticastMAC, RegAddr, RegData);
    std::cout << "Multicast hash table: register " << std::hex << (int)RegAddr << ", data = " << RegData << std::endl;

    AmpIO board1(boardNum);
    AmpIO board2(boardNum);

#if Amp1394_HAS_RAW1394
    FirewirePort FwPort(0, std::cout);
    if (!FwPort.IsOK()) {
        std::cout << "Failed to initialize firewire port" << std::endl;
        return 0;
    }
    FwPort.AddBoard(&board1);
#if 0
    if (!InitEthernet(board1)) {
        std::cout << "Failed to initialize Ethernet chip" << std::endl;
        FwPort.RemoveBoard(boardNum);
        return 0;
    }
#else
    // Read the Chip ID (16-bit read)
    AmpIO_UInt16 chipID = board1.ReadKSZ8851ChipID();
    std::cout << "   Chip ID = " << std::hex << chipID << std::endl;
#endif

    QuadletReadCallbackBoardId = board1.GetBoardId();
    Eth1394Port EthPort(0, std::cout, QuadletReadCallback);
    if (!EthPort.IsOK()) {
        std::cout << "Failed to initialize ethernet port" << std::endl;
#if 0
        FwPort.RemoveBoard(boardNum);
        return 0;
#endif
    }
#else
    Eth1394Port EthPort(0, std::cout);
    if (!EthPort.IsOK()) {
        std::cout << "Failed to initialize ethernet port" << std::endl;
        return 0;
    }
#endif
    EthPort.AddBoard(&board2);

    // For now, nothing more can be done without FireWire (RAW1394)
#if Amp1394_HAS_RAW1394
    // Turn off buffered I/O for keyboard
    struct termios oldTerm, newTerm;
    tcgetattr(0, &oldTerm);
    newTerm = oldTerm;
    newTerm.c_lflag &= ~ICANON;
    newTerm.c_lflag &= ECHO;
    tcsetattr(0, TCSANOW, &newTerm);

    bool done = false;
    quadlet_t read_data, write_data;
    while (!done) {

        std::cout << std::endl << "Ethernet Test Program" << std::endl;
        std::cout << "  0) Quit" << std::endl;
        std::cout << "  1) Quadlet write to board" << std::endl;
        std::cout << "  2) Quadlet read from board" << std::endl;
        std::cout << "  3) Block read from board" << std::endl;
        std::cout << "  4) Block write to board" << std::endl;
        std::cout << "  5) Ethernet port status" << std::endl;
        std::cout << "  6) Initialize Ethernet port" << std::endl;
        std::cout << "  7) Ethernet debug info" << std::endl;
        std::cout << "  8) Multicast quadlet read" << std::endl;
        std::cout << "  c) Continuous test (quadlet reads)" << std::endl;
        std::cout << "  d) Continuous write test (quadlet write)" << std::endl;
        std::cout << "  e) Read RXFCTR packet count" << std::endl;
        std::cout << "  f) Print Firewire PHY registers" << std::endl;
        std::cout << "Select option: ";
        
        int c = getchar();
        std::cout << std::endl << std::endl;

        nodeaddr_t addr;
        quadlet_t fw_block_data[16];
        quadlet_t eth_block_data[16];
        quadlet_t write_block[4] = { 0x11111111, 0x22222222, 0x33333333, 0x44444444 };
        int i;
        char buf[5];

        switch (c) {
        case '0':   // Quit
                done = true;
                break;

        case '1':   // Write quadlet from PC to FPGA
//            std::cout << "Data to write: 0x";
//            std::cin >> std::hex >> write_data;
//            getchar();
            write_data++;
            if (!EthPort.WriteQuadlet(boardNum, 0x0F, write_data ))
                std::cout << "Failed to write quadlet via Ethernet port" << std::endl;
            else
                std::cout << "Write data = 0x" << std::hex << write_data << "\n";
//            else {
//                read_data = 0;
//                EthPort.ReadQuadlet(boardNum, 0x0F, read_data);
//                std::cout << "Read quadlet data: 0x" << std::hex << bswap_32(read_data) << std::endl;
//            }

            break;

        case '2':   // Read request from PC via Ethernet (note that QuadletReadCallback is called)
                read_data = 0;
                addr = 0x04;  // Return QLA1
                if (EthPort.ReadQuadlet(boardNum, addr, read_data))
                    std::cout << "Read quadlet data: " << std::hex << read_data << std::endl;
                else
                    std::cout << "Failed to read quadlet via Ethernet port" << std::endl;
                memcpy(buf, (char *)(&read_data), 4);
                buf[4] = 0;
                std::cout << "  as string: " << buf << std::endl;
                break;

        case '3':
                if (!FwPort.ReadBlock(boardNum, 0, fw_block_data, sizeof(fw_block_data))) {
                    std::cout << "Failed to read block data via FireWire port" << std::endl;
                    break;
                }
                if (!EthPort.ReadBlock(boardNum, 0, eth_block_data, sizeof(eth_block_data))) {
                    std::cout << "Failed to read block data via Ethernet port" << std::endl;
                    break;
                }
                for (i = 0; i < sizeof(fw_block_data)/sizeof(quadlet_t); i++)
                    std::cout << std::dec << i << ": " << std::hex << bswap_32(fw_block_data[i])
                              << ", " << bswap_32(eth_block_data[i]) << std::endl;
                break;

        case '4':
                // Read from DAC (quadlet reads), modify values, write them using
                // a block write, then read them again to check.
                // Note that test can be done using FireWire by changing EthPort to FwPort.
                for (i = 0; i < 4; i++) {
                    addr = 0x0001 | ((i+1) << 4);  // channel 1-4, DAC Control
                    EthPort.ReadQuadlet(boardNum, addr, write_block[i]);
                }
                std::cout << "Read from DAC: " << std::hex << write_block[0] << ", "
                          << write_block[1] << ", " << write_block[2] << ", "
                          << write_block[3] << std::endl;
                for (i = 0; i < 4; i++) {
                    write_block[i] = bswap_32(VALID_BIT | (write_block[i]+(i+1)*0x100));
                }
                std::cout << "Setting new values" << std::endl;
                if (!EthPort.WriteBlock(boardNum, 0, write_block, sizeof(write_block))) {
                    std::cout << "Failed to write block data via Ethernet port" << std::endl;
                    break;
                }
                for (i = 0; i < 4; i++) {
                    addr = 0x0001 | ((i+1) << 4);  // channel 1-4, DAC Control
                    EthPort.ReadQuadlet(boardNum, addr, write_block[i]);
                }
                std::cout << "Read from DAC: " << std::hex << write_block[0] << ", "
                          << write_block[1] << ", " << write_block[2] << ", "
                          << write_block[3] << std::endl;
                break;

        case '5':
                PrintEthernetStatus(board1);
                break;

        case '6':
                InitEthernet(board1);
                break;

        case '7':
                PrintEthernetDebug(board1);
                break;

        case '8':   // Read request via Ethernet multicast
                read_data = 0;
                addr = 0;  // Return status register
                if (EthPort.ReadQuadlet(0xff, addr, read_data))
                    std::cout << "Read quadlet data: " << std::hex << read_data << std::endl;
                else
                    std::cout << "Failed to read quadlet via Ethernet port" << std::endl;
                break;

        case 'c':
            ContinuousReadTest(&EthPort, boardNum);
            break;

        case 'd':
            ContinuousWriteTest(&EthPort, &FwPort, boardNum);
            break;

        case 'e':
            AmpIO_UInt16 reg;
            board1.ReadKSZ8851Reg(0x92, reg);
            std::cout << "ISR    = 0x" << std::hex << reg << "  ";
            board1.ReadKSZ8851Reg(0x9C, reg);
            std::cout << "RXFCTR = 0x" << std::hex << reg << "  ";
            board1.ReadKSZ8851Reg(0x7C, reg);
            std::cout << "RXFHSR = 0x" << std::hex << reg << "\n";
            break;

        case 'f':
            std::cout << "Firewire PHY data via FireWire:" << std::endl;
            PrintFirewirePHY(&FwPort, boardNum);
            std::cout << "Firewire PHY data via Ethernet:" << std::endl;
            PrintFirewirePHY(&EthPort, boardNum);
            break;
        }
    }

    tcsetattr(0, TCSANOW, &oldTerm);  // Restore terminal I/O settings
    FwPort.RemoveBoard(boardNum);
#endif
    EthPort.RemoveBoard(boardNum);
    return 0;
}
