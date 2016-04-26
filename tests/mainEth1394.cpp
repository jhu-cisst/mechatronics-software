/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <iostream>
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

#if Amp1394_HAS_RAW1394
// Ethernet debug
void PrintEthernetDebug(AmpIO &Board)
{
    AmpIO_UInt16 status = Board.ReadKSZ8851Status();
    if (!(status&0x8000)) {
        std::cout << "   No Ethernet controller, status = " << std::hex << status << std::endl;
        return;
    }
    std::cout << "Status: ";
    if (status&0x4000) std::cout << "error ";
    if (status&0x2000) std::cout << "initOK ";
    if (status&0x1000) std::cout << "initReq ";
    if (status&0x0800) std::cout << "ethIoErr ";
    if (status&0x0400) std::cout << "cmdReq ";
    if (status&0x0200) std::cout << "cmdAck ";
    if (status&0x0100) std::cout << "qRead ";
    if (status&0x0080) std::cout << "qWrite ";
    if (status&0x0040) std::cout << "bRead ";
    if (status&0x0020) std::cout << "PME ";
    if (!(status&0x0010)) std::cout << "IRQ ";
    std::cout << "state=" << (int)(status&0x000f) << std::endl;
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
    bool ret = true;
    ret &= CheckRegister(Board, 0x10, 0xffff, 0x9400);  // MAC address low = 0x94nn (nn = board id)
    ret &= CheckRegister(Board, 0x12, 0xffff, 0x0E13);  // MAC address middle = 0xOE13
    ret &= CheckRegister(Board, 0x14, 0xffff, 0xFA61);  // MAC address high = 0xFA61
    ret &= CheckRegister(Board, 0x84, 0x4000, 0x4000);  // Enable QMU transmit frame data pointer auto increment
    ret &= CheckRegister(Board, 0x70, 0x01fe, 0x01EE);  // Enable QMU transmit flow control, CRC, and padding
    ret &= CheckRegister(Board, 0x86, 0x4000, 0x4000);  // Enable QMU receive frame data pointer auto increment
    ret &= CheckRegister(Board, 0x9C, 0x00ff, 0x0001);  // Configure receive frame threshold for 1 frame
    ret &= CheckRegister(Board, 0x74, 0xfffe, 0x7CE0);
    ret &= CheckRegister(Board, 0x82, 0x03f7, 0x0020);  // Enable QMU frame count threshold (1), no auto-dequeue
    ret &= CheckRegister(Board, 0x90, 0xffff, 0x2000);  // Enable receive interrupts (TODO: also consider link change interrupt)
    ret &= CheckRegister(Board, 0x70, 0x0001, 0x0001);
    ret &= CheckRegister(Board, 0x74, 0x0001, 0x0001);
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

    // Reset the board
    Board.ResetKSZ8851();
    // Wait 100 msec
    usleep(100000L);

    // Read the status
    status = Board.ReadKSZ8851Status();
    std::cout << "   After reset, status = " << std::hex << status << std::endl;

    if (!(status&0x2000)) {
        std::cout << "   Ethernet failed initialization" << std::endl;
        return false;
    }

    // Read the Chip ID (16-bit read)
    AmpIO_UInt16 chipID = Board.ReadKSZ8851ChipID();
    std::cout << "   Chip ID = " << std::hex << chipID << std::endl;
    if ((chipID&0xfff0) != 0x8870)
        return false;


    // Check that KSZ8851 registers are as expected
    if (!CheckEthernet(Board))
        return false;

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

static char QuadletReadCallbackBoardId = 0;

bool QuadletReadCallback(Eth1394Port &, unsigned char boardId, std::ostream &debugStream)
{
    if (QuadletReadCallbackBoardId != boardId) {
        debugStream << "Warning: QuadletReadCallback called for board " << (unsigned int) boardId
                    << ", expected board " << (unsigned int) QuadletReadCallbackBoardId << std::endl;
        return false;
    }
    return true;
}
#endif

int main()
{
    // Hard-coded for board #0
    AmpIO board1(0);
    AmpIO board2(0);

#if Amp1394_HAS_RAW1394
    FirewirePort FwPort(0, std::cout);
    if (!FwPort.IsOK()) {
        std::cout << "Failed to initialize firewire port" << std::endl;
        return 0;
    }
    FwPort.AddBoard(&board1);
    if (!InitEthernet(board1)) {
        std::cout << "Failed to initialize Ethernet chip" << std::endl;
        FwPort.RemoveBoard(0);
        return 0;
    }

    QuadletReadCallbackBoardId = board1.GetBoardId();
    Eth1394Port EthPort(0, std::cout, QuadletReadCallback);
    if (!EthPort.IsOK()) {
        std::cout << "Failed to initialize ethernet port" << std::endl;
        FwPort.RemoveBoard(0);
        return 0;
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
    while (!done) {

        std::cout << std::endl << "Ethernet Test Program" << std::endl;
        std::cout << "  0) Quit" << std::endl;
        std::cout << "  1) Quadlet write to board" << std::endl;
        std::cout << "  2) Quadlet read from board" << std::endl;
        std::cout << "  3) Block read from  board" << std::endl;
        std::cout << "  4) Ethernet port status" << std::endl;
        std::cout << "  5) Initialize Ethernet port" << std::endl;
        std::cout << "  6) Ethernet debug info" << std::endl;
        std::cout << "Select option: ";
        
        int c = getchar();
        std::cout << std::endl << std::endl;

        nodeid_t boardid;
        nodeaddr_t addr;
        unsigned int tcode;
        quadlet_t quad_data;
        AmpIO_UInt16 srcAddress[3];
        quadlet_t read_data, write_data;
        quadlet_t fw_block_data[16];
        quadlet_t eth_block_data[16];
        int i;
        char buf[5];

        switch (c) {
        case '0':   // Quit
                done = true;
                break;

        case '1':   // Write quadlet from PC to FPGA
                write_data = 0x0;
                if (!EthPort.WriteQuadlet(0, 0, write_data))
                    std::cout << "Failed to write quadlet via Ethernet port" << std::endl;
                break;

        case '2':   // Read request from PC via Ethernet (note that QuadletReadCallback is called)
                read_data = 0;
                addr = 4;  // Return QLA1
                if (EthPort.ReadQuadlet(0, addr, read_data))
                    std::cout << "Read quadlet data: " << std::hex << bswap_32(read_data) << std::endl;
                else
                    std::cout << "Failed to read quadlet via Ethernet port" << std::endl;
                memcpy(buf, (char *)(&read_data), 4);
                buf[4] = 0;
                std::cout << "  as string: " << buf << std::endl;
                break;

        case '3':
                if (!FwPort.ReadBlock(0, 0, fw_block_data, sizeof(fw_block_data))) {
                    std::cout << "Failed to read block data via FireWire port" << std::endl;
                    break;
                }
                if (!EthPort.ReadBlock(0, 0, eth_block_data, sizeof(eth_block_data))) {
                    std::cout << "Failed to read block data via Ethernet port" << std::endl;
                    break;
                }
                for (i = 0; i < sizeof(fw_block_data)/sizeof(quadlet_t); i++)
                    std::cout << std::dec << i << ": " << std::hex << bswap_32(fw_block_data[i])
                              << ", " << bswap_32(eth_block_data[i]) << std::endl;
                break;

        case '4':
                PrintEthernetStatus(board1);
                break;

        case '5':
                InitEthernet(board1);
                break;

        case '6':
                PrintEthernetDebug(board1);
                break;
        }
    }

    tcsetattr(0, TCSANOW, &oldTerm);  // Restore terminal I/O settings
    FwPort.RemoveBoard(0);
#endif
    EthPort.RemoveBoard(0);
    return 0;
}
