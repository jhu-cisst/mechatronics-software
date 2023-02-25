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
#include "Amp1394BSwap.h"

const uint32_t VALID_BIT        = 0x80000000;  /*!< High bit of 32-bit word */
const uint32_t DAC_MASK         = 0x0000ffff;  /*!< Mask for 16-bit DAC values */

// CRC16-CCIT (also called CRC-ITU)
const uint16_t crc16_table[256] = {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

uint16_t ComputeCRC16(unsigned char *data, size_t len)
{
    uint16_t crc16 = 0;
    for (size_t i = 0; i < len; i++)
        crc16 = (crc16 << 8) ^ crc16_table[((crc16 >> 8) ^ data[i]) & 0x00ff];
    return  crc16;
}

void ComputeConfigCRC()
{
    uint16_t crc16, crc16Eth;
    quadlet_t bus_info[4];
    bus_info[0] = bswap_32(0x31333934);  // "1394"
    bus_info[1] = bswap_32(0x00ffa000);
    bus_info[2] = bswap_32(0xfa610e00);  // LCSR CID + BID + E/F
    bus_info[3] = bswap_32(0x00000007);  // Firmware version
    std::cout << std::hex;
    for (unsigned int bver = 0x0e; bver <= 0x0f; bver++) {
        std::cout << "    // Version: " << ((bver == 0x0e) ? "Ethernet" : "Firewire") << std::endl;
        std::cout << "    always @(*)" << std::endl;
        std::cout << "    begin" << std::endl;
        std::cout << "        case (board_id)" << std::endl;
        for (unsigned int bnum = 0; bnum < BoardIO::MAX_BOARDS; bnum++) {
            bus_info[2] = bswap_32(0xfa610e00|(bnum << 4)|bver);
            crc16 = ComputeCRC16(reinterpret_cast<unsigned char *>(bus_info), sizeof(bus_info));
            std::cout << "            4'h" << bnum << ": info_crc = 16'h"
                      << std::setw(4) << std::setfill('0') << crc16 << ";" << std::endl;
        }
        std::cout << "        endcase" << std::endl;
        std::cout << "    end" << std::endl;
    }
    quadlet_t root_dir[5];
    root_dir[0] = bswap_32(0x0c0083c0);
    root_dir[1] = bswap_32(0x03fa610e);
    root_dir[2] = bswap_32(0x81000003);
    root_dir[3] = bswap_32(0x17000001);
    root_dir[4] = bswap_32(0x81000006);
    crc16 = ComputeCRC16(reinterpret_cast<unsigned char *>(root_dir), sizeof(root_dir));
    root_dir[3] = bswap_32(0x17000002);
    crc16Eth = ComputeCRC16(reinterpret_cast<unsigned char *>(root_dir), sizeof(root_dir));
    std::cout << "    Root Directory CRC (Rev 1,2): " << std::setw(4) << std::setfill('0') << crc16
              << ", " << crc16Eth << std::endl;
    quadlet_t vendor_desc[4];
    vendor_desc[0] = bswap_32(0x00000000);
    vendor_desc[1] = bswap_32(0x00000000);
    vendor_desc[2] = bswap_32(0x4A485520);  // "JHU "
    vendor_desc[3] = bswap_32(0x4C435352);  // "LCSR"
    crc16 = ComputeCRC16(reinterpret_cast<unsigned char *>(vendor_desc), sizeof(vendor_desc));
    std::cout << "    Vendor Descriptor CRC: " << std::setw(4) << std::setfill('0') << crc16
              << std::endl;
    quadlet_t model_desc[5];
    model_desc[0] = bswap_32(0x00000000);
    model_desc[1] = bswap_32(0x00000000);
    model_desc[2] = bswap_32(0x46504741);   // "FPGA"
    model_desc[3] = bswap_32(0x312F514C);   // "1/QL"
    model_desc[4] = bswap_32(0x41000000);   // "A"
    crc16 = ComputeCRC16(reinterpret_cast<unsigned char *>(model_desc), sizeof(model_desc));
    model_desc[3] = bswap_32(0x322F514C);   // "2/QL"
    crc16Eth = ComputeCRC16(reinterpret_cast<unsigned char *>(model_desc), sizeof(model_desc));
    std::cout << "    Model Descriptor CRC (Rev 1,2): " << std::setw(4) << std::setfill('0') << crc16
              << ", " << crc16Eth << std::endl;
    std::cout << std::dec;
}

uint32_t KSZ8851CRC(const unsigned char *data, size_t len)
{
    uint32_t crc = 0xffffffff;
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
void ComputeMulticastHash(unsigned char *MulticastMAC, uint8_t &regAddr, uint16_t &regData)
{
    uint32_t crc = KSZ8851CRC(MulticastMAC, 6);
    int regOffset = (crc >> 29) & 0x0006;  // first 2 bits of CRC (x2)
    int regBit = (crc >> 26) & 0x00F;      // next 4 bits of CRC
    regAddr = 0xA0 + regOffset;            // 0xA0 --> MAHTR0 (MAC Address Hash Table Register 0)
    regData = (1 << regBit);
}

// Ethernet status from FPGA register 12
void PrintEthernetStatus(AmpIO &Board)
{
    uint32_t status;
    if (Board.ReadEthernetStatus(status))
        EthBasePort::PrintStatus(std::cout, status);
}

#if Amp1394_HAS_RAW1394
// Ethernet status, as reported by KSZ8851 on FPGA board
void PrintEthernetPhyStatusV2(AmpIO &Board)
{
    uint16_t reg;
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

void PrintEthernetPhyStatusV3(AmpIO &Board, unsigned int chan)
{
    std::cout << "PHY" << chan << ":";
    uint16_t reg;
    unsigned int phyAddr = FpgaIO::PHY_RTL8211F;
    // This should be on page 0xa43, but seems to work fine on default page
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PHYSR, reg)) {
        std::cout << " failed to read PHYSR" << std::endl;
        return;
    }
    if (reg & 0x0004) std::cout << " link-good";
    if (reg & 0x0008) std::cout << " full-duplex";
    else std::cout << " half-duplex";
    unsigned int speed = (reg & 0x0030)>>4;
    const char *speedStr[4] = { "10Mbps", "100 Mbps", "1000 Mbps", "??" };
    std::cout << " " << speedStr[speed];
    if (reg & 0x0002) std::cout << " polarity-reversed";
    std::cout << std::endl;

    // Speed setting uses bits 6 and 13, as follows:
    //
    //   speed | Mbps | 6,13  | register value
    //     0   |   10 | 0, 0  |     0x0000
    //     1   |  100 | 0, 1  |     0x2000
    //     2   | 1000 | 1, 0  |     0x0040
    //     3   |   ?? | 1, 1  |     0x2040
    unsigned short speedMap[4] = { 0x0000, 0x2000, 0x0040, 0x2040 };
    std::cout << "Reading GMII core register: ";
    uint16_t coreReg;
    if (Board.ReadRTL8211F_Register(chan, FpgaIO::PHY_GMII_CORE, 16, coreReg)) {
        std::cout << std::hex << coreReg << std::dec;
        for (size_t i = 0; i < 4; i++) {
            if ((coreReg&0x2040) == speedMap[i]) {
                std::cout << ", Speed: " << speedStr[i];
                break;
            }
        }
        std::cout << std::endl;
    }
    else {
        std::cout << " failed to read to GMII core register" << std::endl;
    }
}

// Check contents of KSZ8851 register
bool CheckRegister(AmpIO &Board, uint8_t regNum, uint16_t mask, uint16_t value)
{
    uint16_t reg;
    Board.ReadKSZ8851Reg(regNum, reg);
    if ((reg&mask) != value) {
        std::cout << "Register " << std::hex << (int)regNum << ": read = " << reg
                  << ", expected = " << value << " (mask = " << mask << ")" << std::endl;
        return false;
    }
    return true;
}

// Check whether Ethernet initialized correctly
bool CheckEthernetV2(AmpIO &Board)
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
    uint8_t HashReg;
    uint16_t HashValue;
    ComputeMulticastHash(MulticastMAC, HashReg, HashValue);
    ret &= CheckRegister(Board, HashReg, 0xffff, HashValue);
    ret &= CheckRegister(Board, 0x82, 0x03f7, 0x0020);  // Enable QMU frame count threshold (1), no auto-dequeue
    ret &= CheckRegister(Board, 0x90, 0xffff, 0xa000);  // Enable receive and link change interrupts
    std::cout << "Checking ---- end ----" << "\n";
    return ret;
}

bool CheckRTL8211F_RegIO(AmpIO &Board, unsigned int chan, unsigned int phyAddr)
{
    uint16_t curPage;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, curPage)) {
        std::cout << "Failed to read PHY" << chan << " PAGSR" << std::endl;
        return false;
    }
    if (curPage != FpgaIO::RTL8211F_PAGE_DEFAULT) {
        std::cout << std::hex << "Changing page from " << curPage << " to " << FpgaIO::RTL8211F_PAGE_DEFAULT << std::endl;
        if (!Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, FpgaIO::RTL8211F_PAGE_DEFAULT)) {
            std::cout << "Failed to write PHY" << chan << " PAGSR" << std::endl;
            return false;
        }
    }
    // Perform walking bit test on INER (interrupt enable register, 18)
    uint16_t curIner, mask_read;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_INER, curIner)) {
        std::cout << "Failed to read PHY" << chan << " INER" << std::endl;
        return false;
    }
    std::cout << "Testing RTL8211F PHY" << chan << " Register I/O, initial value = " << std::hex << curIner << std::endl;
    bool allOK = true;
    for (uint16_t mask = 0x1; mask != 0; mask <<= 1) {
        Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_INER, mask);
        Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_INER, mask_read);
        if (mask != mask_read) {
            std::cout << "Error: wrote " << mask << ", read " << mask_read << std::endl;
            allOK = false;
            //break;
        }
    }
    std::cout << std::dec << "Test complete" << std::endl;
    // Restore original values
    Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_INER, curIner);
    Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, curPage);
    return allOK;
}

bool CheckEthernetV3(AmpIO &Board, unsigned int chan)
{
    unsigned int phyAddr = FpgaIO::PHY_RTL8211F;

    // Check Register I/O
    if (!CheckRTL8211F_RegIO(Board, chan, phyAddr))
        return false;

    // Check PHYID1 and PHYID2 (assumes we are on correct page)
    uint16_t phyid1 = 0, phyid2 = 0;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PHYID1, phyid1))
        std::cout << "Failed to read PHY" << chan << " PHYID1" << std::endl;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PHYID2, phyid2))
        std::cout << "Failed to read PHY" << chan << " PHYID2" << std::endl;
    std::cout << "PHY" << chan << std::hex
              << " PHYID1: " << std::setw(4) << std::setfill('0') << phyid1 << " (should be 001c),"
              << " PHYID2: " << std::setw(4) << std::setfill('0') << phyid2 << " (should be c916)"
              << std::dec << std::endl;

    // Now, check undocumented registers 0x11 (17) and 0x15 (21) on page 0xd08
    //   Register 17, Bit 8 (0x0100) indicates state of TX_DELAY
    //   Register 21, Bit 3 (0x0008) indicates state of RX_DELAY
    if (!Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, 0xd08))
        std::cout << "Failed to set PHY" << chan << " PAGSR to 0xd08" << std::endl;
    uint16_t phyreg17, phyreg21;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, 17, phyreg17))
        std::cout << "Failed to read PHY" << chan << " Page 0xd08, Reg 17" << std::endl;
    std::cout << "PHY" << chan << std::hex << " Page 0xd08, Register 17: " << phyreg17 << std::dec << std::endl;
    std::cout << "Tx Delay: " << ((phyreg17 & 0x0100) ? "ON" : "OFF") << std::endl;
    if (!Board.ReadRTL8211F_Register(chan, phyAddr, 21, phyreg21))
        std::cout << "Failed to read PHY" << chan << " Page 0xd08, Reg 21" << std::endl;
    std::cout << "PHY" << chan << std::hex << " Page 0xd08, Register 21: " << phyreg21 << std::dec << std::endl;
    std::cout << "Rx Delay: " << ((phyreg21 & 0x0008) ? "ON" : "OFF") << std::endl;

    // Restore default page
    if (!Board.WriteRTL8211F_Register(chan, phyAddr, FpgaIO::RTL8211F_PAGSR, FpgaIO::RTL8211F_PAGE_DEFAULT))
        std::cout << "Failed to write PHY" << chan << " PAGSR" << std::endl;

    // Check GMII to RGMII core PHY register
    // Based on the VHDL source code for this core, PHY Specific Control Register 1 should be at address 16
    uint16_t phyCR;
    if (!Board.ReadRTL8211F_Register(chan, FpgaIO::PHY_GMII_CORE, 16, phyCR)) {
        std::cout << "Failed to read GMII PHY" << chan << " Reg 16" << std::endl;
    }
    else {
        std::cout << std::hex << "GMII PHY Reg 16: " << phyCR << std::dec << std::endl;
    }

    return (phyid1 == 0x001c) && (phyid2 == 0xc916);
}

bool InitEthernet(AmpIO &Board, unsigned int eth_port)
{
    if (Board.GetFirmwareVersion() < 5) {
        std::cout << "   No Ethernet controller, firmware version = " << Board.GetFirmwareVersion() << std::endl;
        return false;
    }

    unsigned int fpga_ver = Board.GetFpgaVersionMajor();
    if (fpga_ver < 2) {
        std::cout << "   No Ethernet in FPGA V" << fpga_ver << std::endl;
        return false;
    }
    PrintEthernetStatus(Board);

    // Reset the board
    Board.WriteEthernetPhyReset(eth_port);
    if (fpga_ver == 2) {
        // Wait 100 msec
        Amp1394_Sleep(0.1);
    }
    else {
        // Wait 500 msec
        Amp1394_Sleep(0.5);
    }

    // Read the status
    uint32_t status;
    Board.ReadEthernetStatus(status);
    std::cout << "   After reset, status = " << std::hex << ((fpga_ver == 2) ? (status>>16) : status) << std::endl;

    if (fpga_ver == 2) {
        if (!(status & FpgaIO::ETH_STAT_INIT_OK_V2)) {
            std::cout << "   Ethernet V2 failed initialization" << std::endl;
            EthBasePort::PrintStatus(std::cout, status);
            return false;
        }

        // Read the Chip ID (16-bit read)
        uint16_t chipID = Board.ReadKSZ8851ChipID();
        std::cout << "   Chip ID = " << std::hex << chipID << std::endl;
        if ((chipID&0xfff0) != 0x8870)
            return false;


        // Check that KSZ8851 registers are as expected
        if (!CheckEthernetV2(Board)) {
            PrintEthernetStatus(Board);
            return false;
        }

        // Display the MAC address
        uint16_t regLow, regMid, regHigh;
        Board.ReadKSZ8851Reg(0x10, regLow);   // MAC address low = 0x94nn (nn = board id)
        Board.ReadKSZ8851Reg(0x12, regMid);   // MAC address middle = 0xOE13
        Board.ReadKSZ8851Reg(0x14, regHigh);  // MAC address high = 0xFA61
        std::cout << "   MAC address = " << std::hex << regHigh << ":" << regMid << ":" << regLow << std::endl;
    }
    else if (fpga_ver == 3) {
        uint8_t portStatus = FpgaIO::GetEthernetPortStatusV3(status, eth_port);
        if (!(portStatus & FpgaIO::ETH_PORT_STAT_INIT_OK)) {
            std::cout << "   Ethernet V3 failed initialization" << std::endl;
            EthBasePort::PrintStatus(std::cout, status);
            return false;
        }

        // Check that RTL8211F registers are as expected
        if (!CheckEthernetV3(Board, eth_port)) {
            PrintEthernetStatus(Board);
            return false;
        }
    }

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

void TestDacRW(BasePort *port, unsigned char boardNum)
{
    size_t i;
    quadlet_t write_block[5];

    // Read from DAC (quadlet reads), modify values, write them using
    // a block write, then read them again to check.
    for (i = 0; i < 4; i++) {
        nodeaddr_t addr = 0x0001 | ((i+1) << 4);  // channel 1-4, DAC Control
        write_block[i] = 0;
        if (!port->ReadQuadlet(boardNum, addr, write_block[i]))
            std::cout << "Failed to read quadlet for channel " << (i+1) << std::endl;
        write_block[i] &= 0x0000ffff;
    }
    std::cout << "Read from DAC: " << std::hex << write_block[0] << ", "
              << write_block[1] << ", " << write_block[2] << ", "
              << write_block[3] << std::endl;
    for (i = 0; i < 4; i++) {
        write_block[i] = bswap_32(VALID_BIT | (boardNum<<24) | (write_block[i]+static_cast<quadlet_t>(i+1)*0x100));
    }
    write_block[4] = 0;   // power control
    std::cout << "Setting new values" << std::endl;
    if (!port->WriteBlock(boardNum, 0, write_block, sizeof(write_block)))
        std::cout << "Failed to write block data" << std::endl;
}

void  WriteAllBoardsTest(BasePort *port, std::vector<AmpIO *> &boardList)
{
    std::cout << "Protocol: " << port->GetProtocol() << std::endl;
    for (unsigned int j = 0; j < boardList.size(); j++) {
        boardList[j]->WritePowerEnable(true);
        boardList[j]->SetAmpEnableMask(0x0f, 0x0f);
        boardList[j]->SetSafetyRelay(true);
        for (int axis = 0; axis < 4; axis++)
            boardList[j]->SetMotorCurrent(axis, 0x8000+16*boardList[j]->GetBoardId()+axis);
    }
    port->WriteAllBoards();
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
    std::cout << "Node: " << std::dec << ((read_data >> 2) & 0x000003f);
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

void ReadConfigROM(BasePort *port, unsigned int boardNum)
{
    nodeaddr_t addr;
    quadlet_t read_data;
    quadlet_t block_data[24];
    addr = 0xfffff0000400;  // Configuration ROM address
    if (port->ReadQuadlet(boardNum, addr, read_data))
        std::cout << "Configuration ROM: " << std::hex << read_data << std::endl;
    std::cout << "Testing with block read (first entry should be ROM, followed by bus info):" << std::endl;
    if (port->ReadBlock(boardNum, addr, block_data, sizeof(block_data))) {
        for (size_t i = 0; i < sizeof(block_data)/sizeof(quadlet_t); i++) {
            std::cout << std::hex << std::setw(8) << std::setfill('0') << bswap_32(block_data[i]) << "  ";
            if (i%4 == 3) std::cout << std::endl;
        }
    }
    unsigned int bus_info_len = (bswap_32(block_data[0])&0xff000000) >> 24;
    std::cout << "Bus_Info length = " << bus_info_len << std::endl;
    if (bus_info_len > 1) {
        unsigned int bus_info_crc = bswap_32(block_data[0])&0x0000ffff;
        uint16_t crc16 = ComputeCRC16(reinterpret_cast<unsigned char *>(&block_data[1]), bus_info_len*sizeof(quadlet_t));
        std::cout << "Bus_Info CRC: ROM = " << std::hex << bus_info_crc << ", Computed = " << crc16 << std::dec << std::endl;
        unsigned int root_len = (bswap_32(block_data[bus_info_len+1])&0xffff0000) >> 16;
        unsigned int root_dir_crc = bswap_32(block_data[bus_info_len+1])&0x0000ffff;
        std::cout << "Root_Directory length = " << root_len << std::endl;
        crc16 = ComputeCRC16(reinterpret_cast<unsigned char *>(&block_data[bus_info_len+2]), root_len*sizeof(quadlet_t));
        std::cout << "Root_Dir CRC: ROM = " << std::hex << root_dir_crc << ", Computed = " << crc16 << std::dec << std::endl;
    }
    std::cout << std::endl;
    std::cout << "Testing again with block read (8th entry should be ROM, followed by bus info):" << std::endl;
    if (port->ReadBlock(boardNum, addr-7*sizeof(quadlet_t), block_data, sizeof(block_data))) {
        for (size_t i = 0; i < sizeof(block_data)/sizeof(quadlet_t); i++) {
            std::cout << std::hex << std::setw(8) << std::setfill('0') << bswap_32(block_data[i]) << "  ";
            if (i%4 == 3) std::cout << std::endl;
        }
    }
}

bool RunTiming(const std::string &portName, AmpIO *boardTest, EthBasePort *ethPort, const std::string &msgType, size_t numIter = 1000)
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
    double minTimeReceive = 1.0;
    double maxTimeReceive = 0.0;
    double minTimeSend = 1.0;
    double maxTimeSend = 0.0;
    for (i = 0; i < numIter; i++) {
        double startTime, deltaTime = 0;
        uint32_t status;
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
        if (ethPort) {
            // Get Ethernet timing (from FPGA)
            // Note that timeSend is a little short because it does not include the time
            // to send the last few bytes of the packet.
            double timeReceive = ethPort->GetFpgaReceiveTime();
            double timeSend = ethPort->GetFpgaTotalTime() - timeReceive;
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
    if (i > 0) {
        std::cout << "   Times (us), " << i << " iterations: mean = " << std::fixed << std::setprecision(2)
                  << (1.0e6*timeSum/i) << ", min = " << (1.0e6*minTime) << ", max = " << (1.0e6*maxTime) << std::endl;
        if (ethPort) {
            std::cout << "      FPGA receive min/max (us) = " << (1.0e6*minTimeReceive) << "/" << (1.0e6*maxTimeReceive);
            if (msgType == "quadlet read")
                std::cout << ", send min/max (us) = " << (1.0e6*minTimeSend) << "/" << (1.0e6*maxTimeSend);
            std::cout << std::endl;
        }
    }
    return true;
}

void TestBlockWrite(BasePort *wport, AmpIO *wboard, AmpIO *rboard)
{
    quadlet_t waveform[MAX_POSSIBLE_DATA_SIZE/sizeof(quadlet_t)];
    quadlet_t waveform_read[MAX_POSSIBLE_DATA_SIZE/sizeof(quadlet_t)];
    const unsigned int WLEN_MAX = (wport->GetMaxWriteDataSize()/sizeof(quadlet_t));
    unsigned int i;
    unsigned int min_left = WLEN_MAX;
    unsigned int min_wlen = WLEN_MAX;
    unsigned int max_wait = 0;
    unsigned int max_wlen = WLEN_MAX;
    unsigned int numSilentMismatch = 0;

    // Check if debug data available, in which case we can query bw_left
    bool debugValid = false;
    if (rboard->ReadEthernetData(waveform_read, 0x80, 16)) {
        char *cptr = reinterpret_cast<char *>(waveform_read);
        if (strncmp(cptr, "DBG1", 4) == 0) {
            debugValid = true;
        }
        else {
            std::cout << "Debug data not available" << std::endl;
            std::cout << "Will only perform read/write test" << std::endl;
        }
    }

    for (unsigned int wlen = 2; wlen < WLEN_MAX; wlen++) {
        bool doOut = ((wlen<10)||(wlen%20 == 0)||(wlen==WLEN_MAX-1));
        unsigned int len16 = 2*wlen;  // length in words
        unsigned int trigger_comp = 10 + len16/2+len16/8 + 2;   // computation on FPGA
        if (doOut) {
            std::cout << "Len=" << std::dec << wlen << ": ";
            if (debugValid) {
                std::cout << "trigger = " << (3*wlen-2)/5.0
                          << ", trigger_fpga = " << (trigger_comp-10)/2 << ", ";
            }
        }
        for (i = 0; i < wlen; i++) {
            waveform[i] = ((wlen+i)<< 16) | (wlen-i);
            waveform_read[i] = 0;
        }
        if (!wboard->WriteWaveformTable(waveform, 0, wlen)) {
            if (!doOut) std::cout << "Len=" << std::dec << wlen << ": ";
            std::cout << "WriteWaveformTable failed" << std::endl;
            return;
        }
        Amp1394_Sleep(0.05);
        if (debugValid) {
            if (!rboard->ReadEthernetData(waveform_read, 0x80, 16))
                break;
            unsigned short *ptr = reinterpret_cast<unsigned short *>(&waveform_read[14]);
            unsigned short bw_left = ptr[0];
            if (bw_left < min_left) {
                min_left = bw_left;
                min_wlen = wlen;
            }
            unsigned short bw_wait = ptr[1];
            if (bw_wait > max_wait) {
                max_wait = bw_wait;
                max_wlen = wlen;
            }
            if (doOut) {
                std::cout << "bw_left = " << bw_left << ", bw_wait = " << bw_wait;
            }
        }
        if (!rboard->ReadWaveformTable(waveform_read, 0, wlen)) {
            if (!doOut) std::cout << "Len=" << std::dec << wlen << ": ";
            std::cout << "ReadWaveformTable failed" << std::endl;
            break;
        }
        bool allOK = true;
        for (i = 0; i < wlen; i++) {
            if (waveform_read[i] != waveform[i]) {
                allOK = false;
                if (doOut) {
                    std::cout << "mismatch at quadlet " << std::dec << i << ", read "
                              << waveform_read[i] << ", expected " << waveform[i];
                    if (numSilentMismatch > 0) {
                        std::cout << std::endl << "There were " << numSilentMismatch
                                  << " additional lengths with mismatches";
                        numSilentMismatch = 0;
                    }
                }
                else
                    numSilentMismatch++;
                break;
            }
        }
        if (doOut) {
            if (!debugValid && allOK) std::cout << " Pass";
            std::cout << std::endl;
        }
    }
    if (numSilentMismatch > 0) {
        std::cout << "There were " << numSilentMismatch << " additional lengths with mismatches" << std::endl;
    }
    if (debugValid) {
        std::cout << "Mininum bw_left = " << std::dec << min_left << " at wlen = " << min_wlen << std::endl;
        std::cout << "Maximum bw_wait = " << std::dec << max_wait << " at wlen = " << max_wlen << std::endl;
    }
}

void TestWaveform(AmpIO *board)
{
    const unsigned int WLEN = 256;
    quadlet_t waveform[WLEN];
    quadlet_t waveform_read[WLEN];
    size_t i;
    // Set up square waves that change every 1 msec
    double clkPer = board->GetFPGAClockPeriod();
    uint32_t numTicks = static_cast<uint32_t>(0.001/clkPer);
    std::cout << "Setting waveform with " << numTicks << " counts (1 msec edges)" << std::endl;
    if ((numTicks&0x7fffff) != numTicks)
        std::cout << "Warning: numTicks does not fit in 23 bits" << std::endl;
    unsigned char dout1 = 0;
    unsigned char dout2 = 0;
    unsigned char dout3 = 0;
    unsigned char dout4 = 0;
    for (i = 0; i < WLEN-1; i++) {
        if (i%4 == 0) dout1 = 1-dout1;
        if (i%4 == 1) dout2 = 1-dout2;
        if (i%4 == 2) dout3 = 1-dout3;
        if (i%4 == 3) dout4 = 1-dout4;
        waveform[i] = 0x80000000 | (numTicks << 8) | (dout4 << 3) | (dout3 << 2) | (dout2 << 1) | dout1;
        waveform_read[i] = 0;
    }
    waveform[WLEN-1] = 0;
    waveform_read[WLEN-1] = 0;
    std::cout << "Writing test pattern" << std::endl;
    if (!board->WriteWaveformTable(waveform, 0, WLEN)) {
        std::cout << "WriteWaveformTable failed" << std::endl;
        return;
    }
    Amp1394_Sleep(0.05);
    std::cout << "Reading data" << std::endl;
    if (!board->ReadWaveformTable(waveform_read, 0, WLEN)) {
        std::cout << "ReadWaveformTable failed" << std::endl;
        return;
    }
    for (i = 0; i < WLEN; i++) {
        if (waveform_read[i] != waveform[i]) {
            std::cout << "Mismatch at quadlet " << i << ", read " << std::hex
                      << waveform_read[i] << ", expected " << waveform[i]
                      << std::dec << std::endl;
            return;
        }
    }
    std::cout << "Pattern verified!" << std::endl;
#if 0
    // Following code will actually generate waveform on DOUT lines
    board->WriteDigitalOutput(0x0f,0x00);
    // Start waveform on all DOUT channels
    board->WriteWaveformControl(0x0f, 0x0f);
#endif
}

// Following prints feedback from the MAX7317 I/O Expander on QLA Rev 1.5+
// Feedback from the MAX7301 I/O Expander on DQLA is a little different
void PrintIOExp(uint32_t iodata)
{
    std::cout << std::hex << (iodata&0x0000ffff) << "  :";
    bool output_error = iodata&0x01000000;
    if (output_error)
        std::cout << " output_error (" << ((iodata&0x00ff0000)>>16) << ") ";
    if (iodata&0x02000000) std::cout << " read_error";
    if (iodata&0x04000000) std::cout << " do_reg_io";
    if (iodata&0x08000000) std::cout << " reg_io_read";
    if (iodata&0x10000000) std::cout << " other_busy";
    std::cout << std::dec << std::endl;
}

// Following checks the configuration of MAX7317 I/O Expander on QLA Rev 1.5+
// (channel 0)
bool QLA_IOExp_Check(AmpIO &Board, unsigned long chan = 0)
{
    // TODO: update for non-zero channels (DQLA)
    uint32_t iodata;
    Board.WriteIOExpander(0x1345);   // Write 0x45 to RAM
    Board.WriteIOExpander(0x9300);   // Read from RAM
    Board.ReadIOExpander(iodata);
    std::cout << "Read from RAM (45): ";
    PrintIOExp(iodata);
    Board.WriteIOExpander(0x1300);   // Write 0x00 to RAM
    Board.WriteIOExpander(0x9300);   // Read from RAM
    std::cout << "Read from RAM (00): ";
    Board.ReadIOExpander(iodata);
    PrintIOExp(iodata);
    Board.WriteIOExpander(0x13ff);   // Write 0xff to RAM
    Board.WriteIOExpander(0x9300);   // Read from RAM
    Board.ReadIOExpander(iodata);
    std::cout << "Read from RAM (ff): ";
    PrintIOExp(iodata);
    for (int i = 0; i < 10; i++) {
        uint32_t cmd = 0x8000 | (i << 8);
        Board.WriteIOExpander(cmd);   // Read Port i status
        Board.ReadIOExpander(iodata);
        std::cout << "Read from Port " << i << ": ";
        PrintIOExp(iodata);
    }
    Board.WriteIOExpander(0x8e00);   // Read Ports 0-7
    Board.ReadIOExpander(iodata);
    std::cout << "Read from Ports 0-7: ";
    PrintIOExp(iodata);
    Board.WriteIOExpander(0x8f00);   // Read Ports 8-9
    Board.ReadIOExpander(iodata);
    std::cout << "Read from Ports 8-9: ";
    PrintIOExp(iodata);
    Board.WriteIOExpander(0x40000000);   // Select debug register
    Board.ReadIOExpander(iodata);
    std::cout << "outputs = " << std::hex << iodata << std::dec << std::endl;
    Board.WriteIOExpander(0xc0000000);   // Clear errors, select debug register
    Board.ReadIOExpander(iodata);
    std::cout << "outputs = " << std::hex << iodata << std::dec << std::endl;
    return true;
}

// Following checks the configuration of one of MAX7301 I/O Expanders on the DQLA
bool DQLA_IOExp_Check(AmpIO &Board, unsigned long chan)
{
    uint32_t iodata;
    unsigned int DQLA_Num = (chan == 3) ? 1 : 2;
    chan <<= 16;
    Board.WriteIOExpander(chan|0x08000000);     // NOP command to set channel for reading
    Board.ReadIOExpander(iodata);
    std::cout << "Checking DQLA I/O Expander " << DQLA_Num << " ..." << std::endl;
    if (iodata&0x00400000) {
        std::cout << "Error: not configured" << std::endl;
        return false;
    }
    if (iodata&0x00800000) {
        std::cout << "Error: failed to initialize" << std::endl;
        return false;
    }
    if (iodata&0x02000000) {
        std::cout << "Warning: read error detected" << std::endl;
    }
    // Check config commands: IOP[7:4], IOP[11:8], IOP[15:12], ...
    uint16_t configCmd[8] = {  0x09ff, 0x0aff, 0x0b55, 0x0c57, 0x0dff, 0x0eff, 0x0f55, 0x0401 };
    for (unsigned int j = 0; j < sizeof(configCmd)/sizeof(uint16_t); j++) {
        uint16_t readCmd = (configCmd[j]&0xff00)|0x8000;
        std::cout << "Checking config cmd " << j << ": " << std::hex << readCmd << std::dec << std::endl;
        unsigned int cnt;
        for (cnt = 0; cnt < 100; cnt++) {
            Board.WriteIOExpander(chan|readCmd);            // Read config
            Board.ReadIOExpander(iodata);
            if (iodata&0x04000000) {
                std::cout << "  Warning: register I/O still in process" << std::endl;
            }
            if (iodata&0x08000000) {
                std::cout << "  Warning: still waiting for register read data" << std::endl;
            }
            if ((iodata&0x0000ff00) == (readCmd&0x0000ff00))
                break;
        }
        if (cnt == 100) {
            std::cout << "  Warning: command mismatch, received: " << std::hex << (iodata&0x0000ff00)
                      << std::dec <<std::endl;
            continue;
        }
        else if (cnt > 0) {
            std::cout << "  Warning: took " << cnt << " attempts to obtain correct response" << std::endl;
        }
        if (iodata&0x01000000) {
            std::cout << "  Warning: output error detected (" << std::hex << ((iodata&0x001f0000)>>16)
                      << std::dec << ") " << std::endl;
        }
        if (iodata&0x10000000) {
            std::cout << "  Warning:  register I/O error (command ignored because previous not finished)" << std::endl;
        }
        else if ((iodata&0x000000ff) != (configCmd[j]&0x00ff)) {
            std::cout << "  Warning: configuration mismatch, received: " << std::hex << (iodata&0x000000ff)
                      << " (should be " << (configCmd[j]&0x00ff) << ")" << std::dec << std::endl;
        }
        else {
            std::cout << "  Configuration verified" << std::endl;
        }
    }
    Board.WriteIOExpander(chan|0x40000000);    // Select debug register
    Board.ReadIOExpander(iodata);
    unsigned int num_output_errors = (iodata&0x00ff0000)>>16;
    std::cout << "Debug register, number of output errors: " << num_output_errors << std::endl;
    if (iodata&0x02000000) {
        std::cout << "Warning: read_error detected" << std::endl;
    }
    if ((num_output_errors > 0) || (iodata&0x02000000)) {
        std::cout << "Clearing errors" << std::endl;
        Board.WriteIOExpander(chan|0x80000000);   // Clear errors (deselect debug)
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
    uint8_t RegAddr;
    uint16_t RegData;
    ComputeMulticastHash(MulticastMAC, RegAddr, RegData);
    std::cout << "Multicast hash table: register " << std::hex << (int)RegAddr << ", data = " << RegData << std::endl;

    std::vector<AmpIO *> FwBoardList;
    std::vector<AmpIO *> EthBoardList;

    AmpIO *curBoard = 0;     // Current board via Ethernet or Firewire
    AmpIO *curBoardFw = 0;   // Current board via Firewire
    AmpIO *curBoardEth = 0;  // Current board via Ethernet (sets boardNum)
    AmpIO *HubFw = 0;        // Ethernet Hub board via Firewire

    BasePort *curPort = 0;   // Current port (Ethernet or Firewire)

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
    unsigned int eth_port = 0;

    while (!done) {

        // Set curBoard to curBoardFw if it is valid; otherwise curBoardEth.
        // Also set curPort to be consistent with curBoard.
#if Amp1394_HAS_RAW1394
        if (curBoardFw) {
            curBoard = curBoardFw;
            curPort = &FwPort;
        }
        else {
            curBoard = curBoardEth;
            curPort = EthPort;
        }
#else
        curBoard = curBoardEth;
        curPort = EthPort;
#endif

        std::cout << std::endl << "Ethernet Test Program" << std::endl;
        if (curBoard) {
            uint32_t status;
            curBoard->ReadEthernetStatus(status);
            eth_port = FpgaIO::GetEthernetPortCurrent(status);
        }
        if (curBoardFw) {
            std::cout << "  Firewire board: " << static_cast<unsigned int>(curBoardFw->GetBoardId()) << std::endl;
        }
        if (curBoardEth) {
            std::cout << "  Ethernet board: " << static_cast<unsigned int>(curBoardEth->GetBoardId());
            if (eth_port != 0)
                std::cout << ", Eth" << eth_port << std::endl;
            std::cout << std::endl;
        }
        std::cout << "  0) Quit" << std::endl;
        if (curBoardEth) {
            std::cout << "  1) Quadlet write (power/relay toggle) to board via Ethernet" << std::endl;
            std::cout << "  2) Quadlet read from board via Ethernet" << std::endl;
        }
        if (curBoardEth || curBoardFw) {
            std::cout << "  3) Block read from board via Ethernet and/or Firewire" << std::endl;
            std::cout << "  4) Block write to board via Ethernet or Firewire" << std::endl;
        }
        if (curBoardFw) {
            std::cout << "  5) Ethernet port status" << std::endl;
            std::cout << "  6) Initialize Ethernet port" << std::endl;
        }
        if (curBoard)
            std::cout << "  7) Ethernet status info" << std::endl;
        std::cout << "  8) Multicast quadlet read via Ethernet" << std::endl;
        if (curBoardEth) {
            std::cout << "  a) WriteAllBoards test" << std::endl;
            std::cout << "  B) BlockWrite test (Waveform table)" << std::endl;
        }
        if ((EthBoardList.size() > 0) || (FwBoardList.size() > 0))
            std::cout << "  b) Change Firewire/Ethernet board" << std::endl;
        std::cout << "  C) Compute Configuration ROM CRC" << std::endl;
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
        if (curBoard) {
            std::cout << "  m) Initialize and test I/O Expander (QLA 1.5+)" << std::endl;
        }
        std::cout << "  r) Check Firewire bus generation and rescan if needed" << std::endl;
        if (curBoardFw)
            std::cout << "  R) Read Firewire Configuration ROM" << std::endl;
        if (curBoardEth)
            std::cout << "  t) Run Ethernet timing analysis" << std::endl;
        if (curBoard) {
            std::cout << "  v) Measure motor power supply voltage (QLA 1.5+)" << std::endl;
            std::cout << "  w) Test waveform buffer" << std::endl;
            std::cout << "  x) Read Ethernet debug data" << std::endl;
        }
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
#if Amp1394_HAS_RAW1394
            if (curBoardFw) {
                std::cout << "Testing via Firewire" << std::endl;
                TestDacRW(&FwPort, curBoardFw->GetBoardId());
                break;
            }
#endif
            if (curBoardEth) {
                std::cout << "Testing via Ethernet" << std::endl;
                TestDacRW(EthPort, curBoardEth->GetBoardId());
            }
            break;

#if Amp1394_HAS_RAW1394
        case '5':
            if (curBoardFw) {
                unsigned int fpgaVer = curBoardFw->GetFpgaVersionMajor();
                if (fpgaVer == 2) {
                    PrintEthernetPhyStatusV2(*curBoardFw);
                }
                else if (fpgaVer == 3) {
                    PrintEthernetPhyStatusV3(*curBoardFw, 1);
                    PrintEthernetPhyStatusV3(*curBoardFw, 2);
                }
            }
            break;

        case '6':
            if (curBoardFw)
                InitEthernet(*curBoardFw, eth_port);
            break;
#endif

        case '7':
            if (curBoard)
                PrintEthernetStatus(*curBoard);
            break;

        case '8':   // Read request via Ethernet multicast
            read_data = 0;
            addr = 0;  // Return status register
            if (EthPort->ReadQuadlet(FW_NODE_BROADCAST, addr, read_data))
                std::cout << "Read quadlet data: " << std::hex << read_data << std::endl;
            else
                std::cout << "Failed to read quadlet via Ethernet port" << std::endl;
            break;

        case 'a':
            if (curBoardEth)
                WriteAllBoardsTest(EthPort, EthBoardList);
            break;

        case 'B':
            if (curBoardEth)
                TestBlockWrite(EthPort, curBoardEth, curBoard);
            break;

        case 'b':
            curBoardFw = SelectBoard("Firewire", FwBoardList, curBoardFw);
            curBoardEth = SelectBoard("Ethernet", EthBoardList, curBoardEth);
            break;

        case 'c':
            if (curBoardEth)
                ContinuousReadTest(EthPort, boardNum);
            break;

        case 'C':
            ComputeConfigCRC();
            break;

        case 'd':
            if (curBoardEth)
                ContinuousWriteTest(EthPort, boardNum);
            break;

        case 'e':
            if (curBoardFw) {
                uint16_t reg;
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

        case 'm':   // TEST I/O EXPANDER
            if (curBoard) {
                if (curBoard->GetHardwareVersion() == DQLA_String) {
                    DQLA_IOExp_Check(*curBoard, 3);
                    DQLA_IOExp_Check(*curBoard, 4);
                    // Could also test I/O expanders on QLA Rev 1.5+
                    // In DQLA systems, need to use channels 1 and 2
                    // QLA_IOExp_Check(*curBoard, 1);
                    // QLA_IOExp_Check(*curBoard, 2);
                }
                else {
                    // Test I/O expander on QLA Rev 1.5+ (channel 0)
                    QLA_IOExp_Check(*curBoard);
                }
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

#if Amp1394_HAS_RAW1394
        case 'R':
            if (curBoardFw)
                ReadConfigROM(&FwPort, curBoardFw->GetBoardId());
            break;
#endif

        case 't':
            if (curBoardEth)
                RunTiming("Ethernet", curBoardEth, EthPort, "quadlet read");
            if (curBoardFw)
                RunTiming("Firewire", curBoardFw, 0, "quadlet read");
            if (curBoardEth)
                RunTiming("Ethernet", curBoardEth, EthPort, "quadlet write");
            if (curBoardFw)
                RunTiming("Firewire", curBoardFw, 0, "quadlet write");
            break;

        case 'v':
            // Measure power supply voltage (QLA 1.5+)
            if (curBoard) {
                // Make sure board power is on and amplifier power is off
                curBoard->WritePowerEnable(true);
                curBoard->WriteAmpEnable(0x0f, 0);
                uint32_t volts;
                uint32_t vinc = 0x1000;
                for (volts = 0; volts <= 0x0000ffff; volts += vinc) {
                    // Write voltage to DAC 4
                    curPort->WriteQuadlet(curBoard->GetBoardId(), 0x41, volts | 0x80000000);
                    Amp1394_Sleep(0.001);
                    // Read comparator bit (register 10, bit 28)
                    uint32_t iodata;
                    curPort->ReadQuadlet(curBoard->GetBoardId(), 10, iodata);
                    // iodata&0x10000000 is false (0) when DAC voltage is greater than motor power suply.
                    if ((iodata&0x10000000) == 0) {
                        if (vinc <= 1) break;
                        // Backup and reduce resolution
                        volts -= vinc;
                        vinc >>= 1;
                    }
                }
                if (volts <= 0x0000ffff) {
                    const double bits2volts = (2.5/65535)*21.0;
                    std::cout << "Measured power supply voltage = " << volts << " (" << (volts*bits2volts)
                              << ")" << std::endl;
                }
                else {
                    std::cout << "Failed to measure motor supply voltage" << std::endl;
                }
                curPort->WriteQuadlet(curBoard->GetBoardId(), 0x41, 0x80008000);
                curBoard->WritePowerEnable(false);
            }
            break;

        case 'w':
            TestWaveform(curBoard);
            break;

        case 'x':
            if (curBoard) {
                unsigned int fpgaVer = curBoard->GetFpgaVersionMajor();
                double clkPeriod = curBoard->GetFPGAClockPeriod();
                if (curBoard->ReadEthernetData(buffer, (eth_port << 8) | 0xc0, 16))     // PacketBuffer
                    EthBasePort::PrintEthernetPacket(std::cout, buffer, 16);
                if (curBoard->ReadEthernetData(buffer, (eth_port << 8), 64))            // FireWire packet
                    EthBasePort::PrintFirewirePacket(std::cout, buffer, 64);
                if (curBoard->ReadEthernetData(buffer, (eth_port << 8) | 0x80, 16))     // EthernetIO DebugData
                    EthBasePort::PrintDebugData(std::cout, buffer, clkPeriod);
                if (curBoard->ReadEthernetData(buffer, (eth_port << 8) | 0x90, 16)) {   // Low-level DebugData
                    if (fpgaVer == 2)
                        EthBasePort::PrintDebugDataKSZ(std::cout, buffer, clkPeriod);  // KSZ8851 (FPGA V2)
                    else {
                        EthBasePort::PrintDebugDataRTL(std::cout, buffer, clkPeriod);  // RTL8211F (FPGA V3)
                        uint32_t ethStatus;
                        curBoard->ReadEthernetStatus(ethStatus);
                        if (ethStatus & FpgaIO::ETH_STAT_CLK_OK_V3)
                            std::cout << "200 MHz clock running" << std::endl;
                        else
                            std::cout << "**** 200 MHz clock not running -- initialize PS" << std::endl;
                    }
                }
#if 0
                if ((fpgaVer == 2) && (curBoard->ReadEthernetData(buffer, 0xa0, 32))) {
                    std::cout << "Initialization Program:" << std::endl;
                    for (int i = 0; i < 32; i++) {
                        if (i == 16)
                           std::cout << "Run Program:" << std::endl;
                        if (buffer[i] != 0) {
                           if (buffer[i]&0x02000000) std::cout << "   Write ";
                           else std::cout << "   Read  ";
                           std::cout << std::hex << std::setw(2) << std::setfill('0')
                                     << ((buffer[i]&0x00ff0000)>>16) << " ";           // address
                           std::cout << std::hex << std::setw(4) << std::setfill('0')
                                     << (buffer[i]&0x0000ffff);
                           if (buffer[i]&0x01000000) std::cout << " MOD";
                           std::cout << std::endl;
                        }
                    }
                }
                if (curBoard->ReadEthernetData(buffer, (eth_port << 8) | 0xe0, 21)) {   // ReplyIndex
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
                unsigned int fpgaVer = curBoardFw->GetFpgaVersionMajor();
                std::cout << "FPGA Rev " << fpgaVer << std::endl;
                if (fpgaVer == 2) {
                    // Check that KSZ8851 registers are as expected
                    if (!CheckEthernetV2(*curBoardFw))
                        PrintEthernetStatus(*curBoardFw);
                }
                else if (fpgaVer == 3) {
                    CheckEthernetV3(*curBoardFw, 1);
                    CheckEthernetV3(*curBoardFw, 2);
                }
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
