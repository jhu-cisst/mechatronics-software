/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include "FirewirePort.h"
#include "Eth1394Port.h"
#include "AmpIO.h"

#ifdef _MSC_VER
#include <stdlib.h>   // for byteswap functions
inline uint16_t bswap_16(uint16_t data) { return _byteswap_ushort(data); }
inline uint32_t bswap_32(uint32_t data) { return _byteswap_ulong(data); }
#else
#include <byteswap.h>
#endif

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
    // Read the Chip ID (16-bit read)
    AmpIO_UInt16 chipID = Board.ReadKSZ8851ChipID();
    std::cout << "   Chip ID = " << std::hex << chipID << std::endl;
    if ((chipID&0xfff0) != 0x8870)
        return false;
    // Now, program the chip
    AmpIO_UInt16 reg;
    Board.WriteKSZ8851Reg(0x10, (AmpIO_UInt16) 0x89AB);  // MAC address low = 0x89AB
    Board.WriteKSZ8851Reg(0x12, (AmpIO_UInt16) 0x4567);  // MAC address middle = 0x4567
    Board.WriteKSZ8851Reg(0x14, (AmpIO_UInt16) 0x0123);  // MAC address high = 0x0123
    Board.WriteKSZ8851Reg(0x84, (AmpIO_UInt16) 0x4000);  // Enable QMU transmit frame data pointer auto increment
    Board.WriteKSZ8851Reg(0x70, (AmpIO_UInt16) 0x01EE);  // Enable QMU transmit flow control, CRC, and padding
    Board.WriteKSZ8851Reg(0x86, (AmpIO_UInt16) 0x4000);  // Enable QMU receive frame data pointer auto increment
    Board.WriteKSZ8851Reg(0x9C, (AmpIO_UInt16) 0x0001);  // Configure receive frame threshold for 1 frame
    Board.WriteKSZ8851Reg(0x74, (AmpIO_UInt16) 0x74F2);  // Enable QMU receive flow control (0x7CE0 recommended)
    Board.WriteKSZ8851Reg(0x76, (AmpIO_UInt16) 0x0016);  // Enable QMU receive ICMP/UDP lite frame checksum verification
    Board.WriteKSZ8851Reg(0x82, (AmpIO_UInt16) 0x0030);  // Enable QMU receive IP header 2-byte offset (0x0230 recommended)
    // Force link in half duplex if auto-negotiation failed; restart port 1 auto-negotiation
    Board.ReadKSZ8851Reg(0xF6, reg);
    reg = (reg & ~(1<<5)) | (1 << 13);
    Board.WriteKSZ8851Reg(0xF6, reg);
     // ---
    Board.WriteKSZ8851Reg(0x92, (AmpIO_UInt16) 0xFFFF);   // Clear the interrupt status
    Board.WriteKSZ8851Reg(0x90, (AmpIO_UInt16) 0x6000);   // Enable transmit and receive interrupts
    // Enable QMU transmit
    Board.ReadKSZ8851Reg(0x70, reg);
    reg |= 1;
    Board.WriteKSZ8851Reg(0x70, reg);
    // Enable QMU receive
    Board.ReadKSZ8851Reg(0x74, reg);
    reg |= 1;
    Board.WriteKSZ8851Reg(0x74, reg);
    // Wait 100 msec
    usleep(100000L);
    return true;
}

bool SendEthernetPacket(AmpIO &Board)
{
    AmpIO_UInt16 reg;
    // Read QMU TXQ available memory
    Board.ReadKSZ8851Reg(0x78, reg);
    reg &= 0x0fff;
    std::cout << "TXQ memory = " << std::dec << reg << std::endl;
    Board.WriteKSZ8851Reg(0x90, (AmpIO_UInt16) 0);  // disable interrupts
    // Set up QMU DMA transfer operation
    Board.ReadKSZ8851Reg(0x82, reg);
    reg |= (1<<3);
    Board.WriteKSZ8851Reg(0x82, reg);
    // Now, transfer via DMA
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x8000);   // Control word (do not need interrupt on completion)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 34);  // Byte count (14+20=34)
    // Dest MAC: "HUB>PC" (byte swapped)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x5548); // "UH"
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x3E42); // ">B"
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x4350); // "CP"
    // Source MAC: "LCSR" 001XXXXXXXXXXXXX
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x434C); // "CL"
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x5253); // "RS"
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x2000); 
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x0108);  // Ethertype
    //Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x1400); // Length (20 bytes), instead of Ethertype
    // We use 20 bytes for a quadlet read/write. The chip will automatically
    // pad the packet since the length is less than 64 bytes.
    AmpIO_UInt16 packet[10];  // 20 bytes
    Eth1394Port::make_1394_header(reinterpret_cast<quadlet_t *>(packet), 0, 0, 6); // 6 -> Eth1394Port::QRESPONSE
    Board.WriteKSZ8851DMA(packet[0]);  // (15,14)
    Board.WriteKSZ8851DMA(packet[1]);  // (17,16)
    Board.WriteKSZ8851DMA(packet[2]);  // (19,18)
    Board.WriteKSZ8851DMA(packet[3]);  // (21,20)
    Board.WriteKSZ8851DMA(packet[4]);  // (23,22)
    Board.WriteKSZ8851DMA(packet[5]);  // (25,24)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x3412);  // (27,26) Quadlet data
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x7856);  // (29,28) Quadlet data
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x0000);  // (30,29) FireWire CRC
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x0000);  // (32,31) FireWire CRC
    // Stop QMU DMA transfer operation
    Board.ReadKSZ8851Reg(0x82, reg);
    reg &= ~(1<<3);
    Board.WriteKSZ8851Reg(0x82, reg);
    // TxQ manual Enqueue
    Board.ReadKSZ8851Reg(0x80, reg);
    reg |= 1;
    Board.WriteKSZ8851Reg(0x80, reg);
    // Enable device interrupts
    Board.WriteKSZ8851Reg(0x90, (AmpIO_UInt16) 0x6000);   // Enable transmit and receive interrupts
    return true;
}

bool ReceiveEthernetPacket(AmpIO &Board)
{
    AmpIO_UInt16 status = Board.ReadKSZ8851Status();
    if (!(status & 0x8000)) {
        std::cout << "Error: Ethernet not present" << std::endl;
        return false;
    }
    if ((status & 0x0010)) {
        std::cout << "IRQ not asserted" << std::endl;
        return false;
    }
    AmpIO_UInt16 reg;
    Board.ReadKSZ8851Reg(0x92, reg);
    if (!(reg & 0x2000)) {
        std::cout << "RXIS interrupt not asserted" << std::endl;
        return false;
    }
    Board.WriteKSZ8851Reg(0x90, (AmpIO_UInt16) 0);  // disable interrupts
    reg |= 0x2000;
    Board.WriteKSZ8851Reg(0x92, reg);  // clear RXIS bit
    Board.ReadKSZ8851Reg(0x9C, reg);   // read frame count
    unsigned int rxFrameCount = (reg >> 8);
    unsigned int numIP = 0;
    unsigned int numIPv6 = 0;
    unsigned int numValid = 0;
    unsigned int numChecksumError = 0;
    unsigned int numCRCError = 0;
    unsigned int numOtherError = 0;
    unsigned int i, j;
    for (i = 0; i < rxFrameCount; i++) {
        Board.ReadKSZ8851Reg(0x7C, reg);  // read frame header status
        bool valid = (reg & 0x8000);
        if (valid) numValid++;
        if (reg & 0x3C00) {
            numChecksumError++;
            valid = false;
        }
        else if (reg & 0x0001) {
            numCRCError++;
            valid = false;
        }
        else if (reg & 0x0016) {
            numOtherError++;
            valid = false;
        }
        if (valid) {
            Board.ReadKSZ8851Reg(0x7E, reg);  // read frame byte size
            int rxPacketLength = (reg&0x0fff);
            Board.WriteKSZ8851Reg(0x86, (AmpIO_UInt16) 0x4000);  // Reset QMU RXQ frame pointer to 0
            // Set up QMU DMA transfer operation
            Board.ReadKSZ8851Reg(0x82, reg);
            reg |= (1<<3);
            Board.WriteKSZ8851Reg(0x82, reg);
            usleep(1);
            // Now, transfer via DMA
            Board.ReadKSZ8851DMA(reg);     // Ignore first 2 bytes
            Board.ReadKSZ8851DMA(status);  // Read status word (same as register 0x7C)
            AmpIO_UInt16 byteCount;
            Board.ReadKSZ8851DMA(byteCount);  // Read byte count
            unsigned int lengthInWord = ((rxPacketLength+3)>>1) & 0xfffe;
            AmpIO_UInt16 *data = new AmpIO_UInt16[lengthInWord];
            for (j = 0; j < lengthInWord; j++) {
                Board.ReadKSZ8851DMA(data[j]);
                bswap_16(data[j]);
            }
            // Stop QMU DMA transfer operation
            Board.ReadKSZ8851Reg(0x82, reg);
            reg &= ~(1<<3);
            Board.WriteKSZ8851Reg(0x82, reg);
            if (status & 0x8000) {  // if really valid
                if (data[6] == 0xdd86) numIPv6++;
                else if (data[6] == 0x0008) numIP++;
                if (data[6] == 0x0108)
                    std::cout << "Quadlet data = " << std::hex << data[14] << data[13] << std::endl;
            }
#if 0
            for (j = 0; j < lengthInWord; j++) {
                std::cout << "  " << std::hex << data[j];
                if (j%4==3) std::cout << std::endl;
            }
#endif
            delete [] data;
        }
        else {
            // Issue RELEASE error frame command for QMU
            Board.ReadKSZ8851Reg(0x82, reg);
            reg |= 1;
            Board.WriteKSZ8851Reg(0x82, reg);

        }
    }
    Board.WriteKSZ8851Reg(0x90, (AmpIO_UInt16) 0x6000);   // Enable transmit and receive interrupts

    std::cout << "Received " << std::dec << rxFrameCount << " frames, " << numValid << " valid" << std::endl;
    if (numChecksumError || numCRCError || numOtherError) {
        std::cout << "Checksum Error = " << numChecksumError << ", CRC Error = " << numCRCError
                  << ", Other Error = " << numOtherError << std::endl;
    }
    std::cout << "  IPv6 frames = " << numIPv6 << ", IP frames = " << numIP << std::endl;

    return true;
}

static AmpIO *QuadletReadCallbackBoard = 0;

bool QuadletReadCallback(Eth1394Port &, unsigned char boardId, std::ostream &debugStream)
{
    if (QuadletReadCallbackBoard) {
        if (QuadletReadCallbackBoard->GetBoardId() != boardId)
            debugStream << "Warning: QuadletReadCallback called for board " << (unsigned int) boardId
                        << ", expected board " << (unsigned int) QuadletReadCallbackBoard->GetBoardId() << std::endl;
        AmpIO &Board = *QuadletReadCallbackBoard;
        // Receive the quadlet read request on the FPGA
        ReceiveEthernetPacket(Board);
        // Send a response packet
        SendEthernetPacket(Board);
    }
    else {
        std::cout << "QuadletReadCallbackBoard not defined" << std::endl;
        return false;
    }
    return true;
}

int main()
{
    quadlet_t read_data, write_data;

    FirewirePort FwPort(0, std::cout);
    if (!FwPort.IsOK()) {
        std::cout << "Failed to initialize firewire port" << std::endl;
        return 0;
    }
    Eth1394Port EthPort(0, std::cout);
    if (!EthPort.IsOK()) {
        std::cout << "Failed to initialize ethernet port" << std::endl;
        return 0;
    }

    // Hard-coded for board #0
    AmpIO board1(0);
    AmpIO board2(0);

    FwPort.AddBoard(&board1);
    EthPort.AddBoard(&board2);

    if (!InitEthernet(board1)) {
        std::cout << "Failed to initialize Ethernet chip" << std::endl;
        return 0;
    }

    QuadletReadCallbackBoard = &board1;
    EthPort.SetEth1394Callback(QuadletReadCallback);

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
        std::cout << "  1) Send quadlet from PC to FPGA:" << std::endl;
        std::cout << "  2) Send quadlet from FPGA to PC:" << std::endl;
        std::cout << "Select option: ";
        
        int c = getchar();
        std::cout << std::endl << std::endl;

        switch (c) {
            case '0':   // Quit
                done = true;
                break;

            case '1':   // Write quadlet from PC to FPGA
                // Write from PC via Ethernet
                write_data = 0x05165590;
                if (!EthPort.WriteQuadlet(0, 0, write_data))
                    std::cout << "Failed to write quadlet via Ethernet port" << std::endl;

                // Wait 20 msec
                usleep(20000L);

                // Receive on FPGA board
                ReceiveEthernetPacket(board1);
                break;

        case '2': 
                // Read request from PC via Ethernet (note that QuadletReadCallback is called)
                read_data = 0;
                if (EthPort.ReadQuadlet(0, 0, read_data))
                    std::cout << "Read quadlet data: " << std::hex << bswap_32(read_data) << std::endl;
                else
                    std::cout << "Failed to read quadlet via Ethernet port" << std::endl;
                break;

        }
    }

    FwPort.RemoveBoard(0);
    EthPort.RemoveBoard(0);
    tcsetattr(0, TCSANOW, &oldTerm);  // Restore terminal I/O settings
    return 0;
}
