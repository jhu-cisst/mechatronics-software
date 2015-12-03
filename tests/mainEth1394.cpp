/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <iostream>
#include "FirewirePort.h"
#include "Eth1394Port.h"
#include "AmpIO.h"

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
    Board.WriteKSZ8851Reg(0x70, (AmpIO_UInt16) 0x01EE);  // Enable QMU transmit flow conrol
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
    return true;
}

bool SendEthernetPacket(AmpIO &Board)
{
    AmpIO_UInt16 reg;
    // Read QMU TXQ available memory
    Board.ReadKSZ8851Reg(0x78, reg);
    reg &= 0x0fff;
    std::cout << "TXQ memory = " << std::hex << reg << std::endl;
    Board.WriteKSZ8851Reg(0x90, (AmpIO_UInt16) 0);  // disable interrupts
    // Set up QMU DMA transfer operation
    Board.ReadKSZ8851Reg(0x82, reg);
    reg |= (1<<3);
    Board.WriteKSZ8851Reg(0x82, reg);
    // Now, transfer via DMA
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x8000);
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 14+20);
    // Dest MAC: "HUB>PC" (byte swapped)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x5548); // "UH"
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x3E42); // ">B"
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x4350); // "CP"
    // Source MAC: "LCSR" 001XXXXXXXXXXXXX
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x434C); // "CL"
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x5253); // "RS"
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x2000); 
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x0108);  // Ethertype
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x0000);  // Payload starts here (15,14)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x6000);  // (17,16)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x0000);  // (19,18)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x0000);  // (21,20)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x0000);  // (23,22)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x0000);  // (25,24)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x3412);  // (27,26)
    Board.WriteKSZ8851DMA((AmpIO_UInt16) 0x7856);  // (29,28)
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
    int rxFrameCount = (reg >> 8);
    std::cout << "Received " << rxFrameCount << " frames" << std::endl;
    int i, j;
    for (i = 0; i < rxFrameCount; i++) {
        Board.ReadKSZ8851Reg(0x7C, reg);  // read frame header status
        std::cout << "Frame " << i << ": ";
        bool valid = (reg & 0x8000);
        if (valid) std::cout << "valid";
        else std::cout << "invalid";
        if (reg & 0x3C00) {
            std:: cout << ", checksum error";
            valid = false;
        }
        if (reg & 0x0001) {
            std::cout << ", CRC error";
            valid = false;
        }
        if (reg & 0x0016) {
            std::cout << ", other error";
            valid = false;
        }
        std::cout << std::endl;
        if (valid) {
            Board.ReadKSZ8851Reg(0x7E, reg);  // read frame byte size
            int rxPacketLength = (reg&0x0fff);
            std::cout << "  packet length = " << rxPacketLength << std::endl;
            Board.WriteKSZ8851Reg(0x86, (AmpIO_UInt16) 0x4000);  // Reset QMU RXQ frame pointer to 0
            // Set up QMU DMA transfer operation
            Board.ReadKSZ8851Reg(0x82, reg);
            reg |= (1<<3);
            Board.WriteKSZ8851Reg(0x82, reg);
            // Now, transfer via DMA
            Board.ReadKSZ8851DMA(reg);  // Ignore first 2 bytes
            AmpIO_UInt16 controlWord;
            Board.ReadKSZ8851DMA(status);  // Read status word
            AmpIO_UInt16 byteCount;
            Board.ReadKSZ8851DMA(byteCount);  // Read byte count
            int lengthInWord = ((rxPacketLength+3)>>1) & 0xfffe;
            std::cout << "  read byte count = " << byteCount << ", length in words = "
                      << lengthInWord << std::endl;
            AmpIO_UInt16 data;
            for (j = 0; j < lengthInWord; j++) {
                Board.ReadKSZ8851DMA(data);
                std::cout << "  " << std::hex << data;
                if (j%4==3) std::cout << std::endl;
            }
            // Stop QMU DMA transfer operation
            Board.ReadKSZ8851Reg(0x82, reg);
            reg &= ~(1<<3);
            Board.WriteKSZ8851Reg(0x82, reg);
        }
        else {
            // Issue RELEASE error frame command for QMU
            Board.ReadKSZ8851Reg(0x82, reg);
            reg |= 1;
            Board.WriteKSZ8851Reg(0x82, reg);

        }
    }
    Board.WriteKSZ8851Reg(0x90, (AmpIO_UInt16) 0x6000);   // Enable transmit and receive interrupts

    return true;
}

int main()
{
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

    if (InitEthernet(board1)) {
        std::cout << "Initialized Ethernet chip" << std::endl;

        for (;;) {
            SendEthernetPacket(board1);

            // Wait 100 msec
            usleep(100000L);

            ReceiveEthernetPacket(board1);

            quadlet_t read_data = 0;
            if (!EthPort.ReadQuadlet(0, 0, read_data))
                std::cout << "Failed to read quadlet via Ethernet port" << std::endl;
            else
                std::cout << "Read quadlet data: " << std::hex << read_data << std::endl;
        }
    }

    FwPort.RemoveBoard(0);
    EthPort.RemoveBoard(0);
    return 0;
}
