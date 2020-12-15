/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/****************************************************************************************
 *
 * This program is used to test the encoders, in particular the velocity and acceleration
 * estimation, assuming that the FPGA/QLA is connected to the FPGA1394-QLA-Test board.
 *
 * Usage: enctest [-pP] <board num>
 *        where P is the Firewire port number (default 0),
 *        or a string such as ethP and fwP, where P is the port number
 *
 *****************************************************************************************/

#ifdef _MSC_VER   // Windows
#define _USE_MATH_DEFINES
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>

#include <Amp1394/AmpIORevision.h>
#if Amp1394_HAS_RAW1394
#include "FirewirePort.h"
#endif
#if Amp1394_HAS_PCAP
#include "EthRawPort.h"
#endif
#include "EthUdpPort.h"
#include "AmpIO.h"
#include "Amp1394Time.h"

void TestEncoderVelocity(BasePort *port, AmpIO *board, double vel, double accel)
{
    const int WLEN = 64;
    const int testAxis = 0;   // All axes should be the same when using test board
    quadlet_t waveform[WLEN];
    double dt = board->GetFPGAClockPeriod();
    unsigned int Astate = 1;
    unsigned int Bstate = 1;
    double t = 0.0;
    double lastT = 0.0;
    AmpIO_UInt32 minTicks = 0;  // changed below
    AmpIO_UInt32 maxTicks = 0;
    unsigned int i;
    for (i = 0; i < WLEN-1; t += 1000*dt) {
        double theta = vel*t + 0.5*accel*t*t;
        double A = cos(M_PI*theta/2.0);
        double B = sin(M_PI*theta/2.0);
        unsigned int newAstate = Astate;
        unsigned int newBstate = Bstate;
        if ((Astate == 0) && (A > 0.0))      // A-up
            newAstate = 1;
        else if ((Astate == 1) && (A < 0.0)) // A-down
            newAstate = 0;
        else if ((Bstate == 0) && (B > 0.0)) // B-up
            newBstate = 1;
        else if ((Bstate == 1) && (B < 0.0)) // B-down
            newBstate = 0;
        if ((newAstate != Astate) || (newBstate != Bstate)) {
            AmpIO_UInt32 ticks = static_cast<AmpIO_UInt32>((t-lastT)/dt);
            if (i == 0) minTicks = ticks;
            if (ticks < minTicks)
                minTicks = ticks;
            if (ticks > maxTicks)
                maxTicks = ticks;
            lastT = t;
            //std::cout << "waveform[" << i << "]: ticks = " << std::hex << ticks << std::dec
            //          << ", A " << Astate << " B " << Bstate << std::endl;
            waveform[i++] = 0x80000000 | (ticks<<8) | (Bstate << 1) | Astate;
            Astate = newAstate;
            Bstate = newBstate;
        }
    }
    waveform[WLEN-1] = 0;   // Turn off waveform generation
    std::cout << "Created table, total time = " << t << ", tick range: "
              << minTicks << "-" << maxTicks << std::endl;
    if (!board->WriteWaveformTable(waveform, 0, WLEN)) {
        std::cout << "WriteWaveformTable failed" << std::endl;
        return;
    }
    board->WriteDigitalOutput(0x03,0x03);
    // Initialize encoder position
    for (i = 0; i < 4; i++)
        board->WriteEncoderPreload(i, 0);
    port->ReadAllBoards();
    std::cout << "Starting position = " << board->GetEncoderPosition(testAxis)
              << ", velocity = " << board->GetEncoderVelocityCountsPerSecond(testAxis)
              << ", acceleration = " << board->GetEncoderAcceleration(testAxis) << std::endl;
    // Start waveform on DOUT1 and DOUT2 (to produce EncA and EncB using test board)
    board->WriteWaveformControl(0x03, 0x03);
    double velSum = 0.0;
    double accelSum = 0.0;
    unsigned int mNum = 0;
    bool waveform_active = true;
    AmpIO_UInt32 last_index = 0;
    AmpIO_UInt32 read_index;
    while (waveform_active) {
        port->ReadAllBoards();
        if (board->ReadWaveformStatus(waveform_active, read_index)) {
            if (waveform_active) {
                double mpos = board->GetEncoderPosition(testAxis);
                double mvel = board->GetEncoderVelocityCountsPerSecond(testAxis);
                double maccel = board->GetEncoderAcceleration(testAxis);
                if (read_index > 5) {
                    // First few not accurate?
                    velSum += mvel;
                    accelSum += maccel;
                    mNum++;
                }
                if (read_index != last_index) {
                    std::cout << "Waveform index: " << read_index << ", pos = " << mpos
                              << ", vel = " << mvel
                              << ", accel = " << maccel << std::endl;
                    last_index = read_index;
                }
            }
        }
        Amp1394_Sleep(0.0005);
    }
    std::cout << "Average velocity = " << velSum/mNum
              << ", acceleration = " << accelSum/mNum
              << " (" << mNum << " samples)" << std::endl;
}

int main(int argc, char** argv)
{
    int i;
#if Amp1394_HAS_RAW1394
    BasePort::PortType desiredPort = BasePort::PORT_FIREWIRE;
#else
    BasePort::PortType desiredPort = BasePort::PORT_ETH_UDP;
#endif
    int port = 0;
    int board = 0;
    std::string IPaddr(ETH_UDP_DEFAULT_IP);

    if (argc > 1) {
        int args_found = 0;
        for (i = 1; i < argc; i++) {
            if (argv[i][0] == '-') {
                if (argv[i][1] == 'p') {
                    if (!BasePort::ParseOptions(argv[i]+2, desiredPort, port, IPaddr)) {
                        std::cerr << "Failed to parse option: " << argv[i] << std::endl;
                        return 0;
                    }
                    std::cerr << "Selected port: " << BasePort::PortTypeString(desiredPort) << std::endl;
                }
                else {
                    std::cerr << "Usage: enctest [<board-num>] [-pP]" << std::endl
                    << "       where <board-num> = rotary switch setting (0-15, default 0)" << std::endl
                    << "             P = port number (default 0)" << std::endl
                    << "                 can also specify -pfwP, -pethP or -pudp" << std::endl;
                    return 0;
                }
            }
            else {
                if (args_found == 0) {
                    board = atoi(argv[i]);
                    std::cerr << "Selecting board " << board << std::endl;
                }
                args_found++;
            }
        }
    }

    BasePort *Port = 0;
    if (desiredPort == BasePort::PORT_FIREWIRE) {
#if Amp1394_HAS_RAW1394
        Port = new FirewirePort(port, std::cerr);
#else
        std::cerr << "FireWire not available (set Amp1394_HAS_RAW1394 in CMake)" << std::endl;
        return -1;
#endif
    }
    else if (desiredPort == BasePort::PORT_ETH_UDP) {
        Port = new EthUdpPort(port, IPaddr, std::cerr);
    }
    else if (desiredPort == BasePort::PORT_ETH_RAW) {
#if Amp1394_HAS_PCAP
        Port = new EthRawPort(port, std::cerr);
#else
        std::cerr << "Raw Ethernet not available (set Amp1394_HAS_PCAP in CMake)" << std::endl;
        return -1;
#endif
    }
    if (!Port || !Port->IsOK()) {
        std::cerr << "Failed to initialize " << BasePort::PortTypeString(desiredPort) << std::endl;
        return -1;
    }
    AmpIO Board(board);
    Port->AddBoard(&Board);

    double vel = 400.0;
    double accel = 0.0;
    char buf[80];
    double temp;
    bool done = false;
    int opt;
    while (!done) {
        std::cout << std::endl
                  << "0) Exit" << std::endl
                  << "1) Set velocity (vel = " << vel << ")" << std::endl
                  << "2) Set acceleration (accel = " << accel << ")" << std::endl
                  << "3) Run test" << std::endl
                  << "Select option: ";

        std::cin.getline(buf, sizeof(buf));
        if (sscanf(buf, "%d", &opt) != 1)
            opt = -1;

        switch (opt) {
            case 0:   // Quit
                done = true;
                std::cout << std::endl;
                break;
            case 1:
                std::cout << "  New velocity: ";
                std::cin.getline(buf, sizeof(buf));
                if (sscanf(buf, "%lf", &temp) == 1)
                    vel = temp;
                else
                    std::cout << "  Invalid velocity: " << buf << std::endl;
                break;
            case 2:
                std::cout << "  New acceleration: ";
                std::cin.getline(buf, sizeof(buf));
                if (sscanf(buf, "%lf", &temp) == 1)
                    accel = temp;
                else
                    std::cout << "  Invalid acceleration: " << buf << std::endl;
                break;
            case 3:
                std::cout << std::endl;
                TestEncoderVelocity(Port, &Board, vel, accel);
                break;
            default:
                std::cout << "  Invalid option!" << std::endl;
        }
    }

    Port->RemoveBoard(board);
    delete Port;
}
