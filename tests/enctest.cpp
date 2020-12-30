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

#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"

// Calculates time of 0 crossing of sine or cosine
double CalculateTime(double vel, double accel, double t0, bool &dirChange)
{
    double t;
    if (accel == 0.0) {
        t = t0 + (1.0/fabs(vel));
        dirChange = false;
    }
    else {
        double curVel = vel + accel*t0;
        if (curVel == 0.0) {
            t = t0 + sqrt(2/fabs(accel));
            dirChange = true;
        }
        else {
            double posTemp = curVel*curVel+2.0*accel;
            double negTemp = curVel*curVel-2.0*accel;
            if (((curVel < 0.0) && (negTemp >= 0)) || (posTemp < 0))
                t = t0 + (-sqrt(negTemp)-curVel)/accel;
            else if (posTemp >= 0)
                t = t0 + (sqrt(posTemp)-curVel)/accel;
            else {
                // Should never happen; if it does, return solution for 0 velocity
                std::cout << "Error finding solution at t0 = " << t0 << std::endl;
                t = t0 + sqrt(2/fabs(accel));
            }
            double newVel = vel + accel*t;
            dirChange = (newVel*curVel < 0.0);
        }
    }
    return t;
}

void TestEncoderVelocity(BasePort *port, AmpIO *board, double vel, double accel)
{
    const int WLEN = 64;
    const int testAxis = 0;   // All axes should be the same when using test board
    quadlet_t waveform[WLEN];
    double dt = board->GetFPGAClockPeriod();
    unsigned int Astate = 1;
    unsigned int Bstate = (vel < 0) ? 0 : 1;
    bool Bnext = false;
    bool dirChange;
    double t = 0.0;
    double lastT = 0.0;
    AmpIO_UInt32 minTicks = 0;  // changed below
    AmpIO_UInt32 maxTicks = 0;
    unsigned int i;
    for (i = 0; i < WLEN-1; i++) {
        //double theta = vel*t + 0.5*accel*t*t;
        //double A = cos(M_PI*theta/2.0);
        //double B = sin(M_PI*theta/2.0);
        t = CalculateTime(vel, accel, t, dirChange);
        if (dirChange) {
            double theta = vel*t + 0.5*accel*t*t;
            std::cout << "Direction change at t = " << t << ", pos = " << theta << std::endl;
            Bnext = !Bnext;
        }
        if (Bnext)
            Bstate = 1-Bstate;
        else
            Astate = 1-Astate;
        Bnext = !Bnext;
        AmpIO_UInt32 ticks = static_cast<AmpIO_UInt32>((t-lastT)/dt);
        if (i == 0) minTicks = ticks;
        if (ticks < minTicks)
            minTicks = ticks;
        if (ticks > maxTicks)
            maxTicks = ticks;
        lastT = t;
        //std::cout << "waveform[" << i << "]: ticks = " << std::hex << ticks << std::dec
        //          << ", A " << Astate << " B " << Bstate << std::endl;
        waveform[i] = 0x80000000 | (ticks<<8) | (Bstate << 1) | Astate;
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
    double mpos, mvel, maccel, run;
    AmpIO::EncoderVelocityData encVelData;
    double last_mpos = -1000.0;
    double velSum = 0.0;
    double accelSum = 0.0;
    unsigned int mNum = 0;
    bool waveform_active = true;
    while (waveform_active || (mNum == 0)) {
        port->ReadAllBoards();
        waveform_active = board->GetDigitalInput()&0x20000000;
        if (waveform_active) {
            mpos = board->GetEncoderPosition(testAxis);
            mvel = board->GetEncoderVelocityCountsPerSecond(testAxis);
            maccel = board->GetEncoderAcceleration(testAxis);
            run = board->GetEncoderRunningCounterSeconds(testAxis);
            if (!board->GetEncoderVelocityData(testAxis, encVelData))
                std::cout << "GetEncoderVelocityData failed" << std::endl;
            if ((mpos > 5) || (mpos < -5)) {
                // First few not accurate?
                velSum += mvel;
                accelSum += maccel;
                mNum++;
            }
            bool doEndl = false;
            if (mpos != last_mpos) {
                std::cout << "pos = " << mpos
                          << ", vel = " << mvel
                          << ", accel = " << maccel
                          << ", run = " << run;
                last_mpos = mpos;
                doEndl = true;
            }
            if (encVelData.velOverflow) {doEndl = true; std::cout << " VEL_OVF"; }
            if (encVelData.dirChange)  {doEndl = true; std::cout << " DIR_CHG"; }
            if (encVelData.encError)  {doEndl = true; std::cout << " ENC_ERR"; }
            if (encVelData.qtr1Overflow)  {doEndl = true; std::cout << " Q1_OVF"; }
            if (encVelData.qtr5Overflow)  {doEndl = true; std::cout << " Q5_OVF"; }
            if (encVelData.qtr1Edges!= encVelData.qtr5Edges) {
                doEndl = true;
                std::cout << " EDGES(" << std::hex << static_cast<unsigned int>(encVelData.qtr1Edges)
                          << ", " << static_cast<unsigned int>(encVelData.qtr5Edges) << std::dec << ")";
            }
            if (encVelData.runOverflow)  {doEndl = true; std::cout << " RUN_OVF"; }
            if (doEndl) std::cout << std::endl;
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
    int port = 0;
    int board = 0;
    const char *portDescription = "";

    if (argc > 1) {
        int args_found = 0;
        for (i = 1; i < argc; i++) {
            if (argv[i][0] == '-') {
                if (argv[i][1] == 'p') {
                    portDescription = argv[i]+2;
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

    BasePort *Port = PortFactory(portDescription);
    if (!Port) {
        std::cerr << "Failed to create port using: " << portDescription << std::endl;
        return -1;
    }
    if (!Port->IsOK()) {
        std::cerr << "Failed to initialize " << Port->GetPortTypeString() << std::endl;
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
