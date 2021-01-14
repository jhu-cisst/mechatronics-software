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
#include <fstream>
#include <string>
#include <vector>

#include "PortFactory.h"
#include "AmpIO.h"
#include "Amp1394Time.h"

//**************************************** Approach *******************************************************
//
// The encoder position, p(t), is given by the standard equation:  p(t) = p(0) + v(0)*t + 0.5*a*t*t,
// where p(0) is the initial position, v(0) is the initial velocity and a is the acceleration.
//
// Initially, the position was considered as an angle, theta, with the A and B waveforms given by:
//            A = cos(M_PI*theta/2.0);
//            B = sin(M_PI*theta/2.0);
// The zero crossings of these waveforms can be used to create the encoder transitions.
//
// The current implementation is simpler than this. In particular, we consider that each encoder transition
// corresponds to an increase or decrease in the encoder count. Specifically, for an encoder at p,
// the next transition will be to p+1 or p-1. Thus, if we know that a transition happened at the current
// time, tCur, then we can compute the time of the next transition. Details are given in the ConstantVel
// and ConstantAccel classes.

//*************************************** Motion Class Declaration ****************************************

// Base motion class:
//   Derived classes are MotionInit, ConstantVel, ConstantAccel and Dwell
class MotionBase {
protected:
    double t0;     // initial time
    double tf;     // final time
    double p0;     // initial position
    double pf;     // final position
    double v0;     // initial velocity
    double vf;     // final velocity
    double accel;  // acceleration
public:
    MotionBase(const MotionBase *prevMotion = 0);
    virtual ~MotionBase() {}

    // For invalid motions, the constructor sets tf = t0
    bool IsOK() const { return (tf != t0); }

    // Returns -1.0 when motion is finished
    virtual double CalculateNextTime(double tCur, int &pos, int &curDir) = 0;

    // Get position, velocity and acceleration at specified time t
    bool GetValuesAtTime(double t, double &p, double &v, double &a) const;

    void GetInitialValues(double &t, double &p, double &v, double &a) const
    { t = t0; p = p0; v = v0; a = accel; }

    void GetFinalValues(double &t, double &p, double &v, double &a) const
    { t = tf; p = pf; v = vf; a = accel; }

    virtual void Print(std::ostream &out) const = 0;
};

// MotionInit: sets starting values for a trajectory
class MotionInit : public MotionBase {
public:
    MotionInit(double vel = 0.0) : MotionBase(0) { v0 = vel; tf = 0.0; pf = 0.0; vf = vel; }
    ~MotionInit() {}

    double CalculateNextTime(double /*tCur*/, int &/*pos*/, int &/*curDir*/)
    { return -1.0; }

    void Print(std::ostream &out) const
    { out << "Init: v = " << vf << std::endl; }
};

// ConstantVel:
//    Move at current (non-zero) velocity to desired position
class ConstantVel : public MotionBase {
protected:
    int dir;        // current direction (+1 or -1)
public:
    ConstantVel(double pEnd, bool isInfinite = false, const MotionBase *prevMotion = 0);
    ~ConstantVel() {}

    double CalculateNextTime(double tCur, int &pos, int &curDir);

    void Print(std::ostream &out) const
    { out << "ConstantVel: v = " << vf << ", pf = " << pf << ", tf = " << tf << std::endl; }
};

// ConstantAccel:
//    Move at current (non-zero) acceleration to desired velocity
class ConstantAccel : public MotionBase {
protected:
    double pExtreme;  // extreme position (if direction changes)
    int initDir;      // initial direction (0, +1 or -1)
    int dir;          // current direction (+1 or -1)
public:
    ConstantAccel(double acc, double vEnd, bool isInfinite = false, const MotionBase *prevMotion = 0);
    ~ConstantAccel() {}

    double CalculateNextTime(double tCur, int &pos, int &curDir);

    void Print(std::ostream &out) const
    { out << "ConstantAccel: a = " << accel << ", pf = " << pf << ", vf = " << vf << ", tf = " << tf << std::endl; }
};

// Dwell:
//   Dwell at the current position (zero velocity) for the specified time
class Dwell : public MotionBase {
public:
    Dwell(double deltaT, const MotionBase *prevMotion = 0);
    ~Dwell() {};

    double CalculateNextTime(double /*tCur*/, int &/*pos*/, int &/*curDir*/)
    { return -1.0; }

    void Print(std::ostream &out) const
    { out << "Dwell: tf = " << tf << std::endl; }
};

// MotionTrajectory:
//   Manages the list of motions
class MotionTrajectory {
protected:
    std::vector<MotionBase *> motionList;
    size_t curIndex;    // current index into motionList
    double tCur;        // current time
    int pos;            // current encoder position (counts)

    typedef std::pair<double, int> EncTime;   // time, pos
    typedef std::vector<EncTime> EncTimeList;
    EncTimeList encList;
    size_t encIndex;

    // Returns last motion in list (0 if list is empty)
    const MotionBase *GetLastMotion(void) const;

public:
    MotionTrajectory() : curIndex(0), tCur(0.0), pos(0), encIndex(0) {}
    ~MotionTrajectory() { Init(); }

    size_t GetNumPhases(void) const { return motionList.size(); }

    // Delete all existing motion segments
    void Init(double vel = 0.0);

    // Add a motion segment
    bool AddConstantVel(double pEnd, bool isInfinite = false);
    bool AddConstantAccel(double acc, double vEnd, bool isInfinite = false);
    bool AddDwell(double deltaT);

    void Restart(void) { tCur = 0.0; curIndex = 0; pos = 0; }
    double GetCurrentTime(void) const { return tCur; }
    double CalculateNextTime(int &curDir);

    int  GetEncoderPosition(void) const { return pos; }
    bool GetValuesAtTime(double t, double &p, double &v, double &a) const;

    void GetFinalValues(double &t, double &p, double &v, double &a) const;

    void Print(std::ostream &out) const;

    // Create waveform table to be sent to FPGA
    unsigned int CreateWaveform(quadlet_t *waveform, unsigned int max_entries, double dt);

    // Return encoder time corresponding to specified position (pos).
    // Will look for first occurence of position starting at refTime.
    double GetEncoderTime(double refTime, int pos);
};

//*********************************** Motion Class Methods ****************************************

MotionBase::MotionBase(const MotionBase *prevMotion) : accel(0.0)
{
    if (prevMotion) {
        t0 = prevMotion->tf;
        p0 = prevMotion->pf;
        v0 = prevMotion->vf;
    }
    else {
        t0 = 0.0;
        p0 = 0.0;
        v0 = 0.0;
    }
    // Derived classes should override these default values.
    // Note that (tf == t0) is used to indicate an error.
    tf = t0;
    pf = p0;
    vf = v0;
}

bool MotionBase::GetValuesAtTime(double t, double &p, double &v, double &a) const
{
    if ((tf < 0) || (t <= tf)) {
        double tr = t-t0;   // time relative to start of motion
        p = p0 + v0*tr + 0.5*accel*tr*tr;
        v = v0 + accel*tr;
        a = accel;
        return true;
    }
    return false;
}

ConstantVel::ConstantVel(double pEnd, bool isInfinite, const MotionBase *prevMotion) : MotionBase(prevMotion)
{
    if (t0 < 0.0) {
        std::cout << "ConstantVel: previous motion is infinite" << std::endl;
    }
    else if (v0 == 0.0) {
        std::cout << "ConstantVel:  zero velocity not allowed (use Dwell instead)" << std::endl;
    }
    else if (isInfinite) {
        // Valid infinite motion
        tf = -1.0;
    }
    else {
        double dt = (pEnd-p0)/v0;
        if (dt <= 0) {
            std::cout << "ConstantVel: invalid motion" << std::endl;
        }
        else {
            // Valid motion
            pf = pEnd;
            tf = t0 + dt;
        }
    }
    dir = (v0 > 0) ? 1 : -1;
}

double ConstantVel::CalculateNextTime(double tCur, int &pos, int &curDir)
{
    if ((tf >= 0) && (tCur >= tf)) {
        return -1.0;
    }
    // Next position update:
    // p(t) = p(tCur) + v*(t-tCur))
    //   v > 0, p(t) = p(tCur) + 1 --> v*(t-tCur) = +1 --> t = tCur + 1/v
    //   v < 0, p(t) = p(tCur) - 1 --> v*(t-tCur) = -1 --> t = tCur - 1/v
    // Combining both cases, t = tCur + 1/fabs(v)
    curDir = dir;
    double dt = 1.0/fabs(v0);
    tCur += dt;
    if ((tf < 0) || (tCur <= tf)) {
        pos += dir;
        return tCur;
    }
    else {
        return -1.0;
    }
}

ConstantAccel::ConstantAccel(double acc, double vEnd, bool isInfinite, const MotionBase *prevMotion) : MotionBase(prevMotion)
{
    accel = acc;
    if (t0 < 0.0) {
        std::cout << "ConstantAccel: previous motion is infinite" << std::endl;
    }
    else if (accel == 0.0) {
        std::cout << "ConstantAccel:  zero acceleration not allowed" << std::endl;
    }
    else if (isInfinite) {
        // Valid infinite motion
        tf = -1.0;
    }
    else {
        double dt = (vEnd-v0)/accel;
        if (dt <= 0) {
            std::cout << "ConstantAccel: invalid motion" << std::endl;
        }
        else {
            // Valid motion
            vf = vEnd;
            // Final time
            tf = t0 + dt;
            // Final position
            pf = p0 + v0*dt + 0.5*accel*dt*dt;
        }
    }
    if (IsOK()) {
        // Initial direction of motion
        if ((v0 > 0) || ((v0 == 0) && (accel > 0)))
            dir = 1;
        else
            dir = -1;
        // Set initDir (0 means no direction change)
        initDir = (v0*vf < 0) ? dir : 0;
        // Extreme position if there is a direction change (i.e., position when V=0)
        pExtreme = p0 - (v0*v0)/(2.0*accel);
    }
}

double ConstantAccel::CalculateNextTime(double tCur, int &pos, int &curDir)
{
    if ((tf >= 0) && (tCur >= tf)) {
        return -1.0;
    }
    double dt = 0.0;
    double vCur = v0 + accel*(tCur-t0);
    if ((initDir == 1) && ((pos+dir) > pExtreme)) {
        dir = -1;
    }
    else if ((initDir == -1) && ((pos+dir) < pExtreme)) {
        dir = 1;
    }
    curDir = dir;
    if (initDir != dir) {
        // If initDir==0 (no direction change) or we have already changed direction,
        // then check whether against the position limit.
        if (((dir == 1) && ((pos+dir) > pf)) || ((dir == -1) && ((pos+dir) < pf))) {
            std::cout << "ConstantAccel: " << (pos+dir) << " past position limit (pf = " << pf << ")" << std::endl;
            std::cout << "  tCur = " << tCur << ", dir = " << dir << ", initDir = " << initDir << std::endl;
            return -1.0;
        }
    }

    // Next position update:
    // p(t) = p(tCur) + v(tCur)*(t-tCur) + 1/2*a*(t-tCur)^2, where v(tCur) = v(t0) + a*tCur
    // dir = increment in direction of motion (+1 or -1)
    //   dir = 1: p(t) = p(tCur) + 1 --> v(tCur)*(t-tCur) + 1/2*a*(t-tCur)^2 = +1
    //                --> 1/2*a*(t-tCur)^2 + v(tCur)*(t-tCur) - 1 = 0
    //                --> t = tCur + (-v + sqrt(v^2+2*a))/a    (where v = v(tCur))
    //       if a < 0, v^2+2a becomes negative when a < -v^2/2
    //   dir = -1: p(t) = p(tCur) - 1 --> v(tCur)*(t-tCur) + 1/2*a*(t-tCur)^2 = -1
    //                --> 1/2*a*(t-tCur)^2 + v(tCur)*(t-tCur) + 1 = 0
    //                --> t = tCur + (-v - sqrt(v^2-2*a))/a    (where v = v(tCur))
    // These two equations can be combined as follows:
    //                    t = tCur + (-v + dir*sqrt(v^2+dir*2*a))/a
    double temp = vCur*vCur+dir*2.0*accel;
    if (temp < 0) {
        std::cout << "ConstantAccel: tCur = " << tCur << ", pos = " << pos << ", negative square root: " << temp
                  << ", vCur = " << vCur << ", accel = " << accel << std::endl;
        dt = -vCur/accel;
    }
    else {
        dt = (dir*sqrt(temp)-vCur)/accel;
    }
    if (dt < 0) {
        std::cout << "ConstantAccel: tCur = " << tCur << ", pos = " << pos << ", negative dt: " << dt
                  << ", dir = " << dir << ", vCur = " << vCur << std::endl;
        dt = -dt;
    }

    tCur += dt;
    // Update position for next time
    if ((tf < 0) || (tCur <= tf)) {
        pos += dir;
        return tCur;
    }
    else {
        return -1.0;
    }
}

Dwell::Dwell(double deltaT, const MotionBase *prevMotion) : MotionBase(prevMotion)
{
    pf = p0;
    vf = v0;
    tf = t0;   // will be updated if not error
    if (t0 < 0.0) {
        std::cout << "Dwell: previous motion is infinite" << std::endl;
    }
    else if (v0 != 0) {
        std::cout << "Dwell: non-zero velocity not allowed (v0 = " << v0 << ")" << std::endl;
    }
    else {
        tf = t0+deltaT;
    }
}

void MotionTrajectory::Init(double vStart)
{
    for (size_t i = 0; i < motionList.size(); i++)
        delete motionList[i];
    motionList.clear();
    Restart();
    if (vStart != 0.0) {
        MotionBase *motion = new MotionInit(vStart);
        motionList.push_back(motion);
    }
}

const MotionBase *MotionTrajectory::GetLastMotion(void) const
{
    size_t num = motionList.size();
    return (num > 0) ? motionList[num-1] : 0;
}

bool MotionTrajectory::AddConstantVel(double pEnd, bool isInfinite)
{
    const MotionBase *prevMotion = GetLastMotion();
    MotionBase *motion = new ConstantVel(pEnd, isInfinite, prevMotion);
    if (motion->IsOK())
        motionList.push_back(motion);
    return motion->IsOK();
}

bool MotionTrajectory::AddConstantAccel(double acc, double vEnd, bool isInfinite)
{
    const MotionBase *prevMotion = GetLastMotion();
    MotionBase *motion = new ConstantAccel(acc, vEnd, isInfinite, prevMotion);
    if (motion->IsOK())
        motionList.push_back(motion);
    return motion->IsOK();
}

bool MotionTrajectory::AddDwell(double deltaT)
{
    const MotionBase *prevMotion = GetLastMotion();
    MotionBase *motion = new Dwell(deltaT, prevMotion);
    if (motion->IsOK())
        motionList.push_back(motion);
    return motion->IsOK();
}

double MotionTrajectory::CalculateNextTime(int &curDir)
{
    double t = motionList[curIndex]->CalculateNextTime(tCur, pos, curDir);
    while ((t < 0.0) && (curIndex < motionList.size()-1)) {
        curIndex++;
        t = motionList[curIndex]->CalculateNextTime(tCur, pos, curDir);
    }
    if (t >= 0.0)
        tCur = t;
    return t;
}

void MotionTrajectory::Print(std::ostream &out) const
{
    for (size_t i = 0; i < motionList.size(); i++)
        motionList[i]->Print(out);
}

bool MotionTrajectory::GetValuesAtTime(double t, double &p, double &v, double &a) const
{
    for (size_t i = 0; i < motionList.size(); i++) {
        // GetValuesAtTime returns true if (t <= tf) or infinite move (tf < 0)
        if (motionList[i]->GetValuesAtTime(t, p, v, a))
            return true;
    }
    return false;
}

void MotionTrajectory::GetFinalValues(double &t, double &p, double &v, double &a) const
{
    size_t num = motionList.size();
    if (num > 0) {
        motionList[num-1]->GetFinalValues(t, p, v, a);
    }
    else {
        t = 0.0;
        p = 0.0;
        v = 0.0;
        a = 0.0;
    }
}

unsigned int MotionTrajectory::CreateWaveform(quadlet_t *waveform, unsigned int max_entries, double dt)
{
    unsigned int Astate = 1;
    unsigned int Bstate = 0;    // initialized below
    bool Bnext = false;
    int curDir;
    int lastDir = 0;            // initialized below
    double t = 0.0;
    double lastT = 0.0;
    const AmpIO_UInt32 max_ticks = 0x007fffff;   // 23 bits
    AmpIO_UInt32 minTicks = 0;  // changed below
    AmpIO_UInt32 maxTicks = 0;
    unsigned int i = 0;
    encList.clear();
    for (i = 0; i < max_entries-1; i++) {
        t = CalculateNextTime(curDir);
        if (t < 0.0)
            break;
        if (curDir == 0) {
            std::cout << "CreateWaveform: i = " << i << ", invalid direction" << std::endl;
            break;
        }
        AmpIO_UInt32 ticks = static_cast<AmpIO_UInt32>((t-lastT)/dt);
        if (i == 0) {
            minTicks = ticks;
            lastDir = curDir;
            Bstate = (curDir < 0) ? 0 : 1;
        }
        while (ticks > max_ticks) {
            // If we exceed max_ticks (23 bits) add waveform table entries that maintain
            // current setting of A and B.
            std::cout << "waveform[" << i << "]: ticks = " << std::hex << ticks
                      << " (max = " << max_ticks << ")" << std::dec << std::endl;
            if (i < max_entries-2)
                waveform[i++] = 0x80000000 | (max_ticks<<8) | (Bstate << 1) | Astate;
            ticks -= max_ticks;
        }
        // Check for direction change
        if (curDir != lastDir) {
            lastDir = curDir;
            Bnext = !Bnext;
        }
        if (Bnext)
            Bstate = 1-Bstate;
        else
            Astate = 1-Astate;
        Bnext = !Bnext;
        if (ticks < minTicks)
            minTicks = ticks;
        if (ticks > maxTicks)
            maxTicks = ticks;
        lastT = t;
        encList.push_back(EncTime(t, pos));
        //std::cout << "waveform[" << i << "]: ticks = " << std::hex << ticks << std::dec
        //          << ", A " << Astate << " B " << Bstate << std::endl;
        waveform[i] = 0x80000000 | (ticks<<8) | (Bstate << 1) | Astate;
    }
    waveform[i++] = 0;   // Turn off waveform generation
    if (t < 0) t = lastT;
    std::cout << "CreateWaveform: total time = " << t << ", tick range: "
              << minTicks << "-" << maxTicks << std::endl;
    return i;
}

double MotionTrajectory::GetEncoderTime(double refTime, int pos)
{
    // Reset encIndex if necessary
    if (encList[encIndex].first > refTime)
        encIndex = 0;
    // Find refTime in list
    while ((encList[encIndex].first <= refTime) && (encIndex < encList.size()))
        encIndex++;
    // Now, find pos in list
    int rpos = encList[encIndex].second;
    while ((pos != rpos) && (encIndex < encList.size())) {
        encIndex++;
        rpos = encList[encIndex].second;
    }
    return (pos == rpos) ? encList[encIndex].first : -1.0;
}

//************************************************************************************************

std::string GetEdgeString(unsigned char edges)
{
    std::string str;
    if (edges & 0x08) str += " A-up";
    if (edges & 0x04) str += " B-up";
    if (edges & 0x02) str += " A-dn";
    if (edges & 0x01) str += " B-dn";
    return str;
}


void TestEncoderVelocity(BasePort *port, AmpIO *board, MotionTrajectory &motion)
{
    const int WLEN = 1024;
    const int testAxis = 0;   // All axes should be the same when using test board
    quadlet_t waveform[WLEN];
    double dt = board->GetFPGAClockPeriod();

    unsigned int numEntries = motion.CreateWaveform(waveform, WLEN, dt);

    std::cout << "Writing waveform table with " << numEntries << " entries" << std::endl;
    if (!board->WriteWaveformTable(waveform, 0, numEntries)) {
        std::cout << "WriteWaveformTable failed" << std::endl;
        return;
    }

    double pos, vel, accel;
    if (!motion.GetValuesAtTime(0.0, pos, vel, accel)) {
        std::cout << "TestEncoderVelocity: failed to get initial values" << std::endl;
        return;
    }
    // Initial movements to initialize firmware
    if (vel >= 0) {
         board->WriteDigitalOutput(0x03, 0x02);
         board->WriteDigitalOutput(0x03, 0x00);
         board->WriteDigitalOutput(0x03, 0x01);
         board->WriteDigitalOutput(0x03, 0x03);
    }
    else {
        board->WriteDigitalOutput(0x03, 0x00);
        board->WriteDigitalOutput(0x03, 0x02);
        board->WriteDigitalOutput(0x03, 0x03);
        board->WriteDigitalOutput(0x03, 0x01);
    }
    // Initialize encoder position
    unsigned int i;
    for (i = 0; i < 4; i++)
        board->WriteEncoderPreload(i, 0);

    Amp1394_Sleep(0.05);
    port->ReadAllBoards();
    int startPos = board->GetEncoderPosition(testAxis);
    // Sometimes, position is non-zero after preload
    for (i = 0; (i < 5) && (startPos != 0); i++) {
        std::cout << "WriteEncoderPreload: retrying (pos = " << startPos << ")" << std::endl;
        board->WriteEncoderPreload(testAxis, 0);
        Amp1394_Sleep(0.05);
        port->ReadAllBoards();
        startPos = board->GetEncoderPosition(testAxis);
    }
    if (startPos != 0) {
        std::cout << "Failed to preload encoder" << std::endl;
        return;
    }
    std::cout << "Starting position = " << startPos
              << ", velocity = " << board->GetEncoderVelocityCountsPerSecond(testAxis)
              << ", acceleration = " << board->GetEncoderAcceleration(testAxis) << std::endl;

    // Start waveform on DOUT1 and DOUT2 (to produce EncA and EncB using test board)
    board->WriteWaveformControl(0x03, 0x03);

    // Reference encoder position
    double rtime = 0.0;
    int epos = 0;

    double mpos, mvel, maccel, run;
    AmpIO::EncoderVelocityData encVelData;
    double last_mpos = -1000.0;
    unsigned int mNum = 0;
    unsigned int numSame = 0;
    unsigned int numRunOvf = 0;
    double maxRun = 0.0;
    bool waveform_active = true;
    std::ofstream outFile("waveform.csv", std::ios_base::trunc);
    outFile << "rtime, mpos, mvel, maccel, run, time, rpos, rvel, raccel" << std::endl;
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
            mNum++;
            // Find mpos in encoder list
            if (mpos != epos) {
                rtime = motion.GetEncoderTime(rtime, mpos);
                if (rtime < 0.0) {
                    std::cout << "GetEncoderTime failed, mpos = " << mpos
                              << ", epos = " << epos << std::endl;
                    break;
                }
                epos = mpos;
            }
            double rpos, rvel, raccel;
            double curTime = rtime+run;
            if (motion.GetValuesAtTime(curTime, rpos, rvel, raccel)) {
                outFile << rtime << ", " << mpos << ", " << mvel << ", " << maccel << ", " << run
                        << ", " << curTime << ", " << rpos << ", " << rvel << ", " << raccel << std::endl;
            }
            if (mpos != last_mpos) {
                if (numSame > 0) {
                    std::cout << "  + " << numSame << " entries, max run = " << maxRun;
                    if (numRunOvf > 0)
                        std::cout << " (" << numRunOvf << " overflows)";
                }
                std::cout << std::endl;
                std::cout << "t = " << rtime << ", pos = " << mpos
                          << ", vel = " << mvel
                          << ", accel = " << maccel
                          << ", run = " << run;
                if (encVelData.velOverflow)   std::cout << " VEL_OVF";
                if (encVelData.dirChange)     std::cout << " DIR_CHG";
                if (encVelData.encError)      std::cout << " ENC_ERR";
                if (encVelData.qtr1Overflow)  std::cout << " Q1_OVF";
                if (encVelData.qtr5Overflow)  std::cout << " Q5_OVF";
                if (encVelData.qtr1Edges!= encVelData.qtr5Edges) {
                    std::cout << " EDGES(" << GetEdgeString(encVelData.qtr1Edges)
                              << ", " <<  GetEdgeString(encVelData.qtr5Edges) << ")";
                }
                if (encVelData.runOverflow)  std::cout << " RUN_OVF";
                last_mpos = mpos;
                numSame = 0;
                maxRun = run;
                numRunOvf = 0;
            }
            else {
                numSame++;
                if (encVelData.runOverflow) numRunOvf++;
                if (run >= maxRun)
                    maxRun = run;
                else
                    std::cout << "Unexpected run time: " << run << " (previous = " << maxRun << ")" << std::endl;
            }
        }
        Amp1394_Sleep(0.0005);
    }
    if (numSame > 0) {
        std::cout << "  + " << numSame << " entries, max run = " << maxRun;
        if (numRunOvf > 0)
            std::cout << " (" << numRunOvf << " overflows)";
    }
    std::cout << std::endl << "Processed " << mNum << " samples" << std::endl;
}

int main(int argc, char** argv)
{
    int i;
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

    char buf[80];
    double accel;
    double temp;
    bool done = false;
    int opt;

    MotionTrajectory motion;

    while (!done) {
        double tf, pf, vf, af;
        motion.GetFinalValues(tf, pf, vf, af);
        std::cout << std::endl << "Current motion:" << std::endl;
        motion.Print(std::cout);
        std::cout << std::endl
                  << "0) Exit" << std::endl
                  << "1) Clear motion" << std::endl
                  << "2) Add velocity phase" << std::endl
                  << "3) Add acceleration phase" << std::endl
                  << "4) Add dwell" << std::endl
                  << "5) Run test" << std::endl
                  << "6) Set test motion" << std::endl
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
                motion.Init();
                break;

            case 2:
                if (vf != 0.0) {
                    std::cout << "  Enter final position: ";
                    std::cin.getline(buf, sizeof(buf));
                    if (sscanf(buf, "%lf", &temp) == 1)
                        motion.AddConstantVel(temp);
                    else
                        std::cout << "  Invalid final position: " << buf << std::endl;
                }
                else {
                    std::cout << "  Must first accelerate to non-zero velocity" << std::endl;
                }
                break;

            case 3:
                std::cout << "  Enter acceleration: ";
                std::cin.getline(buf, sizeof(buf));
                if (sscanf(buf, "%lf", &temp) == 1) {
                    accel = temp;
                    std::cout << "  Enter final velocity: ";
                    std::cin.getline(buf, sizeof(buf));
                    if (sscanf(buf, "%lf", &temp) == 1)
                        motion.AddConstantAccel(accel, temp);
                    else
                        std::cout << "  Invalid final velocity: " << buf << std::endl;
                }
                else
                    std::cout << "  Invalid acceleration: " << buf << std::endl;
                break;

            case 4:
                if (vf == 0.0) {
                    std::cout << "  Enter dwell time: ";
                    std::cin.getline(buf, sizeof(buf));
                    if (sscanf(buf, "%lf", &temp) == 1) {
                        if (temp > 0.0)
                            motion.AddDwell(temp);
                        else
                            std::cout << "  Invalid dwell time: " << temp << std::endl;
                    }
                    else
                        std::cout << "  Invalid dwell time: " << buf << std::endl;
                }
                else
                    std::cout << "  Cannot dwell with non-zero velocity (vf = " << vf << ")" << std::endl;
                break;

            case 5:
                if (motion.GetNumPhases() > 0) {
                    std::cout << std::endl;
                    TestEncoderVelocity(Port, &Board, motion);
                }
                else
                    std::cout << "  No motion defined" << std::endl;
                break;

            case 6:
                motion.Init();
                motion.AddConstantAccel(1000.0, 400.0);
                motion.AddConstantVel(120.0);
                motion.AddConstantAccel(-1000.0, 0.0);
#if 0
                motion.AddDwell(0.05);
                motion.AddConstantAccel(-1000.0, -400.0);
                motion.AddConstantAccel(1000.0, -300.0);
                motion.AddConstantAccel(10000.0, 100.0);
                motion.AddConstantAccel(1000.0, 200.0);
                motion.AddConstantAccel(-1000.0, 0.0);
#endif
                break;

            default:
                std::cout << "  Invalid option!" << std::endl;
        }
    }

    Port->RemoveBoard(board);
    delete Port;
}
