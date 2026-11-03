/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Jie Ying Wu, Zihan Chen

  (C) Copyright 2011-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "EncoderVelocity.h"

const uint32_t ENC_VEL_MASK_16  = 0x0000ffff;  /*!< Mask for encoder velocity (period) bits, Firmware Version <= 5 (16 bits) */
const uint32_t ENC_VEL_MASK_22  = 0x003fffff;  /*!< Mask for encoder velocity (period) bits, Firmware Version == 6 (22 bits) */
const uint32_t ENC_VEL_MASK_26  = 0x03ffffff;  /*!< Mask for encoder velocity (period) bits, Firmware Version >= 7 (26 bits) */

// The following masks read the most recent quarter-cycle period and the previous one of the same type, which are used
// for estimating acceleration in Firmware Rev 6+.
// Following are for Firmware Rev 6, which split the bits among different fields due to packet size limitations.
const uint32_t ENC_ACC_REC_MS_MASK   = 0xfff00000;   /*!< Mask (into encoder freq/acc) for upper 12 bits of most recent quarter-cycle period */
const uint32_t ENC_ACC_REC_LS_MASK   = 0x3fc00000; /*!< Mask (into encoder period) for lower 8 bits of most recent quarter-cycle period */
const uint32_t ENC_ACC_PREV_MASK     = 0x000fffff; /*!< Mask (into encoder period) for all 20 bits of previous quarter-cycle period */
// Following are for Firmware Rev 7+, which increased the block read packet size, allowing all bits to be grouped together
const uint32_t ENC_VEL_QTR_MASK   = 0x03ffffff;   /*!< Mask (into encoder QTR1/QTR5) for all 26 bits of quarter cycle period */

// Following offsets are for FPGA Firmware Version 6 (22 bits) and 7+ (26 bits)
// (Note that older versions of software assumed that Firmware Version 6 would have different bit assignments)
const uint32_t ENC_VEL_OVER_MASK   = 0x80000000;  /*!< Mask for encoder velocity (period) overflow bit */
const uint32_t ENC_DIR_MASK        = 0x40000000;  /*!< Mask for encoder velocity (period) direction bit */
const uint32_t ENC_DIR_CHANGE_MASK = 0x20000000;  /*!< Mask for encoder velocity (period) direction change (V7+) */

const double VEL_PERD_ESPM          = 1.0/80000000;   /* Clock period for ESPM velocity measurements (dVRK Si) */
const double VEL_PERD               = 1.0/49152000;   /* Clock period for velocity measurements (Rev 7+ firmware) */
const double VEL_PERD_REV6          = 1.0/3072000;    /* Slower clock for velocity measurements (Rev 6 firmware) */
const double VEL_PERD_OLD           = 1.0/768000;     /* Slower clock for velocity measurements (prior to Rev 6 firmware) */

void EncoderVelocity::Init()
{
    clkPeriod = 1.0;
    velPeriod = 0;
    velOverflow = false;
    velPeriodMax = 0;
    velDir = false;
    dirChange = false;
    encError = false;
    partialCycle = true;
    qtr1Period = 0;
    qtr1Overflow = false;
    qtr1Dir = false;
    qtr1Edges = 0;
    qtr5Period = 0;
    qtr5Overflow = false;
    qtr5Dir = false;
    qtr5Edges = 0;
    qtrPeriodMax = 0;
    runPeriod = 0;
    runOverflow = false;
}

// SetData for Firmware V7+

void EncoderVelocity::SetData(uint32_t rawPeriod, uint32_t rawQtr1, uint32_t rawQtr5, uint32_t rawRun,
                              bool isESPM)
{
    Init();
    clkPeriod = isESPM ? VEL_PERD_ESPM : VEL_PERD;
    velPeriodMax = ENC_VEL_MASK_26;
    qtrPeriodMax = ENC_VEL_QTR_MASK;  // 26 bits
    velPeriod = rawPeriod & ENC_VEL_MASK_26;
    velOverflow = rawPeriod & ENC_VEL_OVER_MASK;
    velDir = rawPeriod & ENC_DIR_MASK;
    dirChange = rawPeriod & 0x20000000;
    encError = rawPeriod & 0x10000000;
    partialCycle = rawPeriod & 0x08000000;
    qtr1Period = rawQtr1 & ENC_VEL_QTR_MASK;
    qtr1Overflow = rawQtr1 & ENC_VEL_OVER_MASK;
    qtr1Dir = rawQtr1 & ENC_DIR_MASK;
    qtr1Edges = (rawQtr1>>26)&0x0f;
    qtr5Period = rawQtr5 & ENC_VEL_QTR_MASK;
    qtr5Overflow = rawQtr5 & ENC_VEL_OVER_MASK;
    qtr5Dir = rawQtr5 & ENC_DIR_MASK;
    qtr5Edges = (rawQtr5>>26)&0x0f;
    runPeriod = rawRun & ENC_VEL_QTR_MASK;
    runOverflow = rawRun & ENC_VEL_OVER_MASK;
}

// SetData for Firmware Rev 6

void EncoderVelocity::SetDataRev6(uint32_t rawPeriod, uint32_t rawQtr)
{
    Init();
    clkPeriod = VEL_PERD_REV6;
    velPeriodMax = ENC_VEL_MASK_22;
    qtrPeriodMax = ENC_ACC_PREV_MASK;  // 20 bits
    // Firmware 6 has bits stuffed in different places:
    //   Q1: lower 8 bits in velPeriod[29:22] and upper 12 bits in accQtr1[31:20]
    //   Q5: all 20 bits in accQtr1[19:0]
    velPeriod = rawPeriod & ENC_VEL_MASK_22;
    qtr1Period = (((rawQtr & ENC_ACC_REC_MS_MASK)>>12) |
                  ((rawPeriod & ENC_ACC_REC_LS_MASK) >> 22)) & ENC_ACC_PREV_MASK;
    qtr5Period = rawQtr & ENC_ACC_PREV_MASK;
    velOverflow = rawPeriod & ENC_VEL_OVER_MASK;
    velDir = rawPeriod & ENC_DIR_MASK;
    // Qtr1 and Qtr5 overflow at 0x000fffff (no overflow bit is set by firmware)
    if (qtr1Period == qtrPeriodMax)
        qtr1Overflow = true;
    if (qtr5Period == qtrPeriodMax)
        qtr5Overflow = true;
    // Qtr1 and Qtr5 direction are not recorded, so set them same as velDir;
    // i.e., assume that there hasn't been a direction change.
    qtr1Dir = velDir;
    qtr5Dir = velDir;
}

// SetData for older firmware (Rev 1-5)

void EncoderVelocity::SetDataOld(uint32_t rawPeriod, bool useRunCounter)
{
    Init();
    // Prior to Firmware Version 6, the latched counter value is returned
    // as the lower 16 bits. Starting with Firmware Version 4, the upper 16 bits are
    // the free-running counter, which was not used.
    // Note that the counter values are signed, so we convert to unsigned and set a direction bit
    // to be consistent with later versions of firmware.
    clkPeriod = VEL_PERD_OLD;
    velPeriodMax = ENC_VEL_MASK_16;
    velPeriod = static_cast<uint16_t>(rawPeriod & ENC_VEL_MASK_16);
    // Convert from signed count to unsigned count and direction
    if (velPeriod == 0x8000) { // if overflow
        velPeriod = 0x00007fff;
        velOverflow = true;
        // Firmware also sets velPeriod to overflow when direction change occurred, so could potentially
        // set dirChange = true.
    }
    else if (velPeriod & 0x8000) {  // if negative
        velPeriod = ~velPeriod;     // ones complement (16-bits)
        velPeriod = velPeriod + 1;  // twos complement (32-bits)
        velDir = false;
    }
    else {
        velPeriod = velPeriod;
        velDir = true;
    }
    if (useRunCounter) {
        uint16_t runCtr = static_cast<uint16_t>((rawPeriod>>16) & ENC_VEL_MASK_16);
        // Convert from signed count to unsigned count and direction
        if (runCtr == 0x8000) {  // if overflow
            runOverflow = true;
            runCtr = 0x7fff;
        }
        else if (runCtr & 0x8000) {  // if negative
            runCtr = ~runCtr;        // ones complement (16-bits)
            runCtr += 1;             // twos complement (16-bits)
        }
        runPeriod = static_cast<uint32_t>(runCtr);
    }
}

// Returns encoder velocity in counts/sec -> 4/period
double EncoderVelocity::GetEncoderVelocity() const
{
    uint32_t delta = 0;
    // Avoid divide by 0 (should never happen)
    if (velPeriod == 0) delta = 1;

    double vel = 0.0;
    if (!velOverflow && !dirChange) {
        vel = 4.0/((velPeriod+delta)*clkPeriod);
        if (!velDir)
            vel = -vel;
    }

    return vel;
}

// Returns predicted encoder velocity in counts/sec, taking into account
// acceleration and running counter.
double EncoderVelocity::GetEncoderVelocityPredicted(double percent_threshold) const
{
    double encVel = GetEncoderVelocity();
    double encAcc = GetEncoderAcceleration(percent_threshold);
    // The encoder measurement delay is half the measured period, based on the assumption that measuring the
    // period over a full cycle (4 quadrature counts) estimates the velocity in the middle of that cycle.
    double encDelay = velPeriod*clkPeriod/2.0;
    double encRun = GetEncoderRunningCounterSeconds();
    double deltaVel = encAcc*(encDelay+encRun);
    double predVel = encVel+deltaVel;
    if (encVel < 0) {
        // Do not change velocity direction
        if (predVel > 0.0)
            predVel = 0.0;
        // Maximum velocity limited by 1 count (i.e., we know
        // that a count has not happened for encRun seconds)
        if (predVel*encRun < -1.0)
            predVel = -1.0/encRun;
    }
    else if (encVel > 0.0) {
        // Do not change velocity direction
        if (predVel < 0.0)
            predVel = 0.0;
        // Maximum velocity limited by 1 count (i.e., we know
        // that a count has not happened for encRun seconds)
        if (predVel*encRun > 1.0)
            predVel = 1.0/encRun;
    }
    else {
        // If not moving, do not attempt to predict
        predVel = 0.0;
    }
    return predVel;
}

// Estimate acceleration from two quarters of the same type; units are counts/second**2
// Valid for firmware version 6+.
double EncoderVelocity::GetEncoderAcceleration(double percent_threshold) const
{
    if (velOverflow)
        return 0.0;

    uint32_t velPeriodPrev = velPeriod - qtr1Period + qtr5Period;     // Previous full-cycle period
    if (qtr5Overflow)
        velPeriodPrev = velPeriodMax;

    // Should never happen
    if ((qtr1Period == 0) || (qtr5Period == 0) || (velPeriod == 0) || (velPeriodPrev == 0))
        return 0.0;

    if (qtr1Edges != qtr5Edges)
        return 0.0;

    if ((qtr1Dir != qtr5Dir) || (qtr1Dir != velDir))
        return 0.0;

    double acc = 0.0;
    if (1.0/qtr1Period <= percent_threshold) {
        double qtrDiff = static_cast<double>(qtr5Period) - static_cast<double>(qtr1Period);
        double qtrSum = static_cast<double>(qtr5Period + qtr1Period);
        double velProd = static_cast<double>(velPeriod)*static_cast<double>(velPeriodPrev)*clkPeriod*clkPeriod;
        acc = (8.0*qtrDiff)/(velProd*qtrSum);
        if (!velDir)
            acc = -acc;
    }
    return acc;
}

// Get the encoder running counter, in seconds. This is primarily used for Firmware Rev 7+, but
// also supports the running counter in Firmware Rev 4-5.
double EncoderVelocity::GetEncoderRunningCounterSeconds() const
{
    return runPeriod*clkPeriod;
}
