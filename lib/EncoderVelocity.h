/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides, Jie Ying Wu, Zihan Chen

  (C) Copyright 2021-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __ENCODER_VELOCITY_H__
#define __ENCODER_VELOCITY_H__

#include "Amp1394Types.h"

// Methods for computing encoder velocity (and acceleration) based on FPGA measured periods
// between encoder edges. This class handles the differences between firmware versions.
// The approach is documented in the following paper:
//
//     J. Y. Wu, Z. Chen, A. Deguet, and P. Kazanzides, "FPGA-based velocity estimation for
//     control of robots with low-resolution encoders", IEEE/RSJ Intl. Conf. on Intelligent
//     Robots and Systems (IROS), Oct 2018.
//
// The most accurate velocity estimate will be returned by GetEncoderVelocityPredicted, which
// estimates the acceleration and uses it to compensate for the estimated delay.

class EncoderVelocity {
protected:
    double clkPeriod;            // Clock period, in seconds
    uint32_t velPeriod;      // Encoder full-cycle period (for velocity)
    bool velOverflow;            // Velocity (period) overflow
    uint32_t velPeriodMax;   // Maximum possible velocity period
    bool velDir;                 // Velocity direction (true -> positive direction)
    bool dirChange;              // Direction change during last velocity period, V7+
    bool encError;               // Encoder error detected, V7+
    bool partialCycle;           // No full cycle yet, V7+
    uint32_t qtr1Period;     // Encoder quarter-cycle period 1 (for accel), V6+
    bool qtr1Overflow;           // Qtr1 overflow, V7+
    bool qtr1Dir;                // Qtr1 direction (true -> positive direction), V7+
    unsigned char qtr1Edges;     // Qtr1 edge mask (A-up, B-up, A-down, B-down), V7+
    uint32_t qtr5Period;     // Encoder quarter-cycle period 5 (for accel), V6+
    bool qtr5Overflow;           // Qtr5 overflow, V7+
    bool qtr5Dir;                // Qtr5 direction (true -> positive direction), V7+
    unsigned char qtr5Edges;     // Qtr5 edge mask (A-up, B-up, A-down, B-down), V7+
    uint32_t qtrPeriodMax;   // Maximum Qtr1 or Qtr5 period
    uint32_t runPeriod;      // Time since last encoder edge, Firmware V4,5,7+
    bool runOverflow;            // Running counter overflow, V7+

public:
    EncoderVelocity() { Init(); }
    ~EncoderVelocity() {}

    void Init();

    // SetData for Firmware Rev 7+
    //    Set isESPM true for dVRK-Si
    void SetData(uint32_t rawPeriod, uint32_t rawQtr1, uint32_t rawQtr5, uint32_t rawRun,
                 bool isESPM = false);
    // SetData for Firmware Rev 6
    void SetDataRev6(uint32_t rawPeriod, uint32_t rawQtr);
    // SetData for Firmware <= 5; useRunCounter should be true for Firmware Rev 4-5
    void SetDataOld(uint32_t rawPeriod, bool useRunCounter);

    /*! Returns the encoder velocity, in counts per second, based on the FPGA measurement
        of the encoder period (i.e., time between two consecutive edges). Specifically, the
        velocity is given by 4/(period*clk) counts/sec, where clk is the period of the clock used to measure
        the encoder period. The numerator is 4 because an encoder period is equal to 4 counts (quadrature).
        If the counter overflows, the velocity is set to 0. This method has improved performance starting
        with Firmware V6. For a better velocity estimate, see the GetEncoderVelocityPredicted method.
    */
    double GetEncoderVelocity() const;

    /*! Returns the predicted encoder velocity, in counts per second, based on the FPGA measurement of the
        encoder period (i.e., time between two consecutive edges), with compensation for the measurement delay.
        For Firmware Rev 6+, the predicted encoder velocity uses the estimated acceleration to predict the
        velocity at the current time. For Rev 7+, the prediction also uses the encoder running counter (time
        since last edge). There are two limits enforced:
        1) The predicted velocity will not change sign (i.e., be in the opposite direction from the measured velocity).
        2) The predicted velocity will not be larger than the velocity that would have caused one encoder count to
           occur during the time measured by the running counter.
        For an explanation of percent_threshold, see GetEncoderAcceleration.
    */
    double GetEncoderVelocityPredicted(double percent_threshold = 1.0) const;

    /*! Returns the encoder acceleration in counts per second**2, based on the scaled difference
        between the most recent full cycle and the previous full cycle. If the encoder counter overflowed
        (i.e., velocity was very slow or zero), the acceleration is set to 0. Since velocity is averaged
        over an entire cycle, acceleration can be applied over half the cycle to reduce delays.
        The percent_threshold is between 0 and 1 and essentially controls the maximum velocity for
        which acceleration is estimated. Specifically, if T is the most recent quarter-cycle period,
        then the acceleration is set to 0 if the magnitude of 1/T is greater than percent_threshold.
        This can avoid noisy acceleration estimates in those cases. Setting percent_threshold to 1.0
        effectively disables this feature. */
    double GetEncoderAcceleration(double percent_threshold = 1.0) const;

    /*! Get the encoder running counter, in seconds. This is primarily used for Firmware Rev 7+, but
        also supports the running counter in Firmware Rev 4-5.
    */
    double GetEncoderRunningCounterSeconds() const;

    /* Returns true if an encoder error was detected (V7+) */
    bool IsEncoderError() { return encError; }

    //*********** Following methods used by qladisp and enctest ************/

    // Returns the raw encoder velocity period
    uint32_t GetEncoderVelocityPeriod() const
    { return velPeriod; }

    // Returns the raw encoder quarter1 period
    uint32_t GetEncoderQuarter1Period() const
    { return qtr1Period; }

    // Returns the raw encoder quarter5 period
    uint32_t GetEncoderQuarter5Period() const
    { return qtr5Period; }

    // Indicates whether running counter has overflowed
    bool IsRunningCounterOverflow() const {return runOverflow; }

    enum EdgeMask { A_UP = 0x08, B_UP = 0x04, A_DN = 0x02, B_DN = 0x01 };

    // Returns the quarter 1 edge mask
    // Normally, only one edge should be detected (see EdgeMask)
    unsigned char GetEncoderQuarter1Edges() const { return qtr1Edges; }

    // Returns the quarter 5 edge mask
    // Normally, only one edge should be detected (see EdgeMask)
    unsigned char GetEncoderQuarter5Edges() const { return qtr5Edges; }

    // Returns whether there was a direction change
    bool GetEncoderDirChange() const { return dirChange; }
};

#endif // __ENCODER_VELOCITY_H__
