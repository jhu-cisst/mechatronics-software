/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef __MOTORVOLTAGE_H__
#define __MOTORVOLTAGE_H__

class BasePort;
class AmpIO;

// Returns the measured motor supply voltage, for QLA V1.5+
//   index:  0 for QLA; 1 or 2 for DQLA
double MeasureMotorSupplyVoltage(BasePort *curPort, AmpIO *curBoard, unsigned int index = 0);

#endif  // __MOTORVOLTAGE_H__
