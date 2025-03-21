/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// These are simple cross-platform implementations to get the current time (in seconds) and
// to sleep for a specified number of seconds. They are based on the implementations from
// osaGetTime and osaSleep in cisstOSAbstraction.

#ifndef __AMP1394TIME_H__
#define __AMP1394TIME_H__

// Return the time in seconds
double Amp1394_GetTime(void);

// Sleep for the desired number of seconds
void Amp1394_Sleep(double sec);

#endif

