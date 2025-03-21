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

#include "Amp1394Time.h"

#include <time.h>

#ifdef _MSC_VER   // Windows
#include <windows.h>
#else             // Linux, OS X, Solaris
#include <sys/time.h>
#include <unistd.h>
#endif

// See osaGetTime.cpp (cisstOSAbstraction) if support for other platforms needed.

double Amp1394_GetTime(void)
{
#ifdef _MSC_VER
    LARGE_INTEGER liTimerFrequency, liTimeNow;
    double timerFrequency, time;
    // According to MSDN, these functions are guaranteed to work
    // on Windows XP or later.
    if ((QueryPerformanceCounter(&liTimeNow) == 0) ||
        (QueryPerformanceFrequency(&liTimerFrequency) == 0)) {
	 // No performance counter available
	 return 0.0;
    }
    timerFrequency = (double)liTimerFrequency.QuadPart;
    // Also, the frequency is guaranteed to be non-zero on Windows XP or later.
    if (timerFrequency == 0.0) return 0.0;
    time = (double)liTimeNow.QuadPart/timerFrequency;
    return time;
#else
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return ((double) currentTime.tv_sec) + ((double)currentTime.tv_usec) * 1e-6;
#endif
}

// See osaSleep.cpp (cisstOSAbstraction) if support for other platforms needed.

void Amp1394_Sleep(double sec)
{
#ifdef _MSC_VER
    // A waitable timer seems to be better than the Windows Sleep().
    HANDLE WaitTimer;
    LARGE_INTEGER dueTime;
    sec *= -10.0 * 1000.0 * 1000.0;
    dueTime.QuadPart = static_cast<LONGLONG>(sec); //dueTime is in 100ns
    // We don't name the timer (third parameter) because CreateWaitableTimer will fail if the name
    // matches an existing name (e.g., if two threads call osaSleep).
    WaitTimer = CreateWaitableTimer(NULL, true, NULL);
    SetWaitableTimer(WaitTimer, &dueTime, 0, NULL, NULL, 0);
    WaitForSingleObject(WaitTimer, INFINITE);
    CloseHandle(WaitTimer);
#else
    const long nSecInSec =  1000L * 1000L * 1000L;
    struct timespec ts;
    ts.tv_sec = static_cast<long> (sec);
    ts.tv_nsec = static_cast<long> ( (sec-ts.tv_sec) * nSecInSec );
    nanosleep(&ts, NULL);
#endif
}
