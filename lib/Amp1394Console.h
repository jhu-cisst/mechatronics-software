/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Peter Kazanzides

  (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef __AMP1394CONSOLE_H__
#define __AMP1394CONSOLE_H__


// Portable class for console display/input

class Amp1394Console {
    bool isOk;
    bool noEcho;   // Whether to echo input characters and show cursor
    bool noBlock;  // Whether character input is blocking
    struct ConsoleInternals;
    ConsoleInternals *Internals;
public:
    enum { FLAG_DEFAULT = 0x00, FLAG_ECHO = 0x01, FLAG_BLOCKING = 0x02 };

    Amp1394Console(unsigned int flags = FLAG_DEFAULT) :
        isOk(false), noEcho(!(flags&FLAG_ECHO)), noBlock(!(flags&FLAG_BLOCKING)), Internals(0)
    { }
    ~Amp1394Console()
    { End(); }

    bool IsOK() const { return isOk; }
    bool Init();
    void End();

    // Print the specified string at the specified cursor position (row, col).
    // Uses control string (cstr) with variable arguments (like printf).
    static void Print(int row, int col, const char *cstr, ...);
    // Update console display. Does nothing for implementations where Print()
    // immediately updates display.
    static void Refresh();
    // Return input character. Note that the behavior of this function is
    // affected by the blocking mode.
    int GetChar();
    // Read (up to n) characters until newline or carriage return.
    // This function is always blocking.
    static bool GetString(char *str, int n);
};

#endif
