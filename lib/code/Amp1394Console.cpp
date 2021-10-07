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

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <cstdarg>
#ifdef _MSC_VER
#define WIN32_LEAN_AND_MEAN
#include <conio.h>
#include <Windows.h>
#else
#include <unistd.h>
#ifdef Amp1394Console_HAS_CURSES
#include <curses.h>
#else
#include <termios.h>
#endif
#endif

#include "Amp1394Console.h"

#ifdef Amp1394Console_HAS_CURSES

// Implementation using Curses library

bool Amp1394Console::Init()
{
    initscr();             // initialize curses
    cbreak();              // disable buffered input
    keypad(stdscr, TRUE);  // enable keypad values
    if (noEcho)
        noecho();              // do not echo input characters
    if (noBlock)
        nodelay(stdscr, TRUE); // getch non-blocking
    isOk = true;
    return true;
}

void Amp1394Console::End()
{
    endwin();
}

void Amp1394Console::Print(int row, int col, const char *cstr, ...)
{
    va_list args;
    va_start(args, cstr);
    wmove(stdscr, row, col);
    vwprintw(stdscr, cstr, args);
    va_end(args);
}

void Amp1394Console::Refresh()
{
    wrefresh(stdscr);
}

int Amp1394Console::GetChar()
{
    return getch();
}

bool Amp1394Console::GetString(char *str, int n)
{
    return (getnstr(str, n) != ERR);
}

#else

// Implementation using VT-100 escape codes
#ifndef _MSC_VER
struct Amp1394Console::ConsoleInternals {
    struct termios savedAttr;
};
#endif

bool Amp1394Console::Init()
{
    isOk = false;
#ifdef _MSC_VER
    HANDLE stdscr = GetStdHandle(STD_OUTPUT_HANDLE);
    if (stdscr == INVALID_HANDLE_VALUE) {
        std::cerr << "Could not get Std handle" << std::endl;
        return false;
    }
    DWORD mode;
    if (!GetConsoleMode(stdscr, &mode)) {
        std::cerr << "Could not get console mode" << std::endl;
        return false;
    }
    mode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    if (!SetConsoleMode(stdscr, mode)) {
        std::cerr << "Failed to set VT100 mode" << std::endl;
        return false;
    }
#else
    Internals = new Amp1394Console::ConsoleInternals;
    struct termios termAttr;
    tcgetattr(STDIN_FILENO, &termAttr);
    Internals->savedAttr = termAttr;
    if (noBlock) {
        termAttr.c_lflag &= ~ICANON;
        termAttr.c_cc[VMIN] = 0;
        termAttr.c_cc[VTIME] = 0;
    }
    else
        termAttr.c_lflag |= ICANON;
    if (noEcho)
        termAttr.c_lflag &= ~ECHO;
    else
        termAttr.c_lflag |= ECHO;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &termAttr) != 0) {
        std::cerr << "Failed to set terminal attributes" << std::endl;
        return false;
    }
#endif
    if (noEcho)
        printf("\x1b[?25l");  // Hide cursor
    printf("\x1b[2J");        // Erase screen
    isOk = true;
    return true;
}

void Amp1394Console::End()
{
    if (isOk) {
        printf("\x1b[0;0H");  // Move cursor to (0,0)
        printf("\x1b[!p");    // Soft reset (restore defaults)
        printf("\x1b[2J");    // Erase screen
    }
#ifndef _MSC_VER
    if (Internals) {
        // Restore settings
        tcsetattr(STDIN_FILENO, TCSANOW, &(Internals->savedAttr));
        delete Internals;
        Internals = 0;
    }
#endif
    isOk = false;
}

void Amp1394Console::Print(int row, int col, const char *cstr, ...)
{
    va_list args;
    va_start(args, cstr);
    printf("\x1b[%d;%dH", row, col);
    vprintf(cstr, args);
    va_end(args);
}

void Amp1394Console::Refresh()
{
}

int Amp1394Console::GetChar()
{
#ifdef _MSC_VER
    int c = -1;   // ERR in curses
    // If blocking, or if key pressed, call _getch
    if (!noBlock || _kbhit())
        c = noEcho ? _getch() : _getche();
#else
    int c = getchar();
#endif
    return c;
}

bool Amp1394Console::GetString(char *str, int n)
{
    char *ret = fgets(str, n, stdin);
    if (ret) {
        size_t len = strlen(str);
        if (len > 0)
            str[len-1] = 0;  // remove newline
    }
    return (ret != 0);
}

#endif
