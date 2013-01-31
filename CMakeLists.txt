#
# $Id$
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project(Mechatronics)

cmake_minimum_required(VERSION 2.6)

# To find Amp1394 library headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# To link with build libraries
link_directories(${CMAKE_CURRENT_BINARY_DIR})

# Utilities to test libraw1394
add_subdirectory(util)

# Code for library Amp1394
add_subdirectory(lib)

# Test program using curse for text based display
find_package (Curses)
if (CURSES_FOUND)
  add_subdirectory(tests)
else (CURSES_FOUND)
  message ("Can't compile curses based test programs for Amp1394, make sure you have libcurses dev installed")
endif (CURSES_FOUND)

# Utility to flash/test board
add_subdirectory(programmer)
