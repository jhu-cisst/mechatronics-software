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

cmake_minimum_required(VERSION 2.6)

project(Amp1394Tests)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../lib)

add_executable(sensors sensors.cpp)
target_link_libraries(sensors Amp1394 raw1394 curses)

add_executable(qlatest qlatest.cpp)
target_link_libraries(qlatest Amp1394 raw1394 curses)
