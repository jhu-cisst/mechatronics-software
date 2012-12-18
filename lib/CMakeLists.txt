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
project(Amp1394)

# Allow generated c++ code to find header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# To link with build libraries
link_directories(${CMAKE_CURRENT_BINARY_DIR})

# Create galilRobot library
add_library(Amp1394 ${IS_SHARED}
            BoardIO.h
            AmpIO.h code/AmpIO.cpp
            FirewirePort.h code/FirewirePort.cpp)

target_link_libraries(Amp1394 raw1394)
