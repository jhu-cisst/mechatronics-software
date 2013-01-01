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

# Allow generated c++ code to find lib header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# To link with build libraries
link_directories(${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(util)
add_subdirectory(lib)
add_subdirectory(tests)
add_subdirectory(programmer)

