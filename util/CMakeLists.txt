#
# $Id$
#
# (C) Copyright 2012-2015 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# Backward compatibility
cmake_minimum_required(VERSION 2.6)

project(util1394)

if(NOT EXECUTABLE_OUTPUT_PATH)
   set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
endif(NOT EXECUTABLE_OUTPUT_PATH)

set(PROGRAMS block1394 info1394 time1394)

foreach(program ${PROGRAMS})
  add_executable(${program} ${program}.c)
  target_link_libraries(${program} raw1394)
endforeach(program)

# Add post-build command to copy block1394 to quad1394
add_custom_command(TARGET block1394 POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy
                      ${EXECUTABLE_OUTPUT_PATH}/block1394
                      ${EXECUTABLE_OUTPUT_PATH}/quad1394
                   COMMENT "Generating quad1394")
