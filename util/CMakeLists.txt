#
# (C) Copyright 2012-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# Backward compatibility
cmake_minimum_required (VERSION 2.6)
project (Amp1394Utils)

include_directories (${Amp1394_INCLUDE_DIR})
link_directories (${Amp1394_LIBRARY_DIR} ${Amp1394_EXTRA_LIBRARY_DIR})


if (NOT EXECUTABLE_OUTPUT_PATH)
   set (EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
endif (NOT EXECUTABLE_OUTPUT_PATH)

# for FireWire 1394
if (Amp1394_HAS_RAW1394)
  set (PROGRAMS block1394 info1394 time1394)
  foreach (program ${PROGRAMS})
    add_executable (${program} ${program}.c)
    target_link_libraries (${program} raw1394)
  endforeach (program)

  # Add post-build command to copy block1394 to quad1394
  add_custom_command (TARGET block1394 POST_BUILD
                      COMMAND ${CMAKE_COMMAND} -E copy
                         ${EXECUTABLE_OUTPUT_PATH}/block1394
                         ${EXECUTABLE_OUTPUT_PATH}/quad1394
                      COMMENT "Generating quad1394")

endif (Amp1394_HAS_RAW1394)


# For PCAP
if (Amp1394_HAS_PCAP)
  add_executable (blocketh1394 blocketh1394.cpp)
  target_link_libraries (blocketh1394 raw1394 Amp1394)

  add_custom_command (TARGET blocketh1394 POST_BUILD
                      COMMAND ${CMAKE_COMMAND} -E copy
                         ${EXECUTABLE_OUTPUT_PATH}/blocketh1394
                         ${EXECUTABLE_OUTPUT_PATH}/quadeth1394
                      COMMENT "Generating quad1394eth")

endif (Amp1394_HAS_PCAP)
