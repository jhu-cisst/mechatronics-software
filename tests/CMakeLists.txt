#
# (C) Copyright 2011-2021 Johns Hopkins University (JHU), All Rights Reserved.
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

include_directories(${Amp1394_INCLUDE_DIR})
link_directories(${Amp1394_LIBRARY_DIR} ${Amp1394_EXTRA_LIBRARY_DIR})

set (CONSOLE_LIBRARIES "")

if (NOT WIN32)
  option (Amp1394_HAS_CURSES "Build Amp1394 test programs with curses (OFF --> VT100 mode)" ON)

  if (Amp1394_HAS_CURSES)
    find_package (Curses)
    if (CURSES_FOUND)
      add_definition(-DAmp1394_HAS_CURSES)
      include_directories (${CURSES_INCLUDE_DIR})
      set (CONSOLE_LIBRARIES ${CONSOLE_LIBRARIES} ${CURSES_LIBRARIES})
    else (CURSES_FOUND)
      message ("Cannot find Curses library, changing to VT100 mode")
    endif (CURSES_FOUND)
  endif (Amp1394_HAS_CURSES)
endif (NOT WIN32)

add_executable(qladisp qladisp.cpp Amp1394Console.h Amp1394Console.cpp)
target_link_libraries(qladisp ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES} ${CONSOLE_LIBRARIES})

add_executable(qlatest qlatest.cpp Amp1394Console.h Amp1394Console.cpp)
target_link_libraries(qlatest ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES} ${CONSOLE_LIBRARIES})

install (TARGETS qladisp qlatest
         COMPONENT Amp1394-utils
         RUNTIME DESTINATION bin)

if (Amp1394_HAS_RAW1394)
  add_executable(fwPortTest fwPortTest.cpp)
  target_link_libraries(fwPortTest ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})

  install (TARGETS fwPortTest
           COMPONENT Amp1394-utils
           RUNTIME DESTINATION bin)
endif (Amp1394_HAS_RAW1394)

add_executable(qlacloserelays qlacloserelays.cpp)
target_link_libraries(qlacloserelays ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})

add_executable(qlacommand qlacommand.cpp)
target_link_libraries(qlacommand ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})

add_executable(eth1394Test mainEth1394.cpp)
target_link_libraries(eth1394Test ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})

add_executable(instrument instrument.cpp)
target_link_libraries(instrument ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})

add_executable (block1394eth block1394eth.cpp)
target_link_libraries (block1394eth ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})

add_custom_command (TARGET block1394eth POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy
                       ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}/block1394eth${CMAKE_EXECUTABLE_SUFFIX}
                       ${EXECUTABLE_OUTPUT_PATH}/${CMAKE_CFG_INTDIR}/quad1394eth${CMAKE_EXECUTABLE_SUFFIX}
                    COMMENT "Generating quad1394eth")

add_executable(enctest enctest.cpp)
target_link_libraries (enctest ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})

install (PROGRAMS ${EXECUTABLE_OUTPUT_PATH}/quad1394eth
         COMPONENT Amp1394-utils
         DESTINATION bin)

install (TARGETS qlacloserelays qlacommand eth1394Test instrument block1394eth enctest
         COMPONENT Amp1394-utils
         RUNTIME DESTINATION bin)
