#
# (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project(Amp1394Tests)

include_directories (${CURSES_INCLUDE_DIR})

add_executable(qladisp qladisp.cpp)
target_link_libraries(qladisp Amp1394 raw1394 ${CURSES_LIBRARIES})

add_executable(qlatest qlatest.cpp)
target_link_libraries(qlatest Amp1394 raw1394 ${CURSES_LIBRARIES})

add_executable(qlacloserelays qlacloserelays.cpp)
target_link_libraries(qlacloserelays Amp1394 raw1394)

add_executable(fwPortTest fwPortTest.cpp)
target_link_libraries(fwPortTest Amp1394 raw1394)

install (TARGETS qladisp qlatest qlacloserelays fwPortTest
         COMPONENT Amp1394-utils
         RUNTIME DESTINATION bin)