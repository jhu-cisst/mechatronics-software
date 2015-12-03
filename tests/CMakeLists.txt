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

if (WIN32)
  message ("Cannot compile curses based test programs for Amp1394 on Windows")
else (WIN32)
  find_package (Curses)
  if (CURSES_FOUND)

    include_directories (${CURSES_INCLUDE_DIR})

    add_executable(qladisp qladisp.cpp)
    target_link_libraries(qladisp Amp1394 ${CURSES_LIBRARIES})

    add_executable(qlatest qlatest.cpp)
    target_link_libraries(qlatest Amp1394 ${CURSES_LIBRARIES})

    # Check if following needed, and for pcap
    if (Amp1394_HAS_RAW1394)
      target_link_libraries(qladisp raw1394)
      target_link_libraries(qlatest raw1394)
    endif (Amp1394_HAS_RAW1394)

    install (TARGETS qladisp qlatest
             COMPONENT Amp1394-utils
             RUNTIME DESTINATION bin)

  else (CURSES_FOUND)
    message ("Cannot compile curses based test programs for Amp1394, make sure you have libcurses dev installed")
  endif (CURSES_FOUND)
endif (WIN32)

if (Amp1394_HAS_RAW1394)
  add_executable(qlacloserelays qlacloserelays.cpp)
  target_link_libraries(qlacloserelays Amp1394)

  add_executable(fwPortTest fwPortTest.cpp)
  target_link_libraries(fwPortTest Amp1394)

  target_link_libraries(qlacloserelays raw1394)
  target_link_libraries(fwPortTest raw1394)

  install (TARGETS qlacloserelays fwPortTest
           COMPONENT Amp1394-utils
           RUNTIME DESTINATION bin)
endif (Amp1394_HAS_RAW1394)

if (Amp1394_HAS_PCAP AND Amp1394_HAS_RAW1394)
  add_executable(eth1394Test mainEth1394.cpp)
  target_link_libraries(eth1394Test Amp1394 ${PCAP_LIBRARIES} raw1394)
  install (TARGETS eth1394Test
           COMPONENT Amp1394-utils
           RUNTIME DESTINATION bin)
endif (Amp1394_HAS_PCAP AND Amp1394_HAS_RAW1394)
