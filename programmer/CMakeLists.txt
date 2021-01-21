#
# (C) Copyright 2012-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (Amp1394Programmer)

include_directories(${Amp1394_INCLUDE_DIR})
link_directories(${Amp1394_LIBRARY_DIR} ${Amp1394_EXTRA_LIBRARY_DIR})

# programmer, pgm1394
add_executable(pgm1394 pgm1394.cpp mcsFile.h mcsFile.cpp)
target_link_libraries(pgm1394 ${Amp1394_LIBRARIES} ${Amp1394_EXTRA_LIBRARIES})
install (TARGETS pgm1394
         COMPONENT Amp1394-utils
         RUNTIME DESTINATION bin)

# script to program multiple boards
add_custom_command(TARGET pgm1394 POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E
                   copy "${Amp1394Programmer_SOURCE_DIR}/pgm1394multi.bash" ${EXECUTABLE_OUTPUT_PATH})
install (FILES "${Amp1394Programmer_SOURCE_DIR}/pgm1394multi.bash"
         COMPONENT Amp1394-utils
         DESTINATION bin)
