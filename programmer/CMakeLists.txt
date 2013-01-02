#
# $Id$
#
# (C) Copyright 2012-2013 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/../lib)

add_executable(pgm1394 pgm1394.cpp mcsFile.h mcsFile.cpp)

target_link_libraries(pgm1394 Amp1394 raw1394)
