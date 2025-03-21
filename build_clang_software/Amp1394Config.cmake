# - Config file for the Amp1394 package
# It defines the following variables
#  Amp1394_INCLUDE_DIR  - include directories for Amp1394
#  Amp1394_LIBRARY_DIR  - link library directories for Amp1394
#  Amp1394_LIBRARIES    - libraries to link against
#  Amp1394Console_LIBRARIES - libraries for Amp1394Console

# Version
set (Amp1394_VERSION_MAJOR "2")
set (Amp1394_VERSION_MINOR "2")
set (Amp1394_VERSION_PATCH "0")
set (Amp1394_VERSION "2.2.0")
 
# Compute paths
set (Amp1394_INCLUDE_DIR "/mnt/c/Users/kiggz/research/mechatronics-software/source_software/lib;/mnt/c/Users/kiggz/research/mechatronics-software/build_clang_software")
set (Amp1394_LIBRARY_DIR "/mnt/c/Users/kiggz/research/mechatronics-software/build_clang_software/lib")
 
# Libraries to link against
set (Amp1394_LIBRARIES "Amp1394;raw1394")
set (Amp1394Console_LIBRARIES "Amp1394Console;/usr/lib/x86_64-linux-gnu/libcurses.so;/usr/lib/x86_64-linux-gnu/libform.so")

# FireWire/Ethernet/Zynq-EMIO support
set (Amp1394_HAS_RAW1394 "ON")
set (Amp1394_HAS_PCAP    "OFF")
set (Amp1394_HAS_EMIO    "")

# Whether using curses for console
set (Amp1394Console_HAS_CURSES "ON")
