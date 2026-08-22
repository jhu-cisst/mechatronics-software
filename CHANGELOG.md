Change log
==========

2.4.0 (2026-08-22)
==================
* API changes:
  * None
* New features:
  * Support Firmware Rev 10, which provides the dVRK-Si SUJ potentiometer feedback (if available) in the real-time block read packet, and other features described below.
  * `BasePort::GetSiHasSUJ` returns true if dVRK-Si controller includes SUJ interface; if so, `ScanNodes` adds "+dSIB" to displayed hardware string.
  * `AmpIO::GetNumExtraIn` returns the number of extra inputs in real-time block read packet (5 if dVRK-Si controller has SUJ interface; 0 otherwise); `GetExtraInput` provides low-level access to the extra inputs from the real-time block read packet and `ReadExtraInput` provides this via a separate quadlet read (see next items for SUJ-related methods that parse the extra inputs to provide more meaningful information).
  * `AmpIO::Get` methods `GetSiSUJ_Status`, `GetSiSUJ_Pots`, and `GetSiSUJ_Z_Id` return the dVRK-Si SUJ status (e.g., whether various required boards are present), the SUJ potentiometer values, and the expected board id for the Z-axis SUJ pot (for identifying cabling errors); these values are extracted from the real-time block read packet.
  * `AmpIO::Read` methods `ReadSiSUJ_Status`, `ReadSiSUJ_Pots`, and `ReadSiSUJ_Z_Id` function the same as above, but obtain data via separate quadlet reads from the FPGA.
  * `AmpIO::IsCurrentFbFiltered` returns true if the measured motor current is filtered on the FPGA (does not consider any possible analog filtering); call `SetCurrentFbFilter` to enable/disable the FPGA filter, on hardware that supports this feature (dVRK-Si).
  * `AmpIO::HasMotorCommandFb` returns true if motor command feedback is available; if so, calling `RequestMotorCommandFb` adds it to the real-time block read packet, and it can be retrieved by calling `GetMotorCommandFb`.
  * Updates to `qladisp`:
    * Changed `DAC` to `Cmd` when displaying value sent to FPGA, since the destination is not always a DAC.
    * Display dVRK-Si SUJ potentiometer feedback if available; note that each joint has two pots, which are shown on separate lines as `SUJ-P1` and `SUJ-P2`.
    * Added `-c` command line option to request motor command feedback; if available, displayed as `CFb`.
    * Added `f` key to enable/disable measured motor current filtering (dVRK-Si only); asterisk (*) added to displayed value in non-default case (i.e., when filter is disabled).
  * Added simulation of embedded system (firmware), through a SIMULATION port (contribution from University of Verona); simulation is enabled by CMake option `Amp1394_HAS_SIM`.
* Bug fixes:
  * None

2.3.0 (2026-01-07)
==================
* API changes:
  * Require CMake 3.16+
  * CMake changes for Python:
    * Use `find_package(Python)` to find all required components, including `numpy`
    * Introduced CMake option `AMP1394_PYTHON_VERSION_REQUIRED` to allow specification of required version (use default version if not specified)
    * Use `swig_add_library` (introduced in CMake 3.8) instead of `swig_add_module`; no need to manually prepend underscore to library name on Windows
* New features:
  * Created test program `fpgatest` based on major rewrite of `eth1394Test` (`mainEth1394.cpp`); program displays a menu of available tests
  * Changes to `dvrktest`:
    * Added function header documentation
    * Consolidated logging code
    * Increased timeout when testing motor power control
  * Added methods to get/set verbose flag and timeout for Zynq EMIO interface (FPGA V3)
  * Use block read for QLA S/N instead of multiple quadlet reads
  * Implemented `_kbhit` for non-Windows systems (used by test programs)
* Bug fixes:
  * None

2.2.0 (2024-08-30)
==================
* API changes:
  * The `udp` port type (`eth` for raw Ethernet) now means to use Ethernet-only if first connected board is FPGA V3 with Firmware Rev 9+; otherwise, use Ethernet/Firewire bridge; to force use of Ethernet/Firewire bridge, specify `udpfw` port type (`ethfw` for raw Ethernet)
  * Changed Ethernet status/control register format for Firmware Rev 9
* New features:
  * Support Firmware Rev 9, which implements the same protocols as Firmware Rev 8, but adds the Ethernet-only configuration
  * Support Ethernet-only network configuration
    * The "broadcast write" feature (PC to FPGAs) is implemented using UDP multicast to 224.0.0.100
    * The "broadcast read" feature (FPGAs to PC) is implemented similar to Firewire, where PC sends "query" command and FPGAs exchange data (using raw multicast) amongst themselves. The primary difference is that the participating FPGA closest to the PC automatically sends a UDP packet when FPGA data exchange is completed (i.e., the PC does not need to read data, as in the Firewire protocol)
  * Set Firewire gap count via Ethernet when PC not directly connected to Firewire (i.e., Ethernet/Firewire bridge configuration); this improves Ethernet/Firewire bridge timing
  * Changed Zynq EMIO interface (FPGA V3) to use faster `mmap` interface by default; to use slower `gpiod` interface, set port number to 1 (i.e., `emio:1`)
  * Increased Zynq EMIO port maximum data size from 128 bytes to 2048 bytes
  * Added methods to read/write Firewire PHY registers
  * Added `ethswitch` application that displays status of Ethernet switch implemented in FPGA V3 (also some relevant data from FPGA V2)
* Bug fixes:
  * Call `pcap_set_immediate_mode` for newer versions of pcap (raw Ethernet) to prevent interface from hanging

2.1.0 (2023-12-29)
==================
* API changes:
  * None
* New features:
  * Added `-r` (read-only) command line argument to `qladisp`
  * Added check of FPGA register 15, which contains git description of FPGA firmware build (introduced after Firmware Rev 8 release)
  * Added check for Dallas chip family code in `dvrktest`
  * Support Zynq-EMIO interface in test programs (cross-compiled for FPGA V3 embedded ARM processor)
* Bug fixes:
  * Modified `drvktest` to compile on all supported platforms

2.0.0 (2023-11-21)
==================
* API changes:
  * New packet format for Firmware Rev 8
    * Supports different numbers of signals for different boards (QLA, DQLA, DRAC)
  * `qlacloserelays` removed, use `qlacommand` instead
  * Consistent naming of firmware
    * FPGA V1 firmware renamed from `FPGA1394-QLA` to `FPGA1394V1-QLA`
    * FPGA V2 firmware renamed from `FPGA1394Eth-QLA` to `FPGA1394V2-QLA`
    * FPGA V3 firmware is named `FPGA1394V3-XXX`, where `XXX` is `BCFG`, `QLA`, `DQLA` or `DRAC`
  * Use types from `stdint` instead of custom `AmpIO_*`
  * Moved FPGA-specific methods from `AmpIO` to new `FpgaIO` class (`AmpIO` inherits from `FpgaIO`)
* New features:
  * Support for new hardware:
    * Si controllers (dRA1+FPGAv3)
    * New Classic controllers (QLA+DQLA+FPGAv3)
  * Support for QLA Version 1.5+
    * Control either motor current (as before) or voltage (`AmpIO::SetMotorVoltage`)
    * Interface to I/O expander for additional digital I/O
  * Support for cross-compiling for FPGAv3 ARM32 processor
    * Added `ZynqEmioPort` for Zynq ARM to access FPGA registers via EMIO interface
  * Support for dongle (with Dallas DS2480B driver) to access DS2505 chip in da Vinci instruments
  * Read/write motor configuration and status registers (Firmware Rev 8+)
  * Added `qlacommand`
  * `BasePort` class gathers more information about connected boards (FPGA version, hardware type)
  * More data displayed in `qladisp`
  * Added `dvrktest` for manufacturing testing
    * Displays summary of connected controllers
    * Requires custom test board for running tests
  * Added `Amp1394Console` library from existing files, so that it can more easily be used by other projects
  * Added `pgm1394` option to download FPGA PROM to MCS file
* Bug fixes:
  * Improved velocity estimation



1.7.0 (2021-08-11)
==================
* API changes:
  * None
* New features:
  * Added `GetPowerFault`
  * Added methods to set FireWire protocol using strings (e.g: `broadcast-query-read-write`)
* Bug fixes:
  * None


1.6.0 (2021-04-08)
==================
* API changes:
  * New packet format for firmware 7
* New features:
  * UDP support for firmware 7
    * Compiles and tested on Linux, Windows and MacOS
    * Uses link local
    * Detects MTU on network interface
  * Most applications support option to set port (e.g. `-pudp`, `-pfw`, `-pfw:1`...)
  * Improved and tested velocity estimation with firmware 6 and 7, code cleanup
  * `qladisp`
    * Option to use different FireWire protocol (broadcast)
    * Display more information
    * Can be used to read from multiple boards, display data for first 2
    * Power functions can be used per axis or for all
  * Utilities
    * Added `qlacommand` (replaces `qlacloserelays`) with commands to reboot, close/open relays, reset encoder offset, ethernet chip...
    * Added `pgm1394multi` script to upload firmware on multiple boards
  * Added method to test if encoders are preloaded to save time when homing
  * Added data collection from FPGA with firmware 7
* Bug fixes:
  * Code refactorization and cleanup
  * `qladisp` resets the encoder preloads to midrange on exit
  * See Github

1.5.0 (2019-04-19)
==================
* API changes:
  * None
* New features:
  * Support FPGA/QLA serial numbers with up to 3 digits
  * Support to read Dallas chip on da Vinci tools (requires firmware 7 to be released)
* Bug fixes:
  * None

1.4.0 (2018-05-16)
==================
* API changes:
  * None
* New features:
  * Support for firmware Rev6 velocity estimation
  * `pgm1394` now displays version for AmpIO library
* Bug fixes:
  * Fixed CMakeLists project names

1.3.0 (2017-11-07)
==================
* API changes:
  * Methods that read/write digital outputs invert the signals so that the actual output matches the logical value; i.e., dout=0 causes low output (0V), dout=1 causes high output (e.g., 5V).
  * Watchdog is now automatically reset when any write command is sent (sends a no-op if needed)
* New features:
  * Added `HasEthernet` method to `AmpIO`; returns true for FPGA V2.x.
  * pgm1394 detects FPGA hardware version (V1.x or V2.x) and automatically selects correct programming file (`FPGA1394-QLA.mcs` for V1.x and `FPGA1394Eth-QLA.mcs` for V2.x).
  * Added options to read FPGA and QLA serial numbers in pgm1394.
  * Ongoing development for ethernet/PCAP and FPGA based velocity estimation (will require firmware > 5 not yet released)
  * Can be compiled on Windows for future ethernet support
  * Updated Python wrappers
* Bug fixes:
  * qladisp and qlatest: fixed streaming of long error messages

1.2.1 (2016-08-31)
==================
* API changes:
  * None
* New features:
  * Compilation: use -fPIC when available
* Bug fixes:
 * None

1.2.0 (2015-10-18)
==================
* API changes:
* New features:
  * Added utility qlacloserelays to close all safety relays on controllers connected
  * When qladisp is started w/o board numbers, display results of port query
  * Added GetFPGASerialNumber and GetQLASerialNumber methods
  * Added code to read/write digital outputs
  * Added code to support PWM (requires firmware 5+)
* Bug fixes:
 * Reversed order of digital outputs (fixed in firmware 5+)

1.1.0 (2015-04-28)
==================

* API changes:
  * Encoder API now uses signed integers, assumes all values are related to mid range (for setters and getters)
  * Default protocol is now broadcast write if all boards have firmware version 4.0 or higher
* Deprecated features:
  * `SetUseBroadcastFlag` has been replaced by `SetProtocol`
* New features:
  * Added revision number, new header file `AmpIORevision.h` needs to be included
  * Added method to get encoder channel A/B
  * Added `ProtocolType` to select between firewire no broadcast, write only broadcast or read/write broadcast
  * Added method to get encoder overflow bit
* Bug fixes:
  * None

1.0.0 (2014-01-24)
==================

* No change log file, initial release.
