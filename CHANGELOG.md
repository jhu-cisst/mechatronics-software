Change log
==========

2.0.0 (2023-11-xx)
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
