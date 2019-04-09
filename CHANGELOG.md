Change log
==========

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
