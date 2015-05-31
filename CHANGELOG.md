Change log
==========

1.2.0 
==================
* API changes: 
* New features: 
  * Added GetFPGASerialNumber and GetQLASerialNumber methods

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
