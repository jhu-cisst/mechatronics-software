# Mechatronics Software

This is the software interface to the FPGA boards developed for the
[Open Source Mechatronics](https://jhu-cisst.github.io/mechatronics) project.
It is designed to have no external dependencies, other than `libraw1394` for
the Firewire communications and `libpcap` if raw Ethernet is used.

The following directories are included:
* `lib` -- library to interface with the FPGA boards
* `programmer` -- application to program the FPGA boards via Firewire
* `python` -- Python test programs
* `tests` -- example programs (qladisp, qlatest, etc.)
* `util` -- low-level Firewire utility programs (do not depend on `lib`)

Documentation for the software is on the [wiki](http://github.com/jhu-cisst/mechatronics-software/wiki).
