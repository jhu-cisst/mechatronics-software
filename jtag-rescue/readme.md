# OpenOCD configuration for flashing the FPGA and PROM

## Prerequisites
### JTAG adapter
OpenOCD supports most FTDI-based adapters. We recommend Digilent HS2/HS3. OpenOCD does not support the Xilinx Platform Cable DLC9G.

### Software
Install [OpenOCD](http://openocd.org/). Binary is available for Linux, Mac OS, and Windows.

For Ubuntu, install from apt repository.

```
sudo apt install openocd
```

Create a udev rule for the JTAG (if you are using the Digilent JTAG). Create a file at `/etc/udev/rules.d/52-digilent-usb.rules` with the following content.

```
ATTR{idVendor}=="1443", MODE:="666"
ACTION=="add", ATTR{idVendor}=="0403", ATTR{manufacturer}=="Digilent", MODE:="666"
```

Reload the udev rules.

```
udevadm control --reload-rules
```

Now unplug and plug in your JTAG adapter.


## Usage

`cd` into the `openocd` directory. Use the `program_fpga.sh`. The working directory must be `openocd`. Depending on your FPGA hardware (no-ethernet vs ethernet)

```
./program_fpga.sh FPGA1394QLA.bit

```

or

```
./program_fpga.sh FPGA1394EthQLA.bit
```

This temporarily programs the FPGA with v7 firmware. Now, without powering off the FPGA, reprogram the PROM using firewire.

