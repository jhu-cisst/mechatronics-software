#!/usr/bin/env bash
if [ $1 ]; then
openocd -f openocd.cfg -c "init;program_fpga_filename $1;exit"
else
openocd -f openocd.cfg -c "init;program_fpga;exit"
fi

