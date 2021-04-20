#!/bin/bash
rmmod firewire_ohci
rmmod firewire_core
modprobe firewire_ohci
modprobe firewire_core
