#!/bin/sh
sudo modprobe peak_usb
sudo ip link set can0 up type can bitrate 250000
sudo ip link set up can0