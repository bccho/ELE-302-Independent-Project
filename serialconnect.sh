#!/bin/bash
# Opens serial port with /dev/tty.usbserial with baudrate 115200, using miniterm.py

miniterm.py --exit-char 3 /dev/tty.usbserial 115200
