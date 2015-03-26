#!/usr/bin/env python

# Copyright (C) 2014 Martine Lenders <mlenders@inf.fu-berlin.de>
#
# This file is subject to the terms and conditions of the GNU Lesser
# General Public License v2.1. See the file LICENSE in the top level
# directory for more details.

import os, signal, sys, time, serial
from pexpect import spawn, TIMEOUT, EOF

ser = serial.Serial('/dev/ttyACM0', 115200)

try:
    ser.flushInput()
    ser.write('ifconfig 4 set addr 12:34\r\n')
    i = ser.readline()
    i = ser.readline()
    #print repr(i)
    j = "success: set (short) address on interface 4 to 12:34\n"
    #print repr(j)
    print "get address test:", i == j
    ser.flushInput()
    time.sleep(.1)

    ser.write('ifconfig 4 set chan 13\r\n')
    i = ser.readline()
    i = ser.readline()
    #print repr(i)
    j = "success: set channel on interface 4 to 2\n"
    #print repr(j)
    print "get channel test:", i == j
    ser.flushInput()
    time.sleep(.1)

    ser.flushInput()
    time.sleep(.1)
    ser.write('ifconfig\r\n')
    i = ser.readline()
    i = ser.readline()
    #print repr(i)
    j = "Iface  4   HWaddr: 12:34  Channel: 13  NID: 0x1 \n"
    #print repr(j)
    print "ifconfig test:", i == j
    ser.flushInput()
    time.sleep(.1)

except:
    print 'exception occured'
