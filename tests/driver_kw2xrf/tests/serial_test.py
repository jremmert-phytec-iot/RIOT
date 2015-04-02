#!/usr/bin/env python

# Copyright (C) 2014 Martine Lenders <mlenders@inf.fu-berlin.de>
#
# This file is subject to the terms and conditions of the GNU Lesser
# General Public License v2.1. See the file LICENSE in the top level
# directory for more details.

import os, signal, sys, time, serial
from pexpect import spawn, TIMEOUT, EOF

ser = serial.Serial('/dev/ttyACM0', 115200)

ser.write('ifconfig 4 set power -10\r\n')

#while 1:
#    ser.write('txtsnd 4 01:23 test \r\n')
#    time.sleep(.02)

try:
    time.sleep(.05)
    ser.write('ifconfig 4 set addr 12:34\r\n')
    time.sleep(.05)
    ser.flushInput()
    ser.write('ifconfig\r\n')
    i = ser.readline()
    i = ser.readline()
    if i.find('HWaddr: 12:34') != -1:
        print('set short addr: True')
    else:
        print('set short addr: False')
        print repr(i)
    ser.flushInput()
    time.sleep(.05)
    
    ser.write('ifconfig 4 set channel 15\r\n')
    time.sleep(.05)
    ser.flushInput()
    ser.write('ifconfig\r\n')
    i = ser.readline()
    i = ser.readline()
    if i.find('Channel: 15') != -1:
        print('set channel: \tTrue')
    else:
        print('set channel: \tFalse')
        print repr(i)
    ser.flushInput()
    time.sleep(.05)
    
    ser.write('ifconfig 4 set pan 06\r\n')
    time.sleep(.05)
    ser.flushInput()
    ser.write('ifconfig\r\n')
    i = ser.readline()
    i = ser.readline()
    if i.find('NID: 0x6') != -1:
        print('set PAN: \tTrue')
    else:
        print('set PAN: \tFalse')
        print repr(i)
    ser.flushInput()
    time.sleep(.05)

    ser.write('ifconfig 4 set power -20\r\n')
    time.sleep(.05)
    ser.flushInput()
    ser.write('ifconfig\r\n')
    i = ser.readline()
    i = ser.readline()
    if i.find('TX-Power: -20dBm') != -1:
        print('set neg TX-Power: \tTrue')
    else:
        print('set neg TX-Power: \tFalse')
        print repr(i)
    ser.flushInput()
    time.sleep(.05)
    
    ser.write('ifconfig 4 set power 100\r\n')
    time.sleep(.05)
    ser.flushInput()
    ser.write('ifconfig\r\n')
    i = ser.readline()
    i = ser.readline()
    if i.find('TX-Power: 8dBm') != -1:
        print('set out of range TX-Power: \tTrue')
    else:
        print('set out of range TX-Power: \tFalse')
        print repr(i)
    ser.flushInput()
    time.sleep(.05)

    ser.write('ifconfig 4 set state idle\r\n')
    time.sleep(.05)
    ser.flushInput()
    ser.write('ifconfig\r\n')
    i = ser.readline()
    i = ser.readline()
    if i.find('State: IDLE') != -1:
        print('set State: \tTrue')
    else:
        print('set State: \tFalse')
        print repr(i)
    ser.write('ifconfig 4 set state sleep\r\n')
    ser.flushInput()
    time.sleep(.05)
    
    ser.write('ifconfig 4 set addr_long ff:ff:ff:aa:ff:ff:ff:dd\r\n')
    time.sleep(.05)
    ser.flushInput()
    ser.write('ifconfig\r\n')
    i = ser.readline()
    i = ser.readline()
    i = ser.readline()
    if i.find('Long HWaddr: ff:ff:ff:aa:ff:ff:ff:dd') != -1:
        print('set Long_addr: \tTrue')
    else:
        print('set Long_addr: \tFalse')
        print repr(i)
    ser.flushInput()
    time.sleep(.05)
except:
    print 'exception occured'
