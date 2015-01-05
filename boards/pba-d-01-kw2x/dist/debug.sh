#!/bin/bash

# Based on iot-lab_M3 debug.sh script.
# Author: Jonas Remmert j.remmert@phytec.de

echo PBA-D-01 Debug script executed

openocd -f"../../boards/pba-d-01-kw2x/dist/mkw22d512.cfg" \ #"${RIOTBOARD}/${BOARD}/dist/mkw22d512.cfg" \
    -c "tcl_port 6333" \
    -c "telnet_port 4444" \
    -c "init" \
    -c "targets" \
    -c "reset halt" \
    -l /dev/null &

# save pid to terminate afterwards
OCD_PID=$?

# needed for openocd to set up
sleep 1

retval=$(arm-none-eabi-readelf -x .fcfield $2  | awk '/0x00000400/ {print $2 $3 $4 $5}')
unlocked_fcfield="fffffffffffffffffffffffffeffffff"

if [ "$retval" == "$unlocked_fcfield" ]; then
echo "Flash configuration tested... [OK]"
arm-none-eabi-gdb -tui --command=${1} ${2}
#cgdb -d arm-none-eabi-gdb --command=${1} ${2}

else
echo "Hexdump, .fcfield of $2"
retval=$(arm-none-eabi-readelf -x .fcfield $2)
echo $retval
echo "Fcfield not fitting to MKW22Dxxx, danger of bricking the device during flash...[abort]"
fi
kill ${OCD_PID}
