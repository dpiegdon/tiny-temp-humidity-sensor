#!/usr/bin/env python3

import serial
import re
import binascii

device = "/dev/ttyUSB0"
baudrate = 1200
parity = serial.PARITY_EVEN
stopbits = serial.STOPBITS_TWO
timeout = 5

port = serial.Serial( # ( defaults are commented, see https://pythonhosted.org/pyserial/pyserial_api.html )
                    port = device,
                    baudrate = baudrate,
                    #bytesize = serial.EIGHTBITS,
                    parity = parity,
                    stopbits = stopbits,
                    timeout = timeout,
                    #xonxoff = False
                    #rtscts = False
                    #write_timeout = None
                    #dsrdtr = False
                    #inter_byte_timeout = None
                    )

while True:
    line = port.readline()
    try:
        line = line.lstrip().rstrip().decode('utf8')
        m = re.match(r'^([Tht])([0-9A-Fa-f])([0-9A-Fa-f])([0-9A-Fa-f])([0-9A-Fa-f])', line)
        if m is not None:
            val = float(int(m.group(5) + m.group(4) + m.group(3) + m.group(2), 16))
            if "T" == m.group(1): # ADT7410
                val /= 8 # remove lover 3 bits
                # FIXME negative temps?
                temp = val / 16
                print("T {:3.4} C".format(temp))
            elif "t" == m.group(1):
                temp = (175.72 * val)/65536 - 46.85
                print("t {:3.4} C".format(temp))
            elif "h" == m.group(1):
                humidity = (125 * val)/65536 - 6
                print("h {:3.4} %".format(humidity))
            else:
                raise RuntimeError("invalid identifier '{}' for line '{}'".format(m.group(1), line))
        else:
            print("? "+line)
    except UnicodeDecodeError:
        print("utf decode failed")
        pass

