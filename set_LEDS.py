import time
from math import *

import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)

address = 0x08

mode = int(input("LEDMode: "))

buf = bytearray(1)

buf[0] = mode

i2c.writeto(address, buf, start = 0, end = len(buf), stop = True)
