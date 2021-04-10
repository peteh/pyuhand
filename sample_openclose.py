import pyuhand
import glob
import time
uhand = pyuhand.UHand("/dev/ttyUSB0")

uhand.setTargetPercentAll(100)
uhand.write(2000)
uhand.setTargetPercentAll(0)
uhand.write(2000)