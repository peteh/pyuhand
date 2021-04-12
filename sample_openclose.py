import pyuhand
import glob
import time
uhand = pyuhand.UHand("/dev/ttyUSB0")

uhand.setTargetPercentAll(100)
uhand.execute(2000)
uhand.setTargetPercentAll(0)
uhand.execute(2000)
