import pyuhand
import glob
import time
uhand = pyuhand.UHand("/dev/ttyUSB0")

# play all motion files with a second of sleep inbetween
for motionFile in glob.glob("motions/*.xml"):
    motion = pyuhand.Motion.fromFile(motionFile)
    uhand.executeMotion(motion)
    time.sleep(1)
