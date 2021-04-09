import pyuhand
import glob

uhand = pyuhand.UHand("/dev/ttyUSB0")


for motionFile in glob.glob("motions/*.xml"):
    motion = pyuhand.Motion.fromFile(motionFile)
    uhand.executeMotion(motion)


#while(True):
#    uhand.setTargetPercentAll(0)
#    uhand.write()
#    uhand.setTargetPercentAll(100)
#    uhand.write()