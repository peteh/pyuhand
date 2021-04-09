import serial
import struct
import time
from .motion import Motion, MotionFrame

class UHand(object):
    def __init__(self, comPort):
        self._axes = []
        self._axes.append(Axis(1, "Thumb", 800, 2400, reverse = True))
        self._axes.append(Axis(2, "Index", 750, 2100))
        self._axes.append(Axis(3, "Middle", 700, 2000))
        self._axes.append(Axis(4, "Ring", 800, 2050))
        self._axes.append(Axis(5, "Pinky", 800, 2050))
        self._axes.append(Axis(6, "Wrist", 500, 2500))
        self._comPort = comPort

        baudrate = 9600
        try:
            self.serial = serial.Serial(self._comPort, baudrate, timeout=3)
        except Exception as e:
            print(e)
    
    def _singleServoCtrlVal(self, no,speed,value):
        val_byte = struct.pack('<H', int(value))
        speed_byte = struct.pack('<H',speed)
        val_bytearray = bytearray([85,85,8,3,1,speed_byte[0],speed_byte[1],no,val_byte[0],val_byte[1]])
        self.serial.write(val_bytearray)

    def getAxisByAxisId(self, axisId):
        for axis in self._axes:
            if axis.getId() == axisId:
                return axis
        # TODO: illegal argument exception
        return None

    def setTargetValue(self, axisId, value):
        self.getAxisByAxisId(axisId).setTargetValue(value)

    def setTargetPercent(self, axisId, percent):
        self.getAxisByAxisId(axisId).setTargetPercent(percent)

    def setTargetPercentAll(self ,percent):
        for axis in self._axes:
            axis.setTargetPercent(percent)

    def write(self, timeDeltaMs = 1000.):
        for axis in self._axes:
            self._singleServoCtrlVal(axis.getId(), int(timeDeltaMs), axis._value)
        time.sleep(timeDeltaMs/1000.)
    
    def executeMotion(self, motion):
        for frame in motion.getFrames():
            for axisId, value in frame._axisValues.items():
                self.setTargetValue(axisId, value)
            self.write(frame.getTimeMs())

    


class Axis(object):
    def __init__(self, axisId, name, lowLimit, highLimit, reverse = False):
        self._axisId = axisId
        self._name = name
        self._lowLimit = lowLimit
        self._highLimit = highLimit
        self._value = lowLimit
        self._reverse = reverse

    def setTargetValue(self, value):
        # TODO: sanity check
        self._value = value
    
    def setTargetPercent(self, percent):
        # todo check range
        if(self._reverse):
            self._value = (int) (self._highLimit-(percent*((self._highLimit - self._lowLimit)/100)))
        else:
            self._value = (int) (self._lowLimit+(percent*((self._highLimit - self._lowLimit)/100)))

    def getId(self):
        return int(self._axisId)





