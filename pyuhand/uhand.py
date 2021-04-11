import serial
import struct
import time
from .motion import Motion, MotionFrame

class UHand(object):
    def __init__(self, comPort):
        self._axes = []

        # TODO: double check all limits
        self._axes.append(Axis(1, "Thumb", 800, 2400, reverse = True))
        self._axes.append(Axis(2, "Index", 800, 2000)) # overwrite offical limit because mine has a screw there
        self._axes.append(Axis(3, "Middle", 700, 2000))
        self._axes.append(Axis(4, "Ring", 800, 2050))
        self._axes.append(Axis(5, "Pinky", 800, 2050))
        self._axes.append(Axis(6, "Wrist", 500, 2500))
        self._comPort = comPort

        baudrate = 9600

        self.serial = serial.Serial(self._comPort, baudrate, timeout=3)

    
    def _singleServoCtrlVal(self, no,speed,value):
        val_byte = struct.pack('<H', int(value))
        speed_byte = struct.pack('<H',speed)
        val_bytearray = bytearray([85,85,8,3,1,speed_byte[0],speed_byte[1],no,val_byte[0],val_byte[1]])
        return val_bytearray

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
        builder = ProtocolCommandBuilder(timeDeltaMs)
        for axis in self._axes:
            if axis.needsExecution():
                builder.addAxisCommand(axis.getId(), axis.getValue())
                axis.markExecuted()
            command = self._singleServoCtrlVal(axis.getId(), int(timeDeltaMs), axis.getValue())
        buildCommand = builder.build()
        self.serial.write(buildCommand)
        time.sleep(timeDeltaMs/1000.)
    
    def executeMotion(self, motion):
        for frame in motion.getFrames():
            for axisId, value in frame._axisValues.items():
                self.setTargetValue(axisId, value)
            self.write(frame.getTimeMs())

class ProtocolCommandBuilder(object): 
    def __init__(self, timeMs):
        self._commands = []
        self._timeMs = timeMs
    
    def addAxisCommand(self, axisId, value):
        self._commands.append((axisId, value))
    
    def build(self):
        length = len(self._commands) * 3 + 5
        header = 85
        commandId = 3
        servos = len(self._commands)
        speed_byte = struct.pack('<H',self._timeMs)
        val_bytearray = bytearray([header, header, length, commandId, servos, speed_byte[0],speed_byte[1],])
        
        for (axisId, value) in self._commands:
            val_byte = struct.pack('<H', int(value))
            val_bytearray.extend(bytearray([axisId,val_byte[0],val_byte[1]]))
        return val_bytearray
                


class Axis(object):
    def __init__(self, axisId, name, lowLimit, highLimit, reverse = False):
        self._axisId = axisId
        self._name = name
        self._lowLimit = lowLimit
        self._highLimit = highLimit
        self._value = lowLimit
        self._reverse = reverse
        self._needsExecution = True

    def setTargetValue(self, value):
        # TODO: sanity check
        if value < self._lowLimit:
            print("Axis %d command is smaller than limit - clamping, limit: %d, command: %d" % (self._axisId, self._lowLimit, value))
            value = self._lowLimit
        if value > self._highLimit:
            print("Axis %d command is bigger than limit - clamping, limit: %d, command: %d" % (self._axisId, self._highLimit, value))
            value = self._highLimit
        self._needsExecution = True
        self._value = value

    def markExecuted(self):
        self._needsExecution = False
    
    def needsExecution(self):
        return self._needsExecution

    def getValue(self):
        return self._value

    def _clampPercent(self, value):
        if value < 0:
            return 0
        if value > 100:
            return 100
        return value

    def setTargetPercent(self, percent):
        percent = self._clampPercent(percent)
        if(self._reverse):
            value = (int) (self._highLimit-(percent*((self._highLimit - self._lowLimit)/100)))
            self.setTargetValue(value)
        else:
            value = (int) (self._lowLimit+(percent*((self._highLimit - self._lowLimit)/100)))
            self.setTargetValue(value)

    def getId(self):
        return int(self._axisId)





