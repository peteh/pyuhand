# pyuhand - control your uhand
# Copyright (C) 2021  Pete <github@kthxbye.us>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import serial
import struct
import time
from .motion import Motion, MotionFrame

class UHand(object):
    def __init__(self, comPort):
        self._axes = []

        # TODO: double check all limits
        self._axes.append(Axis(1, "Thumb", 700, 2500, reverse = True))
        self._axes.append(Axis(2, "Index", 700, 2000)) # overwrite offical limit because mine has a screw there
        self._axes.append(Axis(3, "Middle", 700, 2000))
        self._axes.append(Axis(4, "Ring", 700, 2100))
        self._axes.append(Axis(5, "Pinky", 700, 2100))
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

    def executeSingleServo(self, timeDeltaMs = 1000., blocking = True):
        for axis in self._axes:
            if axis.needsExecution():
                axis.markExecuted()
                self._singleServoCtrlVal(axis.getId(), timeDeltaMs, axis.getValue())
        if blocking: 
            time.sleep(timeDeltaMs/1000.)    

    def execute(self, timeDeltaMs = 1000., blocking = True):
        builder = ProtocolCommandBuilder(timeDeltaMs)
        needsExecution = False
        for axis in self._axes:
            if axis.needsExecution():
                builder.addAxisCommand(axis.getId(), axis.getValue())
                needsExecution = True
                axis.markExecuted()
        if needsExecution:
            buildCommand = builder.build()
            self.serial.write(buildCommand)
        if blocking: 
            time.sleep(timeDeltaMs / 1000.)
    
    def executeMotion(self, motion):
        for frame in motion.getFrames():
            for axisId, value in frame._axisValues.items():
                self.setTargetValue(axisId, value)
            self.execute(frame.getTimeMs())

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
        speed_byte = struct.pack('<H', int(self._timeMs))
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
        if value < self._lowLimit:
            print("Axis %d command is smaller than limit - clamping, limit: %d, command: %d" % (self._axisId, self._lowLimit, value))
            value = self._lowLimit
        if value > self._highLimit:
            print("Axis %d command is bigger than limit - clamping, limit: %d, command: %d" % (self._axisId, self._highLimit, value))
            value = self._highLimit
        if self._value != value:
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





