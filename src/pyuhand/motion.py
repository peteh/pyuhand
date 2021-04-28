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

import xml.etree.ElementTree as ET

class MotionReader(object):
    def __init__(self):
        pass

class Motion(object):
    def __init__(self):
        self._frames = []
    
    def addFrame(self, frame):
        self._frames.append(frame)
    
    def getFrames(self): 
        return self._frames

    @staticmethod
    def fromFile(filePath):
        tree = ET.parse(filePath)
        root = tree.getroot()
        if(root[0].tag != "Table"): 
            print("Format mismatch, no Table")
            return
        
        motion = Motion()
        table = root[0]
        for i in range(len(table)):
            if table[i].tag != "ID":
                continue
            if table[i+1].tag != "Move":
                continue
            idEntry = table[i]
            moveEntry = table[i+1]
            timeEntry = table[i+2]
            if timeEntry.tag != "Time":
                # TODO: fail hard
                continue

            # read time
            timeMs = int(timeEntry.text.strip().replace("T", ""))
            # read the axis values
            frame = MotionFrame(timeMs)
            for entry in moveEntry.text.strip().split("#"):
                if entry == "":
                    continue
                splitEntry = entry.strip().split(" P")
                axisId = int(splitEntry[0])
                value = int(splitEntry[1])
                frame.setAxisTarget(axisId, value)
            motion.addFrame(frame)
        return motion

class MotionFrame(object):
    def __init__(self, timeMs = 1000):
        self._timeMs = timeMs
        self._axisValues = {}
    
    def setAxisTarget(self, axisId, targetValue):
        self._axisValues[axisId] = targetValue

    def setTimeMs(self, timeMs):
        self._timeMs = timeMs
    
    def getTimeMs(self):
        return self._timeMs

frame = MotionFrame()
frame.setAxisTarget(1, 2000)

