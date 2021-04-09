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

