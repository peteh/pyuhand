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

import pyuhand
import glob
import time
import numpy as np
import cv2
import mediapipe as mp
import math

from filterpy.common import kinematic_kf

class KeyPointFilter(object):
    def __init__(self, nKeypoints, td):
        self._filters = []
        for i in range(nKeypoints):
            self._filters.append(KalFilter3D(td))
    
    def predict(self):
        for kalFilter in self._filters:
            kalFilter.predict()
    
    def update(self, keypoints):
        for i in range(len(self._filters)):
            self._filters[i].update(keypoints[i])
    
    def getFilteredPoints(self):
        keypoints = []
        for kalFilter in self._filters:
            keypoints.append(kalFilter.getStatePos())
        return keypoints

class KalFilter3D(object):
    def __init__(self, dt, initX = 0, initY=0, initZ=0):
        self._f = kinematic_kf(dim=3, order=1, dt=dt, order_by_dim = False)
        self._f.x = np.array([[initX],    # x position
                        [initY],
                        [initZ],
                        [0.], # velocity
                        [0.],
                        [0.]])

        # noise in the measurement from camera
        noiseX = 0.2
        noiseY = 0.5
        noiseZ = 0.8
        self._f.R = np.diag([noiseX, noiseY, noiseZ])

        # process uncertainty
        # how much the model changes between steps
        processNoisePos = 0.06 
        processNoiseVel = 0.1  
        self._f.Q = np.diag([processNoisePos, processNoisePos, processNoisePos, processNoiseVel, processNoiseVel, processNoiseVel])
    
    def predict(self):
        self._f.predict()
    
    def update(self, posMeasure):
        self._f.update(posMeasure)
    
    def getStatePos(self):
        return np.array([self._f.x[0][0], self._f.x[1][0], self._f.x[2][0]])
    

class VarMeasure(object):
    def __init__(self, size):
        self._size = size
        self._store = []

    def addValue(self, value):
        self._store.append(value)
        while(len(self._store) > self._size):
            self._store.pop(0)
    
    def getVariance(self):
        if len(self._store) == 0:
            return 0
        return math.sqrt(np.var(self._store))

def calculateHandAngle(base, direction):
    zAxis = np.array([0., 0., 1.])
    xAxis2d = np.array([1., 0.])
    directionVec = direction - base
    directionVec2d = np.array([directionVec[0], directionVec[2]])
    handAngleDeg = angle2(xAxis2d, directionVec2d)
    handAnglePercent = handAngleDeg / 360. * 100.
    return handAnglePercent

class HandAngle(object):

    def __init__(self, baseKeyPoint):
        self._baseKeyPoint = baseKeyPoint
    
    def _angle(self, p1, p2):
        ang1 = np.arctan2(*p1[::-1])
        ang2 = np.arctan2(*p2[::-1])
        return np.rad2deg((ang1 - ang2) % (2 * np.pi))

    def calculateAngle(self, keyPoints):
        base = keyPoints[self._baseKeyPoint]
        direction = keyPoints[self._baseKeyPoint + 1]
        zAxis = np.array([0., 0., 1.])
        xAxis2d = np.array([1., 0.])
        directionVec = direction - base
        directionVec2d = np.array([directionVec[0], directionVec[2]])
        handAngleDeg = self._angle(xAxis2d, directionVec2d)
        handAnglePercent = handAngleDeg / 360. * 100.
        return handAnglePercent


class Finger(object):
    def __init__(self, baseKeyPoint, angleMin, angleMax):
        self._baseKeyPoint = baseKeyPoint
        self._angleMin = angleMin
        self._angleMax = angleMax

    def _angle(self, v1, v2, acute = True):
        angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        if (acute == True):
            return angle
        else:
            return 2 * np.pi - angle

    def _calculateAngleFrom3Points(self, p1, p2, p3):
        vec12 = p2-p1
        vec23 = p2-p3
        return self._angle(vec12, vec23)

    def calculateAngle(self, keyPoints):
        a1 = self._calculateAngleFrom3Points(keyPoints[self._baseKeyPoint], keyPoints[self._baseKeyPoint + 1], keyPoints[self._baseKeyPoint + 2])
        a2 = self._calculateAngleFrom3Points(keyPoints[self._baseKeyPoint+1], keyPoints[self._baseKeyPoint + 2], keyPoints[self._baseKeyPoint + 3])
        fingerAngle = (a1+a2)/2.
        #fingerAngle = a1
        anglePercent = (fingerAngle - self._angleMin) / (self._angleMax - self._angleMin) * 100
        return anglePercent

def processLandMarks(keyPoints, fixedHandAngle):
    # map to 0-100
    angleMax = 3
    angleMin = 2
    
    position = {}
    fingers = {}
    fingers[1] = Finger(1, 2.3, 3.1) # use different percentage mapping for thumb
    fingers[2] = Finger(5, angleMin, angleMax)
    fingers[3] = Finger(9, angleMin, angleMax)
    fingers[4] = Finger(13, angleMin, angleMax)
    fingers[5] = Finger(17, angleMin, angleMax)
    fingers[6] = HandAngle(0) # handAngle provides the same functions, but calculates the actual angle

    for i in range(1,7):
        position[i] = fingers[i].calculateAngle(keyPoints)

    for i in range(1, 7):
        uhand.setTargetPercent(i, position[i])
    
    if fixedHandAngle:
        uhand.setTargetPercent(6, 100)

    uhand.execute(timedelta * 1000, blocking=False)
    
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

uhand = pyuhand.UHand("/dev/ttyUSB0")

timedelta = 0.06
# For webcam input:
cap = cv2.VideoCapture(0)
with mp_hands.Hands(
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5) as hands:
    
    useKalman = True
    fixedHandAngle = True
    
    filters = KeyPointFilter(21, timedelta)
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue
        # Flip the image horizontally for a later selfie-view display, and convert
        # the BGR image to RGB.
        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        results = hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
            #for hand_landmarks in results.multi_hand_landmarks:
            # only track first hand
            if(len(results.multi_hand_landmarks) > 0):
                hand_landmarks = results.multi_hand_landmarks[0]
                keypoints = []
                for data_point in hand_landmarks.landmark:
                    keypoints.append(np.array([data_point.x, data_point.y, data_point.z]))
                if useKalman: 
                    filters.predict()
                    filters.update(keypoints)
                    keypoints = filters.getFilteredPoints()
                processLandMarks(keypoints, fixedHandAngle)
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        cv2.imshow('MediaPipe Hands', image)
        key = cv2.waitKey(int(timedelta * 1000. * 0.9))
        if key == ord('a'):
            useKalman = not useKalman
            print ("Use Kalman: %d" % (useKalman))

        if key == ord('b'):
            fixedHandAngle = not fixedHandAngle
            print ("Fix Hand Angle: %d" % (fixedHandAngle))

        if key & 0xFF == 27:
            break
cap.release()
