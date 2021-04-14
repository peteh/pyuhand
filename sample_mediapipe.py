import pyuhand
import glob
import time
import numpy as np
import cv2
import mediapipe as mp
import math

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise , kinematic_kf

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

uhand = pyuhand.UHand("/dev/ttyUSB0")

timedelta = 0.06

class FingerTracker(object):
    def __init__(self, dt):
        self._tracks = {}
        for i in range(4):
            self._tracks[i] = KalFilter3D(dt)
    
    def predict(self):
        for i in range(len(self._tracks)):
            self._tracks[i].predict()
    
    def update(self, point1, point2, point3, point4):
        self._tracks[0].update(point1)
        self._tracks[1].update(point2)
        self._tracks[2].update(point3)
        self._tracks[3].update(point4)
    
    def getState(self, index):
        return self._tracks[index].getStatePos()


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

lastposition = {}
lastposition[1] = 0
lastposition[2] = 0
lastposition[3] = 0
lastposition[4] = 0
lastposition[5] = 0
lastTime = time.time()

fingers = {}
fingers[1] = FingerTracker(timedelta)
fingers[2] = FingerTracker(timedelta)
fingers[3] = FingerTracker(timedelta)
fingers[4] = FingerTracker(timedelta)
fingers[5] = FingerTracker(timedelta)

varMeasure = VarMeasure(100)

lastKeyPoints = None
def angle(v1, v2, acute = True):
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    if (acute == True):
        return angle
    else:
        return 2 * np.pi - angle


def calculateAngle(p1, p2, p3):
    vec12 = p2-p1
    vec23 = p2-p3
    return angle(vec12, vec23)

def calculateFinger(keypoint1, keypoint2, keypoint3, keypoint4, angleMin, angleMax):
    a1 = calculateAngle(keypoint1, keypoint2, keypoint3)
    a2 = calculateAngle(keypoint2, keypoint3, keypoint4)
    fingerAngle = (a1+a2)/2.
    #fingerAngle = a1
    anglePercent = (fingerAngle - angleMin) / (angleMax - angleMin) * 100
    return anglePercent
    
def processLandMarks(hand_landmarks, useKalman):
    global lastTime, lastposition
    keypoints = []
    for data_point in hand_landmarks.landmark:
        keypoints.append(np.array([data_point.x, data_point.y, data_point.z]))

    # map to 0-100
    angleMax = 3
    angleMin = 2
    
    position = {}
    global lastKeyPoints
    if lastKeyPoints != None:
        varMeasure.addValue(lastKeyPoints[12][2])
    lastKeyPoints = keypoints
    print(varMeasure.getVariance())
    if useKalman: 
        for i in range (1, 6):
            fingers[i].predict()

        fingers[1].update(keypoints[1], keypoints[2], keypoints[3], keypoints[4])
        fingers[2].update(keypoints[5], keypoints[6], keypoints[7], keypoints[8])
        fingers[3].update(keypoints[9], keypoints[10], keypoints[11], keypoints[12])
        fingers[4].update(keypoints[13], keypoints[14], keypoints[15], keypoints[16])
        fingers[5].update(keypoints[17], keypoints[18], keypoints[19], keypoints[20])

        position[1] = calculateFinger(fingers[1].getState(0),fingers[1].getState(1),fingers[1].getState(2),fingers[1].getState(3), 2.3, 3.) # use different percentage mapping for thumb
        for i in range(2, 6):
            position[i] = calculateFinger(fingers[i].getState(0),fingers[i].getState(1),fingers[i].getState(2),fingers[i].getState(3), angleMin, angleMax) # use different percentage mapping for thumb
    else:
        position[1] = calculateFinger(keypoints[1], keypoints[2], keypoints[3], keypoints[4], 2.3, 3.) # use different percentage mapping for thumb
        position[2] = calculateFinger(keypoints[5], keypoints[6], keypoints[7], keypoints[8], angleMin, angleMax)
        position[3] = calculateFinger(keypoints[9], keypoints[10], keypoints[11], keypoints[12], angleMin, angleMax)
        position[4] = calculateFinger(keypoints[13], keypoints[14], keypoints[15], keypoints[16], angleMin, angleMax)
        position[5] = calculateFinger(keypoints[17], keypoints[18], keypoints[19], keypoints[20], angleMin, angleMax)
    velocity = {}
    for i in range(1, 6):
        uhand.setTargetPercent(i, position[i])
    timenow = time.time()

    uhand.execute(0)

    # store old data
    lastTime = timenow
    lastposition = position
    

# For webcam input:
cap = cv2.VideoCapture(0)
with mp_hands.Hands(
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5) as hands:
    useKalman = True
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
                processLandMarks(hand_landmarks, useKalman)
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        cv2.imshow('MediaPipe Hands', image)
        if cv2.waitKey(33) == ord('a'):
            useKalman = not useKalman
            print ("Use Kalman: %d" % (useKalman))

        if cv2.waitKey(5) & 0xFF == 27:
            break
cap.release()
