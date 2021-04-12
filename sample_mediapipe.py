import pyuhand
import glob
import time
import numpy as np
import cv2
import mediapipe as mp

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

uhand = pyuhand.UHand("/dev/ttyUSB0")

timedelta = 0.06

class KalFilter(object):
    def __init__(self, initPos = 0):

        # time between frames
        dt=0.06
        self._f = KalmanFilter (dim_x=2, dim_z=1)
        self._f.x = np.array([[initPos],    # position
                        [0.]])   # velocity

        # state transition matrix
        self._f.F = np.array([[1.,1.],
                        [0.,1.]])

        # measurement
        self._f.H = np.array([[1.,0 ]])

        # covariance matrix
        self._f.P = np.array([[1000.,    0.],
                        [   0., 1000.] ])

        # noise in the measurement
        self._f.R = 10

        # process uncertainty
        # variance: how much the model changes between steps
        # TODO: estimate noise from camera
        global timedelta
        self._f.Q = Q_discrete_white_noise(dim=2, dt=timedelta, var=100)
    
    def predict(self):
        self._f.predict()
    
    def update(self, posMeasure):
        self._f.update(posMeasure)
    
    def getStatePos(self):
        return self._f.x[0]

lastposition = {}
lastposition[1] = 0
lastposition[2] = 0
lastposition[3] = 0
lastposition[4] = 0
lastposition[5] = 0
lastTime = time.time()

kalFilter = {}
kalFilter[1] = KalFilter()
kalFilter[2] = KalFilter()
kalFilter[3] = KalFilter()
kalFilter[4] = KalFilter()
kalFilter[5] = KalFilter()

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
    
    position[1] = calculateFinger(keypoints[1], keypoints[2], keypoints[3], keypoints[4], 2.3, 3.) # use different percentage mapping for thumb
    position[2] = calculateFinger(keypoints[5], keypoints[6], keypoints[7], keypoints[8], angleMin, angleMax)
    position[3] = calculateFinger(keypoints[9], keypoints[10], keypoints[11], keypoints[12], angleMin, angleMax)
    position[4] = calculateFinger(keypoints[13], keypoints[14], keypoints[15], keypoints[16], angleMin, angleMax)
    position[5] = calculateFinger(keypoints[17], keypoints[18], keypoints[19], keypoints[20], angleMin, angleMax)
    velocity = {}
    for i in range(1, 6):
        velocity[i] = (lastposition[i] - position[i]) / timedelta
        kalFilter[i].predict()
        kalFilter[i].update(position[i])
    timenow = time.time()

    for i in range(1,6):
        if useKalman: 
            uhand.setTargetPercent(i, kalFilter[i].getStatePos())
        else: 
            uhand.setTargetPercent(i, position[i])
    uhand.execute(0)

    # store old data
    lastTime = timenow
    lastposition = position
    

# For webcam input:
cap = cv2.VideoCapture(0)
with mp_hands.Hands(
        min_detection_confidence=0.9,
        min_tracking_confidence=0.5) as hands:
    useKalman = False
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
