import pyuhand
import glob
import time
import numpy as np
import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

uhand = pyuhand.UHand("/dev/ttyUSB0")

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
    
    

def processLandMarks(hand_landmarks):
    keypoints = []
    for data_point in hand_landmarks.landmark:
        keypoints.append(np.array([data_point.x, data_point.y, data_point.z]))

    # map to 0-100
    angleMax = 3
    angleMin = 2
    
    thumbPercent = calculateFinger(keypoints[1], keypoints[2], keypoints[3], keypoints[4], 2.3, 3.) # use different percentage mapping for thumb
    uhand.setTargetPercent(1, thumbPercent)
    
    
    indexFingerPercent = calculateFinger(keypoints[5], keypoints[6], keypoints[7], keypoints[8], angleMin, angleMax)
    uhand.setTargetPercent(2, indexFingerPercent)
    
    middleFingerPercent = calculateFinger(keypoints[9], keypoints[10], keypoints[11], keypoints[12], angleMin, angleMax)
    uhand.setTargetPercent(3, middleFingerPercent)

    ringFingerPercent = calculateFinger(keypoints[13], keypoints[14], keypoints[15], keypoints[16], angleMin, angleMax)
    uhand.setTargetPercent(4, ringFingerPercent)
    
    pinkyFingerPercent = calculateFinger(keypoints[17], keypoints[18], keypoints[19], keypoints[20], angleMin, angleMax)
    uhand.setTargetPercent(5, pinkyFingerPercent)

    uhand.write(0)
    

# For webcam input:
cap = cv2.VideoCapture(0)
with mp_hands.Hands(
        min_detection_confidence=0.9,
        min_tracking_confidence=0.5) as hands:
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
            # only track first hand

            #for hand_landmarks in results.multi_hand_landmarks:
            if(len(results.multi_hand_landmarks) > 0):
                hand_landmarks = results.multi_hand_landmarks[0]
                processLandMarks(hand_landmarks)
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        cv2.imshow('MediaPipe Hands', image)
        if cv2.waitKey(5) & 0xFF == 27:
            break
cap.release()
