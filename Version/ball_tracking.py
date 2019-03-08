import sys
import time
from networktables import NetworkTablesInstance
from networktables import NetworkTables
import cv2
import cscore as cs
from collections import deque
import cscore
import numpy as np
import json
import argparse
import math

# config here
team = 7280


# unknown area
# ap = argparse.ArgumentParser()
# ap.add_argument("-v", "--video",
#                 help="path to the (optional) video file")
# ap.add_argument("-b", "--buffer", type=int, default=64,
#                 help="max buffer size")
# args = vars(ap.parse_args())
pts = deque(maxlen=64)

# set the hsv boundary
hsvBallLower = (0, 140, 135)
hsvBallUpper = (15, 255, 255)

# set the configFile
configFile = "/boot/frc.json"
cameraConfigs = []

# config the camera
camera = cs.UsbCamera("usbcam", "/dev/video0")
camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)

# set the mjpegServer
mjpegServer = cs.MjpegServer("httpserver", 8081)
mjpegServer.setSource(camera)

print("mjpg server listening at http://0.0.0.0:8081")

cvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)
testSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)

cvsink = cs.CvSink("cvsink")
cvsink.setSource(camera)

testServer = cs.MjpegServer("cvhttpserver", 8083)
testServer.setSource(testSource)

cvMjpegServer = cs.MjpegServer("cvhttpserver", 8082)
cvMjpegServer.setSource(cvSource)
print("OpenCV output mjpg server listening at http://0.0.0.0:8082")

origin = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
test = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
output = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

# keep looping
ballPos = 0

while True:
    _, frame = cvsink.grabFrame(origin)

    # codes here
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask, then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, hsvBallLower, hsvBallUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    center = None

    threshold_area = 2000
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            if center[0] < (320/2 - 20):
                ballPos = 1
            elif center[0] > (320/2 + 20):
                ballPos = 2
            else:
                ballPos = 3
        else:
            ballPos = 0
    print(ballPos)


    # update the points queuec
    pts.appendleft(center)
    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        # thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        thickness = 2
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
    # put the frame to the server

    cvSource.putFrame(frame)
import sys
import time
from networktables import NetworkTablesInstance
from networktables import NetworkTables
import cv2
import cscore as cs
from collections import deque
import cscore
import numpy as np
import json
import argparse
import math

# config here
team = 7280


# unknown area
# ap = argparse.ArgumentParser()
# ap.add_argument("-v", "--video",
#                 help="path to the (optional) video file")
# ap.add_argument("-b", "--buffer", type=int, default=64,
#                 help="max buffer size")
# args = vars(ap.parse_args())
pts = deque(maxlen=64)

# set the hsv boundary
hsvBallLower = (0, 140, 135)
hsvBallUpper = (15, 255, 255)

# set the configFile
configFile = "/boot/frc.json"
cameraConfigs = []

# config the camera
camera = cs.UsbCamera("usbcam", "/dev/video0")
camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)

# set the mjpegServer
mjpegServer = cs.MjpegServer("httpserver", 8081)
mjpegServer.setSource(camera)

print("mjpg server listening at http://0.0.0.0:8081")

cvSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)
testSource = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 30)

cvsink = cs.CvSink("cvsink")
cvsink.setSource(camera)

testServer = cs.MjpegServer("cvhttpserver", 8083)
testServer.setSource(testSource)

cvMjpegServer = cs.MjpegServer("cvhttpserver", 8082)
cvMjpegServer.setSource(cvSource)
print("OpenCV output mjpg server listening at http://0.0.0.0:8082")

origin = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
test = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
output = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

# keep looping
ballPos = 0

while True:
    _, frame = cvsink.grabFrame(origin)

    # codes here
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask, then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, hsvBallLower, hsvBallUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    center = None

    threshold_area = 2000
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            if center[0] < (320/2 - 20):
                ballPos = 1
            elif center[0] > (320/2 + 20):
                ballPos = 2
            else:
                ballPos = 3
        else:
            ballPos = 0
    print(ballPos)


    # update the points queuec
    pts.appendleft(center)
    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        # thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        thickness = 2
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
    # put the frame to the server

    cvSource.putFrame(frame)
