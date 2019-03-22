import time
from networktables import NetworkTablesInstance
from networktables import NetworkTables
import cv2
import cscore as cs
from collections import deque
import cscore
import numpy as np
import json
import math
import logging

# config here
team = 7280

# networktable initialization
NetworkTables.initialize(server='roborio-7280-frc.local')
ballNetwork = NetworkTables.getTable("ball")
tapeNetwork = NetworkTables.getTable("tape")


pts = deque(maxlen=64)

# set the camera resolution

width = 320
height = 240

# set the output resolution

outputWidth = 320
outputHeight = 240

ballnp = np.zeros(shape=(width, height, 3), dtype=np.uint8)
tapenp = np.zeros(shape=(width, height, 3), dtype=np.uint8)

# set the hsv boundary
# form: (H, S, V)
# Modify Needed
hsvBallLower = (0, 140, 135)
hsvBallUpper = (15, 255, 255)

# hsvTapeLower = (0, 0, 245)
# hsvTapeUpper = (255, 20, 255)

hsvTapeLower = (82, 75, 176)
hsvTapeUpper = (96, 255, 255)

hsvGroundLower = (7, 0, 160)
hsvGroundUpper = (110, 35, 255)
# set the configFile
configFile = "/boot/frc.json"
cameraConfigs = []

# config the camera
ballCamera = cs.UsbCamera("ballcam", "/dev/video0")
ballCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)

groundCamera = cs.UsbCamera("groundcam", "/dev/video1")
groundCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)

# set the mjpegServer
# Server initial

ballServer = cs.MjpegServer("ballSource", 8081)
ballServer.setSource(ballCamera)

groundServer = cs.MjpegServer("groundSource", 8181)
groundServer.setSource(groundCamera)

print("ball server listening at http://0.0.0.0:8081")
print("ground server listening at http://0.0.0.0:8181")

ballSink = cs.CvSink("ballSink")
ballSink.setSource(ballCamera)

groundSink = cs.CvSink("groundSink")
groundSink.setSource(groundCamera)

cvBallSource = cs.CvSource("cvballsource", cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)
cvBallServer = cs.MjpegServer("vision", 8082)
cvBallServer.setSource(cvBallSource)
print("OpenCV output ball server listening at http://0.0.0.0:8082")

cvGroundSource = cs.CvSource("cvgroundsource", cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)
cvGroundServer = cs.MjpegServer("vision", 8182)
cvGroundServer.setSource(cvGroundSource)
print("OpenCV output ground server listening at http://0.0.0.0:8182")


# keep looping
while True:
    # Ball Part

    # DO NOT TRY TO USE def
    # Will cause serious decreases in FPS
    # Do not know why
    # fucked myself

    tempTime = time.time()

    _, ballFrame = ballSink.grabFrame(ballnp)

    blurred = cv2.GaussianBlur(ballFrame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask, then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, hsvBallLower, hsvBallUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    (_, ballContours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    center = None

    # only proceed if at least one contour was found
    if len(ballContours) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(ballContours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(ballFrame, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(ballFrame, center, 5, (0, 0, 255), -1)
            if center[0] < (width / 2 - width / 16):
                ballPos = 1
            elif center[0] > (width / 2 + width / 16):
                ballPos = 2
            else:
                ballPos = 3
    else:
        ballPos = 0
    # print("ballPos=", ballPos)
    ballNetwork.putNumber("Y", ballPos)

    # update the points queuec
    pts.appendleft(center)
    # loop over the set of tracked points
    # for i in range(1, len(pts)):
    #         # if either of the tracked points are None, ignore them
    #         if pts[i - 1] is None or pts[i] is None:
    #             continue
    #         thickness = 2
    #         cv2.line(ballFrame, pts[i - 1], pts[i], (0, 0, 255), thickness)
    font = cv2.FONT_HERSHEY_SIMPLEX


    # Tape Part

    _, tapeFrame = ballSink.grabFrame(tapenp)
    # tapeBlurred = cv2.GaussianBlur(tapeFrame, (7, 7), 0)
    tapeHsv = cv2.cvtColor(tapeFrame, cv2.COLOR_BGR2HSV)
    tapeMask = cv2.inRange(tapeHsv, hsvTapeLower, hsvTapeUpper)
    tapeMask = cv2.erode(tapeMask, None, iterations=2)
    tapeMask = cv2.dilate(tapeMask, None, iterations=2)
    _, tapeContours, _ = cv2.findContours(tapeMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    # Copies frame and stores it in image
    tapeImage = tapeFrame.copy()
    # Processes the contours, takes in (contours, output_image, (centerOfImage)

    if len(tapeContours) > 1:
        cntsSorted = sorted(tapeContours, key=lambda x: cv2.contourArea(x), reverse=True)

        rect0 = cv2.minAreaRect(cntsSorted[0])
        box0 = cv2.boxPoints(rect0)
        box0 = np.int0(box0)
        cv2.drawContours(ballFrame, [box0], 0, (0, 0, 255), 2)
        rect1 = cv2.minAreaRect(cntsSorted[1])
        box1 = cv2.boxPoints(rect1)
        box1 = np.int0(box1)
        cv2.drawContours(ballFrame, [box1], 0, (0, 0, 255), 2)

        tape0Center = int(rect0[0][0]), int(rect0[0][1])
        tape1Center = int(rect1[0][0]), int(rect1[0][1])

        tapeMiddleCenter0 = (tape0Center[0] + tape1Center[0]) / 2
        tapeMiddleCenter1 = (tape0Center[1] + tape1Center[1]) / 2

        if tapeMiddleCenter0 < (width / 2 - width / 20):
            tapePos = 1
        elif tapeMiddleCenter0 > (width / 2 + width / 20):
            tapePos = 2
        else:
            tapePos = 3
        cv2.circle(ballFrame, (int(tapeMiddleCenter0), int(tapeMiddleCenter1)), 5, (0, 0, 255), -1)
    else:
        tapePos = 0
    tapeNetwork.putNumber("X", tapePos)
    cvBallSource.putFrame(ballFrame)
    # print(tapePos)

    # Ground Part

    _, groundFrame = groundSink.grabFrame(tapenp)
    # groundBlurred = cv2.GaussianBlur(groundFrame, (7, 7), 0)
    groundHsv = cv2.cvtColor(groundFrame, cv2.COLOR_BGR2HSV)
    groundMask = cv2.inRange(groundHsv, hsvGroundLower, hsvGroundUpper)
    groundMask = cv2.erode(groundMask, None, iterations=2)
    groundMask = cv2.dilate(groundMask, None, iterations=2)
    _, groundContours, _ = cv2.findContours(groundMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    groundImage = groundFrame.copy()
    if len(groundContours) > 0:
        groundc = max(groundContours, key=cv2.contourArea)
        groundRect = cv2.minAreaRect(groundc)
        groundBox = cv2.boxPoints(groundRect)
        groundBox = np.int0(groundBox)
        cv2.drawContours(groundImage, [groundBox], 0, (0, 0, 255), 2)
        groundCenter = int(groundRect[0][0]), int(groundRect[0][1])
        cv2.circle(groundImage, groundCenter, 5, (0, 0, 255), -1)

        if groundCenter[0] < (width / 2 - width / 8):
            groundPos = 4
        elif groundCenter[0] > (width / 2 + width / 8):
            groundPos = 5
        elif groundRect[1][0] < groundRect[1][1]:
            groundPos = 1
        elif groundRect[1][0] > groundRect[1][1]:
            groundPos = 2
        else:
            groundPos = 3
        print(groundPos)
    cvGroundSource.putFrame(groundImage)
    tapeNetwork.putNumber("Y", groundPos)
    FPS = 1 / (time.time() - tempTime)  # type: float
    # print("FPS=", FPS)

