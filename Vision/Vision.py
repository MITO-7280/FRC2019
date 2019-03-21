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

openBallOutput = bool(False)

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
hsvBallLower = (0, 140, 135)
hsvBallUpper = (15, 255, 255)

hsvTapeLower = (0, 0, 245)
hsvTapeUpper = (255, 20, 255)

# set the configFile
configFile = "/boot/frc.json"
cameraConfigs = []

# config the camera
ballCamera = cs.UsbCamera("usbcam1", "/dev/video0")
ballCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)

tapeCamera = cs.UsbCamera("usbcam2", "/dev/video1")
tapeCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)

groundCamera = cs.UsbCamera("usbcam3", "/dev/video2")
groundCamera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)

# set the mjpegServer
# Server initial

ballServer = cs.MjpegServer("ballSource", 8081)
ballServer.setSource(ballCamera)

tapeServer = cs.MjpegServer("tapeSource", 8181)
tapeServer.setSource(tapeCamera)

groundServer = cs.MjpegServer("tapeSource", 8281)
groundServer.setSource(groundCamera)

print("ball server listening at http://0.0.0.0:8081")
print("tape server listening at http://0.0.0.0:8181")
print("ground server listening at http://0.0.0.0:8281")

ballSink = cs.CvSink("ballSink")
ballSink.setSource(ballCamera)

tapeSink = cs.CvSink("tapeSink")
tapeSink.setSource(tapeCamera)

groundSink = cs.CvSink("groundSink")
groundSink.setSource(groundCamera)

# cvBallSource = cs.CvSource("cvballsource", cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)
# cvBallServer = cs.MjpegServer("vision", 8082)
# cvBallServer.setSource(cvBallSource)
# print("OpenCV output ball server listening at http://0.0.0.0:8082")

cvTapeSource = cs.CvSource("cvtapesource", cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)
cvTapeServer = cs.MjpegServer("vision", 8182)
cvTapeServer.setSource(cvTapeSource)
print("OpenCV output tape server listening at http://0.0.0.0:8182")

cvGroundSource = cs.CvSource("cvgroundsource", cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)
cvGroundServer = cs.MjpegServer("vision", 8282)
cvGroundServer.setSource(cvGroundSource)
print("OpenCV output tape server listening at http://0.0.0.0:8282")


# keep looping
while True:
    # Ball Part

    # DO NOT TRY TO USE def
    # Will cause serious decreases in FPS
    # Do not know why
    # fucked myself

    tempTime = time.time()
    _, ballFrame = ballSink.grabFrame(ballnp)

    tempTime = time.time()
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
    (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    center = None

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
            # # draw the circle and centroid on the frame,
            # # then update the list of tracked points
            # cv2.circle(ballFrame, (int(x), int(y)), int(radius),
            #            (0, 255, 255), 2)
            # cv2.circle(ballFrame, center, 5, (0, 0, 255), -1)
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

    # # update the points queuec
    # pts.appendleft(center)
    # # loop over the set of tracked points
    # for i in range(1, len(pts)):
    #         # if either of the tracked points are None, ignore them
    #         if pts[i - 1] is None or pts[i] is None:
    #             continue
    #         thickness = 2
    #         cv2.line(ballFrame, pts[i - 1], pts[i], (0, 0, 255), thickness)
    #
    #     font = cv2.FONT_HERSHEY_SIMPLEX
    #     fps = 1 / (time.time() - tempTime)
    #     cv2.putText(ballFrame, str(fps), (0, 100), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
    #     cvBallSource.putFrame(ballFrame)

    # Tape Part

    _, tapeFrame = tapeSink.grabFrame(tapenp)
    # tapeBlurred = cv2.GaussianBlur(tapeFrame, (7, 7), 0)
    tapeHsv = cv2.cvtColor(tapeFrame, cv2.COLOR_BGR2HSV)
    tapeMask = cv2.inRange(tapeHsv, hsvTapeLower, hsvTapeUpper)
    tapeMask = cv2.erode(tapeMask, None, iterations=2)
    tapeMask = cv2.dilate(tapeMask, None, iterations=2)
    _, tapeContours, _ = cv2.findContours(tapeMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    # Gets the shape of video
    screenHeight, screenWidth, _ = tapeFrame.shape
    # Gets center of height and width
    centerX = (screenWidth / 2) - .5
    centerY = (screenHeight / 2) - .5
    # Copies frame and stores it in image
    tapeImage = tapeFrame.copy()
    # Processes the contours, takes in (contours, output_image, (centerOfImage)

    if len(tapeContours) > 1:
        cntsSorted = sorted(tapeContours, key=lambda x: cv2.contourArea(x), reverse=True)

        rect0 = cv2.minAreaRect(cntsSorted[0])
        box0 = cv2.boxPoints(rect0)
        box0 = np.int0(box0)
        cv2.drawContours(tapeImage, [box0], 0, (0, 0, 255), 2)

        rect1 = cv2.minAreaRect(cntsSorted[1])
        box1 = cv2.boxPoints(rect1)
        box1 = np.int0(box1)
        cv2.drawContours(tapeImage, [box1], 0, (0, 0, 255), 2)

        tape0M = cv2.moments(box0)
        tape0Center = (int(tape0M["m10"] / tape0M["m00"]), int(tape0M["m01"] / tape0M["m00"]))

        tape1M = cv2.moments(box1)
        tape1Center = (int(tape1M["m10"] / tape1M["m00"]), int(tape1M["m01"] / tape1M["m00"]))

        tapeMiddleCenter0 = (tape0Center[0] + tape1Center[0]) / 2
        tapeMiddleCenter1 = (tape0Center[1] + tape1Center[1]) / 2

        if tapeMiddleCenter0 < (width / 2 - width / 20):
            tapePos = 1
        elif tapeMiddleCenter0 > (width / 2 + width / 20):
            tapePos = 2
        else:
            tapePos = 3
        cv2.circle(tapeImage, (int(tapeMiddleCenter0), int(tapeMiddleCenter1)), 5, (0, 0, 255), -1)
    else:
        tapePos = 0
    tapeNetwork.putNumber("X", tapePos)
    print(tapePos)

    cvTapeSource.putFrame(tapeImage)
    # Shows the contours overlayed on the original video

    # Ground Part




    FPS = 1 / (time.time() - tempTime)  # type: float
    # print("FPS=", FPS)
