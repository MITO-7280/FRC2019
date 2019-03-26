import time
from networktables import NetworkTablesInstance
from networktables import NetworkTables
import cv2
import cscore as cs
from collections import deque
import numpy as np
import json
import math
import logging

# config here
team = 7280

# networktable initialization
logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize(server='roborio-7280-frc.local')
ballNetwork = NetworkTables.getTable("ball")
tapeNetwork = NetworkTables.getTable("tape")
needNetwork = NetworkTables.getTable("isNeeded")

pts = deque(maxlen=64)

# set the camera resolution

width = 320
height = 240

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

hsvGroundLower = (0, 0, 192)
hsvGroundUpper = (255, 70, 255)
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

# cvBallSource = cs.CvSource("cvballsource", cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)
# cvBallServer = cs.MjpegServer("vision", 8082)
# cvBallServer.setSource(cvBallSource)
# print("OpenCV output ball server listening at http://0.0.0.0:8082")

# cvGroundSource = cs.CvSource("cvgroundsource", cs.VideoMode.PixelFormat.kMJPEG, width, height, 30)
# cvGroundServer = cs.MjpegServer("vision", 8182)
# cvGroundServer.setSource(cvGroundSource)
# print("OpenCV output ground server listening at http://0.0.0.0:8182")


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
        ((circlex, circley), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
#             cv2.circle(ballFrame, (int(circlex), int(circley)), int(radius),
#                        (0, 255, 255), 2)
#             cv2.circle(ballFrame, center, 5, (0, 0, 255), -1)
            if center[1] > (height * 0.8):
                ballPos = 3
            else:
                if center[0] < (width / 2 - width / 10):
                    ballPos = 1
                elif center[0] > (width / 2 + width / 10):
                    ballPos = 2
                else:
                    ballPos = 3
    else:
        ballPos = 0
    print("ballPos=", ballPos)
    ballNetwork.putNumber("Y", ballPos)

#     cvBallSource.putFrame(ballFrame)

    # update the points queuec
    # pts.appendleft(center)
    # loop over the set of tracked points
    # for i in range(1, len(pts)):
    #         # if either of the tracked points are None, ignore them
    #         if pts[i - 1] is None or pts[i] is None:
    #             continue
    #         thickness = 2
    #         cv2.line(ballFrame, pts[i - 1], pts[i], (0, 0, 255), thickness)
    # font = cv2.FONT_HERSHEY_SIMPLEX


    # Tape Part

    # _, tapeFrame = ballSink.grabFrame(tapenp)
    # # tapeBlurred = cv2.blur(tapeFrame, (10, 10))
    # tapeHsv = cv2.cvtColor(tapeFrame, cv2.COLOR_BGR2HSV)
    # tapeMask = cv2.inRange(tapeHsv, hsvTapeLower, hsvTapeUpper)
    # tapeMask = cv2.erode(tapeMask, None, iterations=2)
    # tapeMask = cv2.dilate(tapeMask, None, iterations=2)
    # _, tapeContours, _ = cv2.findContours(tapeMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    # # Copies frame and stores it in image
    # tapeImage = tapeFrame.copy()
    # # Processes the contours, takes in (contours, output_image, (centerOfImage)
    #
    # if len(tapeContours) > 1:
    #     cntsSorted = sorted(tapeContours, key=lambda x: cv2.contourArea(x), reverse=True)
    #     rect0 = cv2.minAreaRect(cntsSorted[0])
    #     box0 = cv2.boxPoints(rect0)
    #     box0 = np.int0(box0)
    #     # cv2.drawContours(ballFrame, [box0], 0, (0, 0, 255), 2)
    #     rect1 = cv2.minAreaRect(cntsSorted[1])
    #     box1 = cv2.boxPoints(rect1)
    #     box1 = np.int0(box1)
    #     # cv2.drawContours(ballFrame, [box1], 0, (0, 0, 255), 2)
    #
    #     tape0Center = int(rect0[0][0]), int(rect0[0][1])
    #     tape1Center = int(rect1[0][0]), int(rect1[0][1])
    #
    #     tapeMiddleCenter0 = (tape0Center[0] + tape1Center[0]) / 2
    #     tapeMiddleCenter1 = (tape0Center[1] + tape1Center[1]) / 2
    #
    #     if tapeMiddleCenter0 < (width / 2 - width / 20):
    #         tapePos = 1
    #     elif tapeMiddleCenter0 > (width / 2 + width / 20):
    #         tapePos = 2
    #     else:
    #         tapePos = 3
    #     # cv2.circle(ballFrame, (int(tapeMiddleCenter0), int(tapeMiddleCenter1)), 5, (0, 0, 255), -1)
    # else:
    #     tapePos = 0
    # tapeNetwork.putNumber("X", tapePos)
    # #
    # cvBallSource.putFrame(ballFrame)

    # print(tapePos)

    # Ground Part
    # isNeeded = int(needNetwork.getNumber("X", 1))
    # if isNeeded == 1:
    #     _, groundFrame = groundSink.grabFrame(tapenp)
    #     groundBlurred = cv2.blur(groundFrame, (4, 4))
    #     groundHsv = cv2.cvtColor(groundFrame, cv2.COLOR_BGR2HSV)
    #     groundMask = cv2.inRange(groundHsv, hsvGroundLower, hsvGroundUpper)
    #     groundMask = cv2.erode(groundMask, None, iterations=2)
    #     groundMask = cv2.dilate(groundMask, None, iterations=2)
    #     _, groundContours, _ = cv2.findContours(groundMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    #     groundImage = groundFrame.copy()
    #     global groundPos
    #     if len(groundContours) > 0:
    #         groundc = max(groundContours, key=cv2.contourArea)
    #         groundImage = cv2.drawContours(groundImage, [groundc], 0, (0, 255, 0), 2)
    #         if cv2.contourArea(groundc) > 200:
    #             groundM = cv2.moments(groundc)
    #             groundCenter = (int(groundM["m10"] / groundM["m00"]), int(groundM["m01"] / groundM["m00"]))
    #             cv2.circle(groundImage, groundCenter, 5, (0, 0, 255), -1)
    #
    #
    #             rows, cols = groundImage.shape[:2]
    #             [vx, vy, x, y] = cv2.fitLine(groundc, cv2.DIST_L2, 0, 0.01, 0.01)
    #             k = vy / vx
    #             lefty = int((-x * k) + y)
    #             righty = int(((cols - x) * k) + y)
    #             groundImage = cv2.line(groundImage, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)
    #
    #             if groundCenter[0] < (width / 2 - width / 8):
    #                 groundPos = 4
    #             elif groundCenter[0] > (width / 2 + width / 8):
    #                 groundPos = 5
    #             elif (vx / vy) < (-1.732):
    #                 groundPos = 1
    #             elif (vx / vy) > 1.732:
    #                 groundPos = 2
    #             else:
    #                 groundPos = 3
    #         else:
    #             groundPos = 6
    #     else:
    #         groundPos = 6
    #     # print(groundPos)
    #     tapeNetwork.putNumber("Y", groundPos)
    #
    #     cvGroundSource.putFrame(groundImage)

    FPS = 1 / (time.time() - tempTime)  # type: float
    print("FPS=", FPS) 
