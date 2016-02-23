#!/usr/bin/env python

import cv2, sys, time, os, math
from pantilt_stub import *
import logging
from networktables import NetworkTable

# ------------------------------------------------------------------------
# helper class
# ------------------------------------------------------------------------
class point:
    """A point identified by (x,y) coordinates"""
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
    
    def __add__(self, p):
        return point(self.x+p.x, self.y+p.y)
    
    def __sub__(self, p):
        return point(self.x-p.x, self.y-p.y)
    
    def __mul__(self, scalar):
        return point(self.x*scalar, self.y*scalar)
    
    def __div__(self, scalar):                        ## python 2.7
        return point(self.x/scalar, self.y/scalar)
    
    def __truediv__(self, scalar):                    ## python 3.5
        return point(self.x/scalar, self.y/scalar)
    
    def __str__(self):
        return "(%s, %s)" % (self.x, self.y)
    
    def __repr__(self):
        return "%s(%r, %r)" % (self.__class__.__name__, self.x, self.y)
    
    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)
    
    def distanceTo(self, p):
        """Calculate the distance between two points."""
        return (self - p).length()
    
    def asTuple(self):
        return (self.x, self.y)
    
    def clone(self):
        return point(self.x, self.y)
    
    def moveTo(self, x, y):
        self.x = x
        self.y = y
    
    def translate(self, dx, dy):
        self.x = self.x + dx
        self.y = self.y + dy

    def toInteger(self):
        self.x = int(self.x + 0.5)
        self.y = int(self.y + 0.5)
    
    def toFloat(self):
        self.x = float(self.x)
        self.y = float(self.y)        

# ------------------------------------------------
# Frame Size. Smaller is faster, but less accurate.
# ------------------------------------------------
#capsize = point(320, 240)
capsize = point(640, 480)

# Default Pan/Tilt for the camera in degrees.
# Camera range is from 0 to 180
# -------------------------------------------
cam_pan  = 70
cam_tilt = 70

# Target info
# -----------
top_target_height = 97.0  # the height to the top of the target in first stronghold is 97 inches

# Camera info
# -----------
camera_height   = 20.0  # height of the camera on robot - update later
vertical_fov    = 41.4  # PI camera info
horizontal_fov  = 53.5  # see https://www.raspberrypi.org/documentation/hardware/camera.md

# horizontal_fov  = 64.0  # est. Logitech c905 webcam
# vertical_fov    = 48.0  # est. Logitech c905 webcam

# ------------------------------------------------------------------------
# Set up the CascadeClassifier for face tracking
# ------------------------------------------------------------------------
def initFaceTracking():
    global faceCascade
    
    if sys.platform == 'win32':
        cascPath = 'C:/greg/dev/opencv/data/lbpcascades/lbpcascade_frontalface.xml'
    else:
        cascPath = '/usr/local/share/OpenCV/lbpcascades/lbpcascade_frontalface.xml'
    faceCascade = cv2.CascadeClassifier(cascPath)

# ------------------------------------------------------------------------
# Given a frame, find the first face, draw a green rectangle around it,
# and return the center of the face
# ------------------------------------------------------------------------
def findFace(frame):
    global faceCascade
    
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))
    
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Draw a green rectangle around the face
        
    for (x, y, w, h) in faces:
        return (frame, point(x, y) + point(w, h) / 2)             # and return the center of the first one found
    
    return (frame, None)

# ------------------------------------------------------------------------
# Given a frame, find the FRC Stronghold 2016 tower target
# and return the center of it
# ------------------------------------------------------------------------
def findTargetSift(frame, kp1, desc1):
    
    # Convert to greyscale for detection
    # ----------------------------------
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)

    return (gray, None)

# ------------------------------------------------------------------------
# move the camera so that it points to pixel (x, y) of the previous frame
# ------------------------------------------------------------------------
def track(p):
    global cam_pan, cam_tilt

    # Correct relative to center of image
    # -----------------------------------
    diff = p - capsize / 2

    # Convert to percentage offset
    # ----------------------------
    diff.toFloat()
    turn = point(diff.x / capsize.x / 2, diff.y / capsize.y / 2)

    # Scale offset to degrees
    # -----------------------
    turn.x   = turn.x * 2.5          # VFOV
    turn.y   = turn.y * 2.5          # HFOV
    cam_pan  += -turn.x
    cam_tilt +=  turn.y

    # Clamp Pan/Tilt to 0 to 180 degrees
    # ----------------------------------
    cam_pan  = max(0, min(180, cam_pan))
    cam_tilt = max(0, min(180, cam_tilt))
    
    # Update the servos
    # -----------------
    pan(cam_pan)
    tilt(cam_tilt)

# ------------------------------------------------------------------------
# Set up the capture with our frame size
# ------------------------------------------------------------------------
def initCapture():
    global video_capture
    
    video_capture = cv2.VideoCapture(0)
    
    # video_capture.set(cv2.CAP_PROP_SETTINGS, 1)  # pops up dialog for webcam settings
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH,  capsize.x)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, capsize.y)
    time.sleep(1)

# ------------------------------------------------------------------------
# networktables
# ------------------------------------------------------------------------
def initNetworktables():
    logging.basicConfig(level=logging.DEBUG)         #to see messages from networktables
    NetworkTable.setIPAddress('127.0.0.1')
    # NetworkTable.setIPAddress('roborio-5260.local')
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    return NetworkTable.getTable('Pi')
    
# ------------------------------------------------------------------------
# main capture/track/display loop
# ------------------------------------------------------------------------
def MainProgram():
    if sys.platform != 'win32':
        os.system('sudo modprobe bcm2835-v4l2')     # Load the BCM V4l2 driver for /dev/video0
        os.system('v4l2-ctl -p 4')                  # Set the framerate (not sure this does anything!)
        #os.system('v4l2-ctl -c focus_auto=0')      # Disable autofocus??
        # try 'v4l2-ctl -l' to show all possible controls
        # use '-d /dev/video1' if more than one device

    # ------------------------------------------------------------------------
    # Turn the camera to the default position
    # ------------------------------------------------------------------------
    pan(cam_pan)
    tilt(cam_tilt)

    initCapture()                                   # setup camera and create video_capture object

    method = 0                                      # 0 = face tracking, 1 = SURF

    if method == 0:
        initFaceTracking()
    elif method == 1:
        # Initiate SIFT detector
        sift = cv2.xfeatures2d.SIFT_create()
        
        # find the keypoints and descriptors with SIFT
        img1 = cv2.imread('target.png', 0)
        if img1 and not img1.empty():
            kp1, desc1 = sift.detectAndCompute(img1, None)

    raspi = initNetworktables()
    i = 0
    
    while True:
        ret, frame = video_capture.read()           # get a frame

        if ret == False:
            print("Error getting image")            # if we didn't get a frame, try again
            continue                                # maybe it was just a hiccup!

        if method == 0:
            img, p = findFace(frame)                # Do face detection
        elif method == 1 and img1 and not img1.empty():
            img, p = findTargetSift(frame, kp1, desc1) # Do target detection using SIFT
        else:
            img, p = None, None
        
        if p:
            #cv2.circle(frame, p.asTuple(), 3, 255, -1)
            track(p)                                # Point the camera to the returned position

        if img != None:
            frame = img
        
        frame = cv2.resize(frame, (capsize * 1).asTuple())
        frame = cv2.flip(frame, 1)

        cv2.imshow('Video', frame)                  # Display the image, with rectangle

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        raspi.putNumber('piTime', i)
        try:
            print('robotTime:', raspi.getNumber('robotTime'))
        except KeyError:
            print('robotTime: n/a')
        sys.stdout.flush()
        i += 1

    # When everything is done, release the capture
    # --------------------------------------------
    video_capture.release()
    cv2.destroyAllWindows()


# ------------------------------------------------------------------------
# just run main program
# ------------------------------------------------------------------------
MainProgram()
