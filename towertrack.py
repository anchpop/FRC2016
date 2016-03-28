#!/usr/bin/env python

# ------------------------------------------------------------------------
# imports
# ------------------------------------------------------------------------
import cv2, sys, time, os, math, logging
from point import point
from networktables import NetworkTable
from threading import Thread

# ------------------------------------------------------------------------
# config
# ------------------------------------------------------------------------
if sys.platform == 'win32':
    method     = 0               # 0 = face tracking, 1 = SURF
    feedback   = True
    camera     = "c905"
    use_servo  = False           # when False, use networktables
    show_image = True            # display grabbed image
    capsize    = point(640, 480)
    cam_center_angle = 0.0
else:
    method     = 0               # 0 = face tracking, 1 = SURF
    feedback   = False
    camera     = "pi"
    use_servo  = True            # when False, use networktables
    show_image = True           # display grabbed image
    capsize    = point(640, 480)
    cam_center_angle = 0.0

print("using %s camera" % camera)
if camera == "pi":
    fov = point(53.5, 41.4)      # see https://www.raspberrypi.org/documentation/hardware/camera.md
elif camera == "c905":
    fov = point(64.0, 48.0)      # est. Logitech c905 webcam
else:
    fov = point(64.0, 48.0)      # est. Logitech c905 webcam

if use_servo:
    from pantilt import *
    
# ------------------------------------------------
# Webcam lag management
#
# If you have ffmpeg support enabled (which it is by default in most opencv builds), you can retrieve
# the mjpeg stream directly from the camera via FFMPEG.
# Something similar to the following (error checking omitted):
#  
#  vc = cv2.VideoCapture()
#  vc.open('http://%s/mjpg/video.mjpg' % camera_host)
#  
#  h = vc.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
#  w = vc.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
#  
#  capture_buffer = np.empty(shape=(h, w, 3), dtype=np.uint8)
#  
#  while True:
#      retval, img = vc.read(capture_buffer)
# ------------------------------------------------
class CameraStream:
    def __init__(self, cam_idx = 0):
        if camera == 'pi':
            # modprobe required after pi reboot
            os.system('sudo modprobe bcm2835-v4l2')     # Load the BCM V4l2 driver for /dev/video0
            os.system('v4l2-ctl -p 4')                  # Set the framerate (not sure this does anything!)
            #os.system('v4l2-ctl -c focus_auto=0')       # Disable autofocus??
            # try 'v4l2-ctl -l' to show all possible controls
            pass
        
        self.cam_idx = cam_idx
        
    def start(self):
        self.capture = cv2.VideoCapture(self.cam_idx) 

        # self.capture.set(cv2.CAP_PROP_SETTINGS, 1)  # pops up dialog for webcam settings
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, capsize.x)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, capsize.y)
        
        buffsize = self.capture.get(cv2.CAP_PROP_BUFFERSIZE)
        if buffsize != -1:
            print("VideoCapture buffer size = %d, setting to 1" % buffsize)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
        # Start new thread to continuously get images
        time.sleep(2 if camera == 'pi' else 1)
        
        self.quit = False
        self.ret, self.currentFrame = self.capture.read()
        self.img_thread = Thread(target=self.updateFrame, args=()).start()

    def stop(self):
        self.quit = True
        self.img_thread.join()
        self.currentFrame = None
        self.capture.release()

    # Continually updates self.currentFrame
    def updateFrame(self):
        while not self.quit:
            ret, frame = self.capture.read()
            while ret == False:                   # Continually grab frames until we get a good one
                time.sleep(0.005)                 # but sleep for 5 milliseconds
                ret, frame = self.capture.read()
            self.ret = True
            self.currentFrame = frame

    def getFrame(self):
        return (self.ret, self.currentFrame)

    def nextFrame(self):
        self.currentFrame = None
        self.ret = False

# ------------------------------------------------
# Frame Size. Smaller is faster, but less accurate.
# ------------------------------------------------

# Target info
# -----------
top_target_height = 97.0  # the height to the top of the target in first stronghold is 97 inches

# Camera info
# -----------
# camera_height   = 20.0  # height of the camera on robot - update later


# Default Pan/Tilt for the camera in degrees.
# Camera range is from 0 to 180
# -------------------------------------------
cam_pan_cur = 70.0
cam_tilt_cur = 70.0

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

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist( gray )
    faces = faceCascade.detectMultiScale(gray, 1.1, 3, 0, (10, 10))

    if show_image:
        for (x, y, w, h) in faces:
            cv2.rectangle(gray, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Draw a green rectangle around the face
        
    for (x, y, w, h) in faces:
        return (gray, point(x, y) + (point(w, h) / 2))     # and return the center of the first one found
    
    return (gray, None)

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
def track(p, raspi):
    global cam_center_angle
    global cam_pan_cur, cam_tilt_cur

    # Correct relative to center of image
    # -----------------------------------
    diff = p - capsize / 2

    # Convert to percentage offset
    # ----------------------------
    diff.toFloat()
    if feedback:
        print('diff = (%f, %f)' % (diff.x, diff.y))                   # in pixels
    turn = point(diff.x / capsize.x, diff.y / capsize.y)  # in %
    
    ## servo seems to react with excessively large movement to tilt commands, so
    ## divide turn.y by 8 instead of by 2
    turn = point(turn.x / 2.0, turn.y / 8)
    if feedback:
        print('turn = (%f, %f)' % (turn.x, turn.y))

    # Scale offset to degrees
    # -----------------------
    turn.x   = turn.x * fov.x
    turn.y   = turn.y * fov.y
    # print('turn = (%f, %f)' % (turn.x, turn.y))

    cam_pan  = -turn.x
    cam_tilt = cam_center_angle + turn.y
    
    # Update the robot
    # ----------------
    if feedback:
        print('pan = %f, tilt = %f ' % (cam_pan, cam_tilt))
    if use_servo:
        cam_pan_cur += cam_pan
        cam_tilt_cur += cam_tilt
        if feedback:
            print('pan_cur = %f, tilt_cur = %f ' % (cam_pan_cur, cam_tilt_cur))
        
        # Clamp Pan/Tilt to 0 to 180 degrees
        # ----------------------------------
        cam_tilt_cur = max(0, min(130, cam_tilt_cur))
        pan(cam_pan_cur)
        tilt(cam_tilt_cur)
    else:
        if int(cam_pan) == 0:
            raspi.putNumber('shoot',  1)
        else:
            raspi.putNumber('shoot',  0)
            raspi.putNumber('pan',  int(cam_pan))
            raspi.putNumber('tilt', int(cam_tilt))

# ------------------------------------------------------------------------
# networktables
# ------------------------------------------------------------------------
def initNetworktables():
    logging.basicConfig(level=logging.DEBUG)         # to see messages from networktables
    NetworkTable.setIPAddress('127.0.0.1')
    # NetworkTable.setIPAddress('roborio-5260.local')
    NetworkTable.setClientMode()
    NetworkTable.initialize()
    return NetworkTable.getTable('Pi')
    
# ------------------------------------------------------------------------
# main capture/track/display loop
# ------------------------------------------------------------------------
def MainProgram():

    cam_stream = CameraStream(0)
    cam_stream.start()

    if use_servo:
        pan(cam_pan_cur)            # Turn the camera to the default position
        tilt(cam_tilt_cur)
        raspi = None
    else:
        raspi = initNetworktables()

    if method == 0:
        initFaceTracking()
    elif method == 1:
        # Initiate SIFT detector
        sift = cv2.xfeatures2d.SIFT_create()
        
        # find the keypoints and descriptors with SIFT
        img1 = cv2.imread('target.png', 0)
        if img1 and not img1.empty():
            kp1, desc1 = sift.detectAndCompute(img1, None)

    loop_cnt = 0
    
    while True:
        loop_cnt += 1
        
        ret, frame = cam_stream.getFrame()          # get a frame

        if ret == False:
            if loop_cnt < 10 or loop_cnt % 100 == 0:
                print("Error getting image")        # if we didn't get a frame, try again
            time.sleep(0.025)                       # sleep for 25 milliseconds so the image thread
                                                    # can get its image
            continue                                # maybe it was just a hiccup!

        if method == 0:
            img, p = findFace(frame)                # Do face detection
        elif method == 1 and img1 and not img1.empty():
            img, p = findTargetSift(frame, kp1, descl) # Do target detection using SIFT
        else:
            img, p = None, None
        
        if p:
            #cv2.circle(frame, p.asTuple(), 3, 255, -1)
            track(p, raspi)                         # Point the camera to the returned position

        # only on windows, show captured image in window
        # ----------------------------------------------
        if show_image:
            frame = img
        
            frame = cv2.resize(frame, (capsize * 1).asTuple())
            #frame = cv2.flip(frame, 1)

            cv2.imshow('Video', frame)              # Display the image, with rectangle

        # do sleep() after show_image
        # ---------------------------
        if p:
            cam_stream.nextFrame()                  # make sure we get a frame after the servo moved
            time.sleep(0.030)                       # sleep for a few milliseconds

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    # --------------------------------------------
    cam_stream.stop()
    cv2.destroyAllWindows()


# ------------------------------------------------------------------------
# just run main program
# ------------------------------------------------------------------------
MainProgram()
