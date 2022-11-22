import cv2
import imutils

import time
import numpy as np

try: 
    USE_PI_CAMERA = True
    from imutils.video.pivideostream import PiVideoStream
except ModuleNotFoundError:
    USE_PI_CAMERA = False
    from imutils.video.webcamvideostream import WebcamVideoStream


class VideoStream:
    def __init__(self, src=0, resolution=(320, 240), framerate=32):
        # check to see if the picamera module should be used
        if USE_PI_CAMERA:
            self.stream = PiVideoStream(resolution=resolution,
            framerate=framerate)
        # otherwise, we are using OpenCV so initialize the webcam
        # stream
        else:
            self.stream = WebcamVideoStream(src=src)
        
        time.sleep(3)  # Let the sensor warm-up
        self.stream.start()
            

    def start(self):
        # start the threaded video stream
        return self.stream.start()

    def update(self):
        # grab the next frame from the stream
        self.stream.update()

    def read(self):
        # return the current frame
        return self.stream.read()

    def stop(self):
        # stop the thread and release any resources
        self.stream.stop()

