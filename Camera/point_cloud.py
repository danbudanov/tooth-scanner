import numpy as np
import cv2
from matplotlib import pyplot as plt

CAMERA_LEFT  = 1;
CAMERA_RIGHT = 2;
capL = cv2.VideoCapture(CAMERA_LEFT)
capR = cv2.VideoCapture(CAMERA_RIGHT)

while(True):
    # Capture frame-by-frame
    ret, frameL = capL.read()
    ret, frameR = capR.read()
    
    #stereo = cv2.createStereoBM(numDisparities=16, blockSize=15)
    stereo = cv2.StereoBM_create(numDisparities=16 * 15)

    grayL= cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)
    disparity = stereo.compute(grayL, grayR)
    #disparity = (disparity - 127) * 255
    #disparity = (disparity + 127)

    # Display frame
    cv2.imshow('disparity', disparity)
    #cv2.imshow('disparity', frameL)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
capL.release()
capR.release()
cv2.destroyAllWindows()
