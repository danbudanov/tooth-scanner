import numpy as np
import cv2
from matplotlib import pyplot as plt

import serial

CAMERA_DEV  = 1;
cap = cv2.VideoCapture(CAMERA_DEV)

ser = serial.Serial('/dev/ttyACM0')

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    rows, cols, channels = frame.shape
    # Find teeth ROI
    TEETH_THRESHOLD = 150
    ret, roi_map = cv2.threshold(gray, TEETH_THRESHOLD, 255, cv2.THRESH_BINARY)
    kernel_erode = np.ones((5,5), np.uint8)
    kernel_dilate = np.ones((9,9), np.uint8)

    #print "H: {}\tS: {}\tV: {}".format(hsv[rows/2, cols/2, 0], 
    #        hsv[rows/2, cols/2, 1], hsv[rows/2, cols/2, 2])
    #lower = np.array([22, 16, 0]);
    #upper = np.array([38, 30, 255])
    #roi_map = cv2.inRange(hsv, lower, upper)
    #cv2.imshow('Capture', roi_map);

    #roi_map = cv2.inRange

    # Expand ROI by opening to largest expandable area
    roi_map = cv2.erode(roi_map, kernel_erode, iterations=3)
    roi_map = cv2.dilate(roi_map, kernel_dilate, iterations=15) #7
    #cv2.imshow('Capture', roi_map)

    # Create contour from largest expandable area
    roi_map2, contours, hierarchy = cv2.findContours(
            roi_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE);
    #roi_map2 = np.uint8([roi_map, roi_map, roi_map])
    frame2 = cv2.drawContours(frame, contours, -1, (0, 0, 255), 3)
    print roi_map2.shape

    #cv2.imshow('Capture', roi_map2)
    #cv2.imshow('Capture', frame2)

    # Compute gradient inside ROI
    #cavity_gradient = cv2.Laplacian(gray, cv2.CV_64F)
    CAVITY_THRESHOLD = 30 #40
    ret, cavity_map = cv2.threshold(gray, CAVITY_THRESHOLD, 255,
            cv2.THRESH_BINARY_INV)
    kernel_cavity = np.ones((5,5), np.uint8)
    #cavity_map = cv2.erode(cavity_map, kernel_cavity, iterations=1)
    cavity_map = cv2.dilate(cavity_map, kernel_cavity, iterations=2)
    cavity_map2, contours_cavity, hierarchy = cv2.findContours(
            cavity_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE);
    cavity_contours_list = []
    for cnt_jaw in [contours[0]]:
        for cnt_cav in contours_cavity:
            M = cv2.moments(cnt_cav)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if (cv2.pointPolygonTest(cnt_jaw, (cx, cy), False) > 0):
                cavity_contours_list.append(cnt_cav)

    frame2 = cv2.drawContours(frame2, cavity_contours_list, 0, (0, 255, 255), 3)

    #cv2.imshow("Cavity", cavity_map)
    cv2.imshow("Cavity", frame2)

    if ser.in_waiting >= 6:
        ser_reading = ser.read(6)
        if (ser_reading == "Button"):
            cv2.imwrite("cavities.png", frame2)

    # If gradient above threshold, find contour of cavity
    
    # Display cavity

    # Display frame
    #cv2.imshow('Capture', hsv)
    #cv2.imshow('disparity', frameL)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
ser.close()
