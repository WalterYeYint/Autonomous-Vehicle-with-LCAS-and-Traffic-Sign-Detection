import cv2
import numpy as np
#import matplotlib.pyplot as plt
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

def make_points(image, line):
    slope, intercept = line
    y1 = int(image.shape[0])# bottom of the image
    y2 = int(y1*3/5)         # slightly lower than the middle
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return [[x1, y1, x2, y2]]

def average_slope_intercept(image, lines):
    left_fit    = []
    right_fit   = []
    if lines is None:
        return None
    for line in lines:
        for x1, y1, x2, y2 in line:
            fit = np.polyfit((x1,x2), (y1,y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
    if len(left_fit) and len(right_fit):
        left_fit_average  = np.average(left_fit, axis=0)
        right_fit_average = np.average(right_fit, axis=0)
        left_line  = make_points(image, left_fit_average)
        right_line = make_points(image, right_fit_average)
        averaged_lines = [left_line, right_line]
        return averaged_lines

def detecting_edges(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([130, 70, 0])
    upper_red = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    #lower_red = np.array([0, 40, 40])
    #upper_red = np.array([10, 100, 100])
    #mask2 = cv2.inRange(hsv, lower_red, upper_red)
    
    #mask = mask + mask2
    #lower_blue = np.array([60, 190, 40])
    #upper_blue = np.array([120, 255, 255])
    #mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # #Convert to grayscale
    # gray = cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY)
    #
    # kernel = 5
    # #Gaussian Blur for smoothing (otherwise, the picture could bring false edges)
    # blur = cv2.GaussianBlur(gray, (kernel, kernel), 0)
    # Canny(img, lower_threshold, upper_threshold)
    #Draws edge if beyond upper_threshold, not if below lower_threshold, draws only if edge is connected to strong edge if between
    canny = cv2.Canny(mask, 50, 150)
    return canny

def display_lines(img,lines):
    line_image = np.zeros_like(img)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)
    return line_image

def region_of_interest(canny):
    height = canny.shape[0]
    width = canny.shape[1]
    mask = np.zeros_like(canny)

    triangle = np.array([[
    (200, height),
    (550, 250),
    (1100, height),]], np.int32)

    cv2.fillPoly(mask, triangle, 255)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image

def video_live():
    camera = PiCamera()
    camera.resolution = (320, 240)
    camera.rotation = 180
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(320, 240))

    # allow the camera to warmup
    time.sleep(0.1)
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        canny_image = detecting_edges(image)

        # show the frame
        cv2.imshow("Frame", canny_image)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
                break

video_live()
#
# image_re = cv2.imread('test_lane.jpeg')
# frame = np.copy(image_re)
# canny_image = canny(frame)
# cropped_canny = region_of_interest(canny_image)
# lines = cv2.HoughLinesP(cropped_canny, 2, np.pi/180, 100, np.array([]), minLineLength=40,maxLineGap=5)
# averaged_lines = average_slope_intercept(frame, lines)
# line_image = display_lines(frame, averaged_lines)
# combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
#
# cv2.imshow("result", combo_image)
# cv2.waitKey(0)
