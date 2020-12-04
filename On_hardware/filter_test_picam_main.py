#testing the lane detection using images
# To connect to the robot : ssh pi@192.168.1.121
# For sending files to the robot : 
# scp filter_test_picam_main.py Camera.py client.py pi@192.168.1.121:~/Thesis-Pi-2nd



import cv2
import numpy as np
import logging
import math
import datetime
import sys
import time
from Camera import PiVideoStream

import serial
import time

# define serial variable for communication
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)   #Important: wait for serial at least 5 secs, otherwise false data


####################################################################################
# Function for transferring data from Pi to Arduino
####################################################################################
def transfer_data(offset, traffic_class):
    ser.write(b'%f\t%f\t%f\t%f\t' % (float(offset), float(traffic_class), 10, 10))

def detecting_contour(img, frame):
    contours_blk, hierarchy_blk = cv2.findContours(img.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_blk) > 0:    
        blackbox = cv2.minAreaRect(contours_blk[0])     # minAreaRect( center (x,y), (width, height), angle of rotation )
        (x_min, y_min), (w_min, h_min), ang = blackbox
    height, width, _ = frame.shape
    setpoint = width / 2
    error = int(x_min + w_min/2 - setpoint)
    cv2.putText(frame,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    cv2.line(frame, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
    box = cv2.boxPoints(blackbox)
    box = np.int0(box)      #to convert from float to int
    cv2.drawContours(frame,[box],0,(0,0,255),3)  
    # cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_min+w_min), int(y_min+h_min)), (0,255,0),3)
    #cv2.rectangle(frame, (100, 100), (200, 200),(0,255,0),3)

def warp(img, src, dst):
	height, width, _ = img.shape
	
	# Perspective Transform matrix and inverse matrix
	M = cv2.getPerspectiveTransform(src, dst)
	Minv = cv2.getPerspectiveTransform(dst, src)
	warped = cv2.warpPerspective(img, M, (width, height), flags=cv2.INTER_LINEAR)

	return warped, Minv

def detecting_edges(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # cv2.imshow("hsv image", hsv)

    # lower_white = np.array([80, 0, 100])
    # upper_white = np.array([110, 50, 255])
    # mask = cv2.inRange(hsv, lower_white, upper_white)

    # lower_red = np.array([150, 50, 80])
    # upper_red = np.array([180, 255, 255])
    # mask = cv2.inRange(hsv, lower_red, upper_red)

    # if you need full red color range uncomment the following 4 line (not helpful though)
    # lower_red_2 = np.array([0, 100, 0])
    # upper_red_2 = np.array([30, 255, 255])
    # mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    # mask = cv2.bitwise_or(mask, mask_2)

    lower_blue = np.array([80, 100, 0])
    upper_blue = np.array([140, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # #Convert to grayscale then dilate and erode
    kernel = np.ones((3,3), np.uint8)

    # mask for black line
    # mask = cv2.inRange(img, (50,70,0), (100,100,255))
    # gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # mask = cv2.inRange(gray, 0, 100)

    # mask for white line
    # gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # mask = cv2.inRange(gray, 210, 255)

    # Original erosion and dilation
    mask = cv2.erode(mask, kernel, iterations=3)
    mask = cv2.dilate(mask, kernel, iterations=13)

    #cv2.imshow("mask image", mask)

    #
    # kernel = 5
    # #Gaussian Blur for smoothing (otherwise, the picture could bring false edges)
    # blur = cv2.GaussianBlur(gray, (kernel, kernel), 0)
    # Canny(img, lower_threshold, upper_threshold)
    #Draws edge if beyond upper_threshold, not if below lower_threshold, draws only if edge is connected to strong edge if between
    canny = cv2.Canny(mask, 50, 150)
    return mask

def region_of_interest(canny):
    height = canny.shape[0]
    width = canny.shape[1]

    #Create a 2D array of zeros like 'image'
    mask = np.zeros_like(canny)

    #Create an array with triangular region of interest with vertices as input
    #polygons = np.array([[(0, height-10), (1000, height-10), (500, 400)]])
    triangle = np.array([[(0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),]], np.int32)

    #draw the filled triangle from 'polygons' into the 'mask'
    cv2.fillPoly(mask, triangle, 255)
    #bitwise_and for filtering 'mask' using 'polygons' as kernel
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image


def find_contours(image, lane_image):
    _, contours, hierarchy = cv2.findContours(image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    box_dim = []
    for c in contours:
        # if cv2.contourArea(c) <= 50 :
        #     continue    
        # x,y,w,h = cv2.boundingRect(c)

        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box_dim.append(box)
        # print (box)
        box = np.array(box).reshape((-1,1,2)).astype(np.int32)
        # box = np.int0([x,y,w,h])

    #     cv2.drawContours(lane_image, [box], -1, (0, 0, 255), 3)  
    #     # cv2.rectangle(lane_image, (x, y), (x + w, y + h), (0, 255,0), 2)
    #     # center = (x,y)
        
    # cv2.imshow('test',lane_image)
    return box_dim

def sort_contours(box_dim, frame):
    left_box = []
    right_box = []
    height, width, _ = frame.shape
    for p in box_dim:
        p1,p2,p3,p4 = p
        # print(p1[1],p2[1],p3[1],p4[1])
        if(p1[0] <= width/2):
            left_box.append(p)
        elif(p1[0] > width/2):
            right_box.append(p)
    
    # print("Left box=")
    # print(left_box)
    # print("Right box=")
    # print(right_box)

    
    # left_box = np.array(left_box).reshape((-1,1,2)).astype(np.int32)          #This line is needed to drawContours the box
    # right_box = np.array(right_box).reshape((-1,1,2)).astype(np.int32)          #This line is needed to drawContours the box
    # cv2.drawContours(frame, [left_box], -1, (0, 0, 255), 3) 
    # cv2.drawContours(frame, [right_box], -1, (0, 0, 255), 3) 

    if(len(left_box) > 0):
        left_box = max(left_box, key = cv2.contourArea)
        left_box = np.array(left_box).reshape((-1,1,2)).astype(np.int32)            #This line is needed to drawContours the box
        # cv2.drawContours(frame, [left_box], -1, (0, 0, 255), 3) 
    if(len(right_box) > 0):
        right_box = max(right_box, key = cv2.contourArea)
        right_box = np.array(right_box).reshape((-1,1,2)).astype(np.int32)          #This line is needed to drawContours the box
    #     cv2.drawContours(frame, [right_box], -1, (0, 0, 255), 3) 
    # cv2.imshow('test',frame)
    

    # print("Left box=")
    # print(left_box)
    # print("Right box=")
    # print(right_box)

    
    return left_box, right_box

def calc_midpoints(left_box, right_box, frame):
    height, width, _ = frame.shape
    left_cx, left_cy, right_cx, right_cy = 0, 0, 0, 0

    if(len(left_box) > 0):
        left_moment = cv2.moments(left_box)
        if (left_moment["m00"] != 0):
            # Get the center x.
            left_cx = int(left_moment["m10"]/left_moment["m00"])
            # Get the center y.
            left_cy = int(left_moment["m01"]/left_moment["m00"])
        else:
            left_cx, left_cy = 0, 0

    if(len(right_box) > 0):
        right_moment = cv2.moments(right_box)
        if (right_moment["m00"] != 0):
            # Get the center x.
            right_cx = int(right_moment["m10"]/right_moment["m00"])
            # Get the center y.
            right_cy = int(right_moment["m01"]/right_moment["m00"])
        else:
            right_cx, right_cy = width, 0
    
    if(len(left_box) == 0):
        left_cx, left_cy = 0, right_cy
    elif(len(right_box) == 0):
        right_cx, right_cy = width, left_cy

    # cv2.circle(frame, (left_cx, left_cy), 5, (0, 0, 255), 3)
    # cv2.circle(frame, (right_cx, right_cy), 5, (0, 0, 255), 3)

    # if(len(left_box) > 0):           #This line is needed to drawContours the box
    #     cv2.drawContours(frame, [left_box], -1, (0, 0, 255), 3) 
    # if(len(right_box) > 0):         #This line is needed to drawContours the box
    #     cv2.drawContours(frame, [right_box], -1, (0, 0, 255), 3) 

    # print(left_cx, left_cy)
    # print(right_cx, right_cy)

    center_pt = int((left_cx + right_cx)/2)
    # cv2.circle(frame, (center_pt, right_cy), 5, (0, 0, 255), 3)
    
    # cv2.line(frame, (int(width/2), height), (int(width/2), 0), (0, 255, 0), 3)
    # print(height)

    # cv2.imshow('test',frame)

    return center_pt, left_cx, left_cy, right_cx, right_cy

def calc_offset(center_pt, frame):
    height, width, _ = frame.shape
    offset = (center_pt - (width/2)) 
    # offset = ((width/2) - center_pt) 
    return offset

############################
# Test Functions
############################
def test_photo(file):
    lane_follower = HandCodedLaneFollower()
    frame_1 = cv2.imread(file)
    #frame_flipped = cv2.flip(frame_1, -1)
    frame = cv2.resize(frame_flipped, (960, 721))
    combo_image = lane_follower.follow_lane(frame)
    show_image('final', combo_image, True)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def video_live():
    curr_angle = 90
    offset = 0
    data = 9
    image = PiVideoStream((320, 240), 32).start_camera_thread()
    # image.start_second_thread()
    # image.start_third_thread()

    # allow the camera to warmup
    time.sleep(8)
    # capture frames from the camera
    while True:
        frame = image.read()
        height, width, _ = frame.shape
        #Source points taken from images with straight lane lines, these are to become parallel after the warp transform
        src_offset = width * 0.2
        src = np.float32([
            [0+src_offset, height/2], # top-left corner
            [width-src_offset, height/2], # top-right corner
            [width, height], # bottom-right corner
            [0, height], # bottom-left corner
        ])
        # for x in range(0,4):
        #     cv2.circle(frame, (src[0,0], src[0,1]), 5, (0, 255, 0), 3)

        # Destination points are to be parallel, taking into account the image size
        dst = np.float32([
            [0, 0], # top-left corner
            [width, 0], # top-right corner
            [width, height], # bottom-right corner
            [0, height], # bottom-left corner
        ])

        warped_frame, Minv = warp(frame, src, dst)


        canny_image = detecting_edges(warped_frame)
        # cropped_image = region_of_interest(canny_image)
        box_dim = find_contours(canny_image, warped_frame)
        # print(box_dim)
        # box_center = calc_midpoints(box_dim, frame)
        left_box, right_box = sort_contours(box_dim, warped_frame)
        center_pt, left_cx, left_cy, right_cx, right_cy = calc_midpoints(left_box, right_box, warped_frame)
        # print(center_pt, left_cx, left_cy, right_cx, right_cy)
        offset = calc_offset(center_pt, warped_frame)
        print(offset)
        
        # # #displaying data on image
        # cv2.circle(warped_frame, (left_cx, left_cy), 5, (0, 0, 255), 3)
        # cv2.circle(warped_frame, (right_cx, right_cy), 5, (0, 0, 255), 3)

        # if(len(left_box) > 0):           #This line is needed to drawContours the box
        #     cv2.drawContours(warped_frame, [left_box], -1, (0, 0, 255), 3) 
        # if(len(right_box) > 0):         #This line is needed to drawContours the box
        #     cv2.drawContours(warped_frame, [right_box], -1, (0, 0, 255), 3) 
        
        # cv2.circle(warped_frame, (center_pt, right_cy), 5, (0, 0, 255), 3)
        # cv2.line(warped_frame, (int(width/2), height), (int(width/2), 0), (0, 255, 0), 3)
        # cv2.putText(warped_frame,'STA: {0:.2f}'.format(offset),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
        # cv2.imshow('test',warped_frame)
        # cv2.imshow("original result", warped_frame)
        # cv2.imshow("cropped result", cropped_image)

        data = float(image.get_data())
        # if data==6 or data==2:
        #     # print("received message: %f" % data)

        transfer_data(offset, data)


        # show the frame
        # cv2.imshow("hsv", hsv)
        # cv2.imshow("canny result", canny_image)
        # cv2.imshow("result2", final_image)
        
        # if the `q` key was pressed, break from the loop
        if cv2.waitKey(10) & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()
    image.close()

if __name__ == '__main__':
    #logging.basicConfig(level=logging.INFO)

    #test_video('/home/pi/DeepPiCar/driver/data/tmp/video01')
    #test_photo('red_line.jpg')
    #test_photo(sys.argv[1])
    #test_video(sys.argv[1])
    
    video_live()
