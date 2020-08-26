import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
from threading import Thread

import serial
import time

#define serial variable for communication
# ser = serial.Serial('/dev/ttyACM0', 9600)

# time.sleep(7)   #Important: wait for serial at least 5 secs, otherwise false data

####################################################################################
# Function for transferring data from Pi to Arduino
####################################################################################
def transfer_data(offset):
    ser.write(b'%f\t%f\t%f\t%f\t' % (float(offset), 10, 20, 30))

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
    mask = cv2.erode(mask, kernel, iterations=5)
    mask = cv2.dilate(mask, kernel, iterations=9)

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
    contours, hierarchy = cv2.findContours(image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
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
    return offset

def test_video(video_file):
# def test_video():
    curr_angle = 90
    cap = cv2.VideoCapture(video_file)
    # cap = cv2.VideoCapture('red_lane.mp4')
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            # contour_image = detecting_contour(frame)
            # averaged_lines, lane_lines_image = detect_lane(frame)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            canny_image = detecting_edges(frame)
            cropped_image = region_of_interest(canny_image)
            box_dim = find_contours(cropped_image, frame)
            # print(box_dim)
            # box_center = calc_midpoints(box_dim, frame)
            left_box, right_box = sort_contours(box_dim, frame)
            center_pt, left_cx, left_cy, right_cx, right_cy = calc_midpoints(left_box, right_box, frame)
            offset = calc_offset(center_pt, frame)
            
            #displaying data on image
            height, width, _ = frame.shape
            cv2.circle(frame, (left_cx, left_cy), 5, (0, 0, 255), 3)
            cv2.circle(frame, (right_cx, right_cy), 5, (0, 0, 255), 3)

            if(len(left_box) > 0):           #This line is needed to drawContours the box
                cv2.drawContours(frame, [left_box], -1, (0, 0, 255), 3) 
            if(len(right_box) > 0):         #This line is needed to drawContours the box
                cv2.drawContours(frame, [right_box], -1, (0, 0, 255), 3) 
            
            cv2.circle(frame, (center_pt, right_cy), 5, (0, 0, 255), 3)
            cv2.line(frame, (int(width/2), height), (int(width/2), 0), (0, 255, 0), 3)
            cv2.putText(frame,'STA: {0:.2f}'.format(offset),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

            cv2.imshow('test',frame)



            # lines = detect_line_segments(cropped_image)
            # averaged_lines = average_slope_intercept_middle_line(frame, lines)
            # final_image, curr_angle = steer(lane_lines_image, averaged_lines, curr_angle)
            # print(curr_angle)
            # transfer_data(curr_angle)
            # cv2.imshow("result", hsv)
            cv2.imshow("result3", cropped_image)
            # cv2.imshow("result2", final_image)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()
    cv2.destroyAllWindows()


def test_photo(photo_file):
    curr_angle = 90
    #image = cv2.imread('test_lane.jpeg')
    image = cv2.imread(photo_file)
    # image_re = cv2.resize(image, (960, 721))
    image_re = cv2.resize(image, (320, 240))
    frame = np.copy(image_re)

    # lane_lines_image = detect_lane(lane_image)
    # cv2.imshow("lane lines", lane_lines_image)
    # cv2.waitKey(0)


    ######################################################################################
    #For testing bit by bit
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    canny_image = detecting_edges(frame)
    cropped_image = region_of_interest(canny_image)
    box_dim = find_contours(cropped_image, frame)
    # print(box_dim)
    left_box, right_box = sort_contours(box_dim, frame)
    center_pt, left_cx, left_cy, right_cx, right_cy = calc_midpoints(left_box, right_box, frame)
    offset = calc_offset(center_pt, frame)
            
    #displaying data on image
    height, width, _ = frame.shape
    cv2.circle(frame, (left_cx, left_cy), 5, (0, 0, 255), 3)
    cv2.circle(frame, (right_cx, right_cy), 5, (0, 0, 255), 3)

    if(len(left_box) > 0):           #This line is needed to drawContours the box
        cv2.drawContours(frame, [left_box], -1, (0, 0, 255), 3) 
    if(len(right_box) > 0):         #This line is needed to drawContours the box
        cv2.drawContours(frame, [right_box], -1, (0, 0, 255), 3) 
    
    cv2.circle(frame, (center_pt, right_cy), 5, (0, 0, 255), 3)
    cv2.line(frame, (int(width/2), height), (int(width/2), 0), (0, 255, 0), 3)
    cv2.putText(frame,'STA: {0:.2f}'.format(offset),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

    cv2.imshow('test',frame)

    
    #canny_image = detecting_edges_grayscale(lane_image)
    
    # averaged_lines, lane_lines_image = detect_lane(lane_image)
    # final_image, curr_angle = steer(lane_lines_image, averaged_lines, curr_angle)

    # cv2.imshow("Original image", lane_image)
    cv2.imshow("hsv image", hsv)
    # cv2.imshow("test", hsv[:,:,2])
    cv2.imshow("Canny image", canny_image)
    # cv2.imshow("Original image", image_re)
    # cv2.imshow("final image", final_image)

    # plt.imshow(image_re)
    # plt.imshow(cv2.cvtColor(image_re, cv2.COLOR_BGR2RGB))
    # plt.show()

    cv2.waitKey(0)


#test_photo('test_lane.jpeg')
# test_photo('white_line.jpg')
# test_photo('black_white_line.jpg')
# test_video('white_line.mp4')
# test_video('white_line_2.mp4')
# test_video('black_line.mp4')
# test_video('black_line_2.mp4')
# test_video('black_line_night.mp4')
# test_photo('red_line.jpg')
# test_video('red_line.mp4')
# test_video('red_line_2.mp4')
# test_video('red_line_night.mp4')
# test_photo('red_line_IRcam2.jpg')
# test_photo('red_line_IRcam.jpg')
# test_photo('red_line_IRcam3.jpg')
# test_photo('white_line_IRcam.jpg')
# test_photo('white_line_IRcam2.jpg')
# test_video('red_lane.mp4')
# test_video('red_lane_electrictape.mp4')
# test_video('red_lane_phcam.mp4')
# test_video('blue_lane.mp4')
# test_photo('Drawing.jpeg')
# test_photo('autodraw 8_25_2020.png')
# test_video('/home/kan/Videos/SpeedLimitTestSuccess2.m4v')
# test_photo('blue_lane.jpg')
# test_photo('blue_lane_2.jpg')
# test_photo('blue_lane_3.jpg')
# test_photo('blue_lane_4.jpg')
# test_photo('blue_lane_5.jpg')
test_video('Curved_lane.mp4')