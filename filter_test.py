import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
from threading import Thread

curr_steering_angle = 90

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

    lower_red = np.array([150, 100, 100])
    upper_red = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # if you need full red color range uncomment the following 4 line (not helpful though)
    # lower_red_2 = np.array([0, 100, 0])
    # upper_red_2 = np.array([30, 255, 255])
    # mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    # mask = cv2.bitwise_or(mask, mask_2)

    # lower_blue = np.array([60, 40, 40])
    # upper_blue = np.array([150, 255, 255])
    # mask = cv2.inRange(hsv, lower_blue, upper_blue)

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
    return canny

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

def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 40  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                    maxLineGap=4)
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        #logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                #logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_coordinates(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_coordinates(frame, right_fit_average))

    #logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines

def average_slope_intercept_middle_line(frame, line_segments):
    lane_lines = []
    if line_segments is None:
        #logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    middle_fit = []

    boundary = 1/3

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                #logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            middle_fit.append((slope, intercept))

    middle_fit_average = np.average(middle_fit, axis=0)
    if len(middle_fit) > 0:
        lane_lines.append(make_coordinates(frame, middle_fit_average))

    #logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines

def make_coordinates(image, line_parameters):
    height, width, _ = image.shape
    if isinstance(line_parameters, np.float64):
    	slope = 0.0001
    	intercept = 0.00
    else:
        slope, intercept = line_parameters
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]



def detect_lane(lane_image):
    canny_image = detecting_edges(lane_image)
    cropped_image = region_of_interest(canny_image)
    lines = detect_line_segments(cropped_image)
    averaged_lines = average_slope_intercept(lane_image, lines)
    lane_lines_image = display_lines(lane_image, averaged_lines)
    return averaged_lines, lane_lines_image

def calculate_offset(frame, line_segments):
    contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours_blk) > 0:    
        blackbox = cv2.minAreaRect(contours_blk[0])
        (x_min, y_min), (w_min, h_min), ang = blackbox

def steer(frame, lane_lines, curr_steering_angle):
        if len(lane_lines) == 0:
            logging.error('No lane lines detected, nothing to do.')
            return frame

        new_steering_angle = compute_steering_angle(frame, lane_lines)
        # curr_steering_angle = stabilize_steering_angle(curr_steering_angle, new_steering_angle, len(lane_lines))
        # cv2.putText(frame,'STA: {0:.2f}'.format(curr_steering_angle),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
        # curr_heading_image = display_heading_line(frame, curr_steering_angle)
        cv2.putText(frame,'STA: {0:.2f}'.format(new_steering_angle),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
        curr_heading_image = display_heading_line(frame, new_steering_angle)

        #cv2.imshow("heading", curr_heading_image)

        return curr_heading_image

def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        # camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        camera_mid_offset_percent = 0.00
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    return steering_angle


def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane

    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    return stabilized_steering_angle


############################
# Utility Functions
############################
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image







def test_video(video_file):
# def test_video():
    cap = cv2.VideoCapture(video_file)
    # cap = cv2.VideoCapture('red_lane.mp4')
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            # contour_image = detecting_contour(frame)
            averaged_lines, lane_lines_image = detect_lane(frame)
            # canny_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # canny_image = detecting_edges(frame)
            # cropped_image = region_of_interest(canny_image)
            # detecting_contour(cropped_image, frame)
            
            # lines = detect_line_segments(cropped_image)
            # averaged_lines = average_slope_intercept_middle_line(frame, lines)
            final_image = steer(lane_lines_image, averaged_lines, curr_steering_angle)
            # cv2.imshow("result", canny_image)
            cv2.imshow("result2", final_image)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()
    cv2.destroyAllWindows()


def test_photo(photo_file):
    #image = cv2.imread('test_lane.jpeg')
    image = cv2.imread(photo_file)
    image_re = cv2.resize(image, (960, 721))
    lane_image = np.copy(image_re)

    # lane_lines_image = detect_lane(lane_image)
    # cv2.imshow("lane lines", lane_lines_image)
    # cv2.waitKey(0)


    ######################################################################################
    #For testing bit by bit
    hsv = cv2.cvtColor(lane_image, cv2.COLOR_BGR2HSV)
    canny_image = detecting_edges(lane_image)
    cropped_image = region_of_interest(canny_image)
    # detecting_contour(cropped_image, lane_image)
    #canny_image = detecting_edges_grayscale(lane_image)
    cv2.imshow("hsv", hsv)
    cv2.imshow("original", canny_image)
    cv2.waitKey(0)


#test_photo('test_lane.jpeg')
# test_photo('white_line.jpg')
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
test_photo('red_line_IRcam3.jpg')
# test_photo('white_line_IRcam.jpg')
# test_photo('white_line_IRcam2.jpg')
# test_video('red_lane.mp4')