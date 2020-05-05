import cv2
import numpy as np
import matplotlib.pyplot as plt

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
    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # cv2.imshow("hsv image", hsv)

    # lower_white = np.array([0, 0, 200])
    # upper_white = np.array([180, 100, 255])
    # mask = cv2.inRange(hsv, lower_white, upper_white)

    # lower_red = np.array([150, 100, 0])
    # upper_red = np.array([180, 255, 255])
    # mask = cv2.inRange(hsv, lower_red, upper_red)

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

def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 130  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
                                    np.array([]), minLineLength=8, maxLineGap=4)
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
    return averaged_lines

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def calculate_offset(frame, line_segments):
    contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours_blk) > 0:    
        blackbox = cv2.minAreaRect(contours_blk[0])
        (x_min, y_min), (w_min, h_min), ang = blackbox

def test_video(video_file):
    cap = cv2.VideoCapture(video_file)
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            #contour_image = detecting_contour(frame)
            canny_image = detecting_edges(frame)
            # cropped_image = region_of_interest(canny_image)
            # detecting_contour(cropped_image, frame)
            
            # lines = detect_line_segments(cropped_image)
            # averaged_lines = average_slope_intercept_middle_line(frame, lines)
            # lane_lines_image = display_lines(frame, averaged_lines)
            cv2.imshow("result", canny_image)
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

    # averaged_lines = detect_lane(lane_image)
    # print(averaged_lines)
    # lane_lines_image = display_lines(lane_image, averaged_lines)
    # cv2.imshow("lane lines", lane_lines_image)
    # cv2.waitKey(0)


    ######################################################################################
    #For testing bit by bit

    canny_image = detecting_edges(lane_image)
    cropped_image = region_of_interest(canny_image)
    detecting_contour(cropped_image, lane_image)
    #canny_image = detecting_edges_grayscale(lane_image)
    cv2.imshow("original", lane_image)
    cv2.waitKey(0)


#test_photo('test_lane.jpeg')
#test_photo('white_line.jpg')
#test_video('white_line_2.mp4')
# test_video('black_line.mp4')
test_video('black_line_2.mp4')
# test_video('red_line.mp4')