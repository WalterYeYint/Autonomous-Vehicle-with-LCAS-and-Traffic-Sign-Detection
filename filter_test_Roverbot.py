import cv2
import numpy as np
import VisionUtils
from Utils import lineAngle, clamp
from collections import namedtuple
from time import time, sleep

def detecting_edges(img):
	# #Convert to grayscale then dilate and erode
	kernel = np.ones((3,3), np.uint8)
	gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	mask = cv2.inRange(gray, 210, 255)
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
	rho = 1  # distance precision in pixel, i.e. 1 pixel
	angle = np.pi / 200  # angular precision in radian, i.e. 1 degree
	min_threshold = 25  # minimal of votes
	lines = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold,
	                                np.array([]), minLineLength=20, maxLineGap=10)

	if lines is None: return []

	# If lines were found, combine them until you have 1 average for each 'direction' of tape in the photo
	lines = [line[0] for line in lines]
	combinedLines = self.__combineLines(lines)


	return combinedLines

def __combineLines(self, unsortedLines):
	""" Combines similar lines into one large 'average' line """

	maxAngle = 45
	minLinesForCombo = 5

	def getAngle(line):
		# Turn angle from -180:180 to just 0:180
		angle = lineAngle(line[:2], line[2:])
		if angle < 0: angle += 180
		return angle

	def lineFits(checkLine, combo):
		""" Check if the line fits within this group of combos by checking it's angle """
		checkAngle = getAngle(checkLine)
		for line in combo:
			angle = lineAngle(line[:2], line[2:])
			difference = abs(checkAngle - angle)

			if difference < maxAngle or 180 - difference < maxAngle:
				return True
			# if difference > maxAngle * 2 or 180 - difference > maxAngle * 2:
			#     return False
		return False

	# Pre-process lines so that lines always point from 0 degrees to 180, and not over
	for i, line in enumerate(unsortedLines):
		angle = lineAngle(line[:2], line[2:])
		if angle < 0:
			line = np.concatenate((line[2:], line[:2]))
			unsortedLines[i] = line


	# Get Line Combos
	lineCombos = []  # Format: [[[l1, l2, l3], [l4, l5, l6]], [[line 1...], [line 2...]]]

	while len(unsortedLines) > 0:
	    checkLine = unsortedLines.pop(0)

	    isSorted = False
	    for i, combo in enumerate(lineCombos):
	        if lineFits(checkLine, combo):
	            # Process the line so that the [x1, y1, and x2, y2] are in the same positions as other combos
	            lineCombos[i].append(checkLine.tolist())
	            isSorted = True
	            break

	    if not isSorted:
	        lineCombos.append([checkLine.tolist()])


	# # Limit each combo to minSamples, keeping only the longest lines
	# lineCombos = [sorted(combo, key= lambda c: (c[0] - c[2]) ** 2 + (c[1] - c[3]) ** 2, reverse=True)
	#               for combo in lineCombos]
	# lineCombos = [combo[:minLinesForCombo] for combo in lineCombos]


	# Filter and Average Combo Groups Format: [[L1], [L2], [L3]]
	averagedCombos = []
	for combo in lineCombos:
	    if len(combo) < minLinesForCombo: continue

	    avgLine = (np.sum(combo, axis=0) / len(combo)).astype(int)
	    avgLine *= 10  # Rescale to screen size
	    averagedCombos.append(Line(avgLine[:2], avgLine[2:]))


	# # Draw Line Combos and Final Lines
	# img = self.rover.camera.read()
	# for i, combo in enumerate(lineCombos):
	#     for x1, y1, x2, y2 in combo:
	#         x1 *= 10
	#         y1 *= 10
	#         x2 *= 10
	#         y2 *= 10
	#
	#         cv2.line(img, (x1, y1), (x2, y2), (80*i, 80*i, 80*i), 2)
	#
	# if len(averagedCombos):
	#     for p1, p2 in averagedCombos:
	#         x1 = p1[0]
	#         y1 = p1[1]
	#         x2 = p2[0]
	#         y2 = p2[1]
	#
	#         cv2.line(img, (x1, y1), (x2, y2), (80, 80, 80), 8)
	#
	# cv2.imshow('final', img)
	# cv2.waitKey(2500)

	return averagedCombos

def test_video(video_file):
    cap = cv2.VideoCapture(video_file)
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            #contour_image = detecting_contour(frame)
            canny_image = detecting_edges(frame)


            # contours_blk, hierarchy_blk = cv2.findContours(canny_image.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # if len(contours_blk) > 0:    
            #     blackbox = cv2.minAreaRect(contours_blk[0])
            #     (x_min, y_min), (w_min, h_min), ang = blackbox
            # height, width, _ = frame.shape
            # setpoint = width / 2
            # error = int(x_min - setpoint)
            # cv2.putText(canny_image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            # cv2.line(canny_image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)

            cropped_image = region_of_interest(canny_image)
            lines = detect_line_segments(cropped_image)
            averaged_lines = average_slope_intercept(frame, lines)
            lane_lines_image = display_lines(frame, averaged_lines)
            cv2.imshow("result", lane_lines_image)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()
    cv2.destroyAllWindows()
