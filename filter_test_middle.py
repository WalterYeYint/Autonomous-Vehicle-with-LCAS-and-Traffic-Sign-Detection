import cv2
import numpy as np
import matplotlib.pyplot as plt

cap = cv2.VideoCapture('white_line_2.mp4')
while(cap.isOpened()):
	ret, frame = cap.read()
	if ret == True:
		gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
		Blackline = cv2.inRange(gray, 210, 255)
		#Blackline = cv2.inRange(frame, (0,0,0), (60,60,60))
		kernel = np.ones((3,3), np.uint8)
		Blackline = cv2.erode(Blackline, kernel, iterations=5)
		Blackline = cv2.dilate(Blackline, kernel, iterations=9)
		contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		if len(contours_blk) > 0:	 
			blackbox = cv2.minAreaRect(contours_blk[0])
			(x_min, y_min), (w_min, h_min), ang = blackbox
			if ang < -45 :
				ang = 90 + ang
			if w_min < h_min and ang > 0:	  
				ang = (90-ang)*-1
			if w_min > h_min and ang < 0:
				ang = 90 + ang	  
			setpoint = 320
			error = int(x_min - setpoint) 
			ang = int(ang)	 
			box = cv2.boxPoints(blackbox)
			box = np.int0(box)
			cv2.drawContours(frame,[box],0,(0,0,255),3)	 
			cv2.putText(frame,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
			cv2.putText(frame,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			cv2.line(frame, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)


		cv2.imshow("orginal with line", frame)	
		key = cv2.waitKey(1) & 0xFF	
		if key == ord("q"):
			break
	else:
		break
cap.release()
cv2.destroyAllWindows()

		