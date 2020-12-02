

import cv2
from imutils.video import WebcamVideoStream
import time
import threading

cam = WebcamVideoStream(src="https://192.168.1.103:8080/video").start()

cv2.namedWindow("test")

img_counter = 0
frame = cam.read()

def snapshot_thread():
    while True:
        global img_counter
        ### Take snapshots periodically
        img_name = "opencv_frame_{}.png".format(img_counter)
        cv2.imwrite(img_name, frame)
        print("{} written!".format(img_name))
        img_counter += 1
        time.sleep(5)


t = threading.Thread(target=snapshot_thread, name='snapshot_thread', args=())
t.start()

while True:
    frame = cam.read()
    cv2.imshow("test", frame)

    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break

    ### Take snapshots by typing spacebar
    # elif k%256 == 32:
    #     # SPACE pressed
    #     img_name = "opencv_frame_{}.png".format(img_counter)
    #     cv2.imwrite(img_name, frame)
    #     print("{} written!".format(img_name))
    #     img_counter += 1

cam.release()

cv2.destroyAllWindows()