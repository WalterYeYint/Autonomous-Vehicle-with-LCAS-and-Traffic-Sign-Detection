# USAGE
# python client.py --server-ip SERVER_IP

# import the necessary packages
from Camera import PiVideoStream
import imagezmq
import argparse
import socket
import time

# construct the argument parser and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-s", "--server-ip", required=True,
# 	help="ip address of the server to which the client will connect")
# args = vars(ap.parse_args())

# initialize the ImageSender object with the socket address of the
# server
# sender = imagezmq.ImageSender(connect_to="tcp://{}:5555".format(
	# args["server_ip"]))
sender = imagezmq.ImageSender(connect_to="tcp://{}:5555".format('192.168.1.114'))
# get the host name, initialize the video stream, and allow the
# camera sensor to warmup
rpiName = socket.gethostname()
image = PiVideoStream((320, 240), 32).start_camera_thread()
image.start_second_thread()
#vs = VideoStream(src=0).start()
time.sleep(2.0)
 
while True:
	# read the frame from the camera and send it to the server
	frame = image.read()
	sender.send_image(rpiName, frame)