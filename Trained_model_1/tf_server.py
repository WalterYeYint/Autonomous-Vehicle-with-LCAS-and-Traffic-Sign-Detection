# USAGE
# python server.py --prototxt MobileNetSSD_deploy.prototxt --model MobileNetSSD_deploy.caffemodel --montageW 2 --montageH 2
# To run tensorboard:
# tensorboard --logdir=/home/kan/'Important (Back these up)'/SDCP/


# import the necessary packages
from datetime import datetime
import imagezmq
import argparse
import imutils

import numpy as np
import os
import sys
import tensorflow as tf

from matplotlib import pyplot as plt
from PIL import Image

import cv2
import socket

# ## Object detection imports
# Here are the imports from the object detection module.
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

physical_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)


# Initialize UDP socket
UDP_IP = "192.168.1.121"
# UDP_IP = "192.168.43.85"
UDP_PORT = 5006
MESSAGE = 0
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # (Internet, UDP)


# # Model preparation 

# ## Variables
# 
# Any model exported using the `export_inference_graph.py` tool can be loaded here simply by changing `PATH_TO_CKPT` to point to a new .pb file.  
# 
# By default we use an "SSD with Mobilenet" model here. See the [detection model zoo](https://github.com/tensorflow/models/blob/master/object_detection/g3doc/detection_model_zoo.md) for a list of other models that can be run out-of-the-box with varying speeds and accuracies.


# # Path to frozen detection graph. This is the actual model that is used for the object detection.
# PATH_TO_CKPT = 'frozen_inference_graph.pb'

# # List of the strings that is used to add correct label for each box.
# PATH_TO_LABELS = 'label_map.pbtxt'

# NUM_CLASSES = 6

# PATH_TO_CKPT = 'frozen_inference_graph_216_imgs_31_btch.pb'
# PATH_TO_CKPT = 'frozen_inference_graph_362_imgs_31_btch.pb'
PATH_TO_CKPT = 'trained_graphs_&_label_maps/frozen_inference_graph_825_train_32_btch.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = 'trained_graphs_&_label_maps/label_map_204_imgs_3000_steps.pbtxt'

NUM_CLASSES = 8

font = cv2.FONT_HERSHEY_SIMPLEX


# ## Load a (frozen) Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')


# ## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)





# initialize the ImageHub object
imageHub = imagezmq.ImageHub()





# start looping over all the frames
with detection_graph.as_default():
    with tf.compat.v1.Session(graph=detection_graph) as sess:
        while True:
            # receive RPi name and frame from the RPi and acknowledge
            # the receipt
            (rpiName, frame) = imageHub.recv_image()
            imageHub.send_reply(b'OK')

            # resize the frame to have a maximum width of 400 pixels, then
            # grab the frame dimensions and construct a blob
            image_np = imutils.resize(frame, width=400)
            
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image_np, axis=0)
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            scores = detection_graph.get_tensor_by_name('detection_scores:0')
            classes = detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name('num_detections:0')
            # Actual detection.
            (boxes, scores, classes, num_detections) = sess.run(
                [boxes, scores, classes, num_detections],
                feed_dict={image_tensor: image_np_expanded})
            # Visualization of the results of a detection.
            vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                min_score_thresh=.8,
                max_boxes_to_draw=1,
                use_normalized_coordinates=True,
                line_thickness=8)
            
            largest_index = 1
            largest_score = 0
            min_score_thresh = 0.8

            for i in range(boxes.shape[0]):
                if scores[0][i] > largest_score:
                    largest_index = i
                    # boxes[i] is the box which will be drawn
            if scores[0][largest_index] >= min_score_thresh:
                cv2.putText(image_np, str(category_index[classes[0][largest_index]]["name"]), (20, 30), font, 1, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.putText(image_np, str(scores[0][largest_index]), (220, 30), font, 1, (0, 0, 255), 1, cv2.LINE_AA)
                print ("This class is gonna get used", classes[0][largest_index], scores[0][largest_index])
                sock.sendto(b'%f' % float(classes[0][largest_index]), (UDP_IP, UDP_PORT))



            # boxes = np.squeeze(boxes)
            # max_boxes_to_draw = boxes.shape[0]
            # scores = np.squeeze(scores)
            # min_score_thresh=.7
            # for i in range(min(max_boxes_to_draw, boxes.shape[0])):
                # if scores is None or scores[i] > min_score_thresh:
            #         # boxes[i] is the box which will be drawn
            #         print ("This box is gonna get used", boxes[i], scores[i])

            cv2.imshow('object detection', image_np)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                break

        # do a bit of cleanup
        cv2.destroyAllWindows()