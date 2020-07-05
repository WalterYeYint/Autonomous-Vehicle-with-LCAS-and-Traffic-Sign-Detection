import numpy as np
import os
import sys
import tensorflow as tf

from matplotlib import pyplot as plt
from PIL import Image

import cv2


# ## Object detection imports
# Here are the imports from the object detection module.

# In[3]:

from object_detection.utils import label_map_util

from object_detection.utils import visualization_utils as vis_util

physical_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], True)


# # Model preparation 

# ## Variables
# 
# Any model exported using the `export_inference_graph.py` tool can be loaded here simply by changing `PATH_TO_CKPT` to point to a new .pb file.  
# 
# By default we use an "SSD with Mobilenet" model here. See the [detection model zoo](https://github.com/tensorflow/models/blob/master/object_detection/g3doc/detection_model_zoo.md) for a list of other models that can be run out-of-the-box with varying speeds and accuracies.

# In[4]:

# # Path to frozen detection graph. This is the actual model that is used for the object detection.
# PATH_TO_CKPT = 'frozen_inference_graph.pb'

# # List of the strings that is used to add correct label for each box.
# PATH_TO_LABELS = 'label_map.pbtxt'

# NUM_CLASSES = 6

PATH_TO_CKPT = 'frozen_inference_graph_216_imgs_31_btch.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = 'label_map_204_imgs_3000_steps.pbtxt'

NUM_CLASSES = 8

font = cv2.FONT_HERSHEY_SIMPLEX


# ## Load a (frozen) Tensorflow model into memory.

# In[6]:

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.compat.v1.GraphDef()
    with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')


# ## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine

# In[7]:

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)


# ## Helper code

# In[8]:

def load_image_into_numpy_array(image):
    (im_width, im_height) = image.size
    return np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)


# # Detection


# In[10]:
def test_video(video_file):
# def test_video():
    cap = cv2.VideoCapture(video_file)
    with detection_graph.as_default():
        with tf.compat.v1.Session(graph=detection_graph) as sess:
            while(cap.isOpened()):
                ret, image_np = cap.read()
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
                    cv2.putText(image_np, str(category_index[classes[0][largest_index]]["name"]), (100, 100), font, 2, (0, 0, 255), 2, cv2.LINE_AA)
                    cv2.putText(image_np, str(scores[0][largest_index]), (800, 100), font, 2, (0, 0, 255), 2, cv2.LINE_AA)
                    print ("This class is gonna get used", classes[0][largest_index], scores[0][largest_index])



                # boxes = np.squeeze(boxes)
                # max_boxes_to_draw = boxes.shape[0]
                # scores = np.squeeze(scores)
                # min_score_thresh=.7
                # for i in range(min(max_boxes_to_draw, boxes.shape[0])):
                    # if scores is None or scores[i] > min_score_thresh:
                #         # boxes[i] is the box which will be drawn
                #         print ("This box is gonna get used", boxes[i], scores[i])

                cv2.imshow('object detection', cv2.resize(image_np, (800,600)))
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break
            cap.release()
            cv2.destroyAllWindows()


def test_photo(photo_file):
    with detection_graph.as_default():
        with tf.compat.v1.Session(graph=detection_graph) as sess:
            image_np = cv2.imread(photo_file)
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
                max_boxes_to_draw=3,
                use_normalized_coordinates=True,
                line_thickness=8)
            
            largest_index = 1
            largest_score = 0

            for i in range(boxes.shape[0]):
                if scores[0][i] > largest_score:
                    largest_index = i
                    # boxes[i] is the box which will be drawn
            
            print ("This class is gonna get used", classes[0][largest_index], scores[0][largest_index])


            # print(boxes.shape)
            # print(classes.shape)
            # print(scores)
            # print(scores[0][3])

            # boxes = np.squeeze(boxes)
            # max_boxes_to_draw = boxes.shape[0]
            # scores = np.squeeze(scores)
            # min_score_thresh=.7
            # for i in range(min(max_boxes_to_draw, boxes.shape[0])):
                # if scores is None or scores[i] > min_score_thresh:
            #         # boxes[i] is the box which will be drawn
            #         print ("This box is gonna get used", boxes[i], scores[i])

            cv2.imshow('object detection', cv2.resize(image_np, (800,600)))
            cv2.waitKey(0)


# test_video('test_video.mp4')
# test_video('test_25_sign.mp4')
# test_video('test_stop_sign_1.mp4')
# test_photo('test_photo_1.jpg')
test_video('test_traffic_all.mp4')
# test_photo('stop_sign_real.jpg')
# test_photo('speed_limit_25_real.jpg')
# test_photo('green_traffic_light_real.jpg')
# test_photo('red_traffic_light_real.jpeg')
# test_photo('red_traffic_light_real_3.jpg')