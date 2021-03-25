#!usr/bin/ python
from numpy.core.shape_base import block
# from object_detection.protos.preprocessor_pb2 import NormalizeImage
# from tensorflow.python.ops.gen_math_ops import floor
import rospy
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
# from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CameraInfo, Image as img
from mydreams.msg import ObjectDetectionBoxes

# For CNN
# import matplotlib; matplotlib.use('QT5Agg')
# import matplotlib.pyplot as plt

import os
import io
# import scipy.misc not working...
import numpy as np
from six import BytesIO
from PIL import Image, ImageDraw, ImageFont

import sys
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, dir_path)
from mscoco_label_dict import category_map

# import tensorflow as tf
import tflite_runtime.interpreter as tflite # USE THIS IF ONLY THE TFLITE INTERPRETER IS INSTALLED

# from object_detection.utils import label_map_util
# from object_detection.utils import config_util
# from object_detection.utils import visualization_utils as viz_utils
# from object_detection.builders import model_builder
# from object_detection.models import ssd_mobilenet_v2_feature_extractor # used for preprocess

import cv2
# NOTE if opencv doesn't work do:
#     pip install opencv-python 
#     pip install opencv-contrib-python
from cv_bridge import CvBridge

# Model initialization is taken from: https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/auto_examples/plot_object_detection_checkpoint.html#sphx-glr-auto-examples-plot-object-detection-checkpoint-py
# More model can be found at: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md

# This script uses TFLite model: https://www.tensorflow.org/lite/guide/inference#load_and_run_a_model_in_python
# Based on examle script: https://github.com/tensorflow/examples/blob/master/lite/examples/object_detection/raspberry_pi/detect_picamera.py
# Also followed this for existing models conversionto tflite: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/running_on_mobile_tf2.md


class ObjectDetection:

  def __init__(self):

    # pipeline_config = 'efficientdet_d0_coco17_tpu-32/pipeline.config'
    # model_dir = 'efficientdet_d0_coco17_tpu-32/checkpoint/'

    # # Load pipeline config and build a detection model
    # configs = config_util.get_configs_from_pipeline_file(pipeline_config)
    # model_config = configs['model']
    # detection_model = model_builder.build(
    #       model_config=model_config, is_training=False)

    # # Restore checkpoint
    # ckpt = tf.compat.v2.train.Checkpoint(
    #       model=detection_model)
    # ckpt.restore(os.path.join(model_dir, 'ckpt-0')).expect_partial()

    # # Retreiving labels...
    # label_map_path = 'efficientdet_d0_coco17_tpu-32/mscoco_label_map.pbtxt'  #configs['eval_input_config'].label_map_path
    # label_map = label_map_util.load_labelmap(label_map_path)
    # categories = label_map_util.convert_label_map_to_categories(
    #     label_map,
    #     max_num_classes=label_map_util.get_max_label_map_index(label_map),
    #     use_display_name=True)
    # self.category_index = label_map_util.create_category_index(categories)
    # label_map_dict = label_map_util.get_label_map_dict(label_map, use_display_name=True)

    # Load the TFLite model and allocate tensors.
    tflite_model_path = os.path.join(dir_path, "../model.tflite")
    # print(tflite_model_path)
    # self.interpreter = tf.lite.Interpreter(model_path="/home/davide/catkin_ws/src/object_detection_pico/model.tflite")
    self.interpreter = tflite.Interpreter(model_path=tflite_model_path) # USE THIS IF ONLY THE TFLITE INTERPRETER IS INSTALLED
    self.interpreter.allocate_tensors()

    # Get input and output tensors.
    self.input_details = self.interpreter.get_input_details()
    self.output_details = self.interpreter.get_output_details()

    rospy.set_param('nn_input_size', [float(self.input_details[0]['shape'][1]), float(self.input_details[0]['shape'][2])])
    # rospy.set_param('nn_input_height')
    

    # self.feat_extr = ssd_mobilenet_v2_feature_extractor.SSDMobileNetV2FeatureExtractor(is_training=False)

    self.bridge = CvBridge()

    self.pub = rospy.Publisher("DetectionBoxes", ObjectDetectionBoxes, queue_size=1)
    # self.rate = rospy.Rate()
    self.boxes_msg = ObjectDetectionBoxes()

    # cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL)
    # cv2.waitKey(1)
    # self.detect_fn = self.__get_model_detection_function(detection_model)
    # I run the net with a dummy vector to force the tf function compilation...
    # _, _, _ = self.detect_fn(tf.constant(np.zeros((1,171,224,3)), dtype=tf.float32))
  
  # def __get_model_detection_function(self, model):
  #   """Get a tf.function for detection."""

  #   @tf.function(experimental_compile=False)
  #   def detect_fn(image):
  #     """Detect objects in image."""

  #     image, shapes = model.preprocess(image)
  #     prediction_dict = model.predict(image, shapes)
  #     detections = model.postprocess(prediction_dict, shapes)

  #     return detections, prediction_dict, tf.reshape(shapes, [-1])

  #   return detect_fn

  def __preprocess(self, resized_inputs):
    """SSD preprocessing.

    Maps pixel values to the range [-1, 1].

    Args:
      resized_inputs: a [batch, height, width, channels] float tensor
        representing a batch of images.

    Returns:
      preprocessed_inputs: a [batch, height, width, channels] float tensor
        representing a batch of images.
    """
    # norm_img = cv2.normalize(resized_inputs, np.zeros((171,224)), norm_type=cv2.NORM_MINMAX)
    input_shape = np.shape(resized_inputs)
    dim0 = (self.input_details[0]['shape'][1] - input_shape[0])/2
    if dim0 % 1 == 0:
      top = int(dim0)
      bottom = top
    else: # top % 1 == 0.5:
      top = int(np.floor(dim0)) 
      bottom = int(np.ceil(dim0))

    dim1 = (self.input_details[0]['shape'][2] - input_shape[1])/2
    if dim1 % 1 == 0:
      left = int(dim1)
      right = left
    else:
      left = int(np.floor(dim1))
      right = int(np.ceil(dim1))

    resized_inputs = cv2.copyMakeBorder(resized_inputs, top, bottom, left, right, borderType=cv2.BORDER_CONSTANT, value=0)
    resized_inputs = np.expand_dims(resized_inputs.astype('float32'), 0) # converting into tensor

    return (2.0 / 255.0) * resized_inputs - 1.0
    # return norm_img

  def __repeat_grey_image_3_ch(self, image): # not used here...
      # The function supports only grayscale images
      assert len(image.shape) == 2, "Not a grayscale input image" 
      last_axis = -1
      dim_to_repeat = 2
      repeats = 3
      grscale_img_3dims = np.expand_dims(image, last_axis)
      training_image = np.repeat(grscale_img_3dims, repeats, dim_to_repeat).astype('uint8')
      assert len(training_image.shape) == 3
      assert training_image.shape[-1] == 3
      return training_image

  def __get_output_tensor(self,interpreter, index):
    """Returns the output tensor at the given index."""
    output_details = interpreter.get_output_details()[index]
    tensor = np.squeeze(interpreter.get_tensor(output_details['index'])) # Getting the tensor corresponding to that index
    return tensor
  
  def __callback(self, data):
      # rospy.loginfo(rospy.get_caller_id() + "I heard %d x %d", data.height, data.width)
      # rospy.loginfo("Image received.")
      before = rospy.get_rostime()

      image_np = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
      # image_np = self.bridge.imgmsg_to_cv2(data, desired_encoding="8UC3")
      # image_np = cv2.cvtColor(image_np, cv2.CV_8UC1)
      # rospy.loginfo(np.shape(image_np))
      cv2.namedWindow('Object Detection pure', cv2.WINDOW_NORMAL) 
      cv2.imshow('Object Detection pure', image_np)
      cv2.waitKey(10)
      # image_np = cv2.resize(image_np, (320,320))
      # image_np = self.__repeat_grey_image_3_ch(image_np)
      input_tensor = self.__preprocess(image_np)
      # input_tensor = tf.image.resize_with_pad(input_tensor, 300, 300) #Replacing this with cv2 function
      
      
      # print(self.input_details[0]['shape'][0])
      ou = self.interpreter.get_output_details()[1]
      # print(np.shape(self.interpreter.get_tensor(ou['index'])))
      # print(self.interpreter.get_output_details()[0])

      # input_tensor = tf.convert_to_tensor(
      #     np.expand_dims(input_tensor, 0), dtype=tf.float32) ##This is using complete tf
      # input_tensor = self.feat_extr.preprocess(input_tensor)
      self.interpreter.set_tensor(self.input_details[0]['index'], input_tensor) # set input tensor (incoming image)
      self.interpreter.invoke()
      # detections, predictions_dict, shapes = self.detect_fn(input_tensor)
      boxes = self.__get_output_tensor(self.interpreter, 0)       # boxes: a 2 dimensional numpy array of [N, 4]: (ymin, xmin, ymax, xmax).
                                                                  #    The coordinates are in normalized format between [0, 1].
      classes = self.__get_output_tensor(self.interpreter, 1)     # array of the id of the detected object (see label_map file)
      scores = self.__get_output_tensor(self.interpreter, 2)      # corresponding score
      count = int(self.__get_output_tensor(self.interpreter, 3))

      ### NOTE that every graphical method is not used due to absence of GUI!!

      label_id_offset = 1

      # Here I select only detection with a score above a threshold
      self.boxes_msg.object = []
      self.boxes_msg.score, self.boxes_msg.box_vertices = [], []
      object, score, self.boxes_msg.num_boxes = [], [], 0
      for prob, id, box in zip(scores, classes, range(np.size(boxes, axis=0))):
        if prob < 0.70:
          continue
        else:
          object.append(category_map[id + label_id_offset])
          score.append(prob)
          self.boxes_msg.object.append(category_map[id + label_id_offset])
          self.boxes_msg.score.append(score[-1])
          # self.boxes_msg.box_vertices.append(np.squeeze(boxes[box]))
          # self.boxes_msg.box_vertices.append(boxes[box])
          self.boxes_msg.box_vertices = np.append(self.boxes_msg.box_vertices, boxes[box])
          self.boxes_msg.num_boxes += 1
          if True:
            y_min, x_min = int(boxes[box][0]*data.height), int(boxes[box][1]*data.width)
            y_max, x_max = int(boxes[box][2]*data.height) , int(boxes[box][3]*data.width)
            self.image_np = cv2.rectangle(image_np,(x_min, y_min), (x_max, y_max), (0,0,255), 3)
            self.imready = True

      print("Python script: ", self.boxes_msg.object)
      # self.boxes_msg.box_vertices = self.boxes_msg.box_vertices
      # print(np.shape(self.boxes_msg.box_vertices))
      # print(self.boxes_msg.num_boxes)

      self.boxes_msg.header.stamp = rospy.Time.now()
      self.pub.publish(self.boxes_msg)

      # image_np = image_np.numpy()
      # image_np_with_detections = image_np.copy() # Copy the image BEFORE undergoing preprocessing!!

      # # Show the image
      
      

      after = rospy.get_rostime()
      deltaT = after - before
      # rospy.loginfo("Elapsed time: %f", deltaT.to_sec())
      info_str = ""
      for prob, id in zip(score, object):
        info_str = info_str + "Object: " + id + ", with score: " + str(prob) + "\n"
      rospy.loginfo("Elapsed: %fs\n%s", deltaT.to_sec(), info_str)
      cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL) # NO GUI TO SHOW IMAGES!
      cv2.imshow('Object Detection', image_np)
      cv2.waitKey(10)
      # cv2.resizeWindow('Object Detection', 600, 600)

      # matplotlib.use('TkAgg')
      # plt.figure(figsize=(12,16))
      # plt.imshow(self.image_np)
      # plt.show()
      
      # NOTE that I had to change a file in the object detection API to show images:
      # https://stackoverflow.com/questions/56699499/matplotlib-backend-shifts-when-using-object-detection-api-qtagg-to-agg
    
  def listener(self):

      # In ROS, nodes are uniquely named. If two nodes with the same
      # name are launched, the previous one is kicked off. The
      # anonymous=True flag means that rospy will choose a unique
      # name for our 'listener' node so that multiple listeners can
      # run simultaneously.
      rospy.init_node('cnn_object_detection', anonymous=True)

      rospy.Subscriber("/panoramicrgb_img", img, self.__callback, queue_size=1, buff_size=2**24) # INCREASE BUFF SIZE TO REMOVE INCREASING DELAY OF CALLBACK

      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()

if __name__ == '__main__':
    # cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL)
    objdet = ObjectDetection()
    objdet.listener()

    # while not rospy.is_shutdown():
    #   if objdet.imready:
    #     cv2.imshow('Object Detection', objdet.image_np)
    #     cv2.waitKey(1000)
    #     objdet.imready = False