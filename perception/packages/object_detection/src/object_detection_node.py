#!/usr/bin/env python3
# perception/packages/object_detection/src/object_detection_node.py
"""Nodo ROS: detecta objetos con YOLO26 (TensorRT) y publica detecciones."""
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from object_detection.trt_inference import TRTDetector

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node("object_detection_node")
        engine = rospy.get_param("~engine_path", "/data/model.engine")
        self.detector = TRTDetector(engine,
                                    imgsz=rospy.get_param("~imgsz", 416),
                                    conf_thres=rospy.get_param("~conf_thres", 0.25))
        self.pub = rospy.Publisher("~detections", Float32MultiArray, queue_size=1)
        self.sub = rospy.Subscriber("~image/compressed", CompressedImage,
                                    self.cb, queue_size=1, buff_size=2**24)
        rospy.loginfo("object_detection_node listo")

    def cb(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            rospy.logwarn("Frame corrupto: imdecode devolvio None, se descarta")
            return
        dets = self.detector.infer(img)              # (N,6)
        out = Float32MultiArray()
        out.layout.dim = [MultiArrayDimension(label="n", size=dets.shape[0], stride=6),
                          MultiArrayDimension(label="f", size=6, stride=1)]
        out.data = dets.astype(np.float32).flatten().tolist()
        self.pub.publish(out)

if __name__ == "__main__":
    ObjectDetectionNode()
    rospy.spin()
