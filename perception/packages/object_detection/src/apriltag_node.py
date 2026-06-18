#!/usr/bin/env python3
"""Nodo ROS: detecta AprilTags y publica la accion semantica sugerida."""
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from object_detection.apriltag_policy import select_action


class AprilTagNode:
    def __init__(self):
        rospy.init_node("apriltag_node")
        from dt_apriltags import Detector
        self.detector = Detector(families="tag36h11")
        self.pub = rospy.Publisher("~action", String, queue_size=1)
        self.sub = rospy.Subscriber("~image/compressed", CompressedImage,
                                    self.cb, queue_size=1, buff_size=2**24)
        rospy.loginfo("apriltag_node listo")

    def cb(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        if img is None:
            rospy.logwarn("Frame corrupto en apriltag_node, se descarta")
            return
        tags = self.detector.detect(img)
        accion = select_action([t.tag_id for t in tags])
        if accion is not None:
            self.pub.publish(String(data=accion))


if __name__ == "__main__":
    AprilTagNode()
    rospy.spin()
