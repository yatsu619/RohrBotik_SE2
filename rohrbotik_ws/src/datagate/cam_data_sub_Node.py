from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node

from cam_vision_manager import VisionManager



# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ ROS2 - Subscriber_cam @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class Camera_data(Node):
    def __init__(self):
        super().__init__('cam_data_sub')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'pycam_tb3',
            self.frame_callback,
            10
        )
        self.visisonprocessor = VisionManager.get_instance()                     # Hier erstelle ich das Objekt VisionProcessor,um dort die Variable frame zu übergeben.
        
    def frame_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is not None:
            self.visisonprocessor.videoframe(frame)

        else:
            self.get_logger().info("Da knallts bei der Cam gewaltig")