#--------- Imports

import rclpy 
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# -------- Code

class Pic_sub(Node):
    def __init__(self):
        super().__init__('pic_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'versuchsstream',
            self.listener_callback,
            10
        )
        

    def listener_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            cv.imshow("Kamerabild", frame)
            cv.waitKey(5)#
            self.get_logger().info("Bild Empfangen... ")
        except Exception as e:
            self.get_logger().error(f"Bild konnte nicht decodiert werden: {e}")


def main(args=None):
    rclpy.init(args=args)
    pic_sub = Pic_sub()
    rclpy.spin(pic_sub)
    pic_sub.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows
     
if __name__ == '__main__':
    main()
