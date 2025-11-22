import rclpy
import numpy as np
import cv2 as cv 
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from datagates import cam_VisionProcessor_logic
from cam_msgs.msg import MarkerInfo


#@@@@@@@@@@@@@@@@@@@@@@@@@@ Test SUB NODENODE @@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class Subnode(Node):
    def __init__(self):
        super().__init__('testnode')
        self.bridge = CvBridge()
        self.grayframe = None
        self.marker_found = None
        self.marker_id = None
        self.marker_winkel = None
        self.marker_distanz = None

        self.subscription = self.create_subscription(
            MarkerInfo,
            'MarkerInfos',
            self.frame_callback,
            10
        )
        self.get_logger().info("testnode hochgefahren!")

    
    def frame_callback(self, msg_in):
        self.marker_found = msg_in.marker_found
        self.marker_id = msg_in.marker_id
        self.marker_distanz = msg_in.marker_distanz
        self.marker_winkel = msg_in.marker_winkel

        print("gefunden")
        print(f"typ von self.marker_id:  {type(self.marker_id)}")
        print(f"Found: {self.marker_found}, the ID: {self.marker_id}, die Distanz: {self.marker_distanz: .2f}, die Abweichung zum Mittelpunkt in Winkel: {self.marker_winkel: .2f}")




def main(args=None):
    rclpy.init(args=args)
    testnode = Subnode()
    rclpy.spin(testnode)
    testnode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__': 
    main()