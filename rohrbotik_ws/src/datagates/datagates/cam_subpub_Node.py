from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 
import rclpy
from rclpy.node import Node

from .cam_VisionProcessor_logic import VisionProcessor 
from cam_msgs.msg import MarkerInfo




# @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ ROS2 - SupPubNode - Zuständig für alle  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class Camera_data(Node):
    '''
    Die SupPubNode ist als Hauptverteiler der Kameradaten zuständig. 
    Sie subscript die CameraOutPutNode und empfängt jedes eingehende Graubild.
    Dieses Bild wird dann per Funktion "find_ArUco" in einer VisionProcessor-Instanz verarbeitet. 
    Während der Verarbeitung eines gefundenen Markers werden die wichtigsten Informationen herrausgefiltert und über eine Custom-Message hier weiter gepublished.
    Die Custom-Message besteht aus: 
    >  bool marker_found
    >  float32 marker_distanz
    >  float32 marker_winkel
    >  int32 marker_id
    Grundlegend gibt die Funktion "find_ArUco" bei einem gefunden und ausgewertetem ArUco Marker ein True zurück. Wird kein Marker im Bild gefunden oder ist das Bild fehlerhaft, wird False zurück gegeben.
    Gibt "find_ArUco" ein False zurück, werden die relevanten Variablen auf False oder 0 gesetzt und per Custom-Message gepublished. Andernfalls werden nach jedem Aufruf (fehlerfrei 24 mal die Sekunde) die Werte des aktuellen relevanten ArUco Markers weitergeben.  
    '''
    def __init__(self):
        super().__init__('cam_sup_pub')
        self.bridge = CvBridge()
        self.gray_frame = None
        self.vp = VisionProcessor()

        self.subscription = self.create_subscription(
            Image,
            'pycam_tb3',
            self.frame_callback,
            10
        )

        self.camera_data_publish = self.create_publisher(
            MarkerInfo,
            'MarkerInfos',
            10
        )

        self.get_logger().info("SupPubNode gestartet")

        
    def frame_callback(self, msg):
        self.gray_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')


        if self.gray_frame is not None:
            self.get_logger().info("Graubild bei SubPub angekommen!")
        else:
            self.get_logger().info("Kein Bild bei SubPub angekommen")


#Visionprocessor arbeitet

        if not self.vp.find_ArUco(self.gray_frame):   
            self.get_logger().info("Es wurde kein Marker gefunden, oder zu viele...")
            msg_out = MarkerInfo()
            msg_out.marker_found = False
            msg_out.marker_distanz = 0.0
            msg_out.marker_winkel = 0.0
            msg_out.marker_id = 0
            self.camera_data_publish.publish(msg_out)
            return False
        
        marker_id = self.vp.marker_id
        marker_distanz = self.vp.marker_distanz
        marker_winkel = self.vp.marker_winkel
        marker_found = self.vp.marker_gefunden

     
        msg_out = MarkerInfo()
        msg_out.marker_found = marker_found
        msg_out.marker_winkel = marker_winkel
        msg_out.marker_distanz = marker_distanz
        msg_out.marker_id = marker_id
        
        self.camera_data_publish.publish(msg_out)

        self.get_logger().info(f'ArUco gefunden: {marker_found} ID: {marker_id}, distanz: {marker_distanz: .2f}, Winkel: {marker_winkel: .2f}°')



#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

def main(args=None):
    rclpy.init(args=args)
    cam_sub_pub = Camera_data()
    rclpy.spin(cam_sub_pub)
    cam_sub_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 