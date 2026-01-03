import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32


#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

class Distance(Node):
    '''
    Zur manuellen justierung des Abstands zum vorrausfahrenden Bots.

    Distanz einstellbar zwischen maximal einem Meter und minimal 0.2 Meter)

    Befehl zur Änderung im Terminal:
    > ros2 param set /distance_publisher target_distance 0.8

    '''
    def __init__(self):
        super().__init__('distance')
        self.publisher = self.create_publisher(
            Float32,
            'Distanz_poti',
            5
        )
        self.declare_parameter('poti', 0.5)
        self.timer = self.create_timer(0.1, self.timer_callback)



    def timer_callback(self):
        abstand = self.get_parameter('poti').value
        abstand = max(1.0, min(0.2, abstand))     

        msg = Float32()
        msg.data = abstand
        self.publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = Distance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


