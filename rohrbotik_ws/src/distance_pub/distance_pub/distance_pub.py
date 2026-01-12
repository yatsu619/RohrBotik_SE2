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
    > "ros2 param set /distance_publisher poti 0.8" [einer von den beiden]

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
        abstand = clamp_distance(abstand, self.get_logger())

        msg = Float32()
        msg.data = abstand
        self.publisher.publish(msg)

def clamp_distance(value, logger=None, min_val=0.2, max_val=1.0):
    """
    Begrenzt den Abstand auf erlaubten Bereich.
    Loggt eine Warnung wenn der Wert korrigiert wurde.
    """
    if value > max_val:
        if logger:
            logger.warn(f'Wert {value} zu groß! Begrenzt auf {max_val}')
        return max_val
    
    if value < min_val:
        if logger:
            logger.warn(f'Wert {value} zu klein! Begrenzt auf {min_val}')
        return min_val
    
    return value




def main(args=None):
    rclpy.init(args=args)
    node = Distance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


