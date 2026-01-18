import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32

class VelocityPublisher(Node):
    '''
    Zur manuellen Justierung der Fahrgeschwindigkeit des Bots.
    Geschwindigkeit einstellbar zwischen 0.0 m/s und maximal 2.0 m/s.
    Default: 0.0 m/s (Bot steht still wenn keine Geschwindigkeit empfangen).
    
    Befehl zur Änderung im Terminal:
    > ros2 param set /velocity_pub velocity 0.15
    '''
    def __init__(self):
        super().__init__('velocity_pub')

        self.publisher = self.create_publisher(Float32,
                                               'Geschwindigkeit',
                                               5)
        
        #Parameter für die Geschwindigkeit (Default)
        self.declare_parameter('velocity', 0.0)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Velocity Publisher gestartet')
        
    def timer_callback(self):
        geschwindigkeit = self.get_parameter('velocity').value
        geschwindigkeit = clamp_velocity(geschwindigkeit, self.get_logger())

        msg = Float32()
        msg.data = geschwindigkeit
        self.publisher.publish(msg)

def clamp_velocity(value, logger=None, min_val=0.0, max_val=0.2):
    """
    Begrenzt die Geschwindigkeit auf erlaubten Bereich.
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
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
