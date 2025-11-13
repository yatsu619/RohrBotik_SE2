import rclpy
from interfaces.action import rotate 
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose2D, Twist
from datagate import cam
import time
import rotate_logic

class rotate_server(Node):
    def __init__(self):
        super().__init__('rotate_server')
        self.current_pose = Pose2D(0.0, 0.0, 0.0)
        self._action_server = ActionServer ( self, rotate,'rotate',         
                                                self.execute_callback,          
                                                goal_callback=self.goal_callback,        # optional: neue Ziele akzeptieren
                                                cancel_callback=self.cancel_callback,    # optional: Abbrüche akzeptieren
                                                handle_accepted_callback=self.handle_accepted_callback,# optional: wenn Ziel akzeptiert wird
                                            ) 
        
        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose2D,'/pose',self.pose_callback,10,)
        self.get_logger().info(' Rotate Action Server gestartet.')
      
    def goal_callback(self, goal_request):
        self.get_logger().info(' Neues Ziel empfangen!')
        return rclpy.GoalResponse.ACCEPT  

    def pose_callback(self,msg:Pose2D):
        self.current_pose.x=msg.x
        self.current_pose.y=msg.y
        self.current_pose.theta=msg.theta
        self.get_logger().info(f'Pose empfangen: x={msg.x}, y={msg.y}, theta={msg.theta}')

    def cancel_callback(self, goal_handle):
        if not cam.seerohr() == True :# interface :) @Patrice115
            self.get_logger().info('Abbruchanfrage empfangen!')
            return rclpy.CancelResponse.ACCEPT
        else:
            return rclpy.CancelResponse.REJECT
    
    def handle_accepted_callback(self, goal_handle):
        pass # to do !!!
    def execute_callback(self, goal_handle):
        count = 0

        cmd_actuell = Twist()
    
    # event handel start 
        
        if count == 0:
            cmd_actuell =  rotate_logic.RotateCL500.rotate_to_pipe(self.current_pose) 
            count = 1
        else:
            cmd_actuell =rotate_logic.RotateCL500.rotate_more(self.current_pose)
        
    #  wiederholung !! 



 



def main():

    try:
        pass # to do !!!
    except KeyboardInterrupt:
        node.get_logger().info(' Server wird beendet...')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()






if __name__ == '__main__':
    main()
