import rclpy
from interfaces.action import rotate 
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose2D, Twist
#from # action  import  # action 
import time
import rotate_logic

class rotate_server(Node):
    def __init__(self):
        super().__init__('simple_mover_server')
        self._action_server = ActionServer ( self, rotate,'rotate',
                                                self.execute_callback ,
                                                goal_callback=self.goal_callback,        # optional: neue Ziele akzeptieren
                                                cancel_callback=self.cancel_callback,    # optional: Abbrüche akzeptieren
                                                handle_accepted_callback=self.handle_accepted_callback,# optional: wenn Ziel akzeptiert wird
                                            ) 
        
        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose2D,'/pose',self.pose_callback,10,callback_group=self.pose_cb_group)
        self.get_logger().info(' SimpleMover Action Server gestartet.')
      
    def goal_callback(self, goal_request):
        pass  # to do !!!
    def pose_callback(self,msg:Pose2D):
        pass # to do !!!
    def cancel_callback(self, goal_handle):
        pass # to do !!!
    def handle_accepted_callback(self, goal_handle):
        pass # to do !!!
    def execute_callback(self, goal_handle):
        pass  # to do !!!



 



def main():
    count = 0
    current_pose = Pose2D(0.0, 0.0, 0.0)
   
# event handel start 
    if not kamera.seerohr() == True :# interface :)
        if count == 0:
            rotate_logic.RotateCL500.rotate_to_pipe(current_pose)
            count = 1
        else:
            rotate_logic.RotateCL500.rotate_more(current_pose)
    else:
        # to do 
         # shut down implementiern 
         pass
 #  wiederholung !! 
if __name__ == '__main__':
    main()
