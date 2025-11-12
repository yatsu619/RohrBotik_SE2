import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose2D, Twist
#from # action  import  # action 
import time
import rotate_logic

class rotate_server(Node):
    def __init__(self):
        pass # to do !!!
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
    if not kamera.seerohr == True :# interface :)
        if count == 0:
            rotate_logic.Rotate.rotate_to_pipe(current_pose)
            count = 1
        else:
            rotate_logic.Rotate.rotate_more(current_pose)
    else:
        # to do 
         # shut down implementiern 
         pass
 #  wiederholung !! 
if __name__ == '__main__':
    main()
