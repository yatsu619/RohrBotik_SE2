from geometry_msgs.msg import Pose2D
import rotate_logic







def main():
    count = 0
    current_pose = Pose2D()
    current_pose.x = 0.0
    current_pose.y = 0.0
    current_pose.theta = 0.0

    if not kamera.seerohr == True :
        if count == 0:
            rotate_logic.Rotate.rotate_to_pipe(current_pose)
        else:
            rotate_logic.Rotate.rotate_more(current_pose)
    else:
        # to do 
         # shut down implementiern 
if __name__ == '__main__':
    main()
