from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rotate_server',
            executable='rotate_action_server',
            name='rotate_action_server',
            output='screen'
        ),

        Node(
            package='datagate',
            executable='cam_data_pub_Node',
            name='cam_data_pub_Node',
            output='screen'
        ),

        Node(
            package='datagate',
            executable='cam_data_sub_Node',
            name='cam_data_sub_Node',
            output='screen'
        )
        
        
        #,

        # Node(
        #     package='muri_dev',
        #     executable='muri_action_handler',
        #     name='muri_action_handler',
        #     output='screen'
        # )

    ])

# - colcon bild + sourcen
# terminal: ros2 launch action_server_handler start_robot.py