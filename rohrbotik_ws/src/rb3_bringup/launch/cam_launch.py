#für das allgemeine launchscript
from launch import LaunchDescription
from launch_ros.actions import Node
#für eine bestimmte Reinfolge:
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration

#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

def generate_launch_description():
    
    target_vel_arg = DeclareLaunchArgument(             #Parameter wird deklariert, damit der user diesen beim Starten des Launchfiles setzen kann
        'target_vel',
        default_value = '0.12',
        description = 'Geschwindigkeit für die Linearfahrt in m/s: ',
    )
    
    target_vel = LaunchConfiguration('target_vel')

    camera_node = Node(
        package='datagates',                     # Das Package, in dem die Node ist
        executable='cam_data_pub_Node',          # Der Name der ausführbaren Datei  
        name='cam_data_pub_Node',                     # Ein eigener Name für diese Node-Instanz  
        output='screen',                         # Zeige die Ausgaben im Terminal
 #      parameter=[{}]                           # Übergabeparameter, z.B.: 'video_device': '/dev/video0', 'image_size': [640, 480]
    )

    subpub_node_delayed = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='datagates',
                executable='cam_subpub_Node',
                name='cam_subpub_Node',
                output='screen'
            )
        ]
    )

    velocity_pub_delayed = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='velocity_pub',
                executable='velocity_pub',
                name='velocity_pub',
                parameters=[{'velocity': target_vel}],
                output='screen'
            )
        ]
    )
    
    # 4. Rotate Server (nach 2s)
    rotate_node_delayed = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rotate_server',
                executable='rotate_server_node',
                name='rotate_server_node',
                output='screen'
            )
        ]
    )
    
    # 5. Move Server (nach 3s)
    move_node_delayed = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='move_server',
                executable='move_server_node',
                name='move_server_node',
                output='screen'
            )
        ]
    )
    
    # 6. Handler Server (nach 4s)
    handler_node_delayed = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='action_server_handler',
                executable='handler_node',
                name='handler_node',
                output='screen'
            )
        ]
    )
    
    # 7. Action Goal (nach 7s)
    start_mission = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    ['ros2 action send_goal /handler interfaces/action/HandlerAc "{target_vel: ', 
                     target_vel,
                     '}"']
                ],
                output='screen',
                
            )
        ]
    )

    #8. distance_poti(nach 8s)
    activate_poti = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='distance_pub',
                executable='distance_pub',
                name='distance_pub',
                output='screen'
            )
        ]
    )
    

    return LaunchDescription([
        target_vel_arg,      
        camera_node,              # 0s
        subpub_node_delayed,      # 1s
        velocity_pub_delayed,     # 1s
        rotate_node_delayed,      # 2s
        move_node_delayed,        # 3s
        handler_node_delayed,     # 4s
        start_mission,            # 7s
        activate_poti,            # 8s
    ])