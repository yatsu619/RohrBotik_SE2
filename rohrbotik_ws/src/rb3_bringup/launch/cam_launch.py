#für das allgemeine launchscript
from launch import LaunchDescription
from launch_ros.actions import Node
#für eine bestimmte Reinfolge:
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
'''
Launch File für das gesamte RohrBotik System.

Startet alle Nodes in einer gestaffelten Reihenfolge mit TimerAction.
Dies verhindert Race Conditions, jede Node hat Zeit sich zu initialisieren
bevor die nächste gestartet wird.

Startreihenfolge:
    t=0s Kamera Node (cam_data_pub)
    t=1s Bildverarbeitung (cam_subpub), Geschwindigkeit (velocity_pub), Distanz (distance_pub)
    t=2s Rotate Server
    t=3s Move Server
    t=4s Handler Server (braucht Rotate und Move)
    t=6s Mission Start (Handler Goal senden)
'''

def generate_launch_description():
    '''
    Erstellt die Launch Description mit allen Nodes und deren Startreihenfolge.

    Launch Argument:
        target_vel: Startgeschwindigkeit für die Linearfahrt in m/s (Standard: 0.12)
                    Wird an velocity_pub als Parameter übergeben.
                    Während der Fahrt kann die Geschwindigkeit mit
                    'ros2 param set /velocity_pub velocity <wert>' geändert werden.

    Returns:
        LaunchDescription: Enthält alle Nodes und TimerActions in der richtigen Reihenfolge
    '''
    
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

    activate_poti = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='distance_pub',
                executable='distance_pub',
                name='distance_pub',
                output='screen'
            )
        ]
    )
    
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
    
    start_mission = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'ros2 action send_goal /handler interfaces/action/HandlerAc "{}"'],
                output='screen',
                
            )
        ]
    )

    

    return LaunchDescription([
        target_vel_arg,      
        camera_node,              # 0s
        subpub_node_delayed,      # 1s
        velocity_pub_delayed,     # 1s
        activate_poti,            # 1s
        rotate_node_delayed,      # 2s
        move_node_delayed,        # 3s
        handler_node_delayed,     # 4s
        start_mission,            # 6s
    ])