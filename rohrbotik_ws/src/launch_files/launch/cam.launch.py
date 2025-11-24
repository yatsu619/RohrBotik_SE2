#für das allgemeine launchscript
from launch import LaunchDescription
from launch_ros.actions import Node
#für eine bestimmte Reinfolge:
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

def generate_launch_description():
    

    camera_node = Node(
        package='datagates',                     # Das Package, in dem die Node ist
        executable='cam_data_pub_Node',          # Der Name der ausführbaren Datei  
        name='cam_data_pub_Node',                     # Ein eigener Name für diese Node-Instanz  
        output='screen',                         # Zeige die Ausgaben im Terminal
 #      parameter=[{}]                           # Übergabeparameter, z.B.: 'video_device': '/dev/video0', 'image_size': [640, 480]
    )

    subpub_node = Node(
        package='datagates',  
        executable='cam_subpub_Node', 
        name='cam_subpub_Node',
        output='screen'
    )

    register_subpub_handler = RegisterEventHandler(     # Event_händler: Starte subpub_node, wenn camera_node gestartet ist
            OnProcessStart(
                    target_action=camera_node,
                    on_start=[subpub_node]
            )
    )     

    return LaunchDescription([
        camera_node,
        register_subpub_handler
    ])