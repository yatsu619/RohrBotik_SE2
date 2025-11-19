#!/usr/bin/env python3
"""
Zentrale Start-Datei für das RohrBotik-System

Startet alle benötigten Nodes im GLEICHEN Python-Prozess:
- cam_data_pub_Node: Publiziert Kamerabilder
- cam_data_sub_Node: Empfängt Bilder und verarbeitet sie
- rotate_action_server: Dreht den Roboter

Durch den gemeinsamen Prozess können alle Nodes die GLEICHE 
VisionManager-Instanz nutzen und auf dieselben Bilddaten zugreifen!

VERWENDUNG:
    1. Speichere diese Datei als: start_robot.py
    2. Im Workspace-Root: source install/setup.bash
    3. Dann: python3 src/start_robot.py
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
import sys
import os

# Imports
from datagate.cam_data_pub_Node import CameraOutPut
#from datagate.cam_data_sub_Node import Camera_data
from rotate_server.rotate_server.rotate_action_server import rotate_action_server
from datagate.cam_vision_manager import VisionManager
from interfaces.action import RotateAc


def main(args=None):
    """Hauptfunktion - startet alle Nodes zusammen"""
    
    print("=" * 70)
    print("  🤖 RohrBotik System wird gestartet...")
    print("=" * 70)
    
    # ROS2 initialisieren
    rclpy.init(args=args)
    
    try:
        # Alle Nodes im GLEICHEN Prozess erstellen
        print("\n[1/3] Erstelle Camera Publisher Node...")
        camera_pub_node = CameraOutPut()
        print("      ✅ Camera Publisher bereit")
        
        print("\n[2/3] Erstelle Camera Subscriber Node...")
        #camera_sub_node = Camera_data()
        print("      ✅ Camera Subscriber bereit")
        
        print("\n[3/3] Erstelle Rotate Action Server Node...")
        rotate_server_node = rotate_action_server()
        print("      ✅ Rotate Action Server bereit")
        
        print("\n" + "=" * 70)
        print("  ✅ Alle Nodes teilen sich EINE VisionManager-Instanz!")
        print("  🛑 Drücke Ctrl+C zum Beenden")
        print("=" * 70 + "\n")
        
        # MultiThreadedExecutor für parallele Ausführung
        executor = MultiThreadedExecutor()
        executor.add_node(camera_pub_node)
        executor.add_node(camera_sub_node)
        executor.add_node(rotate_server_node)
        
        # Alle Nodes parallel ausführen
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n\n" + "=" * 70)
        print("  System wird durch Benutzer beendet...")
        print("=" * 70)
        
    except Exception as e:
        print(f"\n❌ FEHLER: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Aufräumen
        print("\n[Cleanup] Beende Nodes...")
        
        try:
            camera_pub_node.destroy_node()
            print("  ✅ Camera Publisher beendet")
        except:
            pass
            
        try:
            camera_sub_node.destroy_node()
            print("  ✅ Camera Subscriber beendet")
        except:
            pass
            
        try:
            rotate_server_node.destroy_node()
            print("  ✅ Rotate Server beendet")
        except:
            pass
        
        print("\n[Cleanup] ROS2 Shutdown...")
        rclpy.shutdown()
        
        print("\n" + "=" * 70)
        print("  System sauber beendet. Auf Wiedersehen! 👋")
        print("=" * 70 + "\n")


if __name__ == '__main__':
    main()