# Dem Turtlebot3 wird Beine gemacht.
Damit die Rohre der Stadt sauber bleiben, wird unser Bot sie alle durchforsten. Und machen Sie sich keine Sorgen um mögliche Kollisionen mit anderen Robotern. Auch in einer solch Heiklen Situation, hat unser RohrBotik-Bot eine Lösung parat. 

->Installation und Ordnerstruktur (zip entpacken)
-> mit scp ins bot schicken


# Anleitung zur Bedienung des Systems:
Für das Starten des Systems werden mindestens 3 Terminals benötigt

1. Lokaler Rechner mit dem Bot verbinden: (in 2 Terminals)
    ssh ubuntu@10.42.0.1
    Passwort: ipek2023

2. Launch Datei vom Bot starten: (im 1. Terminal)
    ros2 launch turtlebot3_bringup robot.launch.py

3. Zum Workspace Ordner gehen, dann builden und Sourcen: (im 2. Terminal)
    cd rohrbotik_group/rohrbotik_ws/    -> auf Bot 2
    colcon build
    source install/setup.bash

4. Im 2. Terminal Sourcen und die Launch Datei starten:
    ros2 launch rb3_bringup cam_launch.py target_vel:= (beliebige Fahrgeschwindigkeit von 0.0 - 0.2 m/s)

3. Nachdem das System hochgefahren ist die Topic Parameter bestimmen: (im 3. Terminal)
    Um Fahrgeschwindigkeit anzupassen:
        ros2 param set /velocity_pub velocity (beliebige Fahrgeschwindigkeit von 0.0 - 0.2 m/s)
    Um Abstand zum anderen Bot einzustellen:
        ros3 param set /distance_pub poti (beliebiger Abstand von 0.2 - 1.0 m)

4. Um Bot zu stoppen: (3. Terminal)
    ros2 param set /velocity_pub velocity 0.0

5. Bei Notfällen: Launch Datei vom Bot killen (1. Terminal)
    CTRL C
6. Bei Codeänderungen: Neu builden und sourcen im Workspace Ordner
    cd rohrbotik_group/rohrbotik_ws/    -> auf Bot 2

    colcon build
    source install/setup.bash

