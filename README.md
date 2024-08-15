# amiga_nav2

## Requirements

This codebase is running on (ROS2 Humble)[https://docs.ros.org/en/humble/index.html] and requires the Nav2 software stack which can be downloaded (here)[https://docs.nav2.org/index.html]
Additionally, this codebase requires the twist_mux package which can be downloaded with `sudo apt install ros-humble-twist-mux`

In addition to the ROS2 requirements, this codebase requires a number of python libraries which can be downloaded in the requirements.txt file (`pip install -r requirements.txt`)

## Hardware
Developed on Intel NUC, requires Emlid Reach RS3 rtk gps or any GPS that sends NMEA to serial. Adafruit m4 CAN microcontroller with script to read and write to the CANBus also reading and writing to USB serial. 

## Launching RVIZ navigation
To launch basic rviz navigation, launch the following: `ros2 launch amiga_nav2 gps_waypoint_follower.launch.py use_rviz:=True`

## GPS Waypoint Navigation
For GPS waypoint navigation, first run the waypoint follower launch file `ros2 launch amiga_nav2 gps_waypoint_follower.launch.py`. From there, you can launch basic waypoint navigation by running the waypoint follower node `ros2 run amiga_nav2 logged_waypoint_follower <PATH-TO-YOUR-WAYPOINTS.yaml>`. You can find the format for the waypoint yaml file under `src/amiga_nav2/config/demo_waypoints.yaml`

## Thorvald communication
To connect to the Thorvald GPS, connect to the Thorvald Wifi. From here, you can access the trimble GPS. To communicate with the Thorvald, launch from `ros2 launch amiga_nav2 thorvald_comm.launch.py`. This should launch the blocker which will cause the Amiga to stop whenever within 7 meter radius of the Thorvald avoiding collision.
