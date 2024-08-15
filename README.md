# nav2_ws

##Requirements

This codebase is running on (ROS2 Humble)[https://docs.ros.org/en/humble/index.html] and requires the Nav2 software stack which can be downloaded (here)[https://docs.nav2.org/index.html]
Additionally, this codebase requires the twist_mux package which can be downloaded with `sudo apt install ros-humble-twist-mux`

In addition to the ROS2 requirements, this codebase requires a number of python libraries which can be downloaded in the requirements.txt file (`pip install -r requirements.txt`)

##Launching RVIZ navigation
To launch basic rviz navigation, launch the following: `ros2 launch amiga_nav2 gps_waypoint_follower.launch.py use_rviz:=True`
