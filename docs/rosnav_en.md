# Install ROS2

Please refer to the following link to install ROS2:

* https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Additionally, install the necessary tools:

```
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```


# Create Workspace

Create a workspace and retrieve the necessary packages:

```
mkdir -p ~/hsr_ros2_ws/src && cd ~/hsr_ros2_ws/src
git clone -b humble https://github.com/hsr-project/hsrb_rosnav.git
```

Build the workspace:

```
cd ~/hsr_ros2_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


# Mapping

First, start the node for creating a map:

```
ros2 launch hsrb_mapping slam_toolbox_mapping.launch.py
```

Check the mapping progress using Rviz:

```
rviz2 -d src/hsrb_rosnav/hsrb_mapping/rviz/hsr_slam_toolbox_mapping.rviz
```

Once the map has been sufficiently created, save it:

```
ros2 run nav2_map_server map_saver_cli -f map --ros-args -p save_map_timeout:=100000.0
```

Running this command will save two files in the directory where it was executed:

* map.pgm
* map.yaml


# Navigation

Start the navigation:

```
ros2 launch hsrb_rosnav_config navigation_launch.py map:=/full/path/to/map.yaml use_sim_time:=True initial_orientation_xyzw:=[0,0,0,1]
```

Use Rviz to initialize the robot's pose and verify the navigation:

```
rviz2 -d src/hsrb_rosnav/hsrb_rosnav_config/rviz/hsr_navigation2.rviz
```

1. Use **2D Pose Estimate** at the top of RViz to initialize the robot's pose.
2. Use **Navigation2 Goal** at the top of RViz to set the target pose.
