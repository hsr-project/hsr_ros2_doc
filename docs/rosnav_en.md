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

## In the case of a simulator

First, start the world. Please launch it with use_navigation:=false.

```
ros2 launch hsrb_gazebo_launch hsrb_apartment_no_objects_world.launch.py use_navigation:=false
```

After that, start the rosnav nodes. 

For the map, specify the full path to the world you launched.

```
ros2 launch hsrb_rosnav_config navigation_launch.py map:=/full/path/to/tmc_database/tmc_potential_maps/maps/white_space/map.yaml use_sim_time:=True initial_orientation_xyzw:=[0,0,0,1]
```

Start the node for map creation.

```
ros2 launch hsrb_mapping slam_toolbox_mapping.launch.py
```

Check the progress of map creation in Rviz.

```
rviz2 -d src/hsrb_rosnav/hsrb_mapping/rviz/hsr_slam_toolbox_mapping.rviz
```

Once the map has been sufficiently created, save it.

```
ros2 run nav2_map_server map_saver_cli -f map --ros-args -p save_map_timeout:=100000.0
```

Executing this will save two files in the directory where the command was run.

* map.pgm
* map.yaml

## In the case of the actual machine

First, on the HSR internal PC, change USE_NAVIGATION to false in /etc/opt/tmc/robot/docker.ros2.hsrb.

```
USE_LASER_ODOM=true
USE_HEAD_CENTER_CAMERA=false
USE_JOYSTICK_TELEOP2=true
USE_DUALSHOCK4=false
USE_BATTERY_NOTIFIER=true
USE_NAVIGATION=false                          # Modification
DOCKER_OPTS= \
        --rm \
        --net=host \
        --pid=host \
```

Once the setting is complete, perform a reboot.

```
$ cd
$ bash stop_ros2_docker.sh
$ bash start_ros2_docker.sh
```

Start the rosnav nodes. For the map, specify tmc_database/tmc_potential_maps/maps/white_space/map.yaml.

```
ros2 launch hsrb_rosnav_config navigation_launch.py map:=/full/path/to/tmc_database/tmc_potential_maps/maps/white_space/map.yaml use_sim_time:=True initial_orientation_xyzw:=[0,0,0,1]
```

Start the node for map creation.

```
ros2 launch hsrb_mapping slam_toolbox_mapping.launch.py
```

Check the progress of map creation in Rviz.

```
rviz2 -d src/hsrb_rosnav/hsrb_mapping/rviz/hsr_slam_toolbox_mapping.rviz
```

Once the map has been sufficiently created, save it.

```
ros2 run nav2_map_server map_saver_cli -f map --ros-args -p save_map_timeout:=100000.0
```

Executing this will save two files in the directory where the command was run.

* map.pgm
* map.yaml

# Navigation

Launch the navigation program.

For the simulator, set use_sim_time:=True.

```
ros2 launch hsrb_rosnav_config navigation_launch.py map:=/full/path/to/map.yaml initial_orientation_xyzw:=[0,0,0,1]
```

Use Rviz to initialize the robot’s position and check the navigation.

```
rviz2 -d src/hsrb_rosnav/hsrb_rosnav_config/rviz/hsr_navigation2.rviz
```

1. Use 2D Pose Estimate at the top of RViz to initialize the robot's pose.
2. Use Navigation2 Goal at the top of RViz to set the target pose.
