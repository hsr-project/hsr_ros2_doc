# Install ROS2

Please refer to the following link to install ROS2:

* https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Additionally, install the necessary tools:

```
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```


# Create Workspace for Simulation

Create a workspace and retrieve the necessary packages:

```
mkdir -p ~/hsr_ros2_ws/src && cd ~/hsr_ros2_ws/src
git clone -b jazzy https://github.com/hsr-project/hsrb_controllers.git
git clone -b jazzy https://github.com/hsr-project/hsrb_common.git
git clone -b jazzy https://github.com/hsr-project/hsrb_drivers.git
git clone -b jazzy https://github.com/hsr-project/hsrb_launch.git
git clone -b jazzy https://github.com/hsr-project/hsrb_manipulation.git
git clone -b jazzy https://github.com/hsr-project/hsrb_rosnav.git
git clone -b jazzy https://github.com/hsr-project/hsrb_simulator.git
git clone -b jazzy https://github.com/hsr-project/hsr_common.git
git clone -b jazzy https://github.com/hsr-project/hsrb_teleop.git
git clone -b jazzy https://github.com/hsr-project/tmc_gazebo.git
git clone -b jazzy https://github.com/hsr-project/tmc_teleop.git
git clone -b jazzy https://github.com/hsr-project/tmc_common.git
git clone -b jazzy https://github.com/hsr-project/tmc_common_msgs.git
git clone -b jazzy https://github.com/hsr-project/tmc_drivers.git
git clone -b jazzy https://github.com/hsr-project/tmc_database.git
git clone -b jazzy https://github.com/hsr-project/tmc_manipulation.git
git clone -b jazzy https://github.com/hsr-project/tmc_manipulation_base.git
git clone -b jazzy https://github.com/hsr-project/tmc_manipulation_planner.git
git clone -b jazzy https://github.com/hsr-project/tmc_point_cloud.git
git clone -b jazzy https://github.com/hsr-project/tmc_realtime_control.git
git clone -b jazzy https://github.com/hsr-project/tmc_voice.git
git clone -b jazzy https://github.com/hsr-project/tmc_navigation.git
rm -rf hsrb_launch/hsrb_robot_launch
rm -rf hsrb_simulator/hsrb_rviz_simulator
rm -rf tmc_drivers/tmc_pgr_camera
```

Build the workspace:

```
cd ~/hsr_ros2_ws/
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


# Gazebo Simulation

Launch the Gazebo simulator. 
The following is an example launch command. 
Use the appropriate launch file based on the robot and world you are using:

```
ros2 launch hsrb_gazebo_launch hsrb_apartment_no_objects_world.launch.py
```


# Rviz Simulation

Sensor simulations are not available. However, it can be used for simple motion verification.

**Case: HSR-B**

```
ros2 launch hsrb_rviz_simulator hsrb_rviz_simulator.launch.py
```

**Case: HSR-C**

```
ros2 launch hsrb_rviz_simulator hsrc_rviz_simulator.launch.py
```
