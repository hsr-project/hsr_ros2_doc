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


# Create Workspace

Create a workspace and retrieve the necessary packages:

```
mkdir -p ~/hsr_ros2_ws/src && cd ~/hsr_ros2_ws/src
git clone -b jazzy https://github.com/hsr-project/hsrb_common.git
git clone -b jazzy https://github.com/hsr-project/hsrb_manipulation.git
git clone -b jazzy https://github.com/hsr-project/hsrb_moveit.git
git clone -b jazzy https://github.com/hsr-project/hsr_common.git
git clone -b jazzy https://github.com/hsr-project/tmc_common.git
git clone -b jazzy https://github.com/hsr-project/tmc_common_msgs.git
git clone -b jazzy https://github.com/hsr-project/tmc_manipulation.git
git clone -b jazzy https://github.com/hsr-project/tmc_manipulation_base.git
git clone -b jazzy https://github.com/hsr-project/tmc_manipulation_planner.git
```

Build the workspace:

```
cd ~/hsr_ros2_ws/
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


# Launch Examples

After starting the simulator or other required tools, execute demo.launch.py.
Use the appropriate launch file for the robot.

For the physical robot

* HSRB

    ```
    ros2 launch hsrb_moveit_config hsrb_demo.launch.py
    ```

* HSRC

    ```
    ros2 launch hsrb_moveit_config hsrc_demo.launch.py
    ```

For the simulator

* HSRB

    * Launch the simulator (case HSRB)

        ```
        ros2 launch hsrb_gazebo_launch hsrb_empty_world.launch.py rviz:=false
        ```

    * Launch the hsrb_demo.launch.py

        ```
        ros2 launch hsrb_moveit_config hsrb_demo.launch.py use_sim_time:=true
        ```

* HSRC

    * Launch the simulator (HSRC)

        ```
        ros2 launch hsrb_gazebo_launch hsrc_empty_world.launch.py rviz:=false
        ```

    * Launch the hsrb_demo.launch.py

        ```
        ros2 launch hsrb_moveit_config hsrc_demo.launch.py use_sim_time:=true
        ```

## Operation via GUI

Send commands from the Rviz MotionPlanning plugin.

## Using MoveGroup from C++

You can run the sample program with the following command:

For the physical robot

```
ros2 launch hsrb_moveit_config hsrb_example.launch.py example_name:=<Example PROGRAM>
```

For the simulator

```
ros2 launch hsrb_moveit_config hsrb_example.launch.py use_sim_time:=true example_name:=<Example PROGRAM>
```

The available sample programs are as follows:

* **moveit_fk_demo**
* **moveit_ik_demo**
* **moveit_gripper_demo**
* **moveit_constraints_demo**