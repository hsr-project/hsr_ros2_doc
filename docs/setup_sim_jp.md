# Install ROS2

以下を参考に，ROS2をインストールしてください．

* https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

また，必要なツールをインストールしてください．

```
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```


# Create Workspace for Simulation

ワークスペースを作成し，必要なパッケージを取得します．

```
mkdir -p ~/hsr_ros2_ws/src && cd ~/hsr_ros2_ws/src
git clone -b humble https://github.com/hsr-project/gazebo_ros2_control.git
git clone -b humble https://github.com/hsr-project/hsrb_controllers.git
git clone -b humble https://github.com/hsr-project/hsrb_common.git
git clone -b humble https://github.com/hsr-project/hsrb_drivers.git
git clone -b humble https://github.com/hsr-project/hsrb_launch.git
git clone -b humble https://github.com/hsr-project/hsrb_manipulation.git
git clone -b humble https://github.com/hsr-project/hsrb_rosnav.git
git clone -b humble https://github.com/hsr-project/hsrb_simulator.git
git clone -b humble https://github.com/hsr-project/hsr_common.git
git clone -b humble https://github.com/hsr-project/hsrb_teleop.git
git clone -b humble https://github.com/hsr-project/tmc_gazebo.git
git clone -b humble https://github.com/hsr-project/tmc_teleop.git
git clone -b humble https://github.com/hsr-project/tmc_common.git
git clone -b humble https://github.com/hsr-project/tmc_common_msgs.git
git clone -b humble https://github.com/hsr-project/tmc_drivers.git
git clone -b humble https://github.com/hsr-project/tmc_database.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation_base.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation_planner.git
git clone -b humble https://github.com/hsr-project/tmc_realtime_control.git
git clone -b humble https://github.com/hsr-project/tmc_voice.git
git clone -b humble https://github.com/hsr-project/tmc_navigation.git
rm -rf hsrb_launch/hsrb_robot_launch
rm -rf hsrb_simulator/hsrb_rviz_simulator
rm -rf tmc_drivers/tmc_pgr_camera
```

ビルドします．

```
cd ~/hsr_ros2_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


# Gazebo Simulation

Gazeboシミュレータを起動します．
以下は起動launchの例で，使うロボット，ワールドに応じて適切なlaunchファイルを利用してください．

```
ros2 launch hsrb_gazebo_launch hsrb_apartment_no_objects_world.launch.py
```


# Rviz Simulation

センサ系のシミュレーションはありません．簡単な動作確認には利用可能です．

**Case: HSR-B**

```
ros2 launch hsrb_rviz_simulator hsrb_rviz_simulator.launch.py
```

**Case: HSR-C**

```
ros2 launch hsrb_rviz_simulator hsrc_rviz_simulator.launch.py
```
