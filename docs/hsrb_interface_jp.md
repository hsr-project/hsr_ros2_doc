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


# Create Workspace

ワークスペースを作成し，必要なパッケージを取得します．

```
mkdir -p ~/hsr_ros2_ws/src && cd ~/hsr_ros2_ws/src
git clone -b humble https://github.com/hsr-project/hsrb_description.git
git clone -b humble https://github.com/hsr-project/hsrb_interfaces.git
git clone -b humble https://github.com/hsr-project/hsrb_meshes.git
git clone -b humble https://github.com/hsr-project/tmc_common.git
git clone -b humble https://github.com/hsr-project/tmc_common_msgs.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation_base.git
git clone -b humble https://github.com/hsr-project/tmc_realtime_control.git
git clone -b humble https://github.com/hsr-project/tmc_voice.git
```

ビルドします．

```
cd ~/hsr_ros2_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

# Bringup Python Interface

```
ros2 run hsrb_interface_py ihsrb.py
```

以下はコマンド例です．詳細は，各関数のhelpを確認ください．

```python
whole_body.move_to_go()
whole_body.move_to_neutral()
whole_body.move_to_joint_positions({'arm_lift_joint': 0.1})
whole_body.gaze_point(point=geometry.Vector3(x=1.0, y=-0.5, z=0.3), ref_frame_id='base_link')
whole_body.move_end_effector_pose([geometry.pose(z=1.0), geometry.pose(z=0.8)], ref_frame_id='hand_palm_link')
whole_body.move_end_effector_by_line((0, 0, 1), 0.3)
whole_body.move_end_effector_by_arc(geometry.pose(y=0.45, z=0.08, ej=math.radians(90.0)), math.radians(60.0), ref_frame_id='hand_palm_link')
gripper.command(1.2)
gripper.apply_force(1.0)
gripper.set_distance(0.05)
gripper.command(0.0)
poses = [geometry.pose(x=1.0, y=0.0, ek=0.0), geometry.pose(x=1.0, y=1.0, ek=math.pi)]
omni_base.go_abs(1.0, -2.0, 0.0, 300.0)
omni_base.go_rel(0.0, 1.0, 0.0, 100.0)
omni_base.follow_trajectory(poses, ref_frame_id='base_footprint')
goal = omni_base.create_follow_trajectory_goal(poses, time_from_starts=[10, 30], ref_frame_id='base_footprint')
omni_base.execute(goal)
omni_base.is_succeeded()
omni_base.cancel_goal()
tts.say(u'Hello')
```
