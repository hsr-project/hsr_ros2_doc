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
git clone -b humble https://github.com/hsr-project/hsrb_manipulation.git
git clone -b humble https://github.com/hsr-project/hsrb_meshes.git
git clone -b humble https://github.com/hsr-project/hsrb_moveit_config.git
git clone -b humble https://github.com/hsr-project/hsrb_moveit_plugins.git
git clone -b humble https://github.com/hsr-project/tmc_common.git
git clone -b humble https://github.com/hsr-project/tmc_common_msgs.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation_base.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation_planner.git
```

ビルドします．

```
cd ~/hsr_ros2_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


# Launch Examples

シミュレータ等を起動後，demo.launch.pyを実行します．ロボットに応じた適切なlaunchファイルを利用してください．
以下は，HSR-Bの例です．

```
ros2 launch hsrb_moveit_config hsrb_demo.launch.py
```

## GUIでの操作

RvizのMotionPlanningプラグインから指令値を投げてください．

## C++からのMoveGroupの利用

次のコマンドで，サンプルプログラムを実行できます．

```
ros2 launch hsrb_moveit_config hsrb_example.launch.py example_name:=moveit_fk_demo
```

準備されているサンプルプログラムは以下の通りです．

* `moveit_fk_demo`
* `moveit_ik_demo`
* `moveit_gripper_demo`
* `moveit_constraints_demo`
