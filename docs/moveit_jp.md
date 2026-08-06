# Install ROS2

以下を参考に，ROS2をインストールしてください．

* https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

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

ビルドします．

```
cd ~/hsr_ros2_ws/
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


# Launch Examples

HSRを起動後，demo.launch.pyを実行します．ロボットに応じた適切なlaunchファイルを利用してください．

実機の場合

* HSRB

    ```
    ros2 launch hsrb_moveit_config hsrb_demo.launch.py
    ```

* HSRC

    ```
    ros2 launch hsrb_moveit_config hsrc_demo.launch.py
    ```

シミュレータの場合

* HSRB

    * シミュレータの起動 (HSRBの場合)

        ```
        ros2 launch hsrb_gazebo_launch hsrb_empty_world.launch.py rviz:=false
        ```

    * launchファイルの起動

        ```
        ros2 launch hsrb_moveit_config hsrb_demo.launch.py use_sim_time:=true
        ```

* HSRC

    * シミュレータの起動 (HSRBの場合)

        ```
        ros2 launch hsrb_gazebo_launch hsrc_empty_world.launch.py rviz:=false
        ```

    * launchファイルの起動

        ```
        ros2 launch hsrb_moveit_config hsrc_demo.launch.py use_sim_time:=true
        ```


## GUIでの操作

RvizのMotionPlanningプラグインから指令値を投げてください．

## C++からのMoveGroupの利用

次のコマンドで，サンプルプログラムを実行できます．

実機の場合

```
ros2 launch hsrb_moveit_config hsrb_example.launch.py example_name:=<Example PROGRAM>
```

シミュレータの場合

```
ros2 launch hsrb_moveit_config hsrb_example.launch.py use_sim_time:=true example_name:=<Example PROGRAM>
```

準備されているサンプルプログラムは以下の通りです．

* **moveit_fk_demo**
* **moveit_ik_demo**
* **moveit_gripper_demo**
* **moveit_constraints_demo**
