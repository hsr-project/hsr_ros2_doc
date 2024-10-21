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
git clone -b humble https://github.com/hsr-project/hsrb_rosnav.git
```

ビルドします．

```
cd ~/hsr_ros2_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


# Mapping

まず，地図作成用のノードを立ち上げます．

```
ros2 launch hsrb_mapping slam_toolbox_mapping.launch.py
```

Rvizで地図作成の進捗を確認します．

```
rviz2 -d src/hsrb_rosnav/hsrb_mapping/rviz/hsr_slam_toolbox_mapping.rviz
```

地図が十分作成されたら，保存します．

```
ros2 run nav2_map_server map_saver_cli -f map --ros-args -p save_map_timeout:=100000.0
```

これを実行することで，実行したディレクトリに2つのファイルが保存されます．

* map.pgm
* map.yaml


# Navigation

ナビゲーション用のプログラムを立ち上げます．

```
ros2 launch hsrb_rosnav_config navigation_launch.py map:=/full/path/to/map.yaml use_sim_time:=True initial_orientation_xyzw:=[0,0,0,1]
```

Rvizを使って自己位置を初期化し，ナビゲーションの確認を行います．

```
rviz2 -d src/hsrb_rosnav/hsrb_rosnav_config/rviz/hsr_navigation2.rviz
```

1. RViz 上部にある **2D Pose Estimate** を利用して，自己位置を初期化してください
2. RViz 上部にある **Navigation2 Goal** を利用して，目標位置を指令してください
