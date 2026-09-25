ナビゲーション 
=================

RVizの ``Nav2 Goal`` ツールを使用することで、シミュレータ上のHSRを動作させることができます。
以下を実施してください。

1. 以下を実行し、環境変数の設定を行います。 ``ROS_DOMAIN_ID`` は各自で設定してください。
   なお、以降起動するターミナルでも同様に設定を行ってください。

   .. code-block:: shell

      $ export ROS_DOMAIN_ID=XXX
      $ source /opt/ros/<ROS_DISTRO>/setup.bash

#. 以下を実行し、Gazeboを起動してください。

   .. code-block:: shell

      $ cd ~/hsr_ros2_ws/
      $ source install/setup.bash
      $ ros2 launch hsrb_gazebo_launch hsrb_apartment_no_objects_world.launch.py use_navigation:=false rviz:=false

#. 以下を実行し、ナビゲーションを起動してください。

   .. code-block:: shell

      $ cd ~/hsr_ros2_ws/
      $ source install/setup.bash
      $ ros2 launch hsrb_rosnav_config navigation_launch.py map:=src/tmc_database/tmc_potential_maps/maps/apartment/map.yaml use_sim_time:=true initial_orientation_xyzw:=[0,0,0,1]

#. 以下を実行し、RVizを起動してください。

   .. code-block:: shell

      $ cd ~/hsr_ros2_ws/
      $ source install/setup.bash
      $ rviz2 -d src/hsrb_rosnav/hsrb_rosnav_config/rviz/hsr_navigation2.rviz

#. RVizを起動すると、以下のような画面が表示されます。

   .. image:: images/sim_nav_start_rviz.png
      :scale: 50

#. 「RobotModel」にチェックを入れると、HSRが表示されます。

   .. image:: images/sim_Robot_Model.png
      :scale: 130

#. RViz画面の中央上部にある ``Nav2 Goal`` を選択してください。

   .. image:: images/sim_Nav2_Goal.png
      :scale: 80

#. 目標地点をクリックして、目標の向きへドラッグしてください。
   緑の矢印が画面に表示され、シミュレータ上のHSRがその位置に移動します。

   .. image:: images/sim_Nav2_Goal_moving.png
      :scale: 50

