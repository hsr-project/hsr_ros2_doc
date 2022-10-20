HSRのセットアップ
=======================

ROS2のインストール
-------------------

https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html を参考に、``ros-foxy-desktop`` と ``ros-foxy-ros-base`` をインストールしてください。

colconのインストール
+++++++++++++++++++++

https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html を参考に、``python3-colcon-common-extensions`` をインストールしてください。

rosdepのインストール
+++++++++++++++++++++

https://docs.ros.org/en/foxy/How-To-Guides/Building-a-Custom-Debian-Package.html を参考に、``python3-rosdep`` をインストールし、rosdepの初期化を実行してください。

HSR用ソフトウェアのインストール
++++++++++++++++++++++++++++++++++++++

ロボット内で、以下のリポジトリを取得します。

.. code-block:: bash

   $ mkdir ~/repositories
   $ cd ~/repositories
   $ git clone -b ros2 https://github.com/mikeferguson/openni2_camera.git
   $ git clone -b foxy https://github.com/ros-perception/image_common.git
   $ git clone -b foxy https://bitbucket.org/tmc-dev-xr/exxx_control_table.git
   $ git clone -b foxy https://bitbucket.org/tmc-dev-xr/hsrb_common.git
   $ git clone -b feature/foxy https://bitbucket.org/tmc-dev-xr/hsrb_control.git
   $ git clone -b feature/foxy https://bitbucket.org/tmc-dev-xr/hsrb_controllers.git
   $ git clone -b foxy https://bitbucket.org/tmc-dev-xr/hsrb_drivers.git
   $ git clone -b foxy https://bitbucket.org/tmc-dev-xr/hsrb_robot.git
   $ git clone -b foxy https://bitbucket.org/tmc-dev-xr/tmc_realtime_control.git

ワークスペースを作成し、対象となるパッケージのシンボリックリンクを作成します。

.. code-block:: bash

   $ mkdir ~/ros2_ws
   $ cd ~/ros2_ws
   $ ln -s ~/repositories/openni2_camera
   $ ln -s ~/repositories/image_common/image_transport
   $ ln -s ~/repositories/exxx_control_table
   $ ln -s ~/repositories/hsrb_common/hsrb_description
   $ ln -s ~/repositories/hsrb_common/hsrb_parts_description
   $ ln -s ~/repositories/hsrb_control/hsrb_robot_hardware
   $ ln -s ~/repositories/hsrb_controllers/hsrb_base_controllers
   $ ln -s ~/repositories/hsrb_controllers/hsrb_gripper_controller
   $ ln -s ~/repositories/hsrb_drivers/hsrb_servomotor_protocol
   $ ln -s ~/repositories/hsrb_robot/hsrb_bringup
   $ ln -s ~/repositories/tmc_realtime_control/tmc_control_msgs

ワークスペースをビルドし、setup.bashを読み込みます。

.. code-block:: bash

   $ cd ~/ros2_ws
   $ source /opt/ros/foxy/setup.bash
   $ rosdep install --from-paths . -y --ignore-src
   $ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --allow-overriding image_transport
   $ source install/setup.bash

HSRの起動
+++++++++++++++++++++++++++++

管理者権限を持っているユーザーで以下コマンドを実行し、ROS1を停止します。

.. code-block:: bash

   $ sudo systemctl stop docker.hsrb.roscore.service

停止ボタンを解除し、以下コマンドを実行して、HSRを起動します。

.. code-block:: bash

   $ cd ~/ros2_ws
   $ source install/setup.bash
   $ ros2 launch hsrb_bringup hsrb.launch.py

以下のコマンドで，HSRの姿勢遷移が確認できたら成功です。

.. code-block:: bash

   $ cd ~/ros2_ws
   $ source install/setup.bash
   $ ros2 action send_goal /arm_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
   trajectory:
     joint_names:
       - arm_lift_joint
       - arm_flex_joint
       - arm_roll_joint
       - wrist_flex_joint
       - wrist_roll_joint
     points:
       - positions: [0.0, 0.0, 0.0, -1.57, 0.0]
         time_from_start: {sec: 3}"


