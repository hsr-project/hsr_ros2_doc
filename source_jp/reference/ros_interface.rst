**************
ROS Interface
**************

.. contents:: 目次
   :local:
   :depth: 1

頭部関節制御
------------

`ros2\_controllers/joint\_trajectory\_controller <http://control.ros.org/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__\ に準じます。

以下の関節を制御します。

-  ``head_pan_joint``
-  ``head_tilt_joint``

Provided Actions
~~~~~~~~~~~~~~~~

-  ``/head_trajectory_controller/follow_joint_trajectory``

   type
       ``control_msgs/action/FollowJointTrajectory``

   description
       軌道再生のためのアクションインターフェース

Published Topics
~~~~~~~~~~~~~~~~

-  ``/head_trajectory_controller/state``

   type
       ``control_msgs/msg/JointTrajectoryControllerState``

   description
       コントローラの現在状態

Subscribed Topics
~~~~~~~~~~~~~~~~~

-  ``/head_trajectory_controller/joint_trajectory``

   type
       ``trajectory_msgs/msg/JointTrajectory``

   description
       目標軌道を設定するトピックインターフェース

Parameters
~~~~~~~~~~

`ros2\_controllers/joint\_trajectory\_controller <http://control.ros.org/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__\ を参照。


腕部関節制御
------------

`ros2\_controllers/joint\_trajectory\_controller <http://control.ros.org/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__\ に準じます。

以下の関節を制御します。

-  ``arm_lift_joint``
-  ``arm_flex_joint``
-  ``arm_roll_joint``
-  ``wrist_flex_joint``
-  ``wrist_roll_joint``

Provided Actions
~~~~~~~~~~~~~~~~

-  ``/arm_trajectory_controller/follow_joint_trajectory``

   type
       ``control_msgs/action/FollowJointTrajectory``

   description
       軌道再生のためのアクションインターフェース

Published Topics
~~~~~~~~~~~~~~~~

-  ``/arm_trajectory_controller/state``

   type
       ``control_msgs/msg/JointTrajectoryControllerState``

   description
       コントローラの現在状態

Subscribe Topics
~~~~~~~~~~~~~~~~

-  ``/arm_trajectory_controller/joint_trajectory``

   type
       ``trajectory_msgs/msg/JointTrajectory``

   description
       目標軌道を設定するトピックインターフェース

Parameters
~~~~~~~~~~

`ros2\_controllers/joint\_trajectory\_controller <http://control.ros.org/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`__\ を参照。


ハンド制御
----------

ハンド部を制御します。

Provided Actions
~~~~~~~~~~~~~~~~

-  ``/gripper_controller/follow_joint_trajectory``

   type
       ``control_msgs/action/FollowJointTrajectory``

   description
       指関節軌道再生のアクションインターフェース

-  ``/gripper_controller/grasp``

   type
       ``tmc_control_msgs/action/GripperApplyEffort``

   description
       握りこみ動作のためのアクションインターフェース

-  ``/gripper_controller/apply_force``

   type
       ``tmc_control_msgs/action/GripperApplyEffort``

   description
       指定した力で把持するアクションインターフェース

Subscribe Topics
~~~~~~~~~~~~~~~~

-  ``/gripper_controller/joint_trajectory``

   type
       ``trajectory_msgs/msg/JointTrajectory``

   description
       目標軌道を設定するトピックインターフェース


台車制御
--------

以下の関節を制御します。

-  ``base_roll_joint``
-  ``base_l_drive_wheel_joint``
-  ``base_r_drive_wheel_joint``

ただし、\ ``JointTrajectoryController``\ は ``/odom`` での位置・方位を仮想的に関節とみなした以下の名前を持つジョイントを制御します。

-  ``odom_x``
-  ``odom_y``
-  ``odom_t``

Provide Actions
~~~~~~~~~~~~~~~

-  ``/omni_base_controller/follow_joint_trajectory``

   type
       ``control_msgs/action/FollowJointTrajectory``

   description
       台車軌道再生を行うアクションインターフェース

Published Topics
~~~~~~~~~~~~~~~~

-  ``/hsrb/omni_base_controller/state``

   type
       ``control_msgs/msg/JointTrajectoryControllerState``

   description
       コントローラの現在状態

-  ``/omni_base_controller/wheel_odom``

   type
       ``nav_msgs/msg/Odometry``

   description
       ロボット起動時からのホイールオドメトリ

Subscribed Topics
~~~~~~~~~~~~~~~~~

-  ``/omni_base_controller/joint_trajectory``

   type
       ``trajectory_msgs/msg/JointTrajectory``

   description
       目標軌道を設定するトピックインターフェース

-  ``/omni_base_controller/cmd_vel``

   type
       ``geometry_msgs/msg/Twist``

   description
       目標速度を設定するトピックインターフェース

Parameters
~~~~~~~~~~

-  ``/omni_base_controller:base_coordinates``

   type
       ``String[]``

   description
       制御対象関節のリスト

-  ``/omni_base_controller:command_timeout``

   type
       ``Double``

   description
       目標速度入力の途絶判定時間［s］
       途絶したら目標速度をゼロにする (デフォルト値：0.5)

-  ``/omni_base_controller:joints.l_wheel``

   type
       ``String``

   description
       制御指令値を送る台車関節名(左駆動輪)

-  ``/omni_base_controller:joints.r_wheel``

   type
       ``String``

   description
       制御指令値を送る台車関節名(右駆動輪)

-  ``/omni_base_controller:joints.steer``

   type
       ``String``

   description
       制御指令値を送る台車関節名(台車旋回軸)

-  ``/omni_base_controller:odometry_publish_rate``

   type
       ``Double``

   description
       ``/omni_base_controller/wheel_odom`` の出版周期［Hz］ (デフォルト値：30.0)

-  ``/omni_base_controller:state_publish_rate``

   type
       ``Double``

   description
       コントローラ状態の出版周期［Hz］

-  ``/omni_base_controller:transform_publish_rate``

   type
       ``Double``

   description
       オドメトリのTF出版周期［Hz］ (デフォルト値：30.0)

-  ``/omni_base_controller:wheel_odom_map_frame``

   type
       ``String``

   description
       ホイールオドメトリ入力の基準フレーム名 (デフォルト値： ``odom`` )

-  ``/omni_base_controller:wheel_odom_base_frame``

   type
       ``String``

   description
       ホイールオドメトリ入力のオドメトリフレーム名 (デフォルト値： ``base_footprint_wheel`` )


手先カメラ
----------

`image\_transport/ImageTransport <http://wiki.ros.org/image_transport>`__\ の仕様に準じます。

Published Topics
~~~~~~~~~~~~~~~~

-  ``/hand_camera/camera_info``

   type
       ``sensor_msgs/msg/CameraInfo``

   description
       カメラ情報

-  ``/hand_camera/image_raw``

   type
       ``sensor_msgs/msg/Image``

   description
       "raw"伝送方式による画像


頭部広角カメラ
--------------

.. note::

   HSRBでのみ利用可能です。

`image\_transport/ImageTransport <http://wiki.ros.org/image_transport>`__\ の仕様に準じます。HSRBのみ利用可能。

Published Topics
~~~~~~~~~~~~~~~~

-  ``/head_center_camera/camera_info``

   type
       ``sensor_msgs/msg/CameraInfo``

   description
       カメラ情報

-  ``/head_center_camera/image_raw``

   type
       ``sensor_msgs/msg/Image``

   description
       "raw"伝送方式による画像


頭部ステレオカメラ（左）
------------------------

.. note::

   シミュレータでのみ利用可能です。

`image\_transport/ImageTransport <http://wiki.ros.org/image_transport>`__\ の仕様に準じます。

Published Topics
~~~~~~~~~~~~~~~~

-  ``/head_l_stereo_camera/camera_info``

   type
       ``sensor_msgs/msg/CameraInfo``

   description
       カメラ情報

-  ``/head_l_stereo_camera/image_rect_color``

   type
       ``sensor_msgs/msg/Image``

   description
       "raw"伝送方式による画像(ひずみ補正済み)


頭部ステレオカメラ（右）
------------------------

.. note::

   シミュレータでのみ利用可能です。

Published Topics
~~~~~~~~~~~~~~~~

-  ``/head_r_stereo_camera/camera_info``

   type
       ``sensor_msgs/msg/CameraInfo``

   description
       カメラ情報

-  ``/head_r_stereo_camera/image_rect_color``

   type
       ``sensor_msgs/msg/Image``

   description
       "raw"伝送方式による画像(ひずみ補正済み)


頭部3次元距離センサ
--------------------

Published Topics
~~~~~~~~~~~~~~~~

-  ``/head_rgbd_sensor/rgb/camera_info``

   type
       ``sensor_msgs/msg/CameraInfo``

   description
       RGBカメラ情報

-  ``/head_rgbd_sensor/rgb/image_rect_color``

   type
       ``sensor_msgs/msg/Image``

   description
       歪み補正されたRGB画像

-  ``/head_rgbd_sensor/depth_registered/camera_info``

   type
       ``sensor_msgs/msg/CameraInfo``

   description
       RGBカメラ情報

-  ``/head_rgbd_sensor/depth_registered/image_rect_raw``

   type
       ``sensor_msgs/msg/Image``

   description
       RGBカメラ座標系に投影された歪み補正された深度画像、画素値の単位は[mm] (16UC1)

-  ``/head_rgbd_sensor/depth_registered/rectified_points``

   type
       ``sensor_msgs/msg/PointCloud2``

   description
       歪み補正された色付きポイントクラウド


IMU
------------

.. note::

   シミュレータでのみ利用可能です。

Published Topics
~~~~~~~~~~~~~~~~

-  ``/base_imu/data``

   type
       ``sensor_msgs/msg/Imu``

   description
       台車胴部に搭載されたIMUの出力


レーザースキャナー
---------------------

Published Topics
~~~~~~~~~~~~~~~~

-  ``/scan``

   type
       ``sensor_msgs/msg/LaserScan``

   description
       台車部レーザスキャナ(レーザレンジファインダ)の出力


関節状態
--------

Published Topics
~~~~~~~~~~~~~~~~

-  ``/joint_states``

   type
       ``sensor_msgs/msg/JointState``

   description
       各関節の現在状態（能動関節のみ含む）

Parameters
~~~~~~~~~~

`ros2\_controllers/joint\_state\_broadcaster <http://control.ros.org/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`__\ を参照。


デジタルI/O
-----------

.. note::

   シミュレータでのみ利用可能です。

Published Topics
~~~~~~~~~~~~~~~~

-  ``/base_f_bumper_sensor``

   type
       ``std_msgs/msg/Bool``

   description
       前部バンパセンサのOn/Off状態

-  ``/base_b_bumper_sensor``

   type
       ``std_msgs/msg/Bool``

   description
       後部バンパセンサのOn/Off状態


ControllerManager制御
---------------------

`controller\_manager/ControllerManager <http://control.ros.org/ros2_control/controller_manager/doc/userdoc.html>`__\ の仕様に準じます。

Provided Services
~~~~~~~~~~~~~~~~~~

-  ``/controller_manager/list_controller_types``

   type
       ``controller_manager_msgs/srv/ListControllerTypes``

   description
       ロード可能なコントローラを列挙する

-  ``/controller_manager/list_controllers``

   type
       ``controller_manager_msgs/srv/ListControllers``

   description
       現在ロードされているコントローラを列挙する

-  ``/controller_manager/load_controller``

   type
       ``controller_manager_msgs/srv/LoadController``

   description
       指定したコントローラをロードする

-  ``/controller_manager/reload_controller_libraries``

   type
       ``controller_manager_msgs/srv/ReloadControllerLibraries``

   description
       コントローラプラグインを再ロードする

-  ``/controller_manager/switch_controller``

   type
       ``controller_manager_msgs/srv/SwitchController``

   description
       コントローラを切り替える

-  ``/controller_manager/unload_controller``

   type
       ``controller_manager_msgs/srv/UnloadController``

   description
       コントローラを取り外す
