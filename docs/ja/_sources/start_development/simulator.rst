***********************
シミュレータでの開発
***********************

ここでは、シミュレータのHSRで開発を行う方法について説明します。

.. _start_simulator:

シミュレータの起動
======================

1. 以下を実行し、環境変数を設定してください。なお、 ``ROS_DOMAIN_ID`` は各自で設定してください。

   .. code-block:: bash
   
     $ ros2 daemon stop
     $ export ROS_DOMAIN_ID=XXX
     $ source /opt/ros/<ROS_DISTRO>/setup.bash

#. 以下を実行し、シミュレータを起動してください。

   .. code-block:: bash
   
     $ cd ~/hsr_ros2_ws/
     $ source install/setup.bash
     $ ros2 launch hsrb_gazebo_launch hsrb_apartment_no_objects_world.launch.py
   
   .. note::
      
      シミュレータでは様々な環境を利用できます。
   
      ``ros2 launch hsrb_gazebo_launch`` の後に「Tab」キーを2回押すことで、他のlaunchファイルを確認することができます。
   
      使用するHSRや環境に応じて選択してください。
   
      また、提供されている環境に関しては `こちら <https://github.com/hsr-project/hsrb_launch/tree/humble/hsrb_gazebo_launch/launch>`__ をご覧ください。

#. シミュレータが起動すると、GazeboとRVizの2つの画面が確認できます。

   .. image:: images/simulator_gazebo_initialization.png
     :scale: 60
      
   .. image:: images/simulator_rviz_initialization.png
     :scale: 50

|

本節以降もシミュレータを起動したままお進みください。

次章では、Pythonインターフェースの使い方について説明します。


