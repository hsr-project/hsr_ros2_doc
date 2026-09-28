#################################
Pythonインターフェースについて
#################################

概要
============

HSRをプログラムするためにROSのインターフェースとROSをラップしたPythonのインターフェースが用意されています。
ここでは、Pythonインターフェースを使って簡単にHSRを動かす方法を学びます。



ワークスペースの作成
====================================

Pythonインターフェースを使うためのワークスペースを作成します。以下の手順を実施してください。

1. 以下を実行し、ビルドに必要なツール類をインストールしてください。

   .. code-block:: shell

      $ sudo apt update
      $ sudo apt install -y python3-colcon-common-extensions python3-rosdep
      $ sudo rosdep init
      $ rosdep update

#. 以下を実行し、ワークスペースの作成と必要なパッケージの配置、ビルドを行ってください。

   .. code-block:: shell

      $ source /opt/ros/<ROS_DISTRO>/setup.bash
      $ mkdir -p ~/hsr_ros2_ws/src && cd ~/hsr_ros2_ws/src
      $ git clone -b $ROS_DISTRO https://github.com/hsr-project/hsrb_interfaces.git

   .. code-block:: shell
   
      $ cd ~/hsr_ros2_ws/
      $ rosdep install --from-paths . -y --ignore-src
      $ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      $ source install/setup.bash


Pythonインターフェースの使い方
==============================

本節では、Pythonインターフェースの使い方を説明します。

1. 以下の手順を実施し、Pythonインターフェース用のコンソールを起動してください。

   * HSRの起動方法

     a. :ref:`シミュレータの起動 <start_simulator>` を参照して、シミュレータでHSRを起動してください。

   * コンソールの起動方法

     a. 以下を実行し、環境変数の設定を行ってください。
        なお、 ``ROS_DOMAIN_ID`` には、シミュレータ起動時に設定した値を設定してください。

        .. code-block:: shell

           $ export ROS_DOMAIN_ID=XXX
           $ source /opt/ros/<ROS_DISTRO>/setup.bash

     #. 以下を実行し、コンソールを起動してください。

        .. code-block:: shell

           $ cd ~/hsr_ros2_ws/
           $ source install/setup.bash
           $ ros2 run hsrb_interface_py ihsrb.py --ros-args -p use_sim_time:=true
   
#. 起動すると以下のようなコンソールが確認できます。
   
   .. code-block:: shell
   
      HSR-B Interactive Shell 0.2.0
      
      
            ____________  ______  _________       __  _______ ____
           /_  __/ __ \ \/ / __ \/_  __/   |     / / / / ___// __ \
            / / / / / /\  / / / / / / / /| |    / /_/ /\__ \/ /_/ /
           / / / /_/ / / / /_/ / / / / ___ |   / __  /___/ / _, _/
          /_/  \____/ /_/\____/ /_/ /_/  |_|  /_/ /_//____/_/ |_|
      
      
      In [1]: 
   
   このコンソールにコマンドを入力することで、HSRを操作できます。
   
#. HSRの動作確認として、ここでは台車の移動機能を使います。
   
   コンソールに ``omni_base.`` まで入力し、「Tab」キーを2回押してください。
   
   .. code-block:: shell
   
      In []: omni_base.<Tab><Tab>
   
   以下のように、いくつかの候補が確認できます。
   
   .. code-block:: shell
   
      In []: omni_base.add_on_set_parameters_callback
       add_on_set_parameters_callback()              destroy_client()                              get_parameter_or()                            is_moving()                                  
       add_waitable()                                destroy_guard_condition()                     get_parameter_type()                          is_succeeded()                               
       cancel_goal()                                 destroy_node()                                get_parameter_types()                         move()                                       
       clients                                       destroy_publisher()                           get_parameters()                              PARAM_REL_TOL                                
       context                                       destroy_rate()                                get_parameters_by_prefix()                    pose                                         
       count_publishers()                            destroy_service()                             get_pose()                                    publishers                                   
       count_subscribers()                           destroy_subscription()                        get_publisher_names_and_types_by_node()       remove_on_set_parameters_callback()          
       create_client()                               destroy_timer()                               get_publishers_info_by_topic()                remove_waitable()                            
       create_follow_trajectory_goal()               execute()                                     get_service_names_and_types()                 resolve_service_name()                       
       create_go_pose_goal()                         executor                                      get_service_names_and_types_by_node()         resolve_topic_name()                         
       create_guard_condition()                      follow_trajectory()                           get_state()                                   services                                     
       create_publisher()                            get_client_names_and_types_by_node()          get_subscriber_names_and_types_by_node()      set_descriptor()                             
       create_rate()                                 get_clock()                                   get_subscriptions_info_by_topic()             set_parameters()                             
       create_service()                              get_fully_qualified_name()                    get_topic_names_and_types()                   set_parameters_atomically()                  
       create_subscription()                         get_logger()                                  go()                                          subscriptions                                
       create_timer()                                get_name()                                    go_abs()                                      timers                                       
       declare_parameter()                           get_namespace()                               go_pose()                                     undeclare_parameter()                        
       declare_parameters()                          get_node_names()                              go_rel()                                      waitables                                    
       default_callback_group                        get_node_names_and_namespaces()               guards                                                                                     
       describe_parameter()                          get_node_names_and_namespaces_with_enclaves() handle                                                                                     
       describe_parameters()                         get_parameter()                               has_parameter()                                                                            
       function(callback: Callable[[List[rclpy.parameter.Parameter]], rcl_interfaces.msg._set_parameters_result.SetParametersResult]) 
   
#. 上記の候補の中から ``go_rel`` の機能を使用します。

   ``omni_base.go_rel`` の後ろに「?」をつけて実行することで、リファレンスが確認できます。
   
   .. code-block:: shell
   
      In []: omni_base.go_rel?
      Signature: omni_base.go_rel(x=0.0, y=0.0, yaw=0.0, timeout=0.0)
      Docstring:
      Move base from current position.
      
      Args:
          x   (float): X-axis position on ``robot`` frame [m]
          y   (float): Y-axis position on ``robot`` frame [m]
          yaw (float): Yaw position on ``robot`` frame [rad]
          timeout (float): Timeout until movement finish [sec].
              Default is 0.0 and wait forever.
      
      Examples:
          .. sourcecode:: python
      
             with hsrb_interface.Robot() as robot:
                 base = robot.try_get('omni_base')
                 base.go_rel(1.0, 0.0, 0.0)
      File:      ~/hsr_ros2_ws/install/hsrb_interface_py/local/lib/python3.10/dist-packages/hsrb_interface/mobile_base.py
      Type:      method
   
   表示されたリファレンスから、4つの引数を持つことを確認できます。
   
   このように、対話的に機能を調べながら実行できます。
   また、その他の機能も同様に調べることができます。

   
#. 以下を実行し、HSRが左方向に0.5[m]移動することを確認してください。
   
   .. code-block:: shell
   
      In []: omni_base.go_rel(0.0, 0.5, 0.0, 100.0)
   
   このコマンドは、xを0.0[m]、yを0.5[m]、yaw軸の回転を0.0[rad]に設定し、タイムアウト時間を100.0[s]に指定するものです。

#. 終了時は、コンソールで「Ctrl」+「D」を押すか、「quit」と入力してください

