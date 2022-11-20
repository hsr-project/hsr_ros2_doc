Build document
=======================

I'm sorry, the documentation with sphinx is only in Japanese.

.. code-block:: bash

   sudo pip3 install sphinx sphinx-rtd-theme
   make html

The outline of the environment construction is described below.


Gazebo simulation
=======================

Install ROS2
+++++++++++++++++++++

Please refer to the following URL and install ROS2 Foxy and tools.

* https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
* https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html
* https://docs.ros.org/en/foxy/How-To-Guides/Building-a-Custom-Debian-Package.html


Create and build the workspace
+++++++++++++++++++++++++++++++++

Download the source code from github by entering the following command:

.. code-block:: bash

   mkdir ~/hsr_project
   cd ~/hsr_project
   git clone -b foxy https://github.com/hsr-project/gazebo_ros2_control.git
   git clone -b foxy https://github.com/hsr-project/hsrb_description.git
   git clone -b foxy https://github.com/hsr-project/hsrb_meshes.git
   git clone -b foxy https://github.com/hsr-project/hsrb_controllers.git
   git clone -b foxy https://github.com/hsr-project/hsrb_gazebo_bringup.git
   git clone -b foxy https://github.com/hsr-project/tmc_gazebo_plugins.git
   git clone -b foxy https://github.com/hsr-project/tmc_control_msgs.git


Create a workspace by entering the following command:

.. code-block:: bash

   mkdir ~/hsr_ros2_ws
   cd ~/hsr_ros2_ws
   ln -s ~/hsr_project/gazebo_ros2_control/gazebo_ros2_control
   ln -s ~/hsr_project/hsrb_description
   ln -s ~/hsr_project/hsrb_meshes
   ln -s ~/hsr_project/hsrb_controllers/hsrb_base_controllers
   ln -s ~/hsr_project/hsrb_gazebo_bringup
   ln -s ~/hsr_project/tmc_gazebo_plugins
   ln -s ~/hsr_project/tmc_control_msgs


Build the workspace by entering the following command:

.. code-block:: bash

   source /opt/ros/foxy/setup.bash
   rosdep install --from-paths . -y --ignore-src
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash


Launch simulation
+++++++++++++++++++++++++++++++++

First, bring up gazebo by entering the following command:

.. code-block:: bash

   ros2 launch hsrb_gazebo_bringup gazebo_bringup.launch.py


Next, open a new ternimal and spawn hsr by entering the following command:

**Case: HSR-B**

.. code-block:: bash

   cd ~/hsr_ros2_ws
   source install/setup.bash
   ros2 launch hsrb_gazebo_bringup spawn_hsrb.launch.py

**Case: HSR-C**

.. code-block:: bash

   cd ~/hsr_ros2_ws
   source install/setup.bash
   ros2 launch hsrb_gazebo_bringup spawn_hsrc.launch.py


Actual Robot
=======================

Install ROS2
+++++++++++++++++++++

Please refer to the following URL and install ROS2 Foxy and tools in HSR.

* https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
* https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html
* https://docs.ros.org/en/foxy/How-To-Guides/Building-a-Custom-Debian-Package.html


Create and build the workspace
+++++++++++++++++++++++++++++++++

In HSR, download the source code from github by entering the following command:

.. code-block:: bash

   mkdir ~/hsr_ros2_ws
   cd ~/hsr_ros2_ws
   git clone -b ros2 https://github.com/mikeferguson/openni2_camera.git
   git clone -b foxy https://github.com/hsr-project/exxx_control_table.git
   git clone -b foxy https://github.com/hsr-project/hsrb_bringup.git
   git clone -b foxy https://github.com/hsr-project/hsrb_controllers.git
   git clone -b foxy https://github.com/hsr-project/hsrb_description.git
   git clone -b foxy https://github.com/hsr-project/hsrb_drivers.git
   git clone -b foxy https://github.com/hsr-project/hsrb_meshes.git
   git clone -b foxy https://github.com/hsr-project/hsrb_robot_hardware.git
   git clone -b foxy https://github.com/hsr-project/tmc_control_msgs.git


Build the workspace by entering the following command:

.. code-block:: bash

   source /opt/ros/foxy/setup.bash
   rosdep install --from-paths . -y --ignore-src
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash


Bringup HSR
+++++++++++++++++++++++++++++++++

First, stop ROS1 auto-start:

.. code-block:: bash

   sudo systemctl stop docker.hsrb.roscore.service


Next, bring up HSR by entering the following command:

**Case: HSR-B**

.. code-block:: bash

   ros2 launch hsrb_bringup hsrb.launch.py

**Case: HSR-C**

.. code-block:: bash

   ros2 launch hsrb_bringup hsrc.launch.py


Moveit
=======================

Install Pinocchio
+++++++++++++++++++++

Please refer to the following URL and install Pinocchio.

* https://stack-of-tasks.github.io/pinocchio/download.html


Create and build the workspace
+++++++++++++++++++++++++++++++++

Download the source code from github by entering the following command:

.. code-block:: bash

   mkdir ~/hsr_project
   cd ~/hsr_project
   git clone -b foxy https://github.com/hsr-project/hsrb_description.git
   git clone -b foxy https://github.com/hsr-project/hsrb_meshes.git
   git clone -b foxy https://github.com/hsr-project/hsrb_kinematics.git
   git clone -b foxy https://github.com/hsr-project/hsrb_moveit_config.git
   git clone -b foxy https://github.com/hsr-project/hsrb_moveit_plugins.git


Create a workspace by entering the following command:

.. code-block:: bash

   mkdir ~/hsr_moveit_ws
   cd ~/hsr_moveit_ws
   ln -s ~/hsr_project/hsrb_description
   ln -s ~/hsr_project/hsrb_meshes
   ln -s ~/hsr_project/hsrb_kinematics
   ln -s ~/hsr_project/hsrb_moveit_config
   ln -s ~/hsr_project/hsrb_moveit_plugins


Build the workspace by entering the following command:

.. code-block:: bash

   source /opt/ros/foxy/setup.bash
   export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
   rosdep install --from-paths . -y --ignore-src
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash


Launch Moveit
+++++++++++++++++++++++++++++++++

Enter the following command with the actual robot or simulator running:

.. code-block:: bash

   ros2 launch hsrb_moveit_config hsrb_demo.launch.py

.. note::

   For HSR-C, use hsrc_demo.launch.py instaed of hsrb_demo.launch.py


There are examples of Move Group C++ Interface.

.. code-block:: bash

   cd ~/hsr_moveit_ws
   source install/setup.bash
   ros2 launch hsrb_moveit_config hsrb_example.launch.py example_name:=moveit_fk_demo


RosNav
=======================

There are RosNav config for HSR.

https://github.com/hsr-project/hsrb_rosnav
