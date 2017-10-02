romeo_moveit_actions
====================

The package allows to perform simple actions with MoveIt! configured for Romeo, NAO, and Pepper robots.

It offers a set of functionalities, like: 
  * pick and place objects,
  * reach and grasp objects,
  * test a target space for reaching and grasping,
  * manage objects
      * virtual objects: add, move, remove,
      * detect objects with the robot's sensors: call ORK Linemod object recognition.

Installation
============

* apt-get install ros-indigo-romeo-moveit-actions
* or from source: git clone https://github.com/ros-aldebaran/romeo_moveit_actions

In case of compiling from source, install the following dependencies:
  * MoveIt!
      * apt-get install ros-indigo-moveit-full
      * apt-get install ros-indigo-moveit-visual-tools
  * MoveIt! Grasp Generator 
      * from source, the recommended branch: git clone https://github.com/nlyubova/moveit_simple_grasps -b romeo-dev

Additionally, install a required robot description package and MoveIt! configuration, at minimum.

Get a robot description:
  * for Romeo:
      * apt-get install ros-indigo-romeo-description
      * or from source https://github.com/ros-aldebaran/romeo_robot
  * for Nao: 
      * apt-get install ros-indigo-nao-description ros-indigo-nao-meshes  
      * or from source https://github.com/ros-naoqi/nao_robot, https://github.com/ros-naoqi/nao_meshes 
  * for Pepper: 
      * apt-get install ros-indigo-pepper-description ros-indigo-pepper-meshes
      * or from source https://github.com/ros-naoqi/pepper_robot, https://github.com/ros-naoqi/pepper_meshes

Get a robot-specific Moveit! configuration:
  * for Romeo: 
      * apt-get install ros-indigo-romeo-moveit-config
      * or from source https://github.com/ros-aldebaran/romeo_moveit_config
  * for Nao: 
      * apt-get install ros-indigo-nao-moveit-config
      * or from source https://github.com/ros-naoqi/nao_moveit_config
  * for Pepper: 
      * apt-get install ros-indigo-pepper-moveit-config
      * or from source https://github.com/ros-naoqi/pepper_moveit_config 


For a real robot (not a simulator)
--------------------------------
If you are working with a real robot (not a simulator) then install the dcm-related package:
  * apt-get install ros-indigo-naoqi-dcm-driver
  * for Romeo: 
      * apt-get install ros-indigo-romeo-control ros-indigo-romeo-dcm-bringup
      * from source https://github.com/ros-aldebaran/romeo_virtual, https://github.com/ros-aldebaran/romeo_dcm_robot
  * for Pepper:
      * apt-get install ros-indigo-pepper-control ros-indigo-pepper-dcm-bringup
      * from source https://github.com/ros-naoqi/pepper_virtual, https://github.com/ros-naoqi/pepper_dcm_robot
  * for Nao: 
      * sudo-apt-get install ros-indigo-nao-control ros-indigo-nao-dcm-bringup
      * or from source https://github.com/ros-naoqi/nao_virtual, https://github.com/ros-aldebaran/nao_dcm_robot

Compile all source packages.


How it works
============

For Romeo simulator
-------------------

Launch MoveIt!:

.. code-block:: bash

    roslaunch romeo_moveit_config demo.launch

Wait until the robot model is loaded and launch moveit_simple_actions:

.. code-block:: bash

    roslaunch romeo_moveit_actions actions_romeo_sim.launch

Welcome to the world of simple actions! Now, you can add virtual objects or detect real objects and reach/grasp/place them !


For a real robot (not a simulator)
--------------------------------

Launch the dcm_bringup (providing a correct robot IP), for example for Romeo:

.. code-block:: bash

    roslaunch romeo_dcm_bringup romeo_dcm_bringup_remote.launch robot_ip:=<ROBOT_IP>

Launch MoveIt!:

.. code-block:: bash

    roslaunch romeo_moveit_config moveit_planner.launch
    
Wait until the robot model is loaded and launch moveit_simple_actions:
    
.. code-block:: bash
    
    roslaunch romeo_moveit_actions actions_romeo.launch
        
Welcome to the world of simple actions! Now, you can add virtual objects or detect real objects and reach/grasp/place them !

