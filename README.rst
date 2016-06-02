Description
===========

This package allows to perform simple actions with MoveIt! configured for Romeo, NAO, and Pepper robots.

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

In case of compilin from source, install the following dependencies:
  * MoveIt!
      * apt-get install ros-indigo-moveit-full
      * apt-get install ros-indigo-moveit-visual-tools
  * MoveIt! Grasp Generator 
      * from source, the recommended branch: git clone -b romeo-dev --single-branch https://github.com/nlyubova/moveit_simple_grasps

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


For Romeo real (not a simulator)
--------------------------------
If you are working with a real robot (not a simulator) then install the dcm-related package:
  * for Romeo: 
      * apt-get install ros-indigo-romeo-dcm-bringup
      * or from source https://github.com/ros-aldebaran/romeo_robot/tree/master/romeo_dcm
  * for Nao: 
      * https://github.com/ros-aldebaran/nao_dcm_robot and https://github.com/ros-naoqi/nao_virtual

Update the robot IP in the config-file (Romeo example):

.. code-block:: bash

    rosed romeo_dcm_bringup romeo_dcm.yaml



How it works
============

For Romeo simulator
-------------------

Launch MoveIt!:

.. code-block:: bash

    roslaunch romeo_moveit_config demo.launch

Wait until the robot model is loaded and launch moveit_simple_actions:

.. code-block:: bash

    roslaunch romeo_moveit_actions actions_romeo.launch

Welcome to the world of simple actions! Now, you can add virtual objects or detect real objects and reach/grasp/place them !


For Romeo real (not a simulator)
--------------------------------

Launch the dcm_bringup (check the robot IP as described in installation):

.. code-block:: bash

    roslaunch romeo_dcm_bringup romeo_dcm_bringup_remote.launch

Launch MoveIt!:

.. code-block:: bash

    roslaunch romeo_moveit_config demo_real.launch
    
Wait until the robot model is loaded and launch moveit_simple_actions:
    
.. code-block:: bash
    
    roslaunch romeo_moveit_actions actions_romeo.launch
        
Welcome to the world of simple actions! Now, you can add virtual objects or detect real objects and reach/grasp/place them !

