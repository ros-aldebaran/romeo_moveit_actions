Description
===========

This package allows to perform simple actions with MoveIt! configured for Romeo, NAO, and Pepper robots.

It provides a possibility to: 
  * pick and place objects,
  * reach and grasp objects,
  * test a target space for reaching and grasping,
  * manage objects
      * virtual objects: add, move, remove,
      * detect real objects: call object detection.

Installation
============

The package requires a robot model and MoveIt configuration, at minimum. 

Get a robot model:
  * for Romeo https://github.com/ros-aldebaran/romeo_robot/tree/master/romeo_description or from the binary ros-indigo-romeo-description
  * for Nao https://github.com/ros-naoqi/nao_robot/tree/master/nao_description or from the binary ros-indigo-nao-description
  * for Pepper https://github.com/ros-naoqi/pepper_robot/tree/master/pepper_description or from the binary ros-indigo-pepper-description

Get a robot-related moveit_config package:
  * for Romeo https://github.com/ros-aldebaran/romeo_moveit_config
  * for Nao https://github.com/ros-naoqi/nao_moveit_config
  * for Pepper https://github.com/nlyubova/pepper_moveit_config 

Install moveit_simple_grasps:
 * https://github.com/nlyubova/moveit_simple_grasps/tree/romeo-dev

Optionally, if you would like to control the real robot but not a simulator, install a dcm_robot package:
  * for Romeo https://github.com/ros-aldebaran/romeo_robot/tree/master/romeo_dcm
  * for Nao https://github.com/ros-aldebaran/nao_dcm_robot 

How it works
============

Launch a moveit_config (example for Romeo):

.. code-block:: bash

    roslaunch romeo_moveit_config demo.launch

Wait until the robot model is loaded and launch moveit_simple_actions (example for Romeo):

.. code-block:: bash

    roslaunch moveit_simple_actions simple_actions_romeo.launch

Optionally, in case of using a real robot but not a simulator, launch a dcm_bringup at the most beginning:

.. code-block:: bash

    roslaunch romeo_dcm_bringup romeo_dcm_bringup_remote.launch

Welcome to the world of simple actions! Now, add virtual objects or detect real ones and reach/grasp/place with them !

