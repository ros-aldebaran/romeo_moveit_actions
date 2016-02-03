^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package romeo_moveit_actions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.7 (2016-02-03)
------------------
* Merge pull request `#1 <https://github.com/nlyubova/romeo_moveit_actions/issues/1>`_ from IanTheEngineer/remove_shape_tools
  Convert deprecated shape_tools dependency
* Convert deprecated shape_tools dependency
  shape_tools functionality was merged into geometric_shapes:
  https://github.com/ros-planning/geometric_shapes/pull/32
  and removed from moveit_core
  https://github.com/ros-planning/moveit_core/pull/242
  which caused this issue.
  This commit updates the pick and place tutorial and adds
  geometric_shapes to the package.xml and CMakeLists.txt to
  prevent the ROS buildfarm from failing to build this package.
* Contributors: Ian McMahon, Natalia Lyubova

0.0.6 (2015-12-30)
------------------
* fixing visualization of geometric shapes and cleaning
* renaming the package to romeo_moveit_actions

0.0.5 (2015-12-14)
------------------
* adding changelogs

0.0.4 (2015-12-10)
------------------
* adding changelogs
* fixing with changes in moveit visual tools

0.0.3 (2015-11-25 17:46)
------------------------
* 0.0.2
* adding CHANGELOG.rst

0.0.2 (2015-11-25 17:09)
------------------------
* adding CHANGELOG.rst
* adding a planning group (arm) name as an argument
* taking into account object height when grasping
* initial commit, new version of romeo_action package
