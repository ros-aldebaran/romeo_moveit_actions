cmake_minimum_required(VERSION 2.8.3)
SET(PROJECTNAME romeo_moveit_actions)
project(${PROJECTNAME})

find_package(catkin REQUIRED COMPONENTS 
  moveit_ros_planning 
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_simple_grasps
  std_msgs
  geometry_msgs
  roscpp
  cmake_modules
  object_recognition_msgs
  actionlib
)

find_package(Boost REQUIRED system thread date_time system filesystem)
find_package(Boost QUIET COMPONENTS program_options)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

###########
## Build ##
###########

include_directories(include/romeo_moveit_actions ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})

set(SRC
  src/main.cpp
  src/simplepickplace.cpp
  src/metablock.cpp
  src/action.cpp
  src/postures.cpp
  src/objprocessing.cpp
  src/visualtools.cpp
  include/romeo_moveit_actions/simplepickplace.hpp
  include/romeo_moveit_actions/metablock.hpp
  include/romeo_moveit_actions/action.hpp
  include/romeo_moveit_actions/custom_environment5.hpp
  include/romeo_moveit_actions/postures.hpp
  include/romeo_moveit_actions/tools.hpp
  include/romeo_moveit_actions/toolsForAction.hpp
  include/romeo_moveit_actions/objprocessing.hpp
  include/romeo_moveit_actions/visualtools.hpp
)

# Main executable
add_executable(${PROJECTNAME} ${SRC})

target_link_libraries(${PROJECTNAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

