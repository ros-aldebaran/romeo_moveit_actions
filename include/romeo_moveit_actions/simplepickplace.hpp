/*
 * Copyright 2016 SoftBank Robotics Europe
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef SIMPLEACTIONS_HPP
#define SIMPLEACTIONS_HPP

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/action.hpp"
#include "romeo_moveit_actions/objprocessing.hpp"
#include "romeo_moveit_actions/evaluation.hpp"

namespace moveit_simple_actions
{

class SimplePickPlace
{
public:
  SimplePickPlace(const std::string robot_name,
                  const double test_step,
                  const double x_min,
                  const double x_max,
                  const double y_min,
                  const double y_max,
                  const double z_min,
                  const double z_max,
                  const std::string left_arm_name,
                  const std::string right_arm_name,
                  const bool verbose);

protected:
  //main cycle
  bool startRoutine();

  //! @brief create a table object
  MetaBlock createTable();

  //switch between the left and right arms
  void switchArm(Action *action_now);

  //create and publish an object
  void createObj(const MetaBlock &block);

  //re-draw the object at new position
  void resetBlock(MetaBlock *block);

  //get collision objects from the topic /collision_object
  void getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg);

  //! @brief clean the object list based on the timestamp
  void cleanObjects(std::vector<MetaBlock> *objects,
                    const bool list_erase=true);

  //! @brief check if the block exists
  bool checkObj(int &block_id);

  // A shared node handle
  ros::NodeHandle nh_, nh_priv_;

  //the robot's name
  std::string robot_name_;

  const bool verbose_;

  //robot's base_frame
  std::string base_frame_;

  //the dimenssion x of a default object
  double block_size_x_;
  //the dimenssion y of a default object
  double block_size_y_;

  //the shift of the robot's base to teh floor
  double floor_to_base_height_;

  /** object processing */
  ObjProcessor obj_proc_;

  //Evaluation of reaching/grasping
  Evaluation evaluation_;

  //the state of re-drawing the world
  bool env_shown_;

  //the working space of the robot
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;

  /** name of the current support surface */
  std::string support_surface_;

  //instances of an Action class for each arm
  Action *action_left_, *action_right_;

  /** visual tools pointer used for scene visualization */
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  /** current MoveIt scene */
  moveit::planning_interface::PlanningSceneInterface current_scene_;

  //the set of available objects
  std::vector<MetaBlock> blocks_;

  //the set of available surfaces
  std::vector<MetaBlock> blocks_surfaces_;

  //the subscriber to get objects through the topic /collision_object
  ros::Subscriber sub_obj_coll_;

  //the publisher of objects poses
  ros::Publisher pub_obj_poses_;

  //the publisher of the current object pose
  ros::Publisher pub_obj_pose_;

  //the current object position
  geometry_msgs::PoseStamped msg_obj_pose_;

  //all objects positions
  geometry_msgs::PoseArray msg_obj_poses_;

  //the default object pose for the left arm
  geometry_msgs::Pose pose_default_;

  //the default object pose for the right arm
  geometry_msgs::Pose pose_default_r_;

  //the default object pose at zero
  geometry_msgs::Pose pose_zero_;

  //all successfully reached positions
  std::vector <geometry_msgs::Pose> stat_poses_success_;

  //publisher of the collision objects to the topic /collision_world
  ros::Publisher pub_obj_moveit_;

  /** processing rate */
  ros::Rate rate_;
};
}

#endif // SIMPLEACTIONS_HPP
