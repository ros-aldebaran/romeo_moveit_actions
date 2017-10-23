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

#ifndef EVALUATION_HPP
#define EVALUATION_HPP

#include <ros/ros.h>

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/action.hpp"

namespace moveit_simple_actions
{

//! @brief Class for evaluation the algorithm.
class Evaluation
{
public:
  //! @brief constructor
  Evaluation(const bool &verbose, const std::string &base_frame);

  //! @brief initialization
  void init(const double &test_step,
            const double &block_size_x,
            const double &block_size_y,
            const double floor_to_base_height,
            const double &x_min,
            const double &x_max,
            const double &y_min,
            const double &y_max,
            const double &z_min,
            const double &z_max);

  //! @brief testing grasping or approximate grasping
  void testReach(ros::NodeHandle &nh,
                 ros::Publisher *pub_obj_pose,
                 ros::Publisher *pub_obj_poses,
                 ros::Publisher *pub_obj_moveit,
                 moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                 Action *action_left,
                 Action *action_right,
                 std::vector<MetaBlock> *blocks_surfaces,
                 const bool pickVsReach,
                 const bool test_poses_rnd=false);

  //! @brief printing the successfully reached positions
  void printStat();

  //! @brief checking if the pose is within the working space (close enough)
  bool inWorkSpace(geometry_msgs::Pose pose,
                   const bool x=true,
                   const bool y=true,
                   const bool z=true);

protected:
  //! @brief generating test poses in a regular maner
  geometry_msgs::PoseArray generatePosesGrid();

  //! @brief generating poses in a random maner
  geometry_msgs::PoseArray generatePosesRnd(const int poses_nbr);

  //! @brief testing a single hand
  int testReachWithGenSingleHand(Action *action,
                                 std::vector<MetaBlock> *blocks_surfaces,
                                 ros::Publisher *pub_obj_pose,
                                 ros::Publisher *pub_obj_poses,
                                 ros::Publisher *pub_obj_moveit,
                                 moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                                 const bool pickVsReach,
                                 const int attempts_nbr,
                                 const double planning_time,
                                 geometry_msgs::PoseArray &msg_poses_validated);

  /** verbose or not */
  bool verbose_;

  /** robot's base_frame */
  std::string base_frame_;

  /** step for testing the working space */
  double test_step_;

  /** size X of a default object */
  double block_size_x_;

  /** size Y of a default object */
  double block_size_y_;

  /** shift of the robot's base to the floor */
  double floor_to_base_height_;

  /** working space in X dim min */
  float x_min_;

  /** working space in X dim max */
  float x_max_;

  /** working space in Y dim min */
  float y_min_;

  /** working space in Y dim max */
  float y_max_;

  /** working space in Z dim min */
  float z_min_;

  /** working space in Z dim max */
  float z_max_;

  /** default zero pose */
  geometry_msgs::Pose pose_zero_;

  /** successfully reached target poses */
  std::vector <geometry_msgs::Pose> stat_poses_success_;

  /** default object to grasp */
  MetaBlock *block_;

  /** default table to grasp on */
  MetaBlock *table_;
};
}
#endif // EVALUATION_HPP
