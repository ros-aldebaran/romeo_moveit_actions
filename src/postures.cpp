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

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

#include "romeo_moveit_actions/postures.hpp"

namespace moveit_simple_actions
{
Posture::Posture(const std::string robot_name, const std::string eef_name, const std::string group_name)
{
  //set joint for the head
  moveit::planning_interface::MoveGroup move_group_head("head");
  //get current
  move_group_head.getCurrentState()->copyJointGroupPositions(move_group_head.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_head.getName()), pose_head_down_);
  if (pose_head_down_.size() > 1)
    if (robot_name == "nao")
      pose_head_down_[1] = 0.3;
    else
      pose_head_down_[1] = 0.35;
  pose_head_zero_.resize(pose_head_down_.size(), 0.0);

  //get the arm joints
  moveit::planning_interface::MoveGroup move_group_plan(group_name);
  std::vector<double> pose_arm_left_init;
  move_group_plan.getCurrentState()->copyJointGroupPositions(
              move_group_plan.getCurrentState()->getRobotModel()->getJointModelGroup(
                  move_group_plan.getName()), pose_arm_left_init);

  //initialize the left arm
  pose_arm_left_.resize(4);
  pose_arm_left_[0] = pose_arm_left_init;
  for (int i=1; i<pose_arm_left_.size(); ++i)
    pose_arm_left_[i].resize(pose_arm_left_[0].size(), 0.0);
  if (pose_arm_left_[0].size() >= 5)
  {
    pose_arm_left_[0].assign(pose_arm_left_[0].size(), 0.0);

    pose_arm_left_[1][0] = 1.74;
    pose_arm_left_[1][1] = 0.75;
    pose_arm_left_[1][2] = -2.08;
    pose_arm_left_[1][3] = -1.15;
    pose_arm_left_[1][4] = -0.43;
    if (pose_arm_left_[0].size() >= 6)
      pose_arm_left_[1][5] = 0.17;

    pose_arm_left_[2][0] = 1.0799225568771362;
    pose_arm_left_[2][1] = 0.6565437912940979;
    pose_arm_left_[2][2] = -0.8390874862670898;
    pose_arm_left_[2][3] = -0.607456386089325;
    pose_arm_left_[2][4] = -0.6672816872596741;
    if (pose_arm_left_[0].size() >= 6)
      pose_arm_left_[2][5] = 0.02454369328916073;

    pose_arm_left_[3][0] = 1.872990608215332;
    pose_arm_left_[3][1] = 0.5553010702133179;
    pose_arm_left_[3][2] = -1.9895731210708618;
    pose_arm_left_[3][3] = -1.052310824394226;
    pose_arm_left_[3][4] = -0.6703495979309082;
    if (pose_arm_left_[0].size() >= 6)
      pose_arm_left_[3][5] = 0.02147573232650757;
  }

  //initialize the right arm
  pose_arm_right_ = std::vector< std::vector<double> >(pose_arm_left_);
  for (int i=0; i<pose_arm_right_.size(); ++i)
  {
    if (pose_arm_right_[i].size() >= 3)
    {
      pose_arm_right_[i][1] *= -1;
      pose_arm_right_[i][2] *= -1;
    }
    if (pose_arm_right_[i].size() >= 5)
    {
      pose_arm_right_[i][3] *= -1;
      pose_arm_right_[i][4] *= -1;
    }
  }

  //get the current set of joint values for the eef
  std::vector<double> pose_hand;
  moveit::planning_interface::MoveGroup move_group_eef(eef_name);
  move_group_eef.getCurrentState()->copyJointGroupPositions(move_group_eef.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_eef.getName()), pose_hand);
  pose_hand_close_.resize(pose_hand.size(), 0.0);
  pose_hand_open_.resize(pose_hand.size(), 0.0);
  initHandPoseOpen(0.0);
  initHandPoseClose(0.8);
}

void Posture::initHandPoseOpen(const double &value){
  pose_hand_open_.assign(pose_hand_open_.size(), value);
  if (pose_hand_open_.size() > 0)
    pose_hand_open_[0] = 0.0;
}

void Posture::initHandPoseClose(const double &value){
  pose_hand_close_.assign(pose_hand_close_.size(), value);
  if (pose_hand_close_.size() > 0)
    pose_hand_close_[0] = 0.0;
}

bool Posture::poseHeadZero(){
  goToPose("head", &pose_head_zero_);
}

bool Posture::poseHeadDown(){
  goToPose("head", &pose_head_down_);
}

bool Posture::poseHand(const std::string &end_eff, const std::string &group, const std::string &arm, const int &pose_id)
{
  //set the end-effector first
  goToPose(end_eff, &pose_hand_open_);

  bool res = false;
  if (arm == "right")
  {
    if (pose_id < pose_arm_right_.size())
      res = goToPose(group, &pose_arm_right_[pose_id]);
  }
  else
  {
    if (pose_id < pose_arm_left_.size())
      res = goToPose(group, &pose_arm_left_[pose_id]);
  }

  return res;
}

bool Posture::poseHandOpen(const std::string &end_eff)
{
  goToPose(end_eff, &pose_hand_open_);
}

bool Posture::poseHandClose(const std::string &end_eff)
{
  goToPose(end_eff, &pose_hand_close_);
}

bool Posture::goToPose(const std::string group_name, std::vector<double> *pose)
{
  moveit::planning_interface::MoveGroup move_group(group_name);

  //get the current set of joint values for the group
  std::vector<double> joints;
  move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joints);

  //plan to the new joint space goal and visualize the plan
  if (joints.size() == pose->size())
  {
    //move_group.setApproximateJointValueTarget(*pose);
    move_group.setJointValueTarget(*pose);

    moveit::planning_interface::MoveGroup::Plan plan;
    bool success = move_group.plan(plan);
    sleep(1.0);
    if (success)
    {
      ROS_INFO("Action with the move_group %s", success?"":"FAILED");
      move_group.move();
      sleep(1.0);

      /*std::vector<double> joints_temp = move_group.getCurrentJointValues();
      std::cout << "Joint values of " << group_name << " ";
      for (int i=0; i< joints_temp.size(); ++i)
        std::cout << joints_temp[i] << " ";
      std::cout << std::endl;*/
    }
    return success;
  }
  else
    ROS_INFO_STREAM("Input joint values have wrong size " << joints.size() << " == " << pose->size());
  return false;
}
}
