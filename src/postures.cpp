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
#include <moveit/move_group_interface/move_group_interface.h>

#include "romeo_moveit_actions/postures.hpp"

typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;

namespace moveit_simple_actions
{
Posture::Posture(const std::string robot_name,
                 const std::string eef_name,
                 const std::string group_name)
{
  //set joint for the head
  MoveGroupInterface move_group_head("head");
  //get current
  move_group_head.getCurrentState()->copyJointGroupPositions(move_group_head.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_head.getName()), pose_head_down_);
  if (pose_head_down_.size() > 1)
    if (robot_name == "nao")
      pose_head_down_[1] = 0.3;
    else
      pose_head_down_[1] = 0.35;
  pose_head_zero_.resize(pose_head_down_.size(), 0.0);

  //get the arm joints
  MoveGroupInterface move_group_plan(group_name);
  std::vector<double> pose_arm_init;
  move_group_plan.getCurrentState()->copyJointGroupPositions(
              move_group_plan.getCurrentState()->getRobotModel()->getJointModelGroup(
                  move_group_plan.getName()), pose_arm_init);

  //initialize the left arm
  pose_arm_.resize(5);
  pose_arm_[0] = std::vector<double>(pose_arm_init);
  for (int i=1; i<pose_arm_.size(); ++i)
    pose_arm_[i].resize(pose_arm_[0].size(), 0.0);
  if (pose_arm_[0].size() >= 5)
  {
    //define the zero pose
    pose_arm_[0].assign(pose_arm_[0].size(), 0.0);

    //define the initial pose (arms down)
    pose_arm_[1][0] = 1.56;
    pose_arm_[1][1] = 0.14;
    pose_arm_[1][2] = -1.22;
    pose_arm_[1][3] = -0.52;
    pose_arm_[1][4] = -0.02;
    if (pose_arm_[1].size() >= 6)
    {
      if (robot_name == "romeo")
        pose_arm_[1][5] = 0.59;
      else
        pose_arm_[1][5] = 0.02;
    }

    //define the initial grasp pose
    pose_arm_[2][0] = 1.74;
    pose_arm_[2][1] = 0.75;
    pose_arm_[2][2] = -2.08;
    pose_arm_[2][3] = -1.15;
    pose_arm_[2][4] = -0.43;
    if (pose_arm_[2].size() >= 6)
    {
      if (robot_name == "romeo")
        pose_arm_[2][5] = 0.17;
      else
        pose_arm_[1][5] = 0.02;
    }

    //define another initial grasp pose
    pose_arm_[3][0] = 1.08;
    pose_arm_[3][1] = 0.66;
    pose_arm_[3][2] = -0.84;
    pose_arm_[3][3] = -0.61;
    pose_arm_[3][4] = -0.67;
    if (pose_arm_[3].size() >= 6)
    {
      if (robot_name == "romeo")
        pose_arm_[3][5] = 0.025;
      else
        pose_arm_[1][5] = 0.02;
    }

    //define another initial grasp pose
    pose_arm_[4][0] = 1.87;
    pose_arm_[4][1] = 0.56;
    pose_arm_[4][2] = -1.99;
    pose_arm_[4][3] = -1.05;
    pose_arm_[4][4] = -0.67;
    if (pose_arm_[4].size() >= 6)
      pose_arm_[4][5] = 0.02;
  }

  //initialize the right arm
  if (group_name.find("right") != std::string::npos)
    for (int i=0; i<pose_arm_.size(); ++i)
    {
      if (pose_arm_[i].size() >= 3)
      {
        pose_arm_[i][1] *= -1;
        pose_arm_[i][2] *= -1;
      }
      if (pose_arm_[i].size() >= 5)
      {
        pose_arm_[i][3] *= -1;
        if (robot_name != "pepper") //not for Pepper
          pose_arm_[i][4] *= -1;
      }
    }

  //get the joint of the eef
  std::vector<double> pose_hand;
  MoveGroupInterface move_group_eef(eef_name);
  move_group_eef.getCurrentState()->copyJointGroupPositions(move_group_eef.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_eef.getName()), pose_hand);
  pose_hand_.resize(2);
  pose_hand_[0].resize(pose_hand.size(), 0.0);
  pose_hand_[1].resize(pose_hand.size(), 0.8);
}

void Posture::initHandPose(const double &value, const int &pose)
{
  if (pose <= pose_hand_.size())
  {
    pose_hand_[pose].assign(pose_hand_[pose].size(), value);
    if (pose_hand_[pose].size() > 0)
      pose_hand_[pose][0] = 0.0;
  }
}

bool Posture::poseHeadZero()
{
  goToPose("head", &pose_head_zero_);
}

bool Posture::poseHeadDown()
{
  goToPose("head", &pose_head_down_);
}

bool Posture::poseHand(const std::string &end_eff,
                       const std::string &group,
                       const int &pose_id)
{
  //set the end-effector first
  goToPose(end_eff, &pose_hand_[0]);
  bool res = false;
  if (pose_id < pose_arm_.size())
    res = goToPose(group, &pose_arm_[pose_id]);
  return res;
}

bool Posture::poseHandOpen(const std::string &end_eff)
{
  goToPose(end_eff, &pose_hand_[0]);
}

bool Posture::poseHandClose(const std::string &end_eff)
{
  goToPose(end_eff, &pose_hand_[1]);
}

bool Posture::goToPose(const std::string group_name,
                       std::vector<double> *pose)
{
  MoveGroupInterface move_group(group_name);

  //get the current set of joint values for the group
  std::vector<double> joints;
  move_group.getCurrentState()->copyJointGroupPositions(
        move_group.getCurrentState()->getRobotModel()->
        getJointModelGroup(move_group.getName()), joints);

  //plan to the new joint space goal and visualize the plan
  if (joints.size() == pose->size())
  {
    //move_group.setApproximateJointValueTarget(*pose);
    move_group.setJointValueTarget(*pose);

    MoveGroupInterface::Plan plan;
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
    ROS_INFO_STREAM("Posture: Joint size is wrong for " << group_name
                    << ": " << joints.size() << " != " << pose->size());
  return false;
}
}
