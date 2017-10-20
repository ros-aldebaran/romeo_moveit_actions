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

#include "romeo_moveit_actions/evaluation.hpp"
#include "romeo_moveit_actions/toolsForObject.hpp"

namespace moveit_simple_actions
{

Evaluation::Evaluation(const bool &verbose, const std::string &base_frame):
  verbose_(verbose),
  base_frame_(base_frame)
{
  pose_zero_.position.x = 0.0;
  pose_zero_.position.y = 0.0;
  pose_zero_.position.z = 0.0;
  pose_zero_.orientation.x = -1.0;
  pose_zero_.orientation.y = 0.0;
  pose_zero_.orientation.z = 0.0;
  pose_zero_.orientation.w = 0.0;
}

void Evaluation::init(const double &test_step,
                      const double &block_size_x,
                      const double &block_size_y,
                      const double floor_to_base_height,
                      const double &x_min,
                      const double &x_max,
                      const double &y_min,
                      const double &y_max,
                      const double &z_min,
                      const double &z_max)
{
  test_step_ = test_step;
  block_size_x_ = block_size_x;
  block_size_y_ = block_size_y;
  floor_to_base_height_ = floor_to_base_height;
  x_min_ = x_min;
  x_max_ = x_max;
  y_min_ = y_min;
  y_max_ = y_max;
  z_min_ = z_min;
  z_max_ = z_max;

  //create a default block
  block_ = new MetaBlock("Virtual1",
                         pose_zero_,
                         sprimitive::CYLINDER,
                         block_size_x_,
                         block_size_y_,
                         0.0);

  //create a default table
  double height = -floor_to_base_height_ + z_min_;
  double width = y_max_*2.0;
  double depth = 0.35;
  geometry_msgs::Pose pose;
  setPose(&pose,
          x_min_ + depth/2.0,
          0.0,
          floor_to_base_height_ + height/2.0);

  table_ = new MetaBlock("table", pose, sprimitive::BOX, depth, width, height);
}

geometry_msgs::PoseArray Evaluation::generatePosesGrid()
{
  std::vector<MetaBlock> blocks_test;
  geometry_msgs::PoseArray poses;
  poses.header.frame_id = base_frame_;

  /*//detect objects to get any mesh
  ROS_INFO_STREAM(" Looking for objects");
  ros::Time start_time = ros::Time::now();
  while ((blocks_.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(1.0)))
  {
    objproc.triggerObjectDetection();
  }*/

  if (verbose_)
    ROS_INFO_STREAM(" Generating goals in the target space");

  MetaBlock block(*block_);

  double y_step = test_step_*1.5;
  double y_min(y_min_), y_max(y_max_);

  int count = 0;
  for (double y=y_min; y<=y_max; y+=y_step)
    for (double z=z_min_; z<=z_max_; z+=test_step_)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        block.pose_.position.x = x;
        block.pose_.position.y = y;
        block.pose_.position.z = z;
        blocks_test.push_back(block);

        poses.poses.push_back(block.pose_);
        std::cout << x << " " << y << " " << z << std::endl;
        ++count;
      }

  y_min = -y_max_;
  y_max = -y_min_;
  for (double y=y_min; y<=y_max; y+=y_step)
    for (double z=z_min_; z<=z_max_; z+=test_step_)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        block.pose_.position.x = x;
        block.pose_.position.y = y;
        block.pose_.position.z = z;
        blocks_test.push_back(block);

        poses.poses.push_back(block.pose_);
        std::cout << x << " " << y << " " << z << std::endl;
        ++count;
      }

  if (verbose_)
    ROS_INFO_STREAM("Total number of generated poses=" << count);
  return poses;
}

geometry_msgs::PoseArray Evaluation::generatePosesRnd(const int poses_nbr)
{
  std::vector<MetaBlock> blocks_test;
  geometry_msgs::PoseArray poses;
  poses.header.frame_id = "base_link";

  int count = 0;
  while (count < poses_nbr){
    geometry_msgs::Pose pose(pose_zero_);
    pose.position.x = 0.35f + float(rand() % 150)/1000.0f;
    pose.position.y = float(rand() % 90)/100.0f - 0.45;
    pose.position.z = -0.23f + (float(rand() % 230)/1000.0f);
    blocks_test.push_back(MetaBlock("BlockTest", pose, sprimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));
    poses.poses.push_back(blocks_test.back().pose_);
    ++count;
  }
  return poses;
}

void Evaluation::testReach(ros::NodeHandle &nh,
                           ros::Publisher *pub_obj_pose,
                           ros::Publisher *pub_obj_poses,
                           ros::Publisher *pub_obj_moveit,
                           moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                           Action *action_left,
                           Action *action_right,
                           std::vector<MetaBlock> *blocks_surfaces,
                           const bool pickVsReach,
                           const bool test_poses_rnd)
{
  geometry_msgs::PoseArray poses_test;
  if (test_poses_rnd)
    poses_test = generatePosesRnd(200);
  else
    poses_test = generatePosesGrid();

  //visualize all generated samples of the goal space
  pub_obj_poses->publish(poses_test);
  sleep(0.05);

  ros::Publisher pub_obj_poses_left =
      nh.advertise<geometry_msgs::PoseArray>("/poses_reachable_left", 100);
  ros::Publisher pub_obj_poses_right =
      nh.advertise<geometry_msgs::PoseArray>("/poses_reachable_right", 100);

  int targets_nbr = 0;
  int attempts_nbr = 1;
  double planning_time = 30.0;
  geometry_msgs::PoseArray msg_poses_validated;
  msg_poses_validated.header.frame_id = action_right->getBaseLink();
  targets_nbr += testReachWithGenSingleHand(action_left,
                                            blocks_surfaces,
                                            pub_obj_pose,
                                            &pub_obj_poses_left,
                                            pub_obj_moveit,
                                            visual_tools,
                                            pickVsReach,
                                            attempts_nbr, planning_time, msg_poses_validated);

  targets_nbr += testReachWithGenSingleHand(action_right,
                                            blocks_surfaces,
                                            pub_obj_pose,
                                            &pub_obj_poses_right,
                                            pub_obj_moveit,
                                            visual_tools,
                                            pickVsReach,
                                            attempts_nbr, planning_time, msg_poses_validated);
  //print all reachable poses
  if (verbose_)
  {
    ROS_INFO_STREAM("Exploration of reachable space \n"
                        << "Successfull/total grasps = " << msg_poses_validated.poses.size() << "/" << targets_nbr << "\n"
                        << "Test params: \n"
                        << "Max attempts number = " << attempts_nbr << "\n planning time = " << planning_time << "\n"
                        << "The tested space: step = " << test_step_ << " the zone: \n"
                        << "x: [ " << x_min_ << " " << x_max_ << " ]\n"
                        << "y: [ " << y_min_ << " " << y_max_ << " ]\n"
                        << "z: [ " << z_min_ << " " << z_max_ << " ]");

    for (int i =0; i < msg_poses_validated.poses.size(); ++i)
      std::cout << msg_poses_validated.poses[i].position.x << " " << msg_poses_validated.poses[i].position.y << " " << msg_poses_validated.poses[i].position.z << std::endl;
  }
}

int Evaluation::testReachWithGenSingleHand(Action *action,
                                                std::vector<MetaBlock> *blocks_surfaces,
                                                ros::Publisher *pub_obj_pose,
                                                ros::Publisher *pub_obj_poses,
                                                ros::Publisher *pub_obj_moveit,
                                                moveit_visual_tools::MoveItVisualToolsPtr visual_tools,
                                                const bool pickVsReach,
                                                const int attempts_nbr,
                                                const double planning_time,
                                                geometry_msgs::PoseArray &msg_poses_validated)
{
  moveit::planning_interface::PlanningSceneInterface current_scene;

  //Set the test space params
  double y_step = test_step_*1.5;
  double y_min(y_min_), y_max(y_max_);
  if (action->arm_ == "right")
  {
    y_min = -y_max_;
    y_max = -y_min_;
  }

  MetaBlock block(*block_);

  int count_total = 0;
  for (double z=z_min_; z<=z_max_; z+=test_step_)
  {
    //update the table height
    table_->size_z_ = -floor_to_base_height_ + (z-block_size_y_/2.0);
    table_->pose_.position.z = floor_to_base_height_ + table_->size_z_/2.0;

    for (double y=y_min; y<=y_max; y+=y_step)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        ++count_total;

        table_->pose_.position.x = x - block_size_x_/2.0 + table_->size_x_/2.0,
        table_->updatePose(table_->pose_);
        std::vector<std::string> objects;
        objects.push_back(table_->name_);
        current_scene.removeCollisionObjects(objects);
        sleep(1.5);
        pub_obj_moveit->publish(table_->collObj_);

        // publish the pose
        geometry_msgs::PoseStamped msg_obj_pose;
        msg_obj_pose.header.frame_id = action->getBaseLink();
        msg_obj_pose.pose.position.x = x;
        msg_obj_pose.pose.position.y = y;
        msg_obj_pose.pose.position.z = z;
        pub_obj_pose->publish(msg_obj_pose);

        // publish the collision object
        block.updatePose(msg_obj_pose.pose);
        pub_obj_moveit->publish(block.collObj_);

        bool success(false);
        if (pickVsReach)
          success = action->pickAction(&block, table_->name_, attempts_nbr, planning_time);
        else
        {
          success = action->reachPregrasp(block.pose_, "");
        }
        if (success)
        {
          //reset object, at first detach it
          action->detachObject(block.name_);

          msg_poses_validated.poses.push_back(block.pose_);
          pub_obj_poses->publish(msg_poses_validated);
          stat_poses_success_.push_back(block.pose_);
        }
        //return the hand
        action->poseHand(1);
        sleep(1.5);

        //remove collision object
        block.removeBlock(&current_scene);
      }
  }

  //return the hand
  action->poseHand(1);
  sleep(1.5);
  return count_total;
}

void Evaluation::printStat()
{
  ROS_INFO_STREAM("Successfully grasped objects at the following locations: ");
  for (std::vector <geometry_msgs::Pose>::iterator it = stat_poses_success_.begin(); it != stat_poses_success_.end(); ++it)
  {
    ROS_INFO_STREAM(" [" <<  it-> position.x << " " << it->position.y << " " << it->position.z << "] ["
                    << it->orientation.x << " " << it->orientation.y << " " << it->orientation.z << " " << it->orientation.w << "]");
  }
}

bool Evaluation::inWorkSpace(geometry_msgs::Pose pose,
                             const bool x,
                             const bool y,
                             const bool z)
{
  bool res(false);
  if (x && y && z)
  {
    if ((pose.position.x < x_max_) && (pose.position.x > x_min_)
        && (pose.position.y < y_max_) && (pose.position.y > y_min_)
        && (pose.position.z < z_max_) && (pose.position.z > z_min_))
      res = true;
  }
  if (!x && !y && z)
  {
    if ((pose.position.z < z_max_) && (pose.position.z > z_min_))
      res = true;
  }

  return res;
}

}
