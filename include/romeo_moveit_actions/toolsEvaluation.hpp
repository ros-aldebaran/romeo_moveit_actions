#ifndef TOOLSEVALUATION_HPP
#define TOOLSEVALUATION_HPP

#include "romeo_moveit_actions/simplepickplace.hpp"

namespace moveit_simple_actions
{

geometry_msgs::PoseArray SimplePickPlace::generatePosesGrid(std::vector<MetaBlock> &blocks_test)
{
  blocks_test_.clear();
  geometry_msgs::PoseArray msg_poses;
  msg_poses.header.frame_id = base_frame_;//grasp_data_.base_link_;

  /*//detect objects to get any mesh
  ROS_INFO_STREAM(" Looking for objects");
  ros::Time start_time = ros::Time::now();
  while (nh_.ok() && (blocks_.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(1.0)))
  {
    objproc.triggerObjectDetection();
  }*/

  if (verbose_)
    ROS_INFO_STREAM(" Generating goals in the target space");

  MetaBlock block = MetaBlock("BlockTest", pose_zero_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x, block_size_y, 0.0);

  double y_step = test_step_*1.5;
  double y_min(y_min_), y_max(y_max_);

  int count = 0;
  for (double y=y_min; y<=y_max; y+=y_step)
    for (double z=z_min_; z<=z_max_; z+=test_step_)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        block.start_pose_.position.x = x;
        block.start_pose_.position.y = y;
        block.start_pose_.position.z = z;
        blocks_test_.push_back(block);

        msg_poses.poses.push_back(block.start_pose_);
        std::cout << x << " " << y << " " << z << std::endl;
        ++count;
      }

  y_min = -y_max_;
  y_max = -y_min_;
  for (double y=y_min; y<=y_max; y+=y_step)
    for (double z=z_min_; z<=z_max_; z+=test_step_)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        block.start_pose_.position.x = x;
        block.start_pose_.position.y = y;
        block.start_pose_.position.z = z;
        blocks_test_.push_back(block);

        msg_poses.poses.push_back(block.start_pose_);
        std::cout << x << " " << y << " " << z << std::endl;
        ++count;
      }

  if (verbose_)
    ROS_INFO_STREAM("Total number of generated poses=" << count);
  return msg_poses;
}

geometry_msgs::PoseArray SimplePickPlace::generatePosesRnd(const int poses_nbr, std::vector<MetaBlock> &blocks_test)
{
  blocks_test_.clear();
  geometry_msgs::PoseArray msg_poses;
  msg_poses.header.frame_id = "base_link";//grasp_data_.base_link_;

  int count = 0;
  while (count < poses_nbr){
    geometry_msgs::Pose pose(pose_zero_);
    pose.position.x = 0.35f + float(rand() % 150)/1000.0f; //[0.35;0.50]
    pose.position.y = float(rand() % 90)/100.0f - 0.45; //[-0.45;0.45]
    pose.position.z = -0.23f + (float(rand() % 230)/1000.0f); //[-0.23;0.00]
    blocks_test_.push_back(MetaBlock("BlockTest", pose, shape_msgs::SolidPrimitive::CYLINDER, block_size_x, block_size_y, 0.0));
    msg_poses.poses.push_back(blocks_test_.back().start_pose_);
    ++count;
  }
  return msg_poses;
}

void SimplePickPlace::testReach(const bool pickVsReach, bool test_poses_rnd)
{
  cleanObjects(&blocks_);
  geometry_msgs::PoseArray msg_poses_test;
  if (test_poses_rnd)
    msg_poses_test = generatePosesRnd(200, blocks_test_);
  else
    msg_poses_test = generatePosesGrid(blocks_test_);

  //visualize all generated samples of the goal space
  pub_obj_poses_.publish(msg_poses_test);
  sleep(0.05);

  ros::Publisher pub_obj_poses_left = nh_.advertise<geometry_msgs::PoseArray>("/poses_reachable_left", 100);
  ros::Publisher pub_obj_poses_right = nh_.advertise<geometry_msgs::PoseArray>("/poses_reachable_right", 100);
  //testReachSingleHand(action_left_, &pub_obj_poses_left, blocks_test, pickVsReach);
  //testReachSingleHand(action_right_, &pub_obj_poses_right, blocks_test, pickVsReach);

  int targets_nbr = 0;
  int attempts_nbr = 1;
  double planning_time = 30.0;
  geometry_msgs::PoseArray msg_poses_validated;
  msg_poses_validated.header.frame_id = action_right_->grasp_data_.base_link_;
  targets_nbr += testReachWithGenSingleHand(action_left_, &pub_obj_poses_left, pickVsReach,
                                            attempts_nbr, planning_time, msg_poses_validated);
  targets_nbr += testReachWithGenSingleHand(action_right_, &pub_obj_poses_right, pickVsReach,
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


int SimplePickPlace::testReachWithGenSingleHand(Action *action,
                                                ros::Publisher *pub_obj_poses,
                                                const bool pickVsReach,
                                                const int attempts_nbr,
                                                const double planning_time,
                                                geometry_msgs::PoseArray &msg_poses_validated)
{
  MetaBlock table_init = blocks_surfaces_.front();

  //Set the test space params
  double y_step = test_step_*1.5;
  double y_min(y_min_), y_max(y_max_);
  if (action->arm == "right")
  {
    y_min = -y_max_;
    y_max = -y_min_;
  }

  MetaBlock block = MetaBlock("BlockTest", pose_zero_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x, block_size_y, 0.0);

  int count_total = 0;
  for (double z=z_min_; z<=z_max_; z+=test_step_)
  {
    blocks_surfaces_.front().size_z_ = -floor_to_base_height_ + (z-block_size_y/2.0);
    blocks_surfaces_.front().start_pose_.position.z = floor_to_base_height_ + blocks_surfaces_.front().size_z_/2.0;
    pub_obj_moveit_.publish(publishCollisionBlock(&blocks_surfaces_.front()));
    //updateTable(visual_tools_, -floor_to_base_height_ + (z-block_size_y/2.0));
    for (double y=y_min; y<=y_max; y+=y_step)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        ++count_total;

        // publish the collision object
        block.start_pose_.position.x = x;
        block.start_pose_.position.y = y;
        block.start_pose_.position.z = z;
        visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&block));

        // publish the pose
        msg_obj_pose_.pose = block.start_pose_;
        pub_obj_pose_.publish(msg_obj_pose_);

        bool success;
        if (pickVsReach)
          success = action->pickAction(&block, support_surface_name_, attempts_nbr, planning_time);
        else
        {
          visual_tools_->cleanupCO(block.name_);
          success = action->reachPregrasp(block.start_pose_, "");
          resetBlock(&block);
        }
        if (success)
        {
          //std::cout << "----- reachable pose: " << block.start_pose_.position.x << " " << block.start_pose_.position.y << " " << block.start_pose_.position.z << std::endl;
          msg_poses_validated.poses.push_back(block.start_pose_);
          pub_obj_poses->publish(msg_poses_validated);
          stat_poses_success_.push_back(block.start_pose_);

          //return the hand
          action->poseHand(1);
          sleep(1.5);
        }
        // Remove attached object and Remove collision object
        action->move_group_->detachObject(block.name_);
        visual_tools_->cleanupCO(block.name_);
      }
  }

  blocks_surfaces_.clear();
  blocks_surfaces_.push_back(table_init);
  pub_obj_moveit_.publish(publishCollisionBlock(&blocks_surfaces_.front()));

  //return the hand
  action->poseHand(1);
  sleep(1.5);
  return count_total;
}

void SimplePickPlace::testReachSingleHand(Action *action, ros::Publisher *pub_obj_poses, std::vector<MetaBlock> &blocks_test, const bool pickVsReach)
{
  int count_reachable = 0;
  geometry_msgs::PoseArray msg_poses_validated;
  msg_poses_validated.header.frame_id = "base_link";//grasp_data_.base_link_;

    for (std::vector<MetaBlock>::iterator block=blocks_test_.begin(); block != blocks_test_.end(); ++block)
      if ((action->arm == "left") && (block->start_pose_.position.y >= 0.12f) ||
          (action->arm == "right") && (block->start_pose_.position.y <= -0.12f))
    {
      // Publish the collision object
      //publishCollisionMetaBlock(&(*block));
      visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));

      msg_obj_pose_.pose = block->start_pose_;
      pub_obj_pose_.publish(msg_obj_pose_);

      bool success;
      if (pickVsReach)
        success = action->pickAction(&(*block), support_surface_name_);
      else
      {
        visual_tools_->cleanupCO(block->name_);
        success = action->reachPregrasp(block->start_pose_, "");
        resetBlock(&(*block));
      }
      if (success)
      {
        std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv reachable pose: " << block->start_pose_.position.x << " " << block->start_pose_.position.y << " " << block->start_pose_.position.z << std::endl;
        msg_poses_validated.poses.push_back(block->start_pose_);
        pub_obj_poses->publish(msg_poses_validated);

        //return the hand
        action->poseHand(1);
        sleep(1.5);
        ++count_reachable;
      }
      // Remove attached object and Remove collision object
      action->move_group_->detachObject(block->name_);
      visual_tools_->cleanupCO(block->name_);
    }

  //return the hand
  action->poseHand(1);
  sleep(1.5);

  //print all reachable poses
  if (verbose_)
  {
    ROS_INFO_STREAM(" number of reachable objects = " << count_reachable << "/" << blocks_test_.size());
    for (int i =0; i < msg_poses_validated.poses.size(); ++i)
      std::cout << msg_poses_validated.poses[i].position.x << " " << msg_poses_validated.poses[i].position.y << " " << msg_poses_validated.poses[i].position.z << std::endl;
  }
}

}
#endif // TOOLSEVALUATION_HPP
