#include "romeo_moveit_actions/evaluation.hpp"

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
}

geometry_msgs::PoseArray Evaluation::generatePosesGrid(std::vector<MetaBlock> &blocks_test)
{
  blocks_test.clear();
  geometry_msgs::PoseArray msg_poses;
  msg_poses.header.frame_id = base_frame_;

  /*//detect objects to get any mesh
  ROS_INFO_STREAM(" Looking for objects");
  ros::Time start_time = ros::Time::now();
  while ((blocks_.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(1.0)))
  {
    objproc.triggerObjectDetection();
  }*/

  if (verbose_)
    ROS_INFO_STREAM(" Generating goals in the target space");

  MetaBlock block = MetaBlock("BlockTest", pose_zero_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0);

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
        blocks_test.push_back(block);

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
        blocks_test.push_back(block);

        msg_poses.poses.push_back(block.start_pose_);
        std::cout << x << " " << y << " " << z << std::endl;
        ++count;
      }

  if (verbose_)
    ROS_INFO_STREAM("Total number of generated poses=" << count);
  return msg_poses;
}

geometry_msgs::PoseArray Evaluation::generatePosesRnd(const int poses_nbr, std::vector<MetaBlock> &blocks_test)
{
  blocks_test.clear();
  geometry_msgs::PoseArray msg_poses;
  msg_poses.header.frame_id = "base_link";//grasp_data_.base_link_;

  int count = 0;
  while (count < poses_nbr){
    geometry_msgs::Pose pose(pose_zero_);
    pose.position.x = 0.35f + float(rand() % 150)/1000.0f; //[0.35;0.50]
    pose.position.y = float(rand() % 90)/100.0f - 0.45; //[-0.45;0.45]
    pose.position.z = -0.23f + (float(rand() % 230)/1000.0f); //[-0.23;0.00]
    blocks_test.push_back(MetaBlock("BlockTest", pose, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));
    msg_poses.poses.push_back(blocks_test.back().start_pose_);
    ++count;
  }
  return msg_poses;
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
  geometry_msgs::PoseArray msg_poses_test;
  std::vector<MetaBlock> blocks_test;
  if (test_poses_rnd)
    msg_poses_test = generatePosesRnd(200, blocks_test);
  else
    msg_poses_test = generatePosesGrid(blocks_test);

  //visualize all generated samples of the goal space
  pub_obj_poses->publish(msg_poses_test);
  sleep(0.05);

  ros::Publisher pub_obj_poses_left = nh.advertise<geometry_msgs::PoseArray>("/poses_reachable_left", 100);
  ros::Publisher pub_obj_poses_right = nh.advertise<geometry_msgs::PoseArray>("/poses_reachable_right", 100);

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
  MetaBlock table_init = blocks_surfaces->front();

  //Set the test space params
  double y_step = test_step_*1.5;
  double y_min(y_min_), y_max(y_max_);
  if (action->arm_ == "right")
  {
    y_min = -y_max_;
    y_max = -y_min_;
  }

  MetaBlock block = MetaBlock("BlockTest", pose_zero_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0);

  int count_total = 0;
  for (double z=z_min_; z<=z_max_; z+=test_step_)
  {
    MetaBlock *table_cur = &blocks_surfaces->front();
    table_cur->size_z_ = -floor_to_base_height_ + (z-block_size_y_/2.0);
    table_cur->start_pose_.position.z = floor_to_base_height_ + table_cur->size_z_/2.0;
    pub_obj_moveit->publish(table_cur->collObj_);
    //TODO
    //updateTable(visual_tools, -floor_to_base_height_ + (z-block_size_y_/2.0));
    for (double y=y_min; y<=y_max; y+=y_step)
      for (double x=x_min_; x<=x_max_; x+=test_step_)
      {
        ++count_total;

        // publish the collision object
        block.start_pose_.position.x = x;
        block.start_pose_.position.y = y;
        block.start_pose_.position.z = z;
        //TODO
        std::vector <shape_msgs::Mesh> meshes;
        visual_tools->processCollisionObjectMsg(block.wrapToCollisionObject(meshes));

        // publish the pose
        geometry_msgs::PoseStamped msg_obj_pose;
        msg_obj_pose.header.frame_id = action->getBaseLink();
        msg_obj_pose.pose = block.start_pose_;
        pub_obj_pose->publish(msg_obj_pose);

        bool success;
        if (pickVsReach)
          success = action->pickAction(&block, table_init.name_, attempts_nbr, planning_time);
        else
        {
          //TODO
          visual_tools->cleanupCO(block.name_);
          success = action->reachPregrasp(block.start_pose_, "");

          //reset object, at first detach it
          action->detachObject(block.name_);
          // Remove/Add collision object
          pub_obj_moveit->publish(block.collObj_);
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
        action->detachObject(block.name_);
        //TODO //TOCHECK
        //visual_tools->cleanupCO(block.name_);
      }
  }

  blocks_surfaces->clear();
  blocks_surfaces->push_back(table_init);
  pub_obj_moveit->publish(blocks_surfaces->front().collObj_);

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

}
