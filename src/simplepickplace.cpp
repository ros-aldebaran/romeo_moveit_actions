//publish messages with objects poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <simplepickplace.hpp>
#include <custom_environment5.hpp>
#include <tools.hpp>

namespace moveit_simple_actions
{
  void swapPoses(geometry_msgs::Pose *pose1, geometry_msgs::Pose *pose2){
    geometry_msgs::Pose temp = *pose1;
    pose1 = pose2;
    *pose2 = temp;
  }

  SimplePickPlace::SimplePickPlace(const std::string robot_name,
                                   const double test_step,
                                   const double x_min,
                                   const double x_max,
                                   const double y_min,
                                   const double y_max,
                                   const double z_min,
                                   const double z_max,
                                   const std::string left_arm_name,
                                   const std::string right_arm_name,
                                   const bool verbose):
      nh_("~"),
      nh_priv_(""),
      robot_name_(robot_name),
      test_step_(test_step),
      verbose_(verbose),
      saveStat_(true),
      base_frame("odom"),
      block_size(0.03),
      block_size_l(0.13), //0.15,
      floor_to_base_height(-1.0), //was -0.9
      env_shown_(false),
      x_min_(x_min),
      x_max_(x_max),
      y_min_(y_min),
      y_max_(y_max),
      z_min_(z_min),
      z_max_(z_max),
      left_arm_name_(left_arm_name),
      right_arm_name_(right_arm_name),
      test_mesh_(false),
      pose_default(),
      pose_default_r(),
      objproc(&nh_priv_),
      vtools_(base_frame)
  {
    pose_default.orientation.x = -1; //0.0;
    pose_default.orientation.y = 0.0;
    pose_default.orientation.z = 0.0;
    pose_default.orientation.w = 0.0;

    if (robot_name_ == "nao")
    {
      block_size = 0.01;
      pose_default.position.x = 0.2;
      pose_default.position.y = 0.1;
      pose_default.position.z = 0.0;
      pose_default_r = pose_default;
      pose_default_r.position.y *= -1;

      if (test_step_ == 0.0)
        test_step_ = 0.03;
      if (x_min_ == 0.0)
        x_min_ = 0.1; //0.3;
      if (x_max_ == 0.0)
        x_max_ = 0.21;
      if (y_min_ == 0.0)
        y_min_ = 0.12;
      if (y_max_ == 0.0)
        y_max_ = 0.24;
      if (z_min_ == 0.0)
        z_min_ = -0.07;
      if (z_max_ == 0.0)
        z_max_ = +0.05;
    }
    else if (robot_name == "pepper")
    {
      block_size = 0.02;
      pose_default.position.x = 0.25;
      pose_default.position.y = 0.2;
      pose_default.position.z = -0.1;
      pose_default_r = pose_default;
      pose_default_r.position.y *= -1;

      if (test_step_ == 0.0)
        test_step_ = 0.04;
      if (x_min_ == 0.0)
        x_min_ = 0.2; //0.3;
      if (x_max_ == 0.0)
        x_max_ = 0.4;
      if (y_min_ == 0.0)
        y_min_ = 0.12;
      if (y_max_ == 0.0)
        y_max_ = 0.24;
      if (z_min_ == 0.0)
        z_min_ = -0.13;
      if (z_max_ == 0.0)
        z_max_ = 0.0;
    }
    else if (robot_name_ == "romeo")
    {
      pose_default.position.x = 0.44;
      pose_default.position.y = 0.15;
      pose_default.position.z = -0.1;
      pose_default_r = pose_default;
      pose_default_r.position.x = 0.49;
      pose_default_r.position.y = -0.25;

      if (test_step_ == 0.0)
        test_step_ = 0.02; //0.04;
      if (x_min_ == 0.0)
        x_min_ = 0.38;
      if (x_max_ == 0.0)
        x_max_ = 0.5;
      if (y_min_ == 0.0)
        y_min_ = 0.12;
      if (y_max_ == 0.0)
        y_max_ = 0.24;
      if (z_min_ == 0.0)
        z_min_ = -0.17;
      if (z_max_ == 0.0)
        z_max_ = -0.08;
    }

    // objects related initialization
    sub_obj_coll = nh_.subscribe<moveit_msgs::CollisionObject>("/collision_object", 10, &SimplePickPlace::getCollisionObjects, this);
    pub_obj_poses = nh_.advertise<geometry_msgs::PoseArray>("/obj_poses", 10);
    pub_obj_pose = nh_.advertise<geometry_msgs::PoseStamped>("/pose_current", 10);

    pub_obj_moveit = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 1000);

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("odom"));
    visual_tools_->setFloorToBaseHeight(floor_to_base_height);
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    cleanEnvironment(visual_tools_);
    env_shown_ = false;
    ros::Duration(1.0).sleep();

    action_left = new Action(&nh_, visual_tools_, left_arm_name_, robot_name_);
    action_right = new Action(&nh_, visual_tools_, right_arm_name_, robot_name_);
ROS_INFO_STREAM("action_left->grasp_data_.base_link_ = " << action_left->grasp_data_.base_link_);
    msg_obj_pose.header.frame_id = action_left->grasp_data_.base_link_;
    msg_obj_poses.header.frame_id = action_left->grasp_data_.base_link_;

    //Move the robots parts to init positions
    //if (promptUserQuestion("Should I move the head to look down?"))
      //action_left->poseHeadDown();
    //action_left->poseHeadDown();

    //if (promptUserQuestion(("Should I move the "+action_left->end_eff+" to the initial pose?").c_str()))
    action_left->poseHandInit();

    //if (promptUserQuestion(("Should I move the "+action_right->plan_group+" to the initial pose?").c_str()))
    action_right->poseHandInit();

    // Create the table
    if (robot_name_ == "romeo")
    {
      createEnvironment(visual_tools_);
      env_shown_ = true;
    }

    printTutorial(robot_name);

    startRoutine();
  }

  geometry_msgs::PoseArray SimplePickPlace::generatePosesGrid(std::vector<MetaBlock> &blocks_test)
  {
    blocks_test.clear();
    geometry_msgs::PoseArray msg_poses;
    msg_poses.header.frame_id = base_frame;//grasp_data_.base_link_;

    /*//detect objects to get any mesh
    ROS_INFO_STREAM(" Looking for objects");
    ros::Time start_time = ros::Time::now();
    while (nh_.ok() && (blocks.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(1.0)))
    {
      objproc.triggerObjectDetection();
    }*/

    if (verbose_)
      ROS_INFO_STREAM(" Generating goals in the target space");

    MetaBlock block = MetaBlock("BlockTest", 0, 0, 0, -1.0, 0.0, 0.0, 0.0, shape_msgs::SolidPrimitive::CYLINDER, block_size, block_size_l);

    double y_step = test_step_*1.5;
    double y_min(y_min_), y_max(y_max_);

    int count = 0;
    for (double y=y_min; y<=y_max; y+=y_step)
      for (double z=z_min_; z<=z_max_; z+=test_step_)
        for (double x=x_min_; x<=x_max_; x+=test_step_)
        {
          block.start_pose.position.x = x;
          block.start_pose.position.y = y;
          block.start_pose.position.z = z;
          blocks_test.push_back(block);

          msg_poses.poses.push_back(block.start_pose);
          std::cout << x << " " << y << " " << z << std::endl;
          ++count;
        }

    y_min = -y_max_;
    y_max = -y_min_;
    for (double y=y_min; y<=y_max; y+=y_step)
      for (double z=z_min_; z<=z_max_; z+=test_step_)
        for (double x=x_min_; x<=x_max_; x+=test_step_)
        {
          block.start_pose.position.x = x;
          block.start_pose.position.y = y;
          block.start_pose.position.z = z;
          blocks_test.push_back(block);

          msg_poses.poses.push_back(block.start_pose);
          std::cout << x << " " << y << " " << z << std::endl;
          ++count;
        }

    if (verbose_)
      ROS_INFO_STREAM("Total number of generated poses=" << count);
    return msg_poses;
  }

  geometry_msgs::PoseArray SimplePickPlace::generatePosesRnd(const int poses_nbr, std::vector<MetaBlock> &blocks_test)
  {
    blocks_test.clear();
    geometry_msgs::PoseArray msg_poses;
    msg_poses.header.frame_id = "base_link";//grasp_data_.base_link_;

    int count = 0;
    while (count < poses_nbr){
      float x = 0.35f + float(rand() % 150)/1000.0f; //[0.35;0.50]
      float y = float(rand() % 90)/100.0f - 0.45; //[-0.45;0.45]
      float z = -0.23f + (float(rand() % 230)/1000.0f); //[-0.23;0.00]
      blocks_test.push_back(MetaBlock("BlockTest", x, y, z, -1,0,0,0, shape_msgs::SolidPrimitive::CYLINDER, block_size, block_size_l));
      msg_poses.poses.push_back(blocks_test.back().start_pose);
      ++count;
    }
    return msg_poses;
  }

  void SimplePickPlace::testReach(const bool pickVsReach, bool test_poses_rnd)
  {
    removeObjects();
    geometry_msgs::PoseArray msg_poses_test;
    if (test_poses_rnd)
      msg_poses_test = generatePosesRnd(200, blocks_test);
    else
      msg_poses_test = generatePosesGrid(blocks_test);

    //visualize all generated samples of the goal space
    pub_obj_poses.publish(msg_poses_test);
    sleep(0.05);

    ros::Publisher pub_obj_poses_left = nh_.advertise<geometry_msgs::PoseArray>("/poses_reachable_left", 100);
    ros::Publisher pub_obj_poses_right = nh_.advertise<geometry_msgs::PoseArray>("/poses_reachable_right", 100);
    //testReachSingleHand(action_left, &pub_obj_poses_left, blocks_test, pickVsReach);
    //testReachSingleHand(action_right, &pub_obj_poses_right, blocks_test, pickVsReach);

    int targets_nbr = 0;
    int attempts_nbr = 1;
    double planning_time = 30.0;
    geometry_msgs::PoseArray msg_poses_validated;
    msg_poses_validated.header.frame_id = action_right->grasp_data_.base_link_;
    targets_nbr += testReachWithGenSingleHand(action_left, &pub_obj_poses_left, pickVsReach,
                                              attempts_nbr, planning_time, msg_poses_validated);
    targets_nbr += testReachWithGenSingleHand(action_right, &pub_obj_poses_right, pickVsReach,
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
    //Set the test space params
    double y_step = test_step_*1.5;
    double y_min(y_min_), y_max(y_max_);
    if (action->arm == "right")
    {
      y_min = -y_max_;
      y_max = -y_min_;
    }

    MetaBlock block = MetaBlock("BlockTest", 0, 0, 0, -1.0, 0.0, 0.0, 0.0, shape_msgs::SolidPrimitive::CYLINDER, block_size, block_size_l);

    int count_total = 0;
    for (double z=z_min_; z<=z_max_; z+=test_step_)
    {
      updateTable(visual_tools_, -floor_to_base_height + (z-block_size_l/2.0));
      for (double y=y_min; y<=y_max; y+=y_step)
        for (double x=x_min_; x<=x_max_; x+=test_step_)
        {
          ++count_total;

          // publish the collision object
          block.start_pose.position.x = x;
          block.start_pose.position.y = y;
          block.start_pose.position.z = z;
          visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&block));

          // publish the pose
          msg_obj_pose.pose = block.start_pose;
          pub_obj_pose.publish(msg_obj_pose);

          bool success;
          if (pickVsReach)
            success = action->pickAction(&block, SUPPORT_SURFACE3_NAME, attempts_nbr, planning_time);
          else
          {
            visual_tools_->cleanupCO(block.name);
            success = action->reachPregrasp(block.start_pose, "");
            resetBlock(&block);
          }
          if (success)
          {
            //std::cout << "----- reachable pose: " << block.start_pose.position.x << " " << block.start_pose.position.y << " " << block.start_pose.position.z << std::endl;
            msg_poses_validated.poses.push_back(block.start_pose);
            pub_obj_poses->publish(msg_poses_validated);
            stat_poses_success.push_back(block.start_pose);

            //return the hand
            action->poseHandInit();
            sleep(1.5);
          }
          // Remove attached object and Remove collision object
          visual_tools_->cleanupACO(block.name);
          visual_tools_->cleanupCO(block.name);
        }
    }

    //return the hand
    action->poseHandInit();
    sleep(1.5);
    return count_total;
  }

  void SimplePickPlace::testReachSingleHand(Action *action, ros::Publisher *pub_obj_poses, std::vector<MetaBlock> &blocks_test, const bool pickVsReach)
  {
    int count_reachable = 0;
    geometry_msgs::PoseArray msg_poses_validated;
    msg_poses_validated.header.frame_id = "base_link";//grasp_data_.base_link_;

      for (std::vector<MetaBlock>::iterator block=blocks_test.begin(); block != blocks_test.end(); ++block)
        if ((action->arm == "left") && (block->start_pose.position.y >= 0.12f) ||
            (action->arm == "right") && (block->start_pose.position.y <= -0.12f))
      {
        // Publish the collision object
        //publishCollisionMetaBlock(&(*block));
        visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));

        msg_obj_pose.pose = block->start_pose;
        pub_obj_pose.publish(msg_obj_pose);

        bool success;
        if (pickVsReach)
          success = action->pickAction(&(*block), SUPPORT_SURFACE3_NAME);
        else
        {
          visual_tools_->cleanupCO(block->name);
          success = action->reachPregrasp(block->start_pose, "");
          resetBlock(&(*block));
        }
        if (success)
        {
          std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv reachable pose: " << block->start_pose.position.x << " " << block->start_pose.position.y << " " << block->start_pose.position.z << std::endl;
          msg_poses_validated.poses.push_back(block->start_pose);
          pub_obj_poses->publish(msg_poses_validated);

          //return the hand
          action->poseHandInit();
          sleep(1.5);
          ++count_reachable;
        }
        // Remove attached object and Remove collision object
        visual_tools_->cleanupACO(block->name);
        visual_tools_->cleanupCO(block->name);
      }

    //return the hand
    action->poseHandInit();
    sleep(1.5);

    //print all reachable poses
    if (verbose_)
    {
      ROS_INFO_STREAM(" number of reachable objects = " << count_reachable << "/" << blocks_test.size());
      for (int i =0; i < msg_poses_validated.poses.size(); ++i)
        std::cout << msg_poses_validated.poses[i].position.x << " " << msg_poses_validated.poses[i].position.y << " " << msg_poses_validated.poses[i].position.z << std::endl;
    }
  }

  bool SimplePickPlace::startRoutine()
  {
    int block_id = -1;
    int hand_id = 0; //0: any, 1: left, 2:right
    Action *action = action_left;

    //publish a virtual object
    blocks.push_back(MetaBlock("Virtual1", ros::Time::now(), pose_default, shape_msgs::SolidPrimitive::CYLINDER, block_size, block_size_l));
    msg_obj_pose.pose = blocks.back().start_pose;
    pub_obj_pose.publish(msg_obj_pose);
    block_id = 0;
    publishCollisionMetaBlock(&blocks.back());

    while(ros::ok()) //main loop
    {
      //int actionState = 0; //nothing is done
      int actionDesired = 0; //reach
      std::string actionName = "";

      while(ros::ok()) //loop objects
      {
        //ROS_INFO_STREAM( block_id << " out of " << blocks.size() << " objects found" );
        if (block_id >= blocks.size()) //check if there are some objects
        {
          block_id = -1;
          removeObjects();
          /*blocks.push_back(MetaBlock("Virtual1", ros::Time::now(), pose_default, shape_msgs::SolidPrimitive::CYLINDER, block_size));
          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          publishCollisionMetaBlock(&blocks.back());*/
        }

        if (block_id != -1)
        {
          msg_obj_pose.pose = blocks[block_id].start_pose;
          pub_obj_pose.publish(msg_obj_pose);
        }

        std::string temp = "";
        if (blocks.size() > 0)
          temp = blocks[block_id].name;
        ROS_INFO_STREAM("What do you want me to do with the object "
                              << temp << " out of " << blocks.size() << " detected objects? ");
        actionName = promptUserQuestionString();
        //actionDesired = promptUserAction();

        ROS_INFO_STREAM("Action chosen '" << actionName
                              << "' object_id=" << block_id
                              //<< " actionState=" << actionState
                              << " hand_id=" << hand_id);

        // Pick -----------------------------------------------------
        if ((block_id != -1) && ((actionDesired == 103) || (actionName == "g"))) //key 'g' //pick the object with a default function //(actionState == 0) &&
        {
          bool success = false;

          if (hand_id != 0) //try with this arm
            success = action->pickAction(&blocks[block_id], SUPPORT_SURFACE3_NAME);
          else //try with any arm
          {
            if (blocks[block_id].start_pose.position.y > 0)
            {
              action = action_left;
              success = action->pickAction(&blocks[block_id], SUPPORT_SURFACE3_NAME);
              blocks[block_id].goal_pose.position.y = 0.25;
            }
            else
            {
              action = action_right;
              success = action->pickAction(&blocks[block_id], SUPPORT_SURFACE3_NAME);
              blocks[block_id].goal_pose.position.y = -0.25;
            }
          }

          if(success)
          {
            stat_poses_success.push_back(blocks[block_id].start_pose);
            //actionState += 2; //approach to object and grasp
          }
        }
        // Place --------------------------------------------------------
        else if ((block_id != -1) && ((actionDesired == 112) || (actionName == "p")))  //key 'p' //place the object with a default function //(actionState == 2) &&
        {
          if(action->placeAction(&blocks[block_id], SUPPORT_SURFACE3_NAME))
          {
            // Swap this block's start and end pose so that we can then move them back to position
            swapPoses(&blocks[block_id].start_pose, &blocks[block_id].goal_pose);
            resetBlock(&blocks[block_id]);
            //++actionState = 0;
          }
        }
        // Return the hand to the zero pose ------------------------------
        else if (actionName == "i0") //key 'z' //move to zero pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          action->poseHandZero();
          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //return the hand to the initial pose ------------------------------
        else if ((actionDesired == 105) || (actionName == "i")) //key 'i' //move to init pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          action->poseHandInit();
          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //return the hand to the initial pose ------------------------------
        else if ((actionName == "i1")) //key 'i1' //move to init pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          std::vector<double> pose_arm_left;
          pose_arm_left.resize(6, 0.0);
          if (robot_name_ == "romeo")
          {
            pose_arm_left[0] = 1.0799225568771362;
            pose_arm_left[1] = 0.6565437912940979;
            pose_arm_left[2] = -0.8390874862670898;
            pose_arm_left[3] = -0.607456386089325;
            pose_arm_left[4] = -0.6672816872596741;
            pose_arm_left[5] = 0.02454369328916073;
          }

          action->poseHand(&pose_arm_left);
          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //return the hand to the initial pose ------------------------------
        else if ((actionName == "i2")) //key 'i2' //move to init pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          if (robot_name_ == "romeo")
          {
            std::vector<double> pose_arm;
            pose_arm.resize(6, 0.0);
            pose_arm[0] = 1.872990608215332;
            pose_arm[1] = 0.5553010702133179;
            pose_arm[2] = -1.9895731210708618;
            pose_arm[3] = -1.052310824394226;
            pose_arm[4] = -0.6703495979309082;
            pose_arm[5] = 0.02147573232650757;

            if (action->arm == "right")
            {
              pose_arm[1] *= -1;
              pose_arm[2] *= -1;
              pose_arm[3] *= -1;
            }
            action->poseHand(&pose_arm);
          }

          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //return the hand to the initial pose ------------------------------
        else if ((actionName == "i3")) //key 'i3' //move to init pose
        {
          // Remove attached object and Remove collision object
          if (block_id >=0)
          {
            visual_tools_->cleanupACO(blocks[block_id].name);
            visual_tools_->cleanupCO(blocks[block_id].name);
          }
          if (robot_name_ == "romeo")
          {
            std::vector<double> pose_arm;
            pose_arm.resize(6, 0.0);
            pose_arm[0] = 1.74;
            pose_arm[1] = 0.75;
            pose_arm[2] = -2.08;
            pose_arm[3] = -1.15;
            pose_arm[4] = -0.43;
            pose_arm[5] = 0.17;

            if (action->arm == "right")
            {
              pose_arm[1] *= -1;
              pose_arm[2] *= -1;
              pose_arm[3] *= -1;
            }
            action->poseHand(&pose_arm);
          }
          if (block_id >=0)
            resetBlock(&blocks[block_id]);
        }
        //exit
        else if ((actionDesired == 113) || (actionName == "q")) //key 'q' //quit the application
          break;
        else if (actionName == "lb") //key 'lb' //clean objects and publish virtual cylinder on the left arm side
        {
          removeObjects();
          blocks.push_back(MetaBlock("Virtual1", ros::Time::now(), pose_default, shape_msgs::SolidPrimitive::BOX, block_size, block_size_l));
          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          publishCollisionMetaBlock(&blocks.back());
        }
        //create a virtual object on the left hand side
        else if (actionName == "lc") //key 'c' //clean objects and publish virtual cylinder on the left arm side
        {
          removeObjects();
          blocks.push_back(MetaBlock("Virtual1", ros::Time::now(), pose_default, shape_msgs::SolidPrimitive::CYLINDER, block_size, block_size_l));
          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          publishCollisionMetaBlock(&blocks.back());
        }
        else if (actionName == "rb") //key 'r' //clean and publish virtual for the right arm
        {
          removeObjects();
          blocks.push_back(MetaBlock("Virtual1", ros::Time(), pose_default_r, shape_msgs::SolidPrimitive::BOX, block_size, block_size_l));
          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          publishCollisionMetaBlock(&blocks.back());
        }
        else if (actionName == "rc") //key 'r' //clean and publish virtual for the right arm
        {
          removeObjects();
          blocks.push_back(MetaBlock("Virtual1", ros::Time(), pose_default_r, shape_msgs::SolidPrimitive::CYLINDER, block_size, block_size_l));
          msg_obj_pose.pose = blocks.back().start_pose;
          pub_obj_pose.publish(msg_obj_pose);
          block_id = 0;
          publishCollisionMetaBlock(&blocks.back());
        }
        else if ((actionDesired == 100) || (actionName == "d")) //key 'd' //detect objects
        {
          block_id = -1;
          removeObjects();

          objproc.triggerObjectDetection();
          // publish all objects as collision blocks
          if (blocks.size() > 0)
          {
            block_id = 0;
            for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
            {
              visual_tools_->cleanupCO(block->name);

              moveit_msgs::CollisionObject msg = wrapToCollisionObject(&(*block));
              //if (!msg.meshes.empty())
                visual_tools_->processCollisionObjectMsg(msg);
              /*else
                publishCollisionMetaBlock(&(*block));*/
            }
          }
          else
            block_id = -1;
        }
        //dd continuous object detection
        else if (actionName == "dd") //key 'dd' //detect objects
        {
          block_id = -1;
          removeObjects();

          ros::Time start_time = ros::Time::now();
          while ((blocks.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(15.0)))
          {
            ROS_INFO_STREAM("**** object detection is running, objects detected  " << blocks.size() << " " << ros::Time::now());
            objproc.triggerObjectDetection();
          }

          // publish all objects as collision blocks
          if (blocks.size() > 0)
          {
            block_id = 0;
            for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
            {
              //visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&(*block)));
              moveit_msgs::CollisionObject msg = wrapToCollisionObject(&(*block));
              if (!msg.primitive_poses.empty())
                visual_tools_->processCollisionObjectMsg(msg);
            }
          }
          else
            block_id = -1;
        }
        //------------ reaching generated poses by moveit simple grasps
        else if ((block_id != -1) && (actionName == "action")) //key 'action' //plan movement
        {
          visual_tools_->cleanupCO(blocks[block_id].name);
          action->graspPlan(&blocks[block_id], SUPPORT_SURFACE3_NAME);
          resetBlock(&blocks[block_id]);
        }
        else if ((block_id != -1) && ((actionDesired == 97) || (actionName == "a"))) //key 'a' //plan all movement
        {
         visual_tools_->cleanupCO(blocks[block_id].name);
          action->graspPlanAllPossible(&blocks[block_id], SUPPORT_SURFACE3_NAME);
          resetBlock(&blocks[block_id]);
        }
        //------------ reaching default poses
        else if ((block_id != -1) && ((actionDesired == 117) || (actionName == "u"))) //key 'u' //reach and grasp
        {
          //clean object temporally or allow to touch it
          visual_tools_->cleanupCO(blocks[block_id].name);

          float dist = std::numeric_limits<float>::max();

          if (hand_id != 0) //try with this arm
            dist = action->reachGrasp(&blocks[block_id], SUPPORT_SURFACE3_NAME);
          else //try with any arm
          {
            if (blocks[block_id].start_pose.position.y > 0)
            {
              action = action_left;
              dist = action->reachGrasp(&blocks[block_id], SUPPORT_SURFACE3_NAME);
              //TODO ideally, we should set the goal pose based on the current position
              blocks[block_id].goal_pose.position.y = 0.25;
            }
            else
            {
              action = action_right;
              dist = action->reachGrasp(&blocks[block_id], SUPPORT_SURFACE3_NAME);
              //TODO ideally, we should set the goal pose based on the current position
              blocks[block_id].goal_pose.position.y = -0.25;
            }
          }

          if (dist < 0.2)
          {
            //visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(&blocks[block_id]));
            //visual_tools_->attachCO(blocks[block_id].name, action->grasp_data_.ee_group_);
            action->poseHandClose();
          }
          resetBlock(&blocks[block_id]);
        }
        else if ((block_id != -1) && (actionName == "pregrasp")) //key 'x' //reach the pregrasp pose
        {
          visual_tools_->cleanupCO(blocks[block_id].name);
          action->reachPregrasp(blocks[block_id].start_pose, SUPPORT_SURFACE3_NAME);
          resetBlock(&blocks[block_id]);
        }
        else if ((block_id != -1) && (actionName == "reachtop")) //key 'y' //reach from top
        {
          visual_tools_->cleanupCO(blocks[block_id].name);
          geometry_msgs::Pose pose = blocks[block_id].start_pose;
          pose.orientation.x = 0;
          pose.position.z += blocks[block_id].size*1.5; //0.12;
          action->reachAction(pose, SUPPORT_SURFACE3_NAME);
          resetBlock(&blocks[block_id]);
        }
        else if ((block_id != -1) && ((actionDesired == 98) || (actionName == "b"))) //key 'b'
        {
          action->pickDefault(&blocks[block_id]);
        }
        else if ((block_id != -1) && ((actionDesired == 119) || (actionName == "w"))) //key 'w' //reach the init pose
        {
          visual_tools_->cleanupCO(blocks[block_id].name);
          action->reachInitPose();
          resetBlock(&blocks[block_id]);
        }
        //--------------
        else if ((block_id != -1) && (actionName == "execute")) //execute tha plan
        {
          action->executeAction();
        }
        else if ((actionDesired == 118) || (actionName == "v")) //print the current pose
        {
          action->getPose();
        }
        else if (actionName == "open") //open hand
        {
          action->poseHandOpen();
        }
        else if (actionName == "close") //close hand
        {
          action->poseHandOpen();
        }
        else if ((actionDesired == 110) || (actionName == "n")) //key 'n' //process the next object
          ++block_id;
        else if (actionName == "test_pick") //test the goal space for picking
          testReach(true);
        else if (actionName == "test_reach") //test the goal space for reaching
          testReach(false);
        else if (actionName == "set_table_height") // set table height
          updateTable(visual_tools_, promptUserValue("give the table height"));
        else if ((actionDesired == 116) || (actionName == "t")) //clean the scene
        {
          if (env_shown_)
          {
            cleanEnvironment(visual_tools_);
            env_shown_ = false;
            removeObjects();
          }
          else
          {
            createEnvironment(visual_tools_);
            env_shown_ = true;
          }
        }
        else if ((actionDesired == 104) || (actionName == "h")) //key 'h'
        {
          if (hand_id < 2)
          {
            hand_id = 2;
            action = action_right;
          }
          else
          {
            hand_id = 1;
            action = action_left;
          }
        }
        //moving the virtual objects
        else if ((actionDesired == 50) || (actionName == "x") || (actionName == "down")) //down
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[block_id].start_pose;
            pose.position.z -= 0.05;
            blocks[block_id].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 52) || (actionName == "s") || (actionName == "left")) //left
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[block_id].start_pose;
            pose.position.y -= 0.05;
            blocks[block_id].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 56) || (actionName == "e") || (actionName == "up")) //up or 'e'
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[block_id].start_pose;
            pose.position.z += 0.05;
            blocks[block_id].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 54) || (actionName == "f") || (actionName == "right")) //right
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[block_id].start_pose;
            pose.position.y += 0.05;
            blocks[block_id].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 51) || (actionName == "c") || (actionName == "farther")) //farther
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.x += 0.05;
            blocks[0].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 57) || (actionName == "r") || (actionName == "closer")) //closer
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.position.x -= 0.05;
            blocks[0].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if (actionDesired == 55) //num-1
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.orientation.x = -1;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 0;
            blocks[0].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if (actionDesired == 49) //num-7
        {
          if (blocks.size() > 0)
          {
            geometry_msgs::Pose pose = blocks[0].start_pose;
            pose.orientation.x = 0;
            pose.orientation.y = 0;
            pose.orientation.z = 0;
            pose.orientation.w = 0;
            blocks[0].updatePose(pose);
            resetBlock(&blocks[block_id]);
          }
        }
        else if ((actionDesired == 106) || (actionName == "j")) //key 'j', set the tolerance
          action->setTolerance(promptUserValue("Set the value: "));
        else if ((actionDesired == 109) || (actionName == "m")) //key 'm', move the head down
          action->poseHeadDown();
        else if ((actionDesired == 107) || (actionName == "k")) //key 'k', move the head to zero
          action->poseHeadZero();
        else if (actionName == "stat") //key 'stat', print the statistics on grasps
        {
          ROS_INFO_STREAM("Successfully grasped objects at the following locations: ");
          for (std::vector <geometry_msgs::Pose>::iterator it = stat_poses_success.begin(); it != stat_poses_success.end(); ++it)
          {
            ROS_INFO_STREAM(" [" <<  it-> position.x << " " << it->position.y << " " << it->position.z << "] ["
                            << it->orientation.x << " " << it->orientation.y << " " << it->orientation.z << " " << it->orientation.w << "]");
          }
        }
      }

      if ((actionDesired == 113) || (actionName == "q"))
        break;
    }
    return true;
  }

  //clean the object list based on the timestamp
  void SimplePickPlace::cleanObjects()
  {
    ros::Time now = ros::Time::now() - ros::Duration(60);

    for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
    {
      if (block->timestamp < now)
      {
        // Remove attached object
        visual_tools_->cleanupACO(block->name);
        // Remove collision object
        visual_tools_->cleanupCO(block->name);
        //blocks.erase(block);
      }
    }
  }

  moveit_msgs::CollisionObject SimplePickPlace::wrapToCollisionObject(MetaBlock *block)
  {
    moveit_msgs::CollisionObject msg_obj_collision;
    msg_obj_collision.header.stamp = ros::Time::now();
    msg_obj_collision.header.frame_id = base_frame;

    msg_obj_collision.id = block->name;
    msg_obj_collision.operation = moveit_msgs::CollisionObject::ADD;

    object_recognition_msgs::GetObjectInformation obj_info;
    obj_info.request.type = block->type;

    if (objproc.getMeshFromDB(obj_info))
    {
      msg_obj_collision.meshes.push_back(obj_info.response.information.ground_truth_mesh);
      msg_obj_collision.mesh_poses.push_back(block->start_pose);
      //ROS_INFO_STREAM("-- mesh found: msg_obj_collision.meshes.size()=" << msg_obj_collision.meshes.size());
    }
    else
    {
      shape_msgs::SolidPrimitive msg_cylinder_;
      msg_cylinder_.type = shape_msgs::SolidPrimitive::CYLINDER;
      msg_cylinder_.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
      msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = block_size;
      msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = block_size_l;

      msg_obj_collision.primitives.push_back(msg_cylinder_);
      msg_obj_collision.primitive_poses.push_back(block->start_pose);
      //ROS_INFO_STREAM("-- mesh not found: " << block->start_pose);
    }

    msg_obj_collision.type = block->type;
    return msg_obj_collision;
  }



  void SimplePickPlace::resetBlock(MetaBlock *block)
  {
    // Remove attached object
    visual_tools_->cleanupACO(block->name);

    // Remove collision object
    visual_tools_->cleanupCO(block->name);

    // Add the collision block
    visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(block));
    //publishCollisionMetaBlock(block);
  }

  void SimplePickPlace::publishCollisionMetaBlock(MetaBlock *block)
  {
    moveit_msgs::CollisionObject collision_obj;
    if (block->shape.type == shape_msgs::SolidPrimitive::BOX)
    {
      collision_obj = vtools_.publishCollisionBlock(block->start_pose, block->name, block_size);
      //visual_tools_->publishCollisionBlock(block->start_pose, block->name, block_size);
    }
    else if (block->shape.type == shape_msgs::SolidPrimitive::CYLINDER)
    {
      collision_obj = vtools_.publishCollisionCylinder(block->start_pose, block->name, block_size, block_size_l);
      //visual_tools_->publishCollisionCylinder(block->start_pose, block->name, block_size, block_size_l);
    }
    pub_obj_moveit.publish(collision_obj);
  }

  void SimplePickPlace::getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg)
  {
    cleanObjects();

    try {
      if (msg->meshes.size() > 0)
      {
        geometry_msgs::Pose pose = msg->mesh_poses[0];
        if ((pose.position.x < x_max_) && (pose.position.x > x_min_)
            && (pose.position.z < z_max_) && (pose.position.z > z_min_))
        {
          //check if exists
          int idx = -1;
          for (int i=0; i<blocks.size(); ++i)
            if (blocks[i].name == msg->id){
              idx = i;
              break;
            }

          //pose.position.z += 0.0576;
          if ((idx == -1) || (idx >= blocks.size()) || (idx >= msg_obj_poses.poses.size())) //if not found, create a new one
          {
            blocks.push_back(MetaBlock(msg->id, msg->header.stamp, pose, msg->meshes.front(), msg->type));
            msg_obj_poses.poses.push_back(pose);
          }
          else if ((idx >= 0) && (idx < blocks.size()))
          {
            blocks[idx].updatePose(pose);
            msg_obj_poses.poses[idx] = pose;
          }
        }
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  void SimplePickPlace::removeObjects()
  {
    //ROS_INFO_STREAM("cleaning blocks");
    //remove from the scene
    for (std::vector<MetaBlock>::iterator block=blocks.begin(); block != blocks.end(); ++block)
    {
      // Remove attached object
      visual_tools_->cleanupACO(block->name);
      // Remove collision object
      visual_tools_->cleanupCO(block->name);
    }

    //remove from the memory
    blocks.clear();
    msg_obj_poses.poses.clear();
    pub_obj_poses.publish(msg_obj_poses);
  }

} //namespace


