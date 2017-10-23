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

//publish messages with objects poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group.h>

#include "romeo_moveit_actions/simplepickplace.hpp"
#include "romeo_moveit_actions/tools.hpp"
#include "romeo_moveit_actions/toolsForObject.hpp"

namespace moveit_simple_actions
{
  MetaBlock SimplePickPlace::createTable()
  {
    //create a table
    double height = -floor_to_base_height_ + (pose_default_.position.z-block_size_y_/2.0);
    double width = std::fabs(pose_default_r_.position.y*2.0) + block_size_x_/2.0;
    double depth = 0.35;
    geometry_msgs::Pose pose;
    setPose(&pose,
            pose_default_.position.x - block_size_x_/2.0 + depth/2.0,
            0.0,
            floor_to_base_height_ + height/2.0);

    MetaBlock table("table",
                    pose,
                    shape_msgs::SolidPrimitive::BOX,
                    depth,
                    width,
                    height);
    return table;
  }

  SimplePickPlace::SimplePickPlace(const std::string robot_name,
                                   double test_step,
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
      verbose_(verbose),
      base_frame_("odom"),
      block_size_x_(0.03),
      block_size_y_(0.13),
      floor_to_base_height_(-1.0),
      env_shown_(false),
      evaluation_(verbose_, base_frame_),
      obj_proc_(&nh_priv_, &evaluation_),
      rate_(12.0)
  {
    setPose(&pose_zero_, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0);
    pose_default_ = pose_zero_;
    pose_default_r_ = pose_default_;
    if (robot_name_ == "nao")
    {
      block_size_x_ = 0.01;
      setPose(&pose_default_, 0.2, 0.1, 0.0);
      setPose(&pose_default_r_, 0.2, -0.1, 0.0);
      test_step = (test_step==0.0)?0.03:test_step;
      x_min_ = (x_min==0.0)?0.1:x_min;
      x_max_ = (x_max==0.0)?0.21:x_max;
      y_min_ = (y_min==0.0)?0.12:y_min;
      y_max_ = (y_max==0.0)?0.24:y_max;
      z_min_ = (z_min==0.0)?-0.07:z_min;
      z_max_ = (z_max==0.0)?0.05:z_max;
    }
    else if (robot_name == "pepper")
    {
      block_size_x_ = 0.02;
      setPose(&pose_default_, 0.25, 0.2, -0.1);
      setPose(&pose_default_r_, 0.25, -0.2, -0.1);
      test_step = (test_step==0.0)?0.04:test_step;
      x_min_ = (x_min==0.0)?0.2:x_min;
      x_max_ = (x_max==0.0)?0.4:x_max;
      y_min_ = (y_min==0.0)?0.12:y_min;
      y_max_ = (y_max==0.0)?0.24:y_max;
      z_min_ = (z_min==0.0)?-0.13:z_min;
      z_max_ = (z_max==0.0)?0.0:z_max;
    }
    else if (robot_name_ == "romeo")
    {
      setPose(&pose_default_, 0.44, 0.15, -0.1);
      setPose(&pose_default_r_, 0.49, -0.25, -0.1);
      test_step = (test_step==0.0)?0.02:test_step;
      x_min_ = (x_min==0.0)?0.38:x_min;
      x_max_ = (x_max==0.0)?0.5:x_max;
      y_min_ = (y_min==0.0)?0.12:y_min;
      y_max_ = (y_max==0.0)?0.24:y_max;
      z_min_ = (z_min==0.0)?-0.17:z_min;
      z_max_ = (z_max==0.0)?-0.08:z_max;
    }

    // objects related initialization
    /*sub_obj_coll_ = nh_.subscribe<moveit_msgs::CollisionObject>(
          "/collision_object", 100, &SimplePickPlace::getCollisionObjects, this);*/

    pub_obj_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_current", 1);

    pub_obj_moveit_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("odom"));
    visual_tools_->setManualSceneUpdating(false);
    visual_tools_->setFloorToBaseHeight(floor_to_base_height_);
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    ros::Duration(1.0).sleep();

    action_left_ = new Action(&nh_, left_arm_name, robot_name_);
    action_right_ = new Action(&nh_, right_arm_name, robot_name_);
    action_left_->initVisualTools(visual_tools_);
    action_right_->initVisualTools(visual_tools_);

    msg_obj_pose_.header.frame_id = action_left_->getBaseLink();

    //move the robots parts to init positions
    //if (promptUserQuestion("Should I move the head to look down?"))
      //action_left_->poseHeadDown();

    //move arms to the initial position
    //if (promptUserQuestion(("Should I move the "+action_left_->end_eff+" to the initial pose?").c_str()))
    action_left_->poseHand(1);

    //if (promptUserQuestion(("Should I move the "+action_right_->plan_group+" to the initial pose?").c_str()))
    action_right_->poseHand(1);

    ros::Duration(1.0).sleep();

    //create a possible table
    blocks_surfaces_.push_back(createTable());
    support_surface_ = blocks_surfaces_.back().name_;
    //remove surfaces and publish again
    if (robot_name_ == "romeo")
    {
      std::vector<std::string> objects = getObjectsOldList(&blocks_surfaces_);
      current_scene_.removeCollisionObjects(objects);
      sleep(1.0);
      //add a collision block
      if (blocks_surfaces_.size() > 0)
      {
        blocks_surfaces_.back().publishBlock(&current_scene_);
        env_shown_ = true;
      }
    }

    evaluation_.init(test_step,
                     block_size_x_,
                     block_size_y_,
                     floor_to_base_height_,
                     x_min_,
                     x_max_,
                     y_min_,
                     y_max_,
                     z_min_,
                     z_max_);

    printTutorial(robot_name);
  }


  void SimplePickPlace::switchArm(Action *action)
  {
    if (action->arm_.find("left") != std::string::npos)
      action = action_right_;
    else
      action = action_left_;

    ROS_INFO_STREAM("Switching the active arm to " << action->arm_);
    sleep(2.0);
  }

  void SimplePickPlace::createObj(const MetaBlock &block)
  {
    obj_proc_.addBlock(block);
    msg_obj_pose_.header.frame_id = block.base_frame_;
    msg_obj_pose_.pose = block.pose_;
    pub_obj_pose_.publish(msg_obj_pose_);

    //publish the collision object
    //obj_proc_.blocks_->back().publishBlock(&current_scene_);
  }

  bool SimplePickPlace::checkObj(int &block_id)
  {
    if ((block_id >= 0) && (block_id < obj_proc_.getAmountOfBlocks()))
      return true;
    else
      false;
  }

  void SimplePickPlace::run()
  {
    int block_id = -1;
    int hand_id = 0; //0: any, 1: left, 2:right
    Action *action = action_left_;

    //create a virtual object
    MetaBlock block_l("Virtual1",
                      pose_default_,
                      shape_msgs::SolidPrimitive::CYLINDER,
                      block_size_x_,
                      block_size_y_,
                      0.0);

    createObj(block_l);
    block_id = 0;

    std::string actionName = "";
    MetaBlock *block;

    //the main loop
    while(ros::ok())
    {
      //if there are some objects, take the first
      if ((block_id == -1) && (obj_proc_.getAmountOfBlocks() > 0))
        block_id = 0;
      //if the object does not exist
      else if (block_id >= obj_proc_.getAmountOfBlocks())
      {
        ROS_WARN_STREAM("The object " << block_id << " does not exist");
        block_id = -1;
        obj_proc_.cleanObjects(true);
      }

      if (block_id >= 0)
      {
        block = obj_proc_.getBlock(block_id);
        if (block == NULL)
        {
          ROS_INFO_STREAM("the object " << block_id << " does not exist");
          block_id = -1;
          continue;
        }

        //update the object's pose
        msg_obj_pose_.header.frame_id = block->base_frame_;
        msg_obj_pose_.pose = block->pose_;
        pub_obj_pose_.publish(msg_obj_pose_);
        ROS_INFO_STREAM("The current active object is "
                        << block->name_
                        << " out of " << obj_proc_.getAmountOfBlocks());
        //obj_proc_.publishAllCollObj(&blocks_);
      }

      //ROS_INFO_STREAM("What do you want me to do ?");
      actionName = promptUserQuestionString();
      ROS_INFO_STREAM("Action chosen '" << actionName
                      << "' object_id=" << block_id
                      << " the arm active=" << action->arm_);
      // Pick the object with a grasp generator
      if ((checkObj(block_id)) && (actionName == "g"))
      {
        bool success = action->pickAction(block, support_surface_);
        //if not succeded then try with another arm
        if(!success)
        {
          switchArm(action);
          success = action->pickAction(block, support_surface_);
        }

        if(success)
          stat_poses_success_.push_back(block->pose_);
      }
      // Place the object with a default function
      else if ((checkObj(block_id)) && (actionName == "p"))
      {
        if(action->placeAction(block, support_surface_))
        {
          /* swap this block's start and the end pose
           * so that we can then move them back to position */
          swapPoses(&block->pose_, &block->goal_pose_);
          resetBlock(block);
        }
      }
      //return the hand to the initial pose and relese the object
      else if (actionName == "i")
      {
        // Remove the attached object and the collision object
        if (checkObj(block_id))
          action->releaseObject(block);
        else
          action->poseHand(1);
      }
      //return the hand to the initial pose
      else if ((actionName.length() == 3) || (actionName.compare(0,1,"i") == 0))
      {
        //remove the attached object and the collision object
        if (checkObj(block_id))
          resetBlock(block);
        //go to the required pose
        action->poseHand(actionName.at(1));
      }
      //exit the application
      else if (actionName == "q")
        break;
      //clean a virtual box on the left arm side
      else if (actionName == "lb")
      {
        obj_proc_.cleanObjects();
        createObj(MetaBlock("Virtual1", pose_default_, shape_msgs::SolidPrimitive::BOX, block_size_x_, block_size_y_, 0.0));
      }
      //create a virtual cylinder on the left hand side
      else if (actionName == "lc")
      {
        obj_proc_.cleanObjects();
        createObj(MetaBlock("Virtual1", pose_default_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));
      }
      //create a virtual box on the right hand side
      else if (actionName == "rb")
      {
        obj_proc_.cleanObjects();
        createObj(MetaBlock("Virtual1", pose_default_r_, shape_msgs::SolidPrimitive::BOX, block_size_x_, block_size_y_, 0.0));
      }
      //create a virtual cylinder on the right hand side
      else if (actionName == "rc")
      {
        obj_proc_.cleanObjects();
        createObj(MetaBlock("Virtual1", pose_default_r_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));
      }
      //detect objects
      else if (actionName == "d")
      {
        obj_proc_.cleanObjects();

        obj_proc_.triggerObjectDetection();
      }
      //continuous object detection
      else if (actionName == "dd")
      {
        obj_proc_.cleanObjects(true);

        ROS_INFO_STREAM("Object detection is running...");
        ros::Time start_time = ros::Time::now();
        while ((obj_proc_.getAmountOfBlocks() <= 0)
               && (ros::Time::now()-start_time < ros::Duration(10.0)))
        {
          obj_proc_.triggerObjectDetection();
          rate_.sleep();
        }
        if (verbose_)
          ROS_INFO_STREAM(obj_proc_.getAmountOfBlocks() << " objects detected");
      }
      //plan possible movement
      else if ((checkObj(block_id)) && (actionName == "plan"))
      {
        //TODO: do not remove an object but allow a collision to it
        visual_tools_->cleanupCO(block->name_);
        action->graspPlan(block, support_surface_);
        resetBlock(block);
      }
      //plan all possible movements
      else if ((checkObj(block_id)) && (actionName == "a"))
      {
        //TODO: do not remove an object but allow a collision to it
        visual_tools_->cleanupCO(block->name_);
        action->graspPlanAllPossible(block, support_surface_);
        resetBlock(block);
      }
      //reaching based on default pose and grasp
      else if ((checkObj(block_id)) && (actionName == "u"))
      {
        float dist = action->reachGrasp(block, support_surface_);
        //if not succeded then try with another arm
        if(dist > 10)
        {
          switchArm(action);
          dist = action->reachGrasp(block, support_surface_);
        }
      }
      //reach from top
      else if ((checkObj(block_id)) && (actionName == "reachtop"))
      {
        //TODO: do not remove an object but allow a collision to it
        visual_tools_->cleanupCO(block->name_);
        geometry_msgs::Pose pose = block->pose_;
        pose.orientation.x = 0;
        pose.position.z += block->size_x_*1.5;
        action->reachAction(pose, support_surface_);
        resetBlock(block);
      }
      //pick an object without a grasp generator
      else if ((checkObj(block_id)) && (actionName == "b"))
      {
        action->pickDefault(block, support_surface_);
      }
      //execute the planned movement
      else if ((checkObj(block_id)) && (actionName == "execute"))
      {
        action->executeAction();
      }
      //print the current pose
      else if (actionName == "get_pose")
      {
        action->getPose();
      }
      //open hand
      else if (actionName == "hand_open")
      {
        action->poseHandOpen();
      }
      //close hand
      else if (actionName == "hand_close")
      {
        action->poseHandClose();
      }
      //process the next object
      else if (actionName == "next_obj")
        ++block_id;
      //test the goal space for picking
      else if (actionName == "test_pick")
      {
        cleanObjects(&blocks_surfaces_, false);
        obj_proc_.cleanObjects();
        evaluation_.testReach(nh_,
                              &pub_obj_pose_,
                              &obj_proc_.pub_obj_poses_,
                              &pub_obj_moveit_,
                              visual_tools_,
                              action_left_,
                              action_right_,
                              &blocks_surfaces_,
                              true);
      }
      //test the goal space for reaching
      else if (actionName == "test_reach")
      {
        obj_proc_.cleanObjects();
        evaluation_.testReach(nh_,
                              &pub_obj_pose_,
                              &obj_proc_.pub_obj_poses_,
                              &pub_obj_moveit_,
                              visual_tools_,
                              action_left_,
                              action_right_,
                              &blocks_surfaces_,
                              false);
      }
      //set the table height
      else if (actionName == "set_table_height")
      {
        //TODO make a function
        if (blocks_surfaces_.size() > 0)
        {
          blocks_surfaces_.back().size_z_ = promptUserValue("Set the table height to");
          blocks_surfaces_.back().pose_.position.z =
              floor_to_base_height_ + blocks_surfaces_.back().size_z_/2.0;
          blocks_surfaces_.back().publishBlock(&current_scene_);
        }
      }
      //clean the scene
      else if (actionName == "table")
      {
        if (env_shown_)
        {
          //cleanObjects(&blocks_surfaces_, false);
          //cleanObjects(&blocks_);
          //env_shown_ = false;
          std::vector<std::string> objects = getObjectsList(blocks_surfaces_);
          current_scene_.removeCollisionObjects(objects);
          env_shown_ = false;
        }
        else
        {
          //pub_obj_moveit_.publish(blocks_surfaces_.front().collObj_);
          //env_shown_ = true;
          if (blocks_surfaces_.size() > 0)
          {
            blocks_surfaces_.back().publishBlock(&current_scene_);
            env_shown_ = true;
          }
        }
      }
      //moving the virtual object down
      else if (checkObj(block_id) && ((actionName == "x") || (actionName == "down")))
      {
        geometry_msgs::Pose pose(block->pose_);
        pose.position.z -= 0.05;
        block->updatePose(&current_scene_, pose);
      }
      //move the virtual object left
      else if (checkObj(block_id) && ((actionName == "s") || (actionName == "left")))
      {
        geometry_msgs::Pose pose(block->pose_);
        pose.position.y -= 0.05;
        block->updatePose(&current_scene_, pose);
      }
      //move the virtual object up
      else if (checkObj(block_id) && ((actionName == "e") || (actionName == "up")))
      {
        geometry_msgs::Pose pose(block->pose_);
        pose.position.z += 0.05;
        block->updatePose(&current_scene_, pose);
      }
      //move the virtual object right
      else if (checkObj(block_id) && ((actionName == "f") || (actionName == "right")))
      {
        geometry_msgs::Pose pose(block->pose_);
        pose.position.y += 0.05;
        block->updatePose(&current_scene_, pose);
      }
      //move the virtual object farther
      else if (checkObj(block_id) && ((actionName == "c") || (actionName == "farther")))
      {
        geometry_msgs::Pose pose(block->pose_);
        pose.position.x += 0.05;
        block->updatePose(&current_scene_, pose);
      }
      //move the virtual object closer
      else if (checkObj(block_id) && ((actionName == "r") || (actionName == "closer")))
      {
        geometry_msgs::Pose pose(block->pose_);
        pose.position.x -= 0.05;
        block->updatePose(&current_scene_, pose);
      }
      //set the tolerance
      else if (actionName == "j")
        action->setTolerance(promptUserValue("Set the value: "));
      //switch the active arm
      else if (actionName == "switcharm")
        switchArm(action);
      //move the head down
      else if (actionName == "look_down")
        action->poseHeadDown();
      //move the head to zero
      else if (actionName == "look_zero")
        action->poseHeadZero();
      //print the statistics on grasps
      else if (actionName == "stat")
      {
        ROS_INFO_STREAM("Successfully grasped objects at the locations: ");
        std::vector <geometry_msgs::Pose>::iterator it = stat_poses_success_.begin();
        for (; it != stat_poses_success_.end(); ++it)
        {
          ROS_INFO_STREAM(" [" << it->position.x << " "
                          << it->position.y << " "
                          << it->position.z << "] ["
                          << it->orientation.x << " "
                          << it->orientation.y << " "
                          << it->orientation.z << " "
                          << it->orientation.w << "]");
        }
      }
      //print the statistics on grasps
      else if (actionName == "stat_evaluation")
      {
        evaluation_.printStat();
      }

      if (actionName == "q")
        break;
    }
  }

  //clean the object list based on the timestamp
  void SimplePickPlace::cleanObjects(std::vector<MetaBlock> *objects,
                                     const bool list_erase)
  {
    std::vector<std::string> objects_list = getObjectsOldList(objects);
    current_scene_.removeCollisionObjects(objects_list);

    //remove from the memory
    if (list_erase)
      objects->clear();
  }

  void SimplePickPlace::resetBlock(MetaBlock *block)
  {
    // Remove attached object
    if(block->name_ != support_surface_)
    {
      action_left_->detachObject(block->name_);
      action_right_->detachObject(block->name_);
    }
    sleep(0.2);

    // Remove/Add collision object
    //visual_tools_->cleanupCO(block->name_);
    //visual_tools_->processCollisionObjectMsg(block->wrapToCollisionObject);

    block->publishBlock(&current_scene_);
  }

} //namespace
