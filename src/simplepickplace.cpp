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
    //create a table

    // objects related initialization
    /*sub_obj_coll_ = nh_.subscribe<moveit_msgs::CollisionObject>(
          "/collision_object", 100, &SimplePickPlace::getCollisionObjects, this);*/

    pub_obj_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/obj_poses", 10);
    pub_obj_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_current", 10);

    pub_obj_moveit_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 1000);

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
    msg_obj_poses_.header.frame_id = action_left_->getBaseLink();

    //Move the robots parts to init positions
    //if (promptUserQuestion("Should I move the head to look down?"))
      //action_left_->poseHeadDown();

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

    evaluation_.init(test_step, block_size_x_, block_size_y_, floor_to_base_height_,
                     x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);

    printTutorial(robot_name);

    startRoutine();
  }


  void SimplePickPlace::switchArm(Action *action_now)
  {
    if (action_now->arm_ == "left")
      action_now = action_right_;
    else
      action_now = action_left_;
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

  bool SimplePickPlace::startRoutine()
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
      std::string actionName = "";
      while(ros::ok()) //loop objects
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

        // Pick -----------------------------------------------------
        if ((checkObj(block_id)) && (actionName == "g")) //key 'g' //pick the object with a grasp generator
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
        // Place --------------------------------------------------------
        else if ((checkObj(block_id)) && (actionName == "p"))  //key 'p' //place the object with a default function
        {
          if(action->placeAction(block, support_surface_))
          {
            // Swap this block's start and end pose so that we can then move them back to position
            swapPoses(&block->pose_, &block->goal_pose_);
            resetBlock(block);
          }
        }
        // Return the hand to the zero pose ------------------------------
        else if (actionName == "i0") //key 'z' //move to zero pose
        {
          // Remove the attached object and the collision object
          if (checkObj(block_id))
            resetBlock(block);
          action->poseHand(0);
        }
        //return the hand to the initial pose ------------------------------
        else if (actionName == "i") //key 'i' //move to init pose
        {
          // Remove the attached object and the collision object
          if (checkObj(block_id))
            resetBlock(block);
          action->poseHand(1);
        }
        //return the hand to the initial pose ------------------------------
        else if (actionName == "i1") //key 'i1' //move to init pose
        {
          // Remove the attached object and the collision object
          if (checkObj(block_id))
            resetBlock(block);
          action->poseHand(2);
        }
        //return the hand to the initial pose ------------------------------
        else if (actionName == "i2") //key 'i2' //move to init pose
        {
          // Remove the attached object and the collision object
          if (checkObj(block_id))
            resetBlock(block);
          action->poseHand(3);
        }
        //exit
        else if (actionName == "q") //key 'q' //quit the application
          break;
        else if (actionName == "lb") //key 'lb' //clean objects and publish virtual cylinder on the left arm side
        {
          cleanObjects(&blocks_);
          createObj(MetaBlock("Virtual1", pose_default_, shape_msgs::SolidPrimitive::BOX, block_size_x_, block_size_y_, 0.0));
        }
        //create a virtual object on the left hand side
        else if (actionName == "lc") //key 'c' //clean objects and publish virtual cylinder on the left arm side
        {
          cleanObjects(&blocks_);
          createObj(MetaBlock("Virtual1", pose_default_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));
        }
        else if (actionName == "rb") //key 'r' //clean and publish virtual for the right arm
        {
          cleanObjects(&blocks_);
          createObj(MetaBlock("Virtual1", pose_default_r_, shape_msgs::SolidPrimitive::BOX, block_size_x_, block_size_y_, 0.0));
        }
        else if (actionName == "rc") //key 'r' //clean and publish virtual for the right arm
        {
          cleanObjects(&blocks_);
          createObj(MetaBlock("Virtual1", pose_default_r_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));
        }
        else if (actionName == "d") //key 'd' //detect objects
        {
          cleanObjects(&blocks_);

          obj_proc_.triggerObjectDetection();
        }
        //dd continuous object detection
        else if (actionName == "dd") //key 'dd' //detect objects
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
        //------------ reaching all poses generated by moveit simple grasps
        else if ((checkObj(block_id))&& (actionName == "plan")) //key 'action' //plan movement
        {
          //TODO: do not remove an object but allow a collision to it
          visual_tools_->cleanupCO(block->name_);
          action->graspPlan(block, support_surface_);
          resetBlock(block);
        }
        else if ((checkObj(block_id)) && (actionName == "a")) //key 'a' //plan all movement
        {
          //TODO: do not remove an object but allow a collision to it
          visual_tools_->cleanupCO(block->name_);
          action->graspPlanAllPossible(block, support_surface_);
          resetBlock(block);
        }
        //------------ reaching the default pose
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
        //moving the virtual object down
        else if ((checkObj(block_id)) && (actionName == "pregrasp"))
        {
          //TODO: do not remove an object but allow a collision to it
          visual_tools_->cleanupCO(block->name_);
          action->reachPregrasp(block->pose_, support_surface_);
          resetBlock(block);
        }
        else if ((checkObj(block_id)) && (actionName == "reachtop")) //key 'y' //reach from top
        {
          //TODO: do not remove an object but allow a collision to it
          visual_tools_->cleanupCO(block->name_);
          geometry_msgs::Pose pose = block->pose_;
          pose.orientation.x = 0;
          pose.position.z += block->size_x_*1.5; //0.12;
          action->reachAction(pose, support_surface_);
          resetBlock(block);
        }
        else if ((checkObj(block_id)) && (actionName == "b")) //key 'b' //pick an object without a grasp generator
        {
          action->pickDefault(block, support_surface_);
        }
        else if ((checkObj(block_id)) && (actionName == "execute")) //execute tha plan
        {
          action->executeAction();
        }
        else if (actionName == "v") //print the current pose
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
        else if (actionName == "n") //key 'n' //process the next object
          ++block_id;
        else if (actionName == "test_pick") //test the goal space for picking
        {
          cleanObjects(&blocks_);
          evaluation_.testReach(nh_,
                                &pub_obj_pose_,
                                &pub_obj_poses_,
                                &pub_obj_moveit_,
                                visual_tools_,
                                action_left_,
                                action_right_,
                                &blocks_surfaces_,
                                true);
        }
        else if (actionName == "test_reach") //test the goal space for reaching
        {
          cleanObjects(&blocks_);
          evaluation_.testReach(nh_,
                                &pub_obj_pose_,
                                &pub_obj_poses_,
                                &pub_obj_moveit_,
                                visual_tools_,
                                action_left_,
                                action_right_,
                                &blocks_surfaces_,
                                false);
        }
        else if (actionName == "set_table_height") // set table height
        {
          blocks_surfaces_.front().size_z_ = promptUserValue("give the table height");
          blocks_surfaces_.front().pose_.position.z = floor_to_base_height_ + blocks_surfaces_.front().size_z_/2.0;
          pub_obj_moveit_.publish(blocks_surfaces_.front().collObj_);
        }
        else if (actionName == "t") //clean the scene
        {
          if (env_shown_)
          {
            cleanObjects(&blocks_surfaces_, false);
            cleanObjects(&blocks_);
            //visual_tools_->removeAllCollisionObjects();
            env_shown_ = false;
          }
          else
          {
            pub_obj_moveit_.publish(blocks_surfaces_.front().collObj_);
            env_shown_ = true;
          }
        }
        //moving the virtual object down
        else if ((actionName == "x") || (actionName == "down"))
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = block->pose_;
            pose.position.z -= 0.05;
            block->updatePose(pose);
            resetBlock(block);
          }
        }
        //move the virtual object left
        else if ((actionName == "s") || (actionName == "left"))
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = block->pose_;
            pose.position.y -= 0.05;
            block->updatePose(pose);
            resetBlock(block);
          }
        }
        //move the virtual object up
        else if ((actionName == "e") || (actionName == "up"))
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = block->pose_;
            pose.position.z += 0.05;
            block->updatePose(pose);
            resetBlock(block);
          }
        }
        //move the virtual object right
        else if ((actionName == "f") || (actionName == "right"))
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = block->pose_;
            pose.position.y += 0.05;
            block->updatePose(pose);
            resetBlock(block);
          }
        }
        //move the virtual object farther
        else if ((actionName == "c") || (actionName == "farther"))
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = blocks_[0].pose_;
            pose.position.x += 0.05;
            blocks_[0].updatePose(pose);
            resetBlock(block);
          }
        }
        //move the virtual object closer
        else if ((actionName == "r") || (actionName == "closer"))
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = blocks_[0].pose_;
            pose.position.x -= 0.05;
            blocks_[0].updatePose(pose);
            resetBlock(block);
          }
        }
        else if (actionName == "j") //key 'j', set the tolerance
          action->setTolerance(promptUserValue("Set the value: "));
        else if (actionName == "h") //key 'h'
          switchArm(action);
        else if (actionName == "m") //key 'm', move the head down
          action->poseHeadDown();
        else if (actionName == "k") //key 'k', move the head to zero
          action->poseHeadZero();
        else if (actionName == "stat") //key 'stat', print the statistics on grasps
        {
          ROS_INFO_STREAM("Successfully grasped objects at the following locations: ");
          for (std::vector <geometry_msgs::Pose>::iterator it = stat_poses_success_.begin(); it != stat_poses_success_.end(); ++it)
          {
            ROS_INFO_STREAM(" [" <<  it-> position.x << " " << it->position.y << " " << it->position.z << "] ["
                            << it->orientation.x << " " << it->orientation.y << " " << it->orientation.z << " " << it->orientation.w << "]");
          }
        }
        else if (actionName == "stat_evaluation") //key 'stat', print the statistics on grasps
        {
          evaluation_.printStat();
        }
      }

      if (actionName == "q")
        break;
    }
    return true;
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
