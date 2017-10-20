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

  void swapPoses(geometry_msgs::Pose *pose1, geometry_msgs::Pose *pose2){
    geometry_msgs::Pose temp = *pose1;
    pose1 = pose2;
    *pose2 = temp;
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
      objproc_(&nh_priv_),
      evaluation_(verbose_, base_frame_)
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
    double table_height = -floor_to_base_height_ + (pose_default_.position.z-block_size_y_/2.0);
    double table_width = std::fabs(pose_default_r_.position.y*2.0) + block_size_x_/2.0;
    double table_depth = 0.35;
    geometry_msgs::Pose table_pose;
    setPose(&table_pose, pose_default_.position.x-block_size_x_/2.0 + table_depth/2.0, 0.0,floor_to_base_height_ + table_height/2.0);
    //if (robot_name_ == "romeo")
    {
      blocks_surfaces_.push_back(MetaBlock("table", table_pose, shape_msgs::SolidPrimitive::BOX, 0.35, table_width, table_height));
      support_surface_name_ = blocks_surfaces_.front().name_;
    }
    env_shown_ = false;

    // objects related initialization
    sub_obj_coll_ = nh_.subscribe<moveit_msgs::CollisionObject>("/collision_object", 10, &SimplePickPlace::getCollisionObjects, this);
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

    action_left_ = new Action(&nh_, visual_tools_, left_arm_name, robot_name_);
    action_right_ = new Action(&nh_, visual_tools_, right_arm_name, robot_name_);
    //ROS_INFO_STREAM("action_left_->grasp_data_.base_link_ = " << action_left_->getBaseLink());
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

    // Publish a table
    //if (robot_name_ == "romeo")
    {
      cleanObjects(&blocks_surfaces_, false);
      pub_obj_moveit_.publish(blocks_surfaces_.front().collObj_);
      env_shown_ = true;
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
    blocks_.push_back(block);
    msg_obj_pose_.pose = block.start_pose_;
    pub_obj_pose_.publish(msg_obj_pose_);
    pub_obj_moveit_.publish(blocks_.back().collObj_);
  }

  bool SimplePickPlace::checkObj(int &block_id)
  {
      if ((block_id >= 0) && (block_id < blocks_.size()))
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
    createObj(MetaBlock("Virtual1", pose_default_, shape_msgs::SolidPrimitive::CYLINDER, block_size_x_, block_size_y_, 0.0));

    while(ros::ok()) //main loop
    {
      std::string actionName = "";
      while(ros::ok()) //loop objects
      {
        if ((block_id == -1) && (blocks_.size() > 0)) //if there are some objects, take the first
          block_id = 0;
        else if (block_id >= blocks_.size()) //if the object does not exist
        {
          block_id = -1;
          cleanObjects(&blocks_, true);
        }

        if (block_id >= 0)
        {
          //update the object's pose
          msg_obj_pose_.pose = blocks_[block_id].start_pose_;
          pub_obj_pose_.publish(msg_obj_pose_);
          ROS_INFO_STREAM("The current active object is "
                                << blocks_[block_id].name_ << " out of " << blocks_.size());

          if (hand_id == 0)
          {
            if (blocks_[block_id].start_pose_.position.y > 0)
              action = action_left_;
            else
              action = action_right_;
          }
        }

        //ROS_INFO_STREAM("What do you want me to do ?");
        actionName = promptUserQuestionString();
        ROS_INFO_STREAM("Action chosen '" << actionName
                              << "' object_id=" << block_id
                              << " the arm active=" << action->arm_);

        // Pick -----------------------------------------------------
        if ((checkObj(block_id)) && (actionName == "g")) //key 'g' //pick the object with a grasp generator
        {
          bool success = action->pickAction(&blocks_[block_id], support_surface_name_);
          //if not succeded then try with another arm
          if(!success)
          {
            switchArm(action);
            success = action->pickAction(&blocks_[block_id], support_surface_name_);
          }

          if(success)
            stat_poses_success_.push_back(blocks_[block_id].start_pose_);
        }
        // Place --------------------------------------------------------
        else if ((checkObj(block_id)) && (actionName == "p"))  //key 'p' //place the object with a default function
        {
          if(action->placeAction(&blocks_[block_id], support_surface_name_))
          {
            // Swap this block's start and end pose so that we can then move them back to position
            swapPoses(&blocks_[block_id].start_pose_, &blocks_[block_id].goal_pose_);
            resetBlock(&blocks_[block_id]);
          }
        }
        // Return the hand to the zero pose ------------------------------
        else if (actionName == "i0") //key 'z' //move to zero pose
        {
          // Remove the attached object and the collision object
          if (checkObj(block_id))
            resetBlock(&blocks_[block_id]);
          action->poseHand(0);
        }
        //return the hand to the initial pose ------------------------------
        else if (actionName == "i") //key 'i' //move to init pose
        {
          // Remove the attached object and the collision object
          if (checkObj(block_id))
            resetBlock(&blocks_[block_id]);
          action->poseHand(1);
        }
        //return the hand to the initial pose ------------------------------
        else if (actionName == "i1") //key 'i1' //move to init pose
        {
          // Remove the attached object and the collision object
          if (checkObj(block_id))
            resetBlock(&blocks_[block_id]);
          action->poseHand(2);
        }
        //return the hand to the initial pose ------------------------------
        else if (actionName == "i2") //key 'i2' //move to init pose
        {
          // Remove the attached object and the collision object
          if (checkObj(block_id))
            resetBlock(&blocks_[block_id]);
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

          objproc_.triggerObjectDetection();
          // publish all objects as collision blocks
          if (blocks_.size() > 0)
          {
            for (std::vector<MetaBlock>::iterator block=blocks_.begin(); block != blocks_.end(); ++block)
            {
              //visual_tools_->cleanupCO(block->name_);
              moveit_msgs::CollisionObject msg = block->wrapToCollisionObject(objproc_.getMeshFromDB(block->type_));
              //if (!msg.meshes.empty())
                visual_tools_->processCollisionObjectMsg(msg);
              /*else
                pub_obj_moveit_.publish(block->collObj_);*/
            }
          }
        }
        //dd continuous object detection
        else if (actionName == "dd") //key 'dd' //detect objects
        {
          cleanObjects(&blocks_);

          ros::Time start_time = ros::Time::now();
          while ((blocks_.size() <= 0) && (ros::Time::now() - start_time < ros::Duration(15.0)))
          {
            ROS_INFO_STREAM("**** object detection is running, objects detected  " << blocks_.size() << " " << ros::Time::now());
            objproc_.triggerObjectDetection();
          }

          // publish all objects as collision blocks
          if (blocks_.size() > 0)
          {
            for (std::vector<MetaBlock>::iterator block=blocks_.begin(); block != blocks_.end(); ++block)
            {
              moveit_msgs::CollisionObject msg = block->wrapToCollisionObject(objproc_.getMeshFromDB(block->type_));
              if (!msg.primitive_poses.empty())
                visual_tools_->processCollisionObjectMsg(msg);
            }
          }
        }
        //------------ reaching all poses generated by moveit simple grasps
        else if ((checkObj(block_id))&& (actionName == "plan")) //key 'action' //plan movement
        {
          //TODO: do not remove an object but allow a collision to it
          visual_tools_->cleanupCO(blocks_[block_id].name_);
          action->graspPlan(&blocks_[block_id], support_surface_name_);
          resetBlock(&blocks_[block_id]);
        }
        else if ((checkObj(block_id)) && (actionName == "a")) //key 'a' //plan all movement
        {
          //TODO: do not remove an object but allow a collision to it
          visual_tools_->cleanupCO(blocks_[block_id].name_);
          action->graspPlanAllPossible(&blocks_[block_id], support_surface_name_);
          resetBlock(&blocks_[block_id]);
        }
        //------------ reaching the default pose
        else if ((checkObj(block_id)) && (actionName == "u")) //key 'u' //reach and grasp
        {
          //TODO: do not remove an object but allow a collision to it
          //visual_tools_->cleanupCO(blocks_[block_id].name_);
          geometry_msgs::Pose pose = blocks_[block_id].start_pose_;
          pose.position.y = 50;
          publishCollisionObject(&blocks_[block_id], pose);
          ros::Duration(0.2).sleep();

          float dist = action->reachGrasp(&blocks_[block_id], support_surface_name_);
          //if not succeded then try with another arm
          if(dist > 10)
          {
            switchArm(action);
            dist = action->reachGrasp(&blocks_[block_id], support_surface_name_);
          }

          publishCollisionObject(&blocks_[block_id]);
          if (dist < 0.2)
          {
            action->attachObject(blocks_[block_id].name_);
            action->poseHandClose();
          }
          //resetBlock(&blocks_[block_id]);
        }
        else if ((checkObj(block_id)) && (actionName == "pregrasp")) //key 'x' //reach the pregrasp pose
        {
          //TODO: do not remove an object but allow a collision to it
          visual_tools_->cleanupCO(blocks_[block_id].name_);
          action->reachPregrasp(blocks_[block_id].start_pose_, support_surface_name_);
          resetBlock(&blocks_[block_id]);
        }
        else if ((checkObj(block_id)) && (actionName == "reachtop")) //key 'y' //reach from top
        {
          //TODO: do not remove an object but allow a collision to it
          visual_tools_->cleanupCO(blocks_[block_id].name_);
          geometry_msgs::Pose pose = blocks_[block_id].start_pose_;
          pose.orientation.x = 0;
          pose.position.z += blocks_[block_id].size_x_*1.5; //0.12;
          action->reachAction(pose, support_surface_name_);
          resetBlock(&blocks_[block_id]);
        }
        else if ((checkObj(block_id)) && (actionName == "b")) //key 'b' //pick an object without a grasp generator
        {
          action->pickDefault(&blocks_[block_id], support_surface_name_);
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
          blocks_surfaces_.front().start_pose_.position.z = floor_to_base_height_ + blocks_surfaces_.front().size_z_/2.0;
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
        //moving the virtual objects
        else if ((actionName == "x") || (actionName == "down")) //down
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = blocks_[block_id].start_pose_;
            pose.position.z -= 0.05;
            blocks_[block_id].updatePose(pose);
            resetBlock(&blocks_[block_id]);
          }
        }
        else if ((actionName == "s") || (actionName == "left")) //left
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = blocks_[block_id].start_pose_;
            pose.position.y -= 0.05;
            blocks_[block_id].updatePose(pose);
            resetBlock(&blocks_[block_id]);
          }
        }
        else if ((actionName == "e") || (actionName == "up")) //up or 'e'
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = blocks_[block_id].start_pose_;
            pose.position.z += 0.05;
            blocks_[block_id].updatePose(pose);
            resetBlock(&blocks_[block_id]);
          }
        }
        else if ((actionName == "f") || (actionName == "right")) //right
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = blocks_[block_id].start_pose_;
            pose.position.y += 0.05;
            blocks_[block_id].updatePose(pose);
            resetBlock(&blocks_[block_id]);
          }
        }
        else if ((actionName == "c") || (actionName == "farther")) //farther
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = blocks_[0].start_pose_;
            pose.position.x += 0.05;
            blocks_[0].updatePose(pose);
            resetBlock(&blocks_[block_id]);
          }
        }
        else if ((actionName == "r") || (actionName == "closer")) //closer
        {
          if (blocks_.size() > 0)
          {
            geometry_msgs::Pose pose = blocks_[0].start_pose_;
            pose.position.x -= 0.05;
            blocks_[0].updatePose(pose);
            resetBlock(&blocks_[block_id]);
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
  void SimplePickPlace::cleanObjects(std::vector<MetaBlock> *objects, const bool list_erase)
  {
    ROS_INFO_STREAM("--- cleaning objects");
    ros::Time now = ros::Time::now() - ros::Duration(5);

    if (objects->size()>0)
    for (std::vector<MetaBlock>::iterator block=objects->begin(); block != objects->end(); ++block)
    {
      if (block->timestamp_ < now)
        resetBlock(&(*block));
    }

    //remove from the memory
    if (list_erase)
      objects->clear();

    msg_obj_poses_.poses.clear();
    pub_obj_poses_.publish(msg_obj_poses_);
  }

  void SimplePickPlace::resetBlock(MetaBlock *block)
  {
    // Remove attached object
    if(block->name_ != support_surface_name_)
    {
      action_left_->detachObject(block->name_);
      action_right_->detachObject(block->name_);
    }

    // Remove/Add collision object
    //visual_tools_->cleanupCO(block->name_);
    //visual_tools_->processCollisionObjectMsg(block->wrapToCollisionObject);
    pub_obj_moveit_.publish(block->collObj_);
  }

  void SimplePickPlace::publishCollisionObject(MetaBlock *block, const geometry_msgs::Pose &pose)
  {
      if (block->collObj_.primitive_poses.size() > 0)
      {
        moveit_msgs::CollisionObject collObj = block->collObj_;
        collObj.primitive_poses[0] = pose;
        pub_obj_moveit_.publish(collObj);
      }
  }
  void SimplePickPlace::publishCollisionObject(MetaBlock *block)
  {
     pub_obj_moveit_.publish(block->collObj_);
  }

  void SimplePickPlace::getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg)
  {
    //cleanObjects(&blocks_, false);

    try {
      if (msg->meshes.size() > 0)
      {
        geometry_msgs::Pose pose = msg->mesh_poses[0];
        if ((pose.position.x < x_max_) && (pose.position.x > x_min_)
            && (pose.position.z < z_max_) && (pose.position.z > z_min_))
        {
          //check if exists
          int idx = findObj(blocks_, msg->id);

          //pose.position.z += 0.0576;
          if (idx == -1) //if not found, create a new one
          {
            blocks_.push_back(MetaBlock(msg->id, pose, msg->meshes.front(), msg->type, msg->header.stamp));
            msg_obj_poses_.poses.push_back(pose);
          }
          else if ((idx >= 0) && (idx < blocks_.size()))
          {
            blocks_[idx].updatePose(pose);
            if (idx >= msg_obj_poses_.poses.size())
              msg_obj_poses_.poses.resize(idx+1);
            msg_obj_poses_.poses[idx] = pose;
          }
        }
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

} //namespace
