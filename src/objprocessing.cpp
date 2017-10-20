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

#include <geometry_msgs/PoseStamped.h>

#include <moveit_msgs/CollisionObject.h>

#include "romeo_moveit_actions/objprocessing.hpp"
#include "romeo_moveit_actions/toolsForAction.hpp"
#include "romeo_moveit_actions/toolsForObject.hpp"

namespace moveit_simple_actions
{

ObjProcessor::ObjProcessor(ros::NodeHandle *nh,
                           Evaluation *evaluation):
  nh_(nh),
  evaluation_(evaluation),
  OBJECT_RECOGNITION_ACTION("/recognize_objects"),
  target_frame_("odom"),
  object_topic_("/recognized_object_array"),
  found_srv_obj_info_(false)
{
  std::string mesh_srv_name("get_object_info");
  found_srv_obj_info_ = true;
  found_obj_reco_client_ = false;

  ros::Time start_time = ros::Time::now();
  while (!ros::service::waitForService(mesh_srv_name, ros::Duration(2.0)))
  {
    ROS_INFO("Waiting for %s service to come up", mesh_srv_name.c_str());
    if (!nh_->ok() || ros::Time::now() - start_time >= ros::Duration(5.0))
    {
      found_srv_obj_info_ = false;
      break;
    }
  }

  if (found_srv_obj_info_)
  {
    get_model_mesh_srv_ = nh_->serviceClient<GetObjInfo>
      (mesh_srv_name, false);
  }

  object_sub_ = nh_->subscribe<RecognizedObjArray>(object_topic_,
                                                   1,
                                                   &ObjProcessor::getRecognizedObjects,
                                                   this);

  msg_obj_poses_.header.frame_id = target_frame_;

  pub_obj_poses_ = nh_->advertise<geometry_msgs::PoseArray>("/obj_poses", 1);
}

bool ObjProcessor::triggerObjectDetection()
{
  if(!obj_reco_client_)
  {
    obj_reco_client_.reset(new ObjRecoActionClient(OBJECT_RECOGNITION_ACTION, false));
  }
  if (!found_obj_reco_client_)
  {
    try
    {
      ROS_DEBUG("Waiting for the Object recognition client");
      waitForAction(obj_reco_client_,
                    *nh_,
                    ros::Duration(5, 0),
                    OBJECT_RECOGNITION_ACTION);
      ROS_DEBUG("Object recognition client is ready");
      found_obj_reco_client_ = true;
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Object recognition action: %s", ex.what());
      return false;
    }
  }

  if (found_obj_reco_client_)
  {
    object_recognition_msgs::ObjectRecognitionGoal goal;
    obj_reco_client_->sendGoal(goal);
    if (!obj_reco_client_->waitForResult())
    {
      ROS_DEBUG_STREAM("Object recognition client returned early");
      return false;
    }
    if (obj_reco_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_DEBUG_STREAM("Fail: "
                       << obj_reco_client_->getState().toString() << ":"
                       << obj_reco_client_->getState().getText());
      return true;
    }
  }
//ROS_INFO_STREAM("triggerObjectDetection finished");
  return false;
}

bool ObjProcessor::getMeshFromDB(GetObjInfo &obj_info)
{
  if (!found_srv_obj_info_)
    return false;

  if ( !get_model_mesh_srv_.call(obj_info) )
  {
    ROS_ERROR("Get model mesh service service call failed altogether");
    return false;
  }
  return true;
}

void ObjProcessor::getRecognizedObjects(const RecognizedObjArray::ConstPtr& msg)
{
  if (msg->objects.empty())
    return;

  std::vector<moveit_msgs::CollisionObject> coll_objects;

  float block_size = 0.03f;
  try
  {
    int obj_id = 0;
    RecognizedObjArray::_objects_type::const_iterator it;
    for (it = msg->objects.begin(); it != msg->objects.end(); ++it)
    {
      geometry_msgs::PoseStamped msg_obj_cam_, msg_obj_pose;

      msg_obj_cam_.header = msg->header;
      msg_obj_cam_.header.stamp = ros::Time(0);
      msg_obj_cam_.pose = it->pose.pose.pose;
      listener_.transformPose(target_frame_, msg_obj_cam_, msg_obj_pose);

      if (!evaluation_->inWorkSpace(msg_obj_pose.pose, 0, 0, 1))
        continue;

      ROS_INFO_STREAM("xyz: " << msg_obj_pose.pose.position.x << " "
                      << msg_obj_pose.pose.position.y << " "
                      << msg_obj_pose.pose.position.z);

      GetObjInfo obj_info;
      obj_info.request.type = it->type;

      //check if exists
      std::stringstream obj_id_str;
      obj_id_str << obj_id << "_" << it->type.key;

      int idx = findObj(&blocks_, obj_id_str.str());
      //msg_obj_pose.pose.position.z -= 0.01;
      if (idx == -1) //if not found, create a new one
      {
        if (getMeshFromDB(obj_info))
        {
          blocks_.push_back(MetaBlock(obj_id_str.str(),
                                      msg_obj_pose.pose,
                                      obj_info.response.information.ground_truth_mesh,
                                      it->type,
                                      msg->header.stamp,
                                      target_frame_));
          msg_obj_poses_.poses.push_back(msg_obj_pose.pose);
        }
        else
        {
          blocks_.push_back(MetaBlock(obj_id_str.str(),
                                      msg_obj_pose.pose,
                                      sprimitive::CYLINDER,
                                      block_size,
                                      block_size*3.0f,
                                      0.0f,
                                      msg->header.stamp,
                                      target_frame_));
          msg_obj_poses_.poses.push_back(msg_obj_pose.pose);
        }

        coll_objects.push_back(blocks_.back().collObj_);
        //ROS_INFO_STREAM("adding an object " << obj_id_str.str());
      }
      else if ((idx >= 0) && (idx < blocks_.size()))
      {
        blocks_.at(idx).updatePose(msg_obj_pose.pose);
        if (idx >= msg_obj_poses_.poses.size())
          msg_obj_poses_.poses.resize(idx+1);
        msg_obj_poses_.poses[idx] = msg_obj_pose.pose;

        coll_objects.push_back(blocks_.at(idx).collObj_);
        //ROS_INFO_STREAM("updating the object " << obj_id_str.str());
      }
      ++obj_id;
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  if (blocks_.size() == 0)
    return;

  // publish all objects as collision blocks
  if (coll_objects.size() > 0)
    current_scene_.addCollisionObjects(coll_objects);
  //publishAllCollObj(blocks_);

  ROS_INFO_STREAM("The detected object: "
                  << blocks_.front().pose_.position.x << " "
                  << blocks_.front().pose_.position.y << " "
                  << blocks_.front().pose_.position.z
                  << " out of " << coll_objects.size() );

  //ROS_INFO_STREAM("blocks_.size() " << blocks_.size() << " " << msg_obj_poses_.poses.size());
}

void ObjProcessor::publishAllCollObj(std::vector<MetaBlock> *blocks)
{
  if (blocks->empty())
    return;

  std::vector<moveit_msgs::CollisionObject> coll_objects;

  std::vector<MetaBlock>::iterator block=blocks->begin();
  for (; block != blocks->end(); ++block)
    coll_objects.push_back(block->collObj_);

  if (coll_objects.size() > 0)
    current_scene_.addCollisionObjects(coll_objects);
}

MetaBlock * ObjProcessor::getBlock(const int &id)
{
  MetaBlock * block;
  if (id < blocks_.size())
    block = &blocks_[id];
  return block;
}

void ObjProcessor::addBlock(const MetaBlock &block)
{
  blocks_.push_back(block);

  blocks_.back().publishBlock(&current_scene_);
}

//clean the object list based on the timestamp
void ObjProcessor::cleanObjects(const bool list_erase)
{
  std::vector<std::string> objects_list = getObjectsOldList(&blocks_);
  current_scene_.removeCollisionObjects(objects_list);

  //remove from the memory
  if (list_erase)
  {
    blocks_.clear();
    msg_obj_poses_.poses.clear(); //TOCHECK
    pub_obj_poses_.publish(msg_obj_poses_);
  }
}
}
