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

#ifndef OBJPROCESSING_HPP
#define OBJPROCESSING_HPP

#include <boost/scoped_ptr.hpp>

// ROS
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseArray.h>

#include <actionlib/client/simple_action_client.h>

#include <object_recognition_msgs/GetObjectInformation.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/evaluation.hpp"

typedef object_recognition_msgs::RecognizedObjectArray RecognizedObjArray;
typedef actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> ObjRecoActionClient;
typedef object_recognition_msgs::GetObjectInformation GetObjInfo;

namespace moveit_simple_actions
{

//! @brief Class for processing objects recognized by ORK.
class ObjProcessor
{
public:
  //! @brief constructor
  ObjProcessor(ros::NodeHandle *nh,
               Evaluation *evaluation);

  //! @brief trigger object detection client
  bool triggerObjectDetection();

  //! @brief convert object recognition messages into collision objects
  void getRecognizedObjects(const RecognizedObjArray::ConstPtr& msg);

  //! @brief get amount of objects
  int getAmountOfBlocks()
  {
    return blocks_.size();
  }

  //! @brief get an object by id
  MetaBlock * getBlock(const int &id);

  //! @brief add a new object to the end
  void addBlock(const MetaBlock &block);

  //! @brief clean the list of objects based on the timestamp
  void cleanObjects(const bool list_erase=true);

  /** publisher of objects' poses */
  ros::Publisher pub_obj_poses_;

protected:
  //! @brief get the object's mesh from the DB
  bool getMeshFromDB(GetObjInfo &obj_info);

  //! @brief publish all collision objects in MoveIt
  void publishAllCollObj(std::vector<MetaBlock> *blocks);

  /** node handle */
  ros::NodeHandle *nh_;

  /** evaluation of reaching/grasping */
  Evaluation *evaluation_;

  /** object recognition action */
  const std::string OBJECT_RECOGNITION_ACTION;

  /** final object frame */
  std::string target_frame_;

  /** recognized object topic */
  std::string object_topic_;

  /** if found object info server */
  bool found_srv_obj_info_;

  /** Client for getting the mesh for a database object */
  ros::ServiceClient get_model_mesh_srv_;

  /** object recognition client */
  boost::scoped_ptr<ObjRecoActionClient> obj_reco_client_;

  /** if found object recognition server */
  bool found_obj_reco_client_;

  /** subscriber to object recognition topic */
  ros::Subscriber object_sub_;

  /** transform listener */
  tf::TransformListener listener_;

  /** current MoveIt scene */
  moveit::planning_interface::PlanningSceneInterface current_scene_;

  /** set of available objects */
  std::vector<MetaBlock> blocks_;

  //! @brief all objects positions
  geometry_msgs::PoseArray msg_obj_poses_;
};
}
#endif // OBJPROCESSING_HPP
