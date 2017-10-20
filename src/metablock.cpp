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

#include <Eigen/Eigen>
#include <stdlib.h>

#include "romeo_moveit_actions/metablock.hpp"
#include "romeo_moveit_actions/toolsForObject.hpp"

namespace moveit_simple_actions
{

MetaBlock::MetaBlock(const std::string name,
                     const geometry_msgs::Pose pose,
                     const uint &shapeType,
                     const float &size_x,
                     const float &size_y,
                     const float &size_z,
                     ros::Time timestamp,
                     std::string base_frame):
    name_(name),
    pose_(pose),
    size_x_(size_x),
    size_y_(size_y),
    size_z_(size_z),
    timestamp_(timestamp),
    base_frame_(base_frame)
{
  if (pose_.position.y < 0)
    pose_.orientation.y *= -1;

  goal_pose_ = pose_;
  if (pose_.position.y < 0)
    goal_pose_.position.y -= 0.2;
  else
    goal_pose_.position.y += 0.2;

  //setshape
  sprimitive solidPrimitive;
  if (shapeType == sprimitive::CYLINDER)
  {
    solidPrimitive.type = sprimitive::CYLINDER;
    solidPrimitive.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<sprimitive::CYLINDER>::value);
    solidPrimitive.dimensions[sprimitive::CYLINDER_HEIGHT] = size_y;
    solidPrimitive.dimensions[sprimitive::CYLINDER_RADIUS] = size_x;
  }
  else
  {
    solidPrimitive.type = sprimitive::BOX;
    solidPrimitive.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<sprimitive::BOX>::value);
    solidPrimitive.dimensions[sprimitive::BOX_X] = size_x;
    solidPrimitive.dimensions[sprimitive::BOX_Y] = size_y;
    solidPrimitive.dimensions[sprimitive::BOX_Z] = size_z;
  }
  shape_ = solidPrimitive;

  //create collision object
  collObj_.header.stamp = ros::Time::now();
  collObj_.header.frame_id = base_frame_;
  collObj_.id = name_;
  collObj_.operation = mcollobj::ADD;
  collObj_.primitives.resize(1);
  if (shape_.dimensions.size() > 0)
    collObj_.primitives[0] = shape_;
  collObj_.primitive_poses.resize(1);
  collObj_.primitive_poses[0] = pose_;
}

MetaBlock::MetaBlock(const std::string name,
                     const geometry_msgs::Pose pose,
                     const shape_msgs::Mesh mesh,
                     const object_recognition_msgs::ObjectType type,
                     ros::Time timestamp,
                     std::string base_frame):
    name_(name),
    pose_(pose),
    size_x_(0.03),
    size_y_(0.115),
    size_z_(0.01),
    timestamp_(timestamp),
    base_frame_(base_frame)
{
  pose_.orientation.x = -1.0;
  pose_.orientation.y = 0.0;
  pose_.orientation.z = 0.0;
  pose_.orientation.w = 0.0;

  goal_pose_ = pose_;
  goal_pose_.position.x = 0.4;
  goal_pose_.position.y = 0.3;
  goal_pose_.position.z = -0.05;
  if (pose_.position.y < 0)
    goal_pose_.position.y *= -1;

  mesh_ = mesh;
  type_ = type;

  //create collision object
  collObj_.header.stamp = ros::Time::now();
  collObj_.header.frame_id = base_frame_;
  collObj_.id = name_;
  collObj_.operation = mcollobj::ADD;
  collObj_.meshes.resize(1);
  collObj_.meshes[0] = mesh;
  collObj_.mesh_poses.resize(1);
  collObj_.mesh_poses[0] = pose_;
}

void MetaBlock::updatePose(const geometry_msgs::Pose &pose)
{
  pose_ = pose;
  if (collObj_.primitive_poses.size() > 0)
    collObj_.primitive_poses[0] = pose;
}

void MetaBlock::updatePoseVis(const geometry_msgs::Pose &pose)
{
  if (collObj_.primitive_poses.size() > 0)
    collObj_.primitive_poses[0] = pose;
}

void MetaBlock::removeBlock(mscene *current_scene)
{
  // Remove/Add collision object
  std::vector<std::string> objects_id;
  objects_id.resize(1);
  objects_id[0] = name_;
  current_scene->removeCollisionObjects(objects_id);
}

void MetaBlock::publishBlock(mscene *current_scene)
{
  std::vector<moveit_msgs::CollisionObject> coll_objects;
  coll_objects.push_back(collObj_);
  current_scene->addCollisionObjects(coll_objects);
  sleep(0.6);
}

void MetaBlock::updatePose(mscene *current_scene,
                           const geometry_msgs::Pose &pose)
{
  updatePose(pose);

  //pub_obj_moveit->publish(collObj_);
  std::vector<moveit_msgs::CollisionObject> coll_objects;
  coll_objects.push_back(collObj_);
  current_scene->addCollisionObjects(coll_objects);
  sleep(0.5);
}

tf::Stamped<tf::Pose> MetaBlock::getTransform(tf::TransformListener *listener,
                                              const std::string &frame)
{
  tf::Stamped<tf::Pose> pose_to_robot;

  tf::Stamped<tf::Pose> tf_obj(
        tf::Pose(
          tf::Quaternion(pose_.orientation.x,
                         pose_.orientation.y,
                         pose_.orientation.z,
                         pose_.orientation.w),
          tf::Vector3(pose_.position.x,
                      pose_.position.y,
                      pose_.position.z)),
      ros::Time(0), base_frame_);

  //transform the pose
  try
  {
    listener->transformPose(frame, tf_obj, pose_to_robot);
  }
  catch (tf::TransformException ex)
  {
   ROS_ERROR("%s",ex.what());
  }

/*  ROS_INFO_STREAM("The pose to the object "
                 << " " << pose_to_robot.getOrigin().x()
                 << " " << pose_to_robot.getOrigin().y()
                 << " " << pose_to_robot.getOrigin().z()
                 );*/
  return pose_to_robot;
}

geometry_msgs::PoseStamped MetaBlock::getTransformed(tf::TransformListener *listener,
                                                     const std::string &frame)
{
  geometry_msgs::PoseStamped pose_to_robot;

  geometry_msgs::PoseStamped pose_obj;
  pose_obj.header.stamp = ros::Time(0);
  pose_obj.header.frame_id = base_frame_;
  pose_obj.pose = pose_;

  //the the pose
  try
  {
    listener->transformPose(frame, pose_obj, pose_to_robot);
  }
  catch (tf::TransformException ex)
  {
   ROS_ERROR("%s",ex.what());
  }

/*  ROS_INFO_STREAM("The pose to the object "
                 << " " << pose_to_robot.getOrigin().x()
                 << " " << pose_to_robot.getOrigin().y()
                 << " " << pose_to_robot.getOrigin().z()
                 );*/
  return pose_to_robot;
}
}
