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

namespace moveit_simple_actions
{

MetaBlock::MetaBlock(const std::string name,
          const geometry_msgs::Pose start_pose,
          const uint shapeType,
          const double size_x,
          const double size_y,
          const double size_z,
          ros::Time timestamp):
    name_(name),
    start_pose_(start_pose),
    size_x_(size_x),
    size_y_(size_y),
    size_z_(size_z),
    timestamp_(timestamp),
    base_frame_("odom")
{
  if (start_pose_.position.y < 0)
    start_pose_.orientation.y *= -1;

  goal_pose_ = start_pose_;
  goal_pose_.position.x = 0.5;
  goal_pose_.position.y = 0.25;
  if (start_pose_.position.y < 0)
    goal_pose_.position.y *= -1;

  //setshape
  shape_msgs::SolidPrimitive solidPrimitive;
  if (shapeType == shape_msgs::SolidPrimitive::CYLINDER)
  {
    solidPrimitive.type = shape_msgs::SolidPrimitive::CYLINDER;
    solidPrimitive.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    solidPrimitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = size_y;
    solidPrimitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = size_x;
  }
  else
  {
    solidPrimitive.type = shape_msgs::SolidPrimitive::BOX;
    solidPrimitive.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    solidPrimitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = size_x;
    solidPrimitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size_y;
    solidPrimitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = size_z;
  }
  shape_ = solidPrimitive;

  //create collision object
  collObj_.header.stamp = ros::Time::now();
  collObj_.header.frame_id = base_frame_;
  collObj_.id = name_;
  collObj_.operation = moveit_msgs::CollisionObject::ADD;
  collObj_.primitives.resize(1);
  if (shape_.dimensions.size() > 0)
    collObj_.primitives[0] = shape_;
  collObj_.primitive_poses.resize(1);
  collObj_.primitive_poses[0] = start_pose_;
}

MetaBlock::MetaBlock(const std::string name,
          const geometry_msgs::Pose start_pose,
          const shape_msgs::Mesh mesh,
          const object_recognition_msgs::ObjectType type,
          ros::Time timestamp):
    name_(name),
    start_pose_(start_pose),
    timestamp_(timestamp),
    base_frame_("odom")
{
  start_pose_.orientation.x = -1.0;
  start_pose_.orientation.y = 0.0;
  start_pose_.orientation.z = 0.0;
  start_pose_.orientation.w = 0.0;

  goal_pose_ = start_pose_;
  goal_pose_.position.x = 0.4;//0.47; //0.5;//
  goal_pose_.position.y = 0.3;
  goal_pose_.position.z = -0.05;
  goal_pose_.orientation.x = -1.0;
  goal_pose_.orientation.y = 0.0;
  goal_pose_.orientation.z = 0.0;
  goal_pose_.orientation.w = 0.0;
  if (start_pose_.position.y < 0)
    goal_pose_.position.y *= -1;

  mesh_ = mesh;
  type_ = type;
}

void MetaBlock::updatePose(const geometry_msgs::Pose &start_pose)
{
  start_pose_ = start_pose;
  if (collObj_.primitive_poses.size() > 0)
    collObj_.primitive_poses[0] = start_pose;
}

void MetaBlock::updatePoseVis(const geometry_msgs::Pose &start_pose)
{
  if (collObj_.primitive_poses.size() > 0)
    collObj_.primitive_poses[0] = start_pose;
}

moveit_msgs::CollisionObject MetaBlock::wrapToCollisionObject(const std::vector <shape_msgs::Mesh> &meshes)
{
  collObj_.header.stamp = ros::Time::now();

  if (!meshes.empty())
  {
    collObj_.meshes.push_back(meshes[0]);
    collObj_.mesh_poses.push_back(start_pose_);
    //ROS_INFO_STREAM("-- mesh found: msg_obj_collision.meshes.size()=" << msg_obj_collision.meshes.size());
  }
  else
    if (collObj_.primitive_poses.size() >0)
      collObj_.primitive_poses[0] = start_pose_;

  return collObj_;
}

}
