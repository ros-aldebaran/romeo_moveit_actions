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

#ifndef METABLOCK_H
#define METABLOCK_H

// ROS
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/CollisionObject.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>

typedef shape_msgs::SolidPrimitive sprimitive;
typedef moveit_msgs::CollisionObject mcollobj;
typedef moveit::planning_interface::PlanningSceneInterface mscene;

namespace moveit_simple_actions
{

//! @brief Class for working with collision objects.
class MetaBlock
{
public:
  //! @brief shape-based constructor
  MetaBlock(const std::string name,
            const geometry_msgs::Pose pose,
            const uint &shapeType,
            const float &size_x,
            const float &size_y,
            const float &size_z,
            ros::Time timestamp=ros::Time::now(),
            std::string base_frame="base_link");

  //! @brief mesh-based constructor
  MetaBlock(const std::string name,
            const geometry_msgs::Pose pose,
            const shape_msgs::Mesh mesh,
            const object_recognition_msgs::ObjectType type,
            ros::Time timestamp=ros::Time::now(),
            std::string base_frame="odom");

  //! @brief update the object pose
  void updatePose(const geometry_msgs::Pose &pose);

  //! @brief update the object's pose visually only
  void updatePoseVis(const geometry_msgs::Pose &pose);

  //! @brief update the pose and publish
  void updatePose(mscene *current_scene,
                  const geometry_msgs::Pose &pose);

  //! @brief remove the collision object
  void removeBlock(mscene *current_scene);

  //! @brief publish the collision object
  void publishBlock(mscene *current_scene);

  //! @brief get the transform to base_link
  tf::Stamped<tf::Pose> getTransform(tf::TransformListener *listener,
                                     const std::string &frame);

  //! @brief get the transform to base_link
  geometry_msgs::PoseStamped getTransformed(tf::TransformListener *listener,
                                            const std::string &frame);

  /** object name */
  std::string name_;

  /** the current position */
  geometry_msgs::Pose pose_;

  /** the goal position */
  geometry_msgs::Pose goal_pose_;

  /** corresponding collision object */
  moveit_msgs::CollisionObject collObj_;

  /** x dimenssion */
  double size_x_;

  /** y dimenssion */
  double size_y_;

  /** z dimenssion */
  double size_z_;

  /** timestamp of creation */
  ros::Time timestamp_;

  /** the base frame */
  std::string base_frame_;

  /** corresponding object type in DB */
  object_recognition_msgs::ObjectType type_;

protected:
  /** the object's shape */
  shape_msgs::SolidPrimitive shape_;

  /** the object's mesh */
  shape_msgs::Mesh mesh_;
};
}

#endif // METABLOCK_H
