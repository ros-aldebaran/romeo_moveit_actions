#ifndef METABLOCK_H
#define METABLOCK_H

// ROS
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/CollisionObject.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>

class MetaBlock
{
public:
  MetaBlock(const std::string name,
            const geometry_msgs::Pose start_pose,
            const uint shapeType,
            const double size_x,
            const double size_y,
            const double size_z,
            ros::Time timestamp=ros::Time::now());
  MetaBlock(const std::string name,
            const geometry_msgs::Pose start_pose,
            const shape_msgs::Mesh mesh,
            const object_recognition_msgs::ObjectType type,
            ros::Time timestamp=ros::Time::now());
  void updatePose(const geometry_msgs::Pose &start_pose);
  void updatePoseVis(const geometry_msgs::Pose &start_pose);
  void setRndPose();

  std::string name_;
  geometry_msgs::Pose start_pose_;
  double size_x_;
  double size_y_;
  double size_z_;
  ros::Time timestamp_;
  geometry_msgs::Pose goal_pose_;
  shape_msgs::SolidPrimitive shape_;
  shape_msgs::Mesh mesh_;
  object_recognition_msgs::ObjectType type_;
  moveit_msgs::CollisionObject collObj_;
};


#endif // METABLOCK_H
