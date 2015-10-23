#ifndef METABLOCK_H
#define METABLOCK_H

// ROS
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <shape_tools/solid_primitive_dims.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>

class MetaBlock
{
public:
  MetaBlock(const std::string name,
            const double start_x,
            const double start_y,
            const double start_z,
            const double orien_x,
            const double orien_y,
            const double orien_z,
            const double orien_w,
            const uint shapeType,
            const double size,
            const double size_r);
  MetaBlock(const std::string name,
            const ros::Time timestamp,
            const geometry_msgs::Pose start_pose,
            const uint shapeType,
            const double size,
            const double size_l);
  MetaBlock(const std::string name,
            const ros::Time timestamp,
            const geometry_msgs::Pose start_pose,
            const shape_msgs::Mesh mesh,
            const object_recognition_msgs::ObjectType type);
  void updatePose(const geometry_msgs::Pose start_pose);
  void setRndPose();

  std::string name;
  double size;
  double size_l;
  ros::Time timestamp;
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;
  shape_msgs::SolidPrimitive shape;
  shape_msgs::Mesh mesh;
  object_recognition_msgs::ObjectType type;
};


#endif // METABLOCK_H
