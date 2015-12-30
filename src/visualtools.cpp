#include "visualtools.hpp"

// Shape tools
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

VisualTools::VisualTools(const std::string &base_frame):
  base_frame_(base_frame)
{
}

moveit_msgs::CollisionObject VisualTools::publishCollisionCylinder(const geometry_msgs::Pose& object_pose,
                                                   const std::string& object_name, double radius,
                                                   double height,
                                                   const rviz_visual_tools::colors& color)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = object_pose;

  return collision_obj;
}

moveit_msgs::CollisionObject VisualTools::publishCollisionBlock(const geometry_msgs::Pose& object_pose,
                                                   const std::string& object_name, double block_size,
                                                   const rviz_visual_tools::colors& color)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
        geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = block_size;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = block_size;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = block_size;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = object_pose;

  return collision_obj;
}
