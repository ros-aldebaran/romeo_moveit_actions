#ifndef VISUALTOOLS_HPP
#define VISUALTOOLS_HPP

#include <moveit_msgs/CollisionObject.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

class VisualTools
{
public:
  VisualTools(const std::string &base_frame);

  moveit_msgs::CollisionObject publishCollisionCylinder(const geometry_msgs::Pose& object_pose,
                                                   const std::string& object_name, double radius,
                                                   double height,
                                                   const rviz_visual_tools::colors& color=rviz_visual_tools::GREEN);

  moveit_msgs::CollisionObject publishCollisionBlock(const geometry_msgs::Pose& object_pose,
                                                   const std::string& object_name, double block_size,
                                                   const rviz_visual_tools::colors& color=rviz_visual_tools::GREEN);

  std::string base_frame_;
};

#endif // VISUALTOOLS_HPP
