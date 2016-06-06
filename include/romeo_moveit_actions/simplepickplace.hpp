#ifndef SIMPLEACTIONS_HPP
#define SIMPLEACTIONS_HPP

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "metablock.hpp"
#include "action.hpp"
#include "objprocessing.hpp"

namespace moveit_simple_actions
{

class SimplePickPlace
{
public:
  SimplePickPlace(const std::string robot_name,
                  const double test_step,
                  const double x_min,
                  const double x_max,
                  const double y_min,
                  const double y_max,
                  const double z_min,
                  const double z_max,
                  const std::string left_arm_name,
                  const std::string right_arm_name,
                  const bool verbose);
  bool startRoutine();

  void switchArm(Action *action_now);

  void createObj(const MetaBlock &block);
  void resetBlock(MetaBlock *block);
  void getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg);
  void cleanObjects(std::vector<MetaBlock> *objects, const bool list_erase=true);
  moveit_msgs::CollisionObject wrapToCollisionObject(MetaBlock *block);
  void publishCollisionObject(MetaBlock *block, const geometry_msgs::Pose &pose);
  void publishCollisionObject(MetaBlock *block);

  //Evaluation in toolsEvaluation.hpp
  void testReach(const bool pickVsReach=true, bool test_poses_rnd=false);
  void testReachSingleHand(Action *action, ros::Publisher *pub_obj_poses, std::vector<MetaBlock> &blocks, const bool pickVsReach);

  int testReachWithGenSingleHand(Action *action,
                                 ros::Publisher *pub_obj_poses,
                                 const bool pickVsReach,
                                 const int attempts_nbr,
                                 const double planning_time,
                                 geometry_msgs::PoseArray &msg_poses_validated);

  geometry_msgs::PoseArray generatePosesRnd(const int poses_nbr, std::vector<MetaBlock> &blocks);
  geometry_msgs::PoseArray generatePosesGrid(std::vector<MetaBlock> &blocks);

  // A shared node handle
  ros::NodeHandle nh_, nh_priv_;

  std::string robot_name_;
  double test_step_;
  const bool verbose_;
  const bool saveStat_;
  std::string base_frame_;
  double block_size_x;
  double block_size_y;
  double floor_to_base_height_;

  std::string left_arm_name_;
  std::string right_arm_name_;
  bool test_mesh_;

  //Posture posture;
  Objprocessing objproc;

  bool env_shown_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;
  std::string support_surface_name_;

  Action *action_left_, *action_right_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  std::vector<MetaBlock> blocks_;
  std::vector<MetaBlock> blocks_test_;
  std::vector<MetaBlock> blocks_surfaces_;

  ros::Subscriber sub_obj_coll_;
  ros::Publisher pub_obj_poses_, pub_obj_pose_;
  geometry_msgs::PoseStamped msg_obj_pose_;
  geometry_msgs::PoseArray msg_obj_poses_;

  geometry_msgs::Pose pose_default_, pose_default_r_, pose_zero_;

  std::vector <geometry_msgs::Pose> stat_poses_success_;

  ros::Publisher pub_obj_moveit_;

};
}

#endif // SIMPLEACTIONS_HPP
