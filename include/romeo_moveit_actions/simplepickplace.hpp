#ifndef SIMPLEACTIONS_HPP
#define SIMPLEACTIONS_HPP

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <metablock.hpp>
#include <action.hpp>
#include <objprocessing.hpp>
#include <visualtools.hpp>

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

  void resetBlock(MetaBlock *block);
  void publishCollisionMetaBlock(MetaBlock *block);

  void setCollisionObjects();
  void getCollisionObjects(const moveit_msgs::CollisionObject::ConstPtr& msg);
  void cleanObjects();
  void removeObjects();
  moveit_msgs::CollisionObject wrapToCollisionObject(MetaBlock *block);

  // A shared node handle
  ros::NodeHandle nh_, nh_priv_;

  std::string robot_name_;
  double test_step_;
  const bool verbose_;
  const bool saveStat_;
  std::string base_frame;
  double block_size;
  double block_size_l;
  double floor_to_base_height;

  bool env_shown_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;
  std::string left_arm_name_;
  std::string right_arm_name_;
  bool test_mesh_;

  Action *action_left, *action_right;
  //Posture posture;
  Objprocessing objproc;

  VisualTools vtools_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  std::vector<MetaBlock> blocks;
  std::vector<MetaBlock> blocks_test;
  //std::vector<moveit_msgs::CollisionObject> obj_coll;

  ros::Subscriber sub_obj_coll;
  ros::Publisher pub_obj_poses, pub_obj_pose; //pub_obj_coll
  geometry_msgs::PoseStamped msg_obj_pose;
  geometry_msgs::PoseArray msg_obj_poses;
  //ros::ServiceClient planning_scene_service_;

  std::vector<geometry_msgs::Pose> poses_validated;
  std::vector<geometry_msgs::Pose> poses_failed;

  geometry_msgs::Pose pose_default, pose_default_r;

  std::vector <geometry_msgs::Pose> stat_poses_success;

  ros::Publisher pub_obj_moveit;
};
}

#endif // SIMPLEACTIONS_HPP
