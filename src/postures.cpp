// MoveIt!
#include <moveit/move_group_interface/move_group.h>

#include "romeo_moveit_actions/postures.hpp"

Posture::Posture(const std::string robot_name, const std::string eef_name, const std::string group_name)
{
  //set joint for the head
  moveit::planning_interface::MoveGroup move_group_head("head");
  //get current
  move_group_head.getCurrentState()->copyJointGroupPositions(move_group_head.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_head.getName()), pose_head_down);
  if (pose_head_down.size() > 1)
    if (robot_name == "nao")
      pose_head_down[1] = 0.3;
    else
      pose_head_down[1] = 0.35;
  pose_head_zero.resize(pose_head_down.size(), 0.0);

  //get the arm joints
  moveit::planning_interface::MoveGroup move_group_plan(group_name);
  std::vector<double> pose_arm_left_init;
  move_group_plan.getCurrentState()->copyJointGroupPositions(
              move_group_plan.getCurrentState()->getRobotModel()->getJointModelGroup(
                  move_group_plan.getName()), pose_arm_left_init);

  //initialize the left arm
  pose_arm_left.resize(4);
  pose_arm_left[0] = pose_arm_left_init;
  for (int i=1; i<pose_arm_left.size(); ++i)
    pose_arm_left[i].resize(pose_arm_left[0].size(), 0.0);
  if (pose_arm_left[0].size() >= 5)
  {
    pose_arm_left[0].assign(pose_arm_left[0].size(), 0.0);

    pose_arm_left[1][0] = 1.74;
    pose_arm_left[1][1] = 0.75;
    pose_arm_left[1][2] = -2.08;
    pose_arm_left[1][3] = -1.15;
    pose_arm_left[1][4] = -0.43;
    if (pose_arm_left[0].size() >= 6)
      pose_arm_left[1][5] = 0.17;

    pose_arm_left[2][0] = 1.0799225568771362;
    pose_arm_left[2][1] = 0.6565437912940979;
    pose_arm_left[2][2] = -0.8390874862670898;
    pose_arm_left[2][3] = -0.607456386089325;
    pose_arm_left[2][4] = -0.6672816872596741;
    if (pose_arm_left[0].size() >= 6)
      pose_arm_left[2][5] = 0.02454369328916073;

    pose_arm_left[3][0] = 1.872990608215332;
    pose_arm_left[3][1] = 0.5553010702133179;
    pose_arm_left[3][2] = -1.9895731210708618;
    pose_arm_left[3][3] = -1.052310824394226;
    pose_arm_left[3][4] = -0.6703495979309082;
    if (pose_arm_left[0].size() >= 6)
      pose_arm_left[3][5] = 0.02147573232650757;
  }

  //initialize the right arm
  pose_arm_right = std::vector< std::vector<double> >(pose_arm_left);
  for (int i=0; i<pose_arm_right.size(); ++i)
  {
    if (pose_arm_right[i].size() >= 3)
    {
      pose_arm_right[i][1] *= -1;
      pose_arm_right[i][2] *= -1;
    }
    if (pose_arm_right[i].size() >= 5)
    {
      pose_arm_right[i][3] *= -1;
      pose_arm_right[i][4] *= -1;
    }
  }

  //get the current set of joint values for the eef
  std::vector<double> pose_hand;
  moveit::planning_interface::MoveGroup move_group_eef(eef_name);
  move_group_eef.getCurrentState()->copyJointGroupPositions(move_group_eef.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_eef.getName()), pose_hand);
  pose_hand_close.resize(pose_hand.size(), 0.0);
  pose_hand_open.resize(pose_hand.size(), 0.0);
  initHandPoseOpen(0.0);
  initHandPoseClose(0.8);

  /*double joints_arm_pregrasp[] = {0.8, 0.5, -0.62, -1.0, -0.96, -0.12}; //, 0.0, 0.80
  pose_arm_left_pregrasp = std::vector<double>(joints_arm_pregrasp, joints_arm_pregrasp + sizeof(joints_arm_pregrasp) / sizeof(double) );*/
}

void Posture::initHandPoseOpen(const double &value){
  pose_hand_open.assign(pose_hand_open.size(), value);
  if (pose_hand_open.size() > 0)
    pose_hand_open[0] = 0.0;
}

void Posture::initHandPoseClose(const double &value){
  pose_hand_close.assign(pose_hand_close.size(), value);
  if (pose_hand_close.size() > 0)
    pose_hand_close[0] = 0.0;
}

bool Posture::poseHeadZero(){
  goToPose("head", &pose_head_zero);
}

bool Posture::poseHeadDown(){
  goToPose("head", &pose_head_down);
}

bool Posture::poseHand(const std::string &end_eff, const std::string &group, const std::string &arm, std::vector<double> *pose_hand)
{
  //end-effector first
  goToPose(end_eff, &pose_hand_open);

  return goToPose(group, pose_hand);
}

bool Posture::poseHand(const std::string &end_eff, const std::string &group, const std::string &arm, const int &pose_id)
{
  //end-effector first
  goToPose(end_eff, &pose_hand_open);

  bool res = false;
  if (arm == "right")
  {
    if (pose_id < pose_arm_right.size())
      res = goToPose(group, &pose_arm_right[pose_id]);
  }
  else
  {
    if (pose_id < pose_arm_left.size())
      res = goToPose(group, &pose_arm_left[pose_id]);
  }

  return res;
}

bool Posture::poseHandOpen(const std::string &end_eff)
{
  goToPose(end_eff, &pose_hand_open);
}

bool Posture::poseHandClose(const std::string &end_eff)
{
  goToPose(end_eff, &pose_hand_close);
}

void Posture::getPose(const std::string group_name)
{
  moveit::planning_interface::MoveGroup move_group(group_name);

  std::vector<std::string> joint_names = move_group.getCurrentState()->getRobotModel()->getEndEffector(move_group.getName())->getJointModelNames();

  std::vector<double> joints_temp = move_group.getCurrentJointValues();
  std::cout << "Joint values of " << group_name << " ";
  for (int i=0; i< joints_temp.size(); ++i)
    std::cout << joint_names[i] << "=" << joints_temp[i] << " ";
  std::cout << std::endl;

  //get the current set of joint values for the group
  std::vector<double> joints;
  move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joints);

  std::cout << "Joint values of " << group_name << " ";
  for (int i=0; i< joints.size(); ++i)
    std::cout << joints[i] << " ";
  std::cout << std::endl;
}

bool Posture::goToPose(const std::string group_name, std::vector<double> *pose)
{
  moveit::planning_interface::MoveGroup move_group(group_name);

  //get the current set of joint values for the group
  std::vector<double> joints;
  move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joints);

  //plan to the new joint space goal and visualize the plan
  if (joints.size() == pose->size())
  {
    //move_group.setApproximateJointValueTarget(*pose);
    move_group.setJointValueTarget(*pose);

    moveit::planning_interface::MoveGroup::Plan plan;
    bool success = move_group.plan(plan);
    sleep(1.0);
    if (success)
    {
      ROS_INFO("Action with the move_group %s", success?"":"FAILED");
      move_group.move();
      sleep(1.0);

      /*std::vector<double> joints_temp = move_group.getCurrentJointValues();
      std::cout << "Joint values of " << group_name << " ";
      for (int i=0; i< joints_temp.size(); ++i)
        std::cout << joints_temp[i] << " ";
      std::cout << std::endl;*/
    }
    return success;
  }
  else
    ROS_INFO_STREAM("Input joint values have wrong size " << joints.size() << " == " << pose->size());
  return false;
}
