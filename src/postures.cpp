// MoveIt!
#include <moveit/move_group_interface/move_group.h>

#include <postures.hpp>

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

  //set of joint for arms
  moveit::planning_interface::MoveGroup move_group_plan(group_name);
  //get current
  move_group_plan.getCurrentState()->copyJointGroupPositions(move_group_plan.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_plan.getName()), pose_arm_left_init);

ROS_INFO_STREAM("pose_arm_left_init.size() "<< pose_arm_left_init.size());

  if ((robot_name == "romeo") && (pose_arm_left_init.size() >= 6))
  {
    pose_arm_left_init[0] = 1.74;
    pose_arm_left_init[1] = 0.75;
    pose_arm_left_init[2] = -2.08;
    pose_arm_left_init[3] = -1.15;
    pose_arm_left_init[4] = -0.43;
    pose_arm_left_init[5] = 0.17;
    if (pose_arm_left_init.size() >= 7)
      pose_arm_left_init[6] = 0.0;
  }
  else if (pose_arm_left_init.size() >= 3)
  {
    pose_arm_left_init[0] = 1.7;
    pose_arm_left_init[1] = 0.3;
    pose_arm_left_init[2] = -1.3;
    //pose_arm_left_init[3] = -0.1;
  }

  pose_arm_right_init.resize(pose_arm_left_init.size(), 0.0);
  for (int i=0; i<pose_arm_left_init.size(); ++i)
    pose_arm_right_init[i] = pose_arm_left_init[i];
  if (pose_arm_right_init.size() >= 3)
  {
    pose_arm_right_init[1] *= -1;
    pose_arm_right_init[2] *= -1;

    if ((robot_name == "romeo") && (pose_arm_left_init.size() >= 6))
    {
      pose_arm_right_init[3] *= -1;
      pose_arm_right_init[4] *= -1;
    }
  }

  std::cout << "------------- pose_arm_left_init";
  for (int i=0; i<pose_arm_left_init.size(); ++i)
    std::cout << pose_arm_left_init[i] << " " << pose_arm_right_init[i] << std::endl;

  pose_arm_zero.resize(pose_arm_left_init.size(), 0.0);

  //get the current set of joint values for the eef
  std::vector<double> pose_hand;
  moveit::planning_interface::MoveGroup move_group_eef(eef_name);
  move_group_eef.getCurrentState()->copyJointGroupPositions(move_group_eef.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_eef.getName()), pose_hand);
  pose_hand_close.resize(pose_hand.size(), 0.0); //0.0
  pose_hand_open.resize(pose_hand.size(), 0.8); //0.8
  if (pose_hand_open.size() > 0)
    pose_hand_open[0] = 0.0;
  if (pose_hand_close.size() > 0)
    pose_hand_close[0] = 0.0;

  /*double joints_arm_pregrasp[] = {0.8, 0.5, -0.62, -1.0, -0.96, -0.12}; //, 0.0, 0.80
  pose_arm_left_pregrasp = std::vector<double>(joints_arm_pregrasp, joints_arm_pregrasp + sizeof(joints_arm_pregrasp) / sizeof(double) );*/
}

bool Posture::poseHeadZero(){
  goToPose("head", &pose_head_zero);
}

bool Posture::poseHeadDown(){
  goToPose("head", &pose_head_down);
}

bool Posture::poseHandZero(const std::string end_eff, const std::string group)
{
  goToPose(end_eff, &pose_hand_open);
  return goToPose(group, &pose_arm_zero);
}

bool Posture::poseHandInit(const std::string &end_eff, const std::string &group, const std::string &arm)
{
  //end-effector first
  goToPose(end_eff, &pose_hand_open);

  bool success = false;
  if (arm == "right")
    success = goToPose(group, &pose_arm_right_init);
  else
    success = goToPose(group, &pose_arm_left_init);
  return success;
}

bool Posture::poseHand(const std::string &end_eff, const std::string &group, const std::string &arm, std::vector<double> *pose_hand)
{
  //end-effector first
  goToPose(end_eff, &pose_hand_open);

  return goToPose(group, pose_hand);
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
