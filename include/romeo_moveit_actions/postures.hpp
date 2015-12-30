#ifndef POSTURES_HPP
#define POSTURES_HPP

#include <stdlib.h>
#include <string>
#include <vector>

class Posture
{
public:
  Posture(const std::string robot_name, const std::string eef_name, const std::string group_name);

  void getPose(const std::string group_name);
  bool poseHeadDown();
  bool poseHeadZero();
  bool poseHandZero(const std::string end_eff, const std::string group);
  bool poseHandInit(const std::string &end_eff, const std::string &group, const std::string &arm);

  bool poseHandOpen(const std::string &end_eff);
  bool poseHandClose(const std::string &end_eff);

  bool poseHand(const std::string &end_eff, const std::string &group, const std::string &arm, std::vector<double> *pose_hand);

  std::vector<double> pose_head_down;
  std::vector<double> pose_head_zero;
  std::vector<double> pose_hand_open;
  std::vector<double> pose_hand_close;
  std::vector<double> pose_arm_zero;
  std::vector<double> pose_arm_right_init;
  std::vector<double> pose_arm_left_init;
  std::vector<double> pose_arm_left_pregrasp;

  private:
    bool goToPose(const std::string group_name, std::vector<double> *pose);
};

#endif // POSTURES_HPP
