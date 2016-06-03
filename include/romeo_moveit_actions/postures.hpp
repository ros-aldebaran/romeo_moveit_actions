#ifndef POSTURES_HPP
#define POSTURES_HPP

#include <stdlib.h>
#include <string>
#include <vector>

class Posture
{
public:
  Posture(const std::string robot_name, const std::string eef_name, const std::string group_name);

  void initHandPoseOpen(const double &value);
  void initHandPoseClose(const double &value);

  void getPose(const std::string group_name);
  bool poseHeadDown();
  bool poseHeadZero();

  bool poseHandOpen(const std::string &end_eff);
  bool poseHandClose(const std::string &end_eff);

  bool poseHand(const std::string &end_eff, const std::string &group, const std::string &arm, std::vector<double> *pose_hand);
  bool poseHand(const std::string &end_eff, const std::string &group, const std::string &arm, const int &pose_id);

  //pre-defined head poses
  std::vector<double> pose_head_down;
  std::vector<double> pose_head_zero;

  //pre-defined hand poses
  std::vector<double> pose_hand_open;
  std::vector<double> pose_hand_close;

  //pre-defined arm poses
  std::vector<double> pose_arm_left_pregrasp;
  std::vector< std::vector<double> > pose_arm_left, pose_arm_right;

  private:
    bool goToPose(const std::string group_name, std::vector<double> *pose);
};

#endif // POSTURES_HPP
