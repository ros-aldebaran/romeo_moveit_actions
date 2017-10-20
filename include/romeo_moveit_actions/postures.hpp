/*
 * Copyright 2016 SoftBank Robotics Europe
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef POSTURES_HPP
#define POSTURES_HPP

#include <stdlib.h>
#include <string>
#include <vector>

namespace moveit_simple_actions
{
class Posture
{
public:
  Posture(const std::string robot_name, const std::string eef_name, const std::string group_name);

  void initHandPoseOpen(const double &value);
  void initHandPoseClose(const double &value);

  bool poseHeadDown();
  bool poseHeadZero();

  bool poseHandOpen(const std::string &end_eff);
  bool poseHandClose(const std::string &end_eff);

  bool poseHand(const std::string &end_eff, const std::string &group, const std::string &arm, const int &pose_id);

private:
  bool goToPose(const std::string group_name, std::vector<double> *pose);

  //pre-defined head poses
  std::vector<double> pose_head_down_;
  std::vector<double> pose_head_zero_;

  //pre-defined hand poses
  std::vector<double> pose_hand_open_;
  std::vector<double> pose_hand_close_;

  //pre-defined arm poses
  std::vector< std::vector<double> > pose_arm_left_, pose_arm_right_;
};
}
#endif // POSTURES_HPP
