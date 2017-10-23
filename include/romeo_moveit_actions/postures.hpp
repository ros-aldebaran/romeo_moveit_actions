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
//! @brief Class for the robot's posture processing.
class Posture
{
public:
  //! @brief constructor
  Posture(const std::string robot_name,
          const std::string eef_name,
          const std::string group_name);

  //! @brief initialize the hand pose
  void initHandPose(const double &value, const int &pose);

  //! @brief define the head down pose
  bool poseHeadDown();

  //! @brief define the head zero pose
  bool poseHeadZero();

  //! @brief define the hand open pose
  bool poseHandOpen(const std::string &end_eff);

  //! @brief define the hand close pose
  bool poseHandClose(const std::string &end_eff);

  //! @brief define the hand pose
  bool poseHand(const std::string &end_eff,
                const std::string &group,
                const int &pose_id);

private:
  //! @brief go to the pose
  bool goToPose(const std::string group_name,
                std::vector<double> *pose);

  /** pre-defined head down pose */
  std::vector<double> pose_head_down_;

  /** pre-defined hand zero pose */
  std::vector<double> pose_head_zero_;

  /** pre-defined hand pose */
  std::vector< std::vector<double> > pose_hand_;

  /** pre-defined arm pose */
  std::vector< std::vector<double> > pose_arm_;
};
}
#endif // POSTURES_HPP
