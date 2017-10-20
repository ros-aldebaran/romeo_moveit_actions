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

#ifndef TOOLSFOROBJECT_H
#define TOOLSFOROBJECT_H

#include <geometry_msgs/Pose.h>

#include "romeo_moveit_actions/metablock.hpp"

namespace moveit_simple_actions
{
//! @brief set the pose
void setPose(geometry_msgs::Pose *pose,
             const double &x,
             const double &y,
             const double &z,
             const double &ox,
             const double &oy,
             const double &oz,
             const double &ow);

//! @brief set the pose
void setPose(geometry_msgs::Pose *pose,
             const double &x,
             const double &y,
             const double &z);

//! @brief find the object by name
int findObj(std::vector<MetaBlock> *blocks,
            const std::string name);

//! @brief get a list of objects
std::vector<std::string> getObjectsList(const std::vector<MetaBlock> &blocks);

//! @brief get a list of old objects
std::vector<std::string> getObjectsOldList(std::vector<MetaBlock> *objects);

//! @brief swap poses
void swapPoses(geometry_msgs::Pose *pose1,
               geometry_msgs::Pose *pose2);
}

#endif // TOOLSFOROBJECT_H
