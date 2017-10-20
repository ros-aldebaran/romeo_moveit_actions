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

namespace moveit_simple_actions
{

void setPose(geometry_msgs::Pose *pose, const double &x, const double &y, const double &z,
                            const double &ox, const double &oy, const double &oz, const double &ow)
{
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = z;
  pose->orientation.x = ox;
  pose->orientation.y = oy;
  pose->orientation.z = oz;
  pose->orientation.w = ow;
}

void setPose(geometry_msgs::Pose *pose, const double &x, const double &y, const double &z)
{
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = z;
}

int findObj(const std::vector<MetaBlock> &blocks, const std::string name)
{
  int idx = -1;
  for (int i=0; i<blocks.size(); ++i)
    if (blocks[i].name_ == name){
      idx = i;
      return idx;
    }
  return idx;
}

}

#endif // TOOLSOBJ_H
