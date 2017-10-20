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

#include "romeo_moveit_actions/toolsForObject.hpp"

namespace moveit_simple_actions
{

void setPose(geometry_msgs::Pose *pose,
             const double &x,
             const double &y,
             const double &z,
             const double &ox,
             const double &oy,
             const double &oz,
             const double &ow)
{
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = z;
  pose->orientation.x = ox;
  pose->orientation.y = oy;
  pose->orientation.z = oz;
  pose->orientation.w = ow;
}

void setPose(geometry_msgs::Pose *pose,
             const double &x,
             const double &y,
             const double &z)
{
  pose->position.x = x;
  pose->position.y = y;
  pose->position.z = z;
}

int findObj(std::vector<MetaBlock> *blocks,
            const std::string name)
{
  int idx = -1;
  for (int i=0; i<blocks->size(); ++i)
    if (blocks->at(i).name_ == name){
      idx = i;
      return idx;
    }
  return idx;
}

std::vector<std::string> getObjectsList(const std::vector<MetaBlock> &blocks)
{
  std::vector<std::string> res;
  res.resize(blocks.size());
  if (blocks.size() > 0)
  {
    std::vector<MetaBlock>::const_iterator it=blocks.begin();
    for (int i=0; it!=blocks.end(); ++it, ++i)
      res[i] = it->name_;
  }
  return res;
}

//clean the object list based on the timestamp
std::vector<std::string> getObjectsOldList(std::vector<MetaBlock> *objects)
{
  std::vector<std::string> objects_id;
  if (objects->size()>0)
  {
    ros::Time now = ros::Time::now() - ros::Duration(5);
    std::vector<MetaBlock>::iterator block=objects->begin();
    for (; block != objects->end(); ++block)
    {
      if (block->timestamp_ < now)
        objects_id.push_back(block->name_);
        //resetBlock(&(*block));
    }
  }
  return objects_id;
}

void swapPoses(geometry_msgs::Pose *pose1,
               geometry_msgs::Pose *pose2)
{
  geometry_msgs::Pose temp = *pose1;
  pose1 = pose2;
  *pose2 = temp;
}

}
