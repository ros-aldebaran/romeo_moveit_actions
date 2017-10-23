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

#ifndef TOOLS_H
#define TOOLS_H

#include <ros/ros.h>

namespace moveit_simple_actions
{

  void printTutorial(const std::string robot_name)
  {
    std::cout << "\n \n"
              << "************************************************\n"
              << "Tutorial for simple actions with Romeo, NAO, and Pepper.\n"
              << "You have chosen -" << robot_name << "- robot. \n\n"

              << "In your scene, you should see:\n"
              << "- a robot\n"
              << "- an object\n"
              << "- [optionally] a table. \n\n"

              << "The scenario is\n"
              << "1) Try to grasp an object (for Romeo only) by pressing -g- + Enter.\n"
              << "2) If the grasp is successful, place the object by typing -place-\n"
              << "   If not, try an approximative grasp by typing -u- + Enter.\n"
              << "3) Try to move the object right/left/top/down/farther/closer\n"
              << " by pressing -s- -f- -e- -x- -r- -c- keys and grasp again.\n"
              << "4) Try to grasp with another hand.\n"
              << "************************************************\n"
              << std::endl;
  }

  void printAllActions()
  {
    ROS_INFO_STREAM("Possible actions are: \n"
                    << " g - pick an object, \n"
                    << " p - place the object, \n"

                    << " u - reach and grasp, \n"
                    << " reachtop - reach the object from top, \n"

                    //<< " plan - plan grasping the object, \n"
                    << " a - plan all possible grasps, \n"
                    //<< " execute - execute the planned action, \n"

                    << " i - go to init pose (i1, i2, i3, i0), \n"
                    << " get_pose - show the pre-grasp pose, \n"
                    << " hand_open - open hand, \n"
                    << " hand_close - close hand, \n"
                    << " look_down - move the head to look down, \n"
                    << " look_zero - move the head to the zero pose, \n"

                    << " d - detect objects, \n"
                    << " lc - add a cylinder on the left, \n"
                    << " rc - add a cylinder on the right, \n"
                    << " table - add/remove the table/scene, \n"
                    << " set_table_height - set the table height, \n"
                    //<< " compute_distance - compute the distance to the object, \n"
                    << " next_obj - process the next object, \n"
                    << " e/s/f/x/r/c - move the object top/left/right/down/closer/farther \n"

                    << " switcharm - switch the active arm, \n"

                    << " test_pick - test the goal space for picking, \n"
                    << " test_reach - test the goal space for reaching, \n"
                    //<< " stat - print statistics on successfull grasps, \n"

                    << " q - exit, \n"
                    << " So, what do you choose?"
                    );
  }

  int promptUserAction()
  {
    printAllActions();
    char ascii;
    std::cin >> ascii;
    int in = (int) ascii;
    return in;
  }

  std::string promptUserQuestionString()
  {
    printAllActions();
    std::string input;
    std::cin >> input;
    return input;
  }

  bool promptUserQuestion(const std::string command)
  {
    ROS_INFO_STREAM_NAMED("pick_place",command);
    char input;
    std::cin >> input;
    if( input == 'n' ) // used for yes/no
      return false;

    return true;
  }

  double promptUserValue(const std::string command)
  {
    ROS_INFO_STREAM_NAMED("pick_place",command);
    double input;
    std::cin >> input;
    return input;
  }

  double promptUserInt(const std::string command)
  {
    ROS_INFO_STREAM_NAMED("pick_place",command);
    double input;
    std::cin >> input;
    return input;
  }

  bool promptUser()
  {
    ROS_INFO_STREAM_NAMED("pick_place","Retry? (y/n)");
    char input; // used for prompting yes/no
    std::cin >> input;
    if( input == 'n' )
      return false;

    return true;
  }
}
#endif // TOOLS_H
