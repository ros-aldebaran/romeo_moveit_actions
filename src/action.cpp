#include <moveit_msgs/PickupGoal.h>

#include <action.hpp>

namespace moveit_simple_actions
{

Action::Action(ros::NodeHandle *nh_, moveit_visual_tools::MoveItVisualToolsPtr &visual_tools, const std::string arm, const std::string robot_name):
  verbose_(false),
  attempts_max_(3),
  planning_time_(30.0),
  //planner_id_("RRTConnectkConfigDefault"),
  tolerance_min_(0.01),
  arm(arm),
  end_eff(arm+"_hand"),
  plan_group(arm+"_arm"),
  posture(robot_name, end_eff, plan_group)
{
  /*ROS_INFO_STREAM("Arm: " << arm);
  ROS_INFO_STREAM("End Effector: " << end_eff);
  ROS_INFO_STREAM("Planning Group: " << plan_group);*/

  // Create MoveGroup for one of the planning groups
  move_group_.reset(new move_group_interface::MoveGroup(plan_group));
  move_group_->setGoalTolerance(0.1);
  move_group_->setPlanningTime(10.0);

  /*std::cout << "----- move_group_->getPlanningTime()=" << move_group_->getPlanningTime() << std::endl;
  std::cout << "----- move_group_->getPlanningFrame()=" << move_group_->getPlanningFrame() << std::endl; //= odom
  std::cout << "----- move_group_->getGoalPositionTolerance()= " << move_group_->getGoalPositionTolerance() << std::endl;
  std::cout << "----- move_group_->getGoalOrientationTolerance()= " << move_group_->getGoalOrientationTolerance() << std::endl;*/
  //move_group_->setPlanningTime(60.0); //30.0); //5
  //move_group_->setNumPlanningAttempts(10);
  /*move_group_->setGoalPositionTolerance(0.1); //0.0001
  move_group_->setGoalOrientationTolerance(0.1); //0.001*/

  // Load grasp generator
  if (!grasp_data_.loadRobotGraspData(*nh_, end_eff))
  {
    ROS_ERROR("The grasp data cannot be loaded");
    ros::shutdown();
  }
  grasp_data_.angle_resolution_ = 30; //20; // generate grasps at PI/angle_resolution increments
  grasp_data_.grasp_depth_ = 0.01; //in negative or 0 this makes the grasps on the other side of the object! (like from below)
  /*grasp_data_.approach_retreat_desired_dist_ = 0.3; //0.2 //as bigger better grasp
  grasp_data_.approach_retreat_min_dist_ = 0.05; //0.06*/
  /*std::cout << "grasp_data.approach_retreat_desired_dist_=" << grasp_data_.approach_retreat_desired_dist_ << std::endl
               << " grasp_data.approach_retreat_min_dist_=" << grasp_data_.approach_retreat_min_dist_ << std::endl
               << " grasp_data_.grasp_depth_= " << grasp_data_.grasp_depth_ << std::endl;*/

  visual_tools_ = visual_tools;

  //update visualization
  //visual_tools_->loadEEMarker(grasp_data_.ee_group_, plan_group);

  // Load Grasp generator
  simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

  pub_obj_pose = nh_->advertise<geometry_msgs::PoseStamped>("/pose_target", 10);
  pub_obj_poses = nh_->advertise<geometry_msgs::PoseStamped>("/pose_targets", 10);
}

bool Action::pickDefault(MetaBlock *block)
{
  bool done = false;

  std::vector<moveit_msgs::Grasp> grasps(1);

  moveit_msgs::Grasp g;
  g.grasp_pose.header.frame_id = block->name; //"base_link";
  g.grasp_pose.pose = block->start_pose;
  g.grasp_pose.pose = grasp_data_.grasp_pose_to_eef_pose_;

  g.pre_grasp_approach.direction.header.frame_id = grasp_data_.ee_parent_link_; //"base_link"; //"LWristYaw_link"; //lscene->getPlanningFrame();
  g.pre_grasp_approach.direction.vector.x = 0;
  g.pre_grasp_approach.direction.vector.y = 0;
  g.pre_grasp_approach.direction.vector.z = -1;
  g.pre_grasp_approach.min_distance = 0.06; //0.01;
  g.pre_grasp_approach.desired_distance = 0.2;

  g.post_grasp_retreat.direction.header.frame_id = grasp_data_.ee_parent_link_; //"base_link"; //"LWristYaw_link"; //lscene->getPlanningFrame();
  g.post_grasp_retreat.direction.vector.x = 0;
  g.post_grasp_retreat.direction.vector.y = 0;
  g.post_grasp_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
  g.post_grasp_retreat.min_distance = 0.06;
  g.post_grasp_retreat.desired_distance = 0.2;

  g.pre_grasp_posture.header.frame_id = grasp_data_.ee_parent_link_; //"base_link";
  g.grasp_posture.header.frame_id = grasp_data_.ee_parent_link_; //"base_link";
  if (grasp_data_.grasp_posture_.joint_names.size() > 0)
  {
    g.pre_grasp_posture.joint_names.resize(1);
    g.pre_grasp_posture.joint_names[0] = grasp_data_.grasp_posture_.joint_names[0]; //"LHand";
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(1);
    g.pre_grasp_posture.points[0].positions[0] = 0.0;

    g.grasp_posture.joint_names.resize(1);
    g.grasp_posture.joint_names[0] = g.pre_grasp_posture.joint_names[0];
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(1);
    g.grasp_posture.points[0].positions[0] = 1.0;
  }
  std::cout << "-- pickDefault g " << g << std::endl;

    grasps[0] = g;

    // Prevent collision with table
    //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(block->name);

    // Add this list to all grasps
    for (std::size_t i = 0; i < grasps.size(); ++i)
      grasps[i].allowed_touch_objects = allowed_touch_objects;

    if (move_group_->pick(block->name, grasps)){
      done = true;

      return true;
    }
    sleep(0.7);
  //}
}

bool Action::executeAction()//execute
{
  bool success(false);

  if (verbose_)
    ROS_INFO_STREAM("Execution of the planned action");

  if (!move_group_)
    return false;

  if (current_plan_)
    success = move_group_->execute(*current_plan_);

  if (verbose_ && success)
      ROS_INFO_STREAM("Execute success! \n\n");

  return success;
}

bool Action::graspPlanAndMove(MetaBlock *block, const std::string surface_name)
{
  bool success(false);

  success = graspPlan(block, surface_name);
  if (success)
    success = move_group_->move();

  if (verbose_ && success)
      ROS_INFO_STREAM("Plan and move success! \n\n");

  return success;
}

bool Action::graspPlan(MetaBlock *block, const std::string surface_name) //computePlanButtonClicked
{
  bool success(false);

  if (verbose_)
    ROS_INFO_STREAM("Planning " << block->name << " at pose " << block->start_pose);

  move_group_->setGoalTolerance(0.1);//0.05 //TODO

  // Prevent collision with table
  if (!surface_name.empty())
    move_group_->setSupportSurfaceName(surface_name);

  if (!move_group_)
    return false;

  //move_group_->setApproximateJointValueTargets(target, move_group_->getEndEffectorLink().c_str());
  move_group_->setPoseTargets(configureForPlanning(generateGrasps(block)), move_group_->getEndEffectorLink().c_str());

  current_plan_.reset(new moveit::planning_interface::MoveGroup::Plan());
  success = move_group_->plan(*current_plan_);
  if (!success)
    current_plan_.reset();

  if (verbose_ && success)
      ROS_INFO_STREAM("Grasp plannin success! \n\n");

  return success;
}

float computeDistance(geometry_msgs::Pose goal, geometry_msgs::Pose current)
{
  float dist = (goal.position.x - current.position.x)*(goal.position.x - current.position.x)
      + (goal.position.y - current.position.y) + (goal.position.y - current.position.y)
      + (goal.position.z - current.position.z) + (goal.position.z - current.position.z);

  return dist;
}

bool Action::poseHeadZero()
{
  return posture.poseHeadZero();
}

bool Action::poseHeadDown()
{
  return posture.poseHeadDown();
}

bool Action::poseHandInit()
{
  move_group_->setGoalTolerance(0.05);
  return posture.poseHandInit(end_eff, plan_group, arm);
}

bool Action::poseHand(std::vector<double> *pose_hand)
{
  move_group_->setGoalTolerance(0.05);
  return posture.poseHand(end_eff, plan_group, arm, pose_hand);
}

bool Action::poseHandZero()
{
  return posture.poseHandZero(end_eff, plan_group);
}

void Action::poseHandOpen()
{
  posture.poseHandOpen(end_eff);
}

void Action::poseHandClose()
{
  posture.poseHandClose(end_eff);
}

bool Action::reachInitPose()
{
  return reachAction(pose_init);
}

geometry_msgs::Pose Action::getPose()
{
  geometry_msgs::PoseStamped pose_now;
  pose_now.header.stamp = ros::Time::now();
  pose_now.header.frame_id = grasp_data_.base_link_;

  pose_now.pose = move_group_->getCurrentPose().pose;
  //ROS_INFO_STREAM("current pose is " << pose_now);

  pose_now.pose.position.x -= grasp_data_.grasp_pose_to_eef_pose_.position.x; //-0.133; //0.15; //0.18; //0.2;
  pose_now.pose.position.y -= grasp_data_.grasp_pose_to_eef_pose_.position.y; //0.07; //0.09; //0.07; //0.09;
  pose_now.pose.position.z -= grasp_data_.grasp_pose_to_eef_pose_.position.z; //-0.04;
  pose_now.pose.orientation = grasp_data_.grasp_pose_to_eef_pose_.orientation;
  //ROS_INFO_STREAM("current pose is " << pose_now);

  pub_obj_pose.publish(pose_now);

  return pose_now.pose;
}

void Action::setTolerance(const double value)
{
  move_group_->setGoalTolerance(value);
  if (verbose_)
    ROS_INFO_STREAM("The GoalPositionTolerance = " << move_group_->getGoalPositionTolerance());
}

float Action::reachGrasp(MetaBlock *block, const std::string surface_name)
{
  //if (verbose_)
    ROS_INFO_STREAM("Reaching at distance = " << block->start_pose.position.x << " " << block->start_pose.position.y << " " << block->start_pose.position.z);

  //clean object temporally or allow to touch it
  /*visual_tools_->cleanupCO(block->name);
  ros::Duration(1.0).sleep();*/

/*moveit_msgs::PickupGoal goal;
  collision_detection::AllowedCollisionMatrixPtr approach_grasp_acm(new collision_detection::AllowedCollisionMatrix(planning_scene->getAllowedCollisionMatrix()));

std::cout << "attach_object_msg.link_name  " << attach_object_msg.link_name << std::endl;
std::cout << "attach_object_msg.object.id " << attach_object_msg.object.id << std::endl;
std::cout << "attach_object_msg.object.operation " << attach_object_msg.object.operation << std::endl;
std::cout << "attach_object_msg.touch_links.size() " << attach_object_msg.touch_links.size() << std::endl;

  // we are allowed to touch the target object
  approach_grasp_acm->setEntry(goal.target_name, attach_object_msg.touch_links, true);
*/

  geometry_msgs::Pose pose = block->start_pose;
  pose.position.z += block->size_l/2.0;

  //reach the object
  if (!reachPregrasp(pose, surface_name))
    return std::numeric_limits<float>::max();

  //compute the distance to teh object
  float dist = computeDistance(move_group_->getCurrentPose().pose, move_group_->getPoseTarget().pose);
  if (verbose_)
    ROS_INFO_STREAM("Reached at distance = " << dist);

  //close the hand, if the object is close enough
  /*if (dist < 0.2)
  //if (dist > grasp_data_.approach_retreat_min_dist_)
  {
    //visual_tools_->processCollisionObjectMsg(wrapToCollisionObject(block));
    visual_tools_->attachCO(block->name, grasp_data_.ee_group_);
    posture.poseHandClose(end_eff);
  }*/

  return dist;
}

bool Action::reachPregrasp(geometry_msgs::Pose pose_target, const std::string surface_name)
{
  pose_target.position.x += grasp_data_.grasp_pose_to_eef_pose_.position.x; //-0.133; //0.15; //0.18; //0.2;
  pose_target.position.y += grasp_data_.grasp_pose_to_eef_pose_.position.y; //0.07; //0.09; //0.07; //0.09;
  pose_target.position.z += grasp_data_.grasp_pose_to_eef_pose_.position.z; //-0.04;
  pose_target.orientation = grasp_data_.grasp_pose_to_eef_pose_.orientation;

  return reachAction(pose_target, surface_name);
}

bool Action::reachAction(geometry_msgs::Pose pose_target, const std::string surface_name)
{
  bool success(false);

  if (verbose_)
    ROS_INFO_STREAM("Planning to the pose " << pose_target);

  if (!move_group_)
    return false;

  moveit::planning_interface::MoveGroup::Plan plan;

  // Prevent collision with table
  if (!surface_name.empty())
    move_group_->setSupportSurfaceName(surface_name);

  move_group_->setPoseTarget(pose_target, move_group_->getEndEffectorLink().c_str());
  pub_obj_pose.publish(move_group_->getPoseTarget());

  //move_group_->setMaxVelocityScalingFactor(0.6); //TODO back
  //move_group_->setNumPlanningAttempts(50);
  double tolerance_min = 0.01;
  double tolerance_max = 0.5;
  double tolerance = tolerance_min;
  int attempts = 0;

  //find a planning solution while increasing tolerance
  while (!success && (attempts < 5))
  {
    move_group_->setGoalTolerance(tolerance);//0.05 //TODO to check
    //move_group_->setGoalPositionTolerance(0.07);
    //move_group_->setGoalOrientationTolerance(0.1);
    success = move_group_->plan(plan);

    if (!success)
    {
      tolerance_min = tolerance;
      tolerance = (tolerance_max-tolerance_min) / 2.0;

      if (verbose_)
        ROS_INFO_STREAM("Planning retry with the tolerance " << tolerance);
    }
    ++attempts;
  }

  //find an approximate solution
  if (!success)
  {
    move_group_->setApproximateJointValueTarget(pose_target, move_group_->getEndEffectorLink().c_str());
    success = move_group_->plan(plan);
  }

  if (success)
    success = move_group_->move();

  if (verbose_ && success)
    ROS_INFO_STREAM("Reaching success with tolerance " << tolerance << "\n\n");

  return success;
}

bool Action::graspPlanAllPossible(MetaBlock *block, const std::string surface_name) //computePlanButtonClicked
{
  bool success(false);

  if (verbose_)
    ROS_INFO_STREAM("Planning all possible grasps to " << block->start_pose);

  move_group_->setGoalTolerance(0.1);
  //move_group_->setPoseReferenceFrame("base_link"); //"LWristYaw_link"
  //move_group_->setStartState(*move_group_->getCurrentState());
  //move_group_->setPlanningTime(30); //in GUI=5
  //move_group_->setNumPlanningAttempts(10); //in GUI=10
  //move_group_->setMaxVelocityScalingFactor(1.0); //in GUI = 1.0
  //move_group_->setPlannerId("RRTConnectkConfigDefault");

  // Prevent collision with table
  if (!surface_name.empty())
    move_group_->setSupportSurfaceName(surface_name);

  std::vector<geometry_msgs::Pose> targets = configureForPlanning(generateGrasps(block));

  moveit::planning_interface::MoveGroup::Plan plan;

  if (targets.size() > 0)
  {
    int counts = 0;
    for (std::vector<geometry_msgs::Pose>::iterator it=targets.begin(); it!=targets.end();++it)
    {
      //move_group_->setPoseTarget(*it, move_group_->getEndEffectorLink().c_str());
      //move_group_->setPositionTarget(it->position.x, it->position.y, it->position.z, move_group_->getEndEffectorLink().c_str());
      success = move_group_->setApproximateJointValueTarget(*it, move_group_->getEndEffectorLink().c_str());
      success = move_group_->plan(plan);
      if (success)
        ++counts;
    }
    if (verbose_)
      ROS_INFO_STREAM( "Planning success for " << counts << " generated poses! \n\n");
  }
  return success;
}

std::vector<moveit_msgs::Grasp> Action::generateGrasps(MetaBlock *block)
{
  std::vector<moveit_msgs::Grasp> grasps;
  if (block->name.empty())
  {
    ROS_INFO_STREAM("No object choosen to grasp");
    return grasps;
  }

  if (verbose_)
    visual_tools_->deleteAllMarkers();

  geometry_msgs::Pose pose = block->start_pose;
  pose.position.z += block->size_l/2.0;
  simple_grasps_->generateBlockGrasps(pose, grasp_data_, grasps );

  if (verbose_)
  {
    double speed = 0.01; //0.05; //
    //visual_tools_->publishGrasps(grasps, grasp_data_.ee_parent_link_, speed);
    //visual_tools_->deleteAllMarkers();
    sleep(0.5);
  }

  if (grasps.size() > 0)
  {
    // an optional list of obstacles that we have semantic information about and that can be touched/pushed/moved in the course of grasping
    std::vector<std::string> allowed_touch_objects(1);
    allowed_touch_objects[0] = block->name;
    for (std::size_t i = 0; i < grasps.size(); ++i)
      grasps[i].allowed_touch_objects = allowed_touch_objects;
  }

  return grasps;
}

std::vector<geometry_msgs::Pose> Action::configureForPlanning(const std::vector<moveit_msgs::Grasp> &grasps)
{
  std::vector<geometry_msgs::Pose> targets(grasps.size());

  if (grasps.size() > 0)
  {
    std::vector<moveit_msgs::Grasp>::const_iterator it_grasp;
    std::vector<geometry_msgs::Pose>::iterator it_pose;
    for (it_grasp=grasps.begin(), it_pose=targets.begin(); it_grasp!=grasps.end(); ++it_grasp, ++it_pose)
    {
      *it_pose = it_grasp->grasp_pose.pose;
    }
  }

  return targets;
}

bool Action::pickAction(MetaBlock *block, 
                        const std::string surface_name,
                        int attempts_nbr,
                        double planning_time,
                        double tolerance_min)
{
  bool success(false);
  //if (verbose_)
    ROS_INFO_STREAM("Pick at pose " << block->start_pose.position.x << " " << block->start_pose.position.y << " " << block->start_pose.position.z);

  if (attempts_nbr == 0)
    attempts_nbr = attempts_max_;

  if (planning_time == 0.0)
    planning_time = planning_time_;

  if (tolerance_min == 0.0)
    tolerance_min = tolerance_min_;

  std::vector<moveit_msgs::Grasp> grasps = generateGrasps(block);

  if (grasps.size() > 0)
  {
    // Prevent collision with table
    if (!surface_name.empty())
      move_group_->setSupportSurfaceName(surface_name);

    move_group_->setPlanningTime(planning_time); //30.0);
    //move_group_->setPlannerId("RRTConnectkConfigDefault");
    //move_group_->setPlannerId("RRTConnect");

    double tolerance_min = 0.01;
    double tolerance_max = 0.5;
    double tolerance = tolerance_min;
    int attempts = 0;

    //find a planning solution while increasing tolerance
    while (!success && (attempts < attempts_max_))
    {
      move_group_->setGoalTolerance(tolerance);//0.05 //TODO to check
      success = move_group_->pick(block->name, grasps);

      if (!success)
      {
        tolerance_min = tolerance;
        tolerance = (tolerance_max-tolerance_min) / 2.0;
      }
      ++attempts;
    }

    if (verbose_ & success)
      ROS_INFO_STREAM("Pick success with tolerance " << tolerance << "\n\n");
  }
  return success;
}

bool Action::placeAction(MetaBlock *block, const std::string surface_name)
{
  if (verbose_)
    ROS_INFO_STREAM("Placing " << block->name << " at pose " << block->goal_pose);

  // Prevent collision with table
  if (!surface_name.empty())
    move_group_->setSupportSurfaceName(surface_name);

  std::vector<moveit_msgs::PlaceLocation> place_locations;

  // Re-usable datastruct
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = grasp_data_.base_link_;
  pose_stamped.header.stamp = ros::Time::now();

  // Create 360 degrees of place location rotated around a center
  //for (double angle = 0; angle < 2*M_PI; angle += M_PI/2)
  //{
    pose_stamped.pose = block->goal_pose;

    // Create new place location
    moveit_msgs::PlaceLocation place_loc;
    place_loc.place_pose = pose_stamped;

    // Approach
    moveit_msgs::GripperTranslation pre_place_approach;
    pre_place_approach.direction.header.stamp = ros::Time::now();
    pre_place_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
    pre_place_approach.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
    pre_place_approach.direction.header.frame_id = grasp_data_.base_link_;
    pre_place_approach.direction.vector.x = 0;
    pre_place_approach.direction.vector.y = 0;
    pre_place_approach.direction.vector.z = 0.1; //-1 // Approach direction (negative z axis)  // TODO: document this assumption
    place_loc.pre_place_approach = pre_place_approach;

    // Retreat
    /*moveit_msgs::GripperTranslation post_place_retreat(pre_place_approach);
    post_place_retreat.direction.vector.x = 0;
    post_place_retreat.direction.vector.y = 0;
    post_place_retreat.direction.vector.z = -1; //1 // Retreat direction (pos z axis)
    place_loc.post_place_retreat = post_place_retreat;*/
    // Post place posture - use same as pre-grasp posture (the OPEN command)
    //ROS_INFO_STREAM("place_loc.post_place_retreat" << place_loc.post_place_retreat);
    place_loc.post_place_posture = grasp_data_.pre_grasp_posture_; //grasp_data_.grasp_posture_;

    place_locations.push_back(place_loc);
  //}

  move_group_->setPlannerId("RRTConnectkConfigDefault");

  bool success = move_group_->place(block->name, place_locations);
  if (verbose_)
  {
    if (success)
      ROS_INFO_STREAM("Place success! \n\n");
    else
      ROS_ERROR_STREAM_NAMED("simple_actions:","Place failed.");
  }

  return success;
}

/*void Action::filterGrasps(MetaBlock *block)
{
  if (verbose_)
    ROS_INFO_STREAM("Picking " << block->name << " at pose " << block->start_pose);

  std::vector<moveit_msgs::Grasp> grasps = generateGrasps(block);

  if (grasps.size() > 0)
  {
    grasp_filter_.filterGrasps(grasps);
  }
}*/

}
