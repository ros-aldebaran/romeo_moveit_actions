#include "romeo_moveit_actions/objprocessing.hpp"
#include "romeo_moveit_actions/toolsForAction.hpp"

namespace moveit_simple_actions
{

Objprocessing::Objprocessing(ros::NodeHandle *nh_):
  nh_(nh_),
  mesh_srv_name("get_object_info"),
  OBJECT_RECOGNITION_ACTION("/recognize_objects"),
  target_frame("base_link"),
  depth_frame_id("CameraDepth_frame")
{
  found_srv_obj_info = true;
  found_object_recognition_client_ = false;

  ros::Time start_time = ros::Time::now();
  while (!ros::service::waitForService(mesh_srv_name, ros::Duration(2.0)))
  {
    ROS_INFO("Waiting for %s service to come up", mesh_srv_name.c_str());
    if (!nh_->ok() || ros::Time::now() - start_time >= ros::Duration(5.0))
    {
      found_srv_obj_info = false;
      break;
    }
  }

  if (found_srv_obj_info)
  {
    get_model_mesh_srv_ = nh_->serviceClient<object_recognition_msgs::GetObjectInformation>
      (mesh_srv_name, false);
  }
}

std::vector <shape_msgs::Mesh> Objprocessing::getMeshFromDB(object_recognition_msgs::ObjectType type)
{
  std::vector <shape_msgs::Mesh> meshes;
  object_recognition_msgs::GetObjectInformation obj_info;
  obj_info.request.type = type;

  if (!found_srv_obj_info)
    return meshes;
  if ( !get_model_mesh_srv_.call(obj_info) )
  {
    ROS_ERROR("The service get_object_info does not respond");
    return meshes;
  }

  meshes.push_back(obj_info.response.information.ground_truth_mesh);
  return meshes;
}

bool Objprocessing::triggerObjectDetection()
{
  if(!object_recognition_client_)
  {
    object_recognition_client_.reset(new actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>(OBJECT_RECOGNITION_ACTION, false));
  }
  if (!found_object_recognition_client_)
  {
    try
    {
      ROS_INFO_STREAM("Waiting for the Object recognition client");
      waitForAction(object_recognition_client_, *nh_, ros::Duration(5, 0), OBJECT_RECOGNITION_ACTION);
      ROS_INFO_STREAM("Object recognition client is ready");
      found_object_recognition_client_ = true;
    }
    catch(std::runtime_error &ex)
    {
      ROS_ERROR("Object recognition action: %s", ex.what());
      return false;
    }
  }

  if (found_object_recognition_client_)
  {
    object_recognition_msgs::ObjectRecognitionGoal goal;
    object_recognition_client_->sendGoal(goal);
    if (!object_recognition_client_->waitForResult())
    {
      ROS_INFO_STREAM("Object recognition client returned early");
      return false;
    }
    if (object_recognition_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_WARN_STREAM("Fail: " << object_recognition_client_->getState().toString() << ": " << object_recognition_client_->getState().getText());
      return true;
    }
  }
//ROS_INFO_STREAM("triggerObjectDetection finished");
  return false;
}
}
