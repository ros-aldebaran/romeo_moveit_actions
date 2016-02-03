#include <Eigen/Eigen>
#include <stdlib.h>

#include <metablock.hpp>

MetaBlock::MetaBlock(const std::string name,
          const double start_x,
          const double start_y,
          const double start_z,
          const double orien_x,
          const double orien_y,
          const double orien_z,
          const double orien_w,
          const uint shapeType,
          const double size,
          const double size_r)
{
  this->name = name;
  this->size = size*size_r;

  //set position
  start_pose.position.x = start_x;
  start_pose.position.y = start_y;
  start_pose.position.z = start_z; //-0.13;//-0.11; //getTableHeight(-0.9); //from -0.11 to -0.1
  this->start_pose.orientation.x = orien_x;
  this->start_pose.orientation.y = orien_y;
  this->start_pose.orientation.z = orien_z;
  this->start_pose.orientation.w = orien_w;


  if (start_pose.position.y < 0)
    start_pose.orientation.y *= -1;

  goal_pose = start_pose;
  goal_pose.position.y = 0.14;
  if (start_pose.position.y < 0)
    goal_pose.position.y *= -1;

  //setshape
  if (shapeType == shape_msgs::SolidPrimitive::CYLINDER)
  {
    shape_msgs::SolidPrimitive shapeCylinder;
    shapeCylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
    shapeCylinder.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    shapeCylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = size*size_r;
    shapeCylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = size;
    this->shape = shapeCylinder;
  }
  else
  {
    shape_msgs::SolidPrimitive shapeBox;
    shapeBox.type = shape_msgs::SolidPrimitive::BOX;
    shapeBox.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_X] = size;
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size;
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = size;
    this->shape = shapeBox;
  }
}

MetaBlock::MetaBlock(const std::string name,
          const ros::Time timestamp,
          const geometry_msgs::Pose start_pose,
          const uint shapeType,
          const double size,
          const double size_l)
{
  this->name = name;
  this->timestamp = timestamp;
  this->size = size;
  this->size_l = size_l;

  this->start_pose = start_pose;

  goal_pose = start_pose;
  goal_pose.position.x = 0.5;
  goal_pose.position.y = 0.18;
  if (start_pose.position.y < 0)
    goal_pose.position.y = -0.25;

  /*this->goal_pose.orientation.x = -1.0;
  this->goal_pose.orientation.y = 0.0;
  this->goal_pose.orientation.z = 0.0;
  this->goal_pose.orientation.w = 0.0;*/

  //setshape
  if (shapeType == shape_msgs::SolidPrimitive::CYLINDER)
  {
    shape_msgs::SolidPrimitive shapeCylinder;
    shapeCylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
    shapeCylinder.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    shapeCylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = size_l;
    shapeCylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = size;
    this->shape = shapeCylinder;
  }
  else
  {
    shape_msgs::SolidPrimitive shapeBox;
    shapeBox.type = shape_msgs::SolidPrimitive::BOX;
    shapeBox.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_X] = size;
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size;
    shapeBox.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = size_l;
    this->shape = shapeBox;
  }
}

MetaBlock::MetaBlock(const std::string name,
          const ros::Time timestamp,
          const geometry_msgs::Pose start_pose,
          const shape_msgs::Mesh mesh,
          const object_recognition_msgs::ObjectType type)
{
  this->name = name;
  this->timestamp = timestamp;

  this->start_pose = start_pose;
  this->start_pose.orientation.x = -1.0;
  this->start_pose.orientation.y = 0.0;
  this->start_pose.orientation.z = 0.0;
  this->start_pose.orientation.w = 0.0;

  this->goal_pose = start_pose;
  this->goal_pose.position.x = 0.4;//0.47; //0.5;//
  this->goal_pose.position.y = 0.3;
  this->goal_pose.position.z = -0.05;
  this->goal_pose.orientation.x = -1.0;
  this->goal_pose.orientation.y = 0.0;
  this->goal_pose.orientation.z = 0.0;
  this->goal_pose.orientation.w = 0.0;

  this->mesh = mesh;
  this->type = type;
}

void MetaBlock::updatePose(const geometry_msgs::Pose start_pose)
{
  this->start_pose = start_pose;
}

void MetaBlock::setRndPose()
{
  this->start_pose.position.x = 0.35f + float(rand() % 150)/1000.0f; //[0.35;0.50]
  this->start_pose.position.y = float(rand() % 90)/100.0f - 0.45; //[-0.45;0.45]
  this->start_pose.position.z = -0.23f + (float(rand() % 230)/1000.0f); //[-0.23;0.00]
}
