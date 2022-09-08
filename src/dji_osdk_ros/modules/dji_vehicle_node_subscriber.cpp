//
// Created by dji on 2020/5/8.
//
#include <dji_osdk_ros/dji_vehicle_node.h>
using namespace dji_osdk_ros;

void
VehicleNode::gimbalAngleCtrlCallback(
  const dji_osdk_ros::Gimbal::ConstPtr& msg
)
{
  ROS_DEBUG("called gimbalAngleCtrlCallback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
  }

  DJI::OSDK::Gimbal::AngleData angle_data;
  //! OSDK takes 0.1 sec as unit
  angle_data.duration = msg->ts;
  angle_data.mode     = msg->mode;
  //! OSDK takes 0.1 deg as unit
  angle_data.roll     = RAD2DEG(msg->roll)*10;
  angle_data.pitch    = RAD2DEG(msg->pitch)*10;
  angle_data.yaw      = RAD2DEG(msg->yaw)*10;
  
  ptr_wrapper_->getVehicle()->gimbal->setAngle(&angle_data);
}

void
VehicleNode::gimbalSpeedCtrlCallback(
  const geometry_msgs::Vector3Stamped::ConstPtr& msg
)
{
  ROS_DEBUG("called gimbalSpeedCtrlCallback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
  }

  DJI::OSDK::Gimbal::SpeedData speed_data;
  //! OSDK takes 0.1 deg as unit
  speed_data.gimbal_control_authority = 1;
  speed_data.roll  = RAD2DEG(msg->vector.x)*10;
  speed_data.pitch = RAD2DEG(msg->vector.y)*10;
  speed_data.yaw   = RAD2DEG(msg->vector.z)*10;

  ptr_wrapper_->getVehicle()->gimbal->setSpeed(&speed_data);
}

void
VehicleNode::flightCommandCallback(
  const dji_osdk_ros::FlightCommand::ConstPtr& msg
)
{
  ROS_DEBUG("called flightCommandCallback");
  if(ptr_wrapper_ == nullptr)
  {
    ROS_ERROR_STREAM("Vehicle modules is nullptr");
  }

  dji_osdk_ros::JoystickMode joystickMode;
  joystickMode.horizontalLogic      = msg->mode.horizontal_mode;
  joystickMode.verticalLogic        = msg->mode.vertical_mode;
  joystickMode.yawLogic             = msg->mode.yaw_mode;
  joystickMode.horizontalCoordinate = msg->mode.horizontal_coordinate;
  joystickMode.stableMode           = msg->mode.stable_mode;
  ptr_wrapper_->setJoystickMode(joystickMode);

  dji_osdk_ros::JoystickCommand joystickCommand;
  joystickCommand.x   = msg->cmd.x;
  joystickCommand.y   = msg->cmd.y;
  joystickCommand.z   = msg->cmd.z;
  joystickCommand.yaw = msg->cmd.yaw;
  ptr_wrapper_->JoystickAction(joystickCommand);
}

