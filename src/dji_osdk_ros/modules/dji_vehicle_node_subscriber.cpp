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

