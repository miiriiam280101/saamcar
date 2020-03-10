#include "base_actuator_node.hpp"
arduino_com_client center_actuator_device;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void angleCallback(const std_msgs::Float64::ConstPtr& msg)
{
  center_actuator_device.send_steering(msg->data * 100.);
  
}

void speedCallback(const std_msgs::Float64::ConstPtr& msg){
  _Float64 speed;
  if (-40. > msg->data > .40){
    speed = .40;
    if (msg->data < 0) speed*= -1.; //If speed is smaller than 0 change sign
  }else
  {
    speed = msg->data;
  }
  center_actuator_device.send_speed(speed);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_actuator");

  ros::NodeHandle n;

  /* Connect to Arduino and check if connected */
  if(center_actuator_device.init(ARDUINO_CENTER_ACTUATORS, 
    SERIAL_DEVICE_PREFIX, NUM_ARDUINO))
  {
    ROS_INFO("Connected to arduino on port: %d\t ID: %d\t Software version: %d",
        center_actuator_device.get_port_num(), center_actuator_device.get_id(), center_actuator_device.get_software_version());
  }
  else
  {
    ROS_ERROR("Could not find an arduino with correct id: %d ", ARDUINO_CENTER_ACTUATORS);
  }

  ros::Subscriber sub = n.subscribe("/base_actuator/angle_setpoint", 10, angleCallback);
  ros::Subscriber speed_sub = n.subscribe("/base_actuator/control_effort_speed", 10, speedCallback);
  
  while(ros::ok()){
    ros::spinOnce();
    center_actuator_device.send_watchdog_trigger();
  }

  return 0;
}