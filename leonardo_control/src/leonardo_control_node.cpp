#include "leonardo_control/leonardo_control_node.hpp"
#include "leonardo_control/LeonardoArmControl.hpp"
#include "leonardo_control/QuaternionTransform.hpp"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "leonardo_control");
  ros::NodeHandle nh;

  QuaternionTransform qt(nh);

  ros::spin();
  return 0;
}
