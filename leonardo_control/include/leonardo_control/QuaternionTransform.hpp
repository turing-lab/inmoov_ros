#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Geometry>

class QuaternionTransform
{
public:
  QuaternionTransform(ros::NodeHandle& t_nodeHandle);
  void subscribe();
  void callbackData1(const std_msgs::Float64MultiArray& msg);
  void callbackData2(const std_msgs::Float64MultiArray& msg);
  std::vector<Eigen::Quaterniond> convertFArray2Qvector(std_msgs::Float64MultiArray array);

private:
  static const std::string TOPIC_BASE_NAME;
  static const std::string TOPIC_DATA_1;
  static const std::string TOPIC_DATA_2;
  const int QUEUE_SIZE = 10;

  ros::NodeHandle m_nodeHandle;
  ros::Subscriber m_subscriber1;
  ros::Subscriber m_subscriber2;
};
