#include "leonardo_control/QuaternionTransform.hpp"

const std::string QuaternionTransform::TOPIC_BASE_NAME = "perception_neuron/QUAT_data_";
const std::string QuaternionTransform::TOPIC_DATA_1 = QuaternionTransform::TOPIC_BASE_NAME + "1";
const std::string QuaternionTransform::TOPIC_DATA_2 = QuaternionTransform::TOPIC_BASE_NAME + "2";


QuaternionTransform::QuaternionTransform(ros::NodeHandle& t_nodeHandle)
  : m_nodeHandle(t_nodeHandle)
{
  subscribe();
}

void QuaternionTransform::subscribe()
{
  m_subscriber1 = m_nodeHandle.subscribe(TOPIC_DATA_1, QUEUE_SIZE,
                        &QuaternionTransform::callbackData1, this);
  m_subscriber2 = m_nodeHandle.subscribe(TOPIC_DATA_2, QUEUE_SIZE,
                        &QuaternionTransform::callbackData2, this);
  ROS_INFO("Subscribed to the Quaternions topic data");
}


void QuaternionTransform::callbackData1(const std_msgs::Float64MultiArray& msg)
{
  ROS_INFO("got data1 msg");
  std::vector<Eigen::Quaterniond> quaternions = convertFArray2Qvector(msg);
}

void QuaternionTransform::callbackData2(const std_msgs::Float64MultiArray& msg)
{
  ROS_INFO("got data2 msg");
}


std::vector<Eigen::Quaterniond> QuaternionTransform::convertFArray2Qvector(std_msgs::Float64MultiArray array)
{
  std::vector<Eigen::Quaterniond> quaternions;
  for(int i = 0; i < 120; i += 4) {
    Eigen::Quaterniond qTemp(
                              array.data[i],
                              array.data[i+1],
                              array.data[i+2],
                              array.data[i+3]
                            );
    quaternions.push_back(qTemp);
  }
  return quaternions;
}
