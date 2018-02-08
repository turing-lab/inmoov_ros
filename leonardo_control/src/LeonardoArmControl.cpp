#include "leonardo_control/LeonardoArmControl.hpp"

LeonardoArmControl::LeonardoArmControl(ros::NodeHandle& t_nh)
  : m_nodeHandle(t_nh)
{
  m_topicName = "leonardo_pos_cmd";
  setup();
}

void LeonardoArmControl::exec(void)
{
  printf("Hola desde LeonardoArmControl\n");
}

void LeonardoArmControl::setup(void)
{
  m_subscriber = m_nodeHandle.subscribe(m_topicName,  10,
                        &LeonardoArmControl::positionCmdListener, this);
  ROS_INFO("subscribed to topic");
}


void LeonardoArmControl::positionCmdListener(const std_msgs::String& msg)
{

}
