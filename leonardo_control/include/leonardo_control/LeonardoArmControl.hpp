#include <ros/ros.h>
#include <std_msgs/String.h>

class LeonardoArmControl
{
public:
  LeonardoArmControl(ros::NodeHandle& t_nh);
  void exec(void);
  void setup(void);
  void positionCmdListener(const std_msgs::String& msg);

private:
  ros::NodeHandle m_nodeHandle;
  std::string m_side;
  std::string m_topicName;
  ros::Subscriber m_subscriber;
};
