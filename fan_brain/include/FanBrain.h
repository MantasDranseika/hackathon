#ifndef FAN_BRAIN_FANBRAIN_H
#define FAN_BRAIN_FANBRAIN_H
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace fan_brain
{
class FanBrain : public rclcpp::Node
{
public:
  FanBrain();

private:
  void topicCallback(std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisher;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_tempSub;   
  int tempSet;
  int fanOn;
};
}
#endif
