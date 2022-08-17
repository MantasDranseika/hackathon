#include "FanBrain.h"
using namespace std::chrono_literals;
using namespace fan_brain;
using std::placeholders::_1;

FanBrain::FanBrain(): Node("fan_brain") 
, tempSet{20}
, fanOn{0}

{
  m_publisher = create_publisher<std_msgs::msg::Int32>("fan_brain", 1);
  m_tempSub = create_subscription<std_msgs::msg::Int32>(          
    "zephyr_fan_control", 1, std::bind(&FanBrain::topicCallback, this, std::placeholders::_1)); 
}

void FanBrain::topicCallback(std_msgs::msg::Int32::SharedPtr tempMsg)
{
  auto message = std_msgs::msg::Int32();

    if (tempMsg->data >= tempSet && fanOn == 0)
    {
      message.data = 1;
      fanOn = message.data;
      m_publisher->publish(message);
    }
    else if (tempMsg->data < tempSet && fanOn == 1 )
    {
      message.data = 0;
      fanOn = message.data;
      m_publisher->publish(message);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FanBrain>());
  rclcpp::shutdown();
  return 0;
}
