#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class my_sub : public rclcpp::Node
{
  public:
    my_sub()
    : Node("sub_node")
    {
      RCLCPP_WARN(this->get_logger(), "connected to topic 'topic'");
      RCLCPP_INFO(this->get_logger(), "Subscriber node started");
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&my_sub::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I got : '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_sub>());
  rclcpp::shutdown();
  return 0;
}