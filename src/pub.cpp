#include <iostream>
#include <thread>  
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyPub : public rclcpp::Node
{
public:
  MyPub() : Node("pub_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    std::thread([this]() {
      RCLCPP_INFO(this->get_logger(), "Type and press Enter to transmit");

      std::string value;
      while (rclcpp::ok()) {
        std::getline(std::cin, value);  

        if (value.empty()) {
          RCLCPP_ERROR(this->get_logger(), "Input cannot be empty");
          continue;
        }

        auto message = std_msgs::msg::String();
        message.data = value;

        RCLCPP_INFO(this->get_logger(), "Transmitting: '%s'", message.data.c_str());
        publisher_->publish(message);
      }
    }).detach();  
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPub>());
  rclcpp::shutdown();
  return 0;
}
