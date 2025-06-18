






#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher()
      : Node("camera_publisher")
  {
    camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
    camera_timer_ = this->create_wall_timer(100ms, std::bind(&CameraPublisher::camera_callback, this));

    cap.open(0);  // <-- This opens the laptop camera

    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open camera.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Camera opened successfully.");
    }
  }

private:
  void camera_callback()
  {
    if (!cap.isOpened()) return;

    cv::Mat frame;  
    cap >> frame;

    if (frame.empty()) return;

    sensor_msgs::msg::Image camera_msg;

    camera_msg.header.stamp = this->now();
    camera_msg.header.frame_id = "camera_frame";

    camera_msg.height = frame.rows;
    camera_msg.width = frame.cols;
    camera_msg.encoding = "bgr8";

    camera_msg.step = camera_msg.width * 3;
    camera_msg.data.assign(frame.datastart, frame.datastart + camera_msg.step * camera_msg.height);

    camera_publisher_->publish(camera_msg);
    RCLCPP_INFO(this->get_logger(), "Published frame: %dx%d", frame.cols, frame.rows);
  }

  rclcpp::TimerBase::SharedPtr camera_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
  cv::VideoCapture cap;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}

