

#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher() : Node("apriltag_camera_publisher")
  {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    cap_.open(0);  // Use laptop camera
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open laptop camera.");
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(100ms, std::bind(&CameraPublisher::publish_frame, this));
    RCLCPP_INFO(this->get_logger(), "Laptop camera opened successfully.");
  }

private:
  void publish_frame()
  {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) return;

    auto now = this->get_clock()->now();

    // Image msg
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = now;
    img_msg.header.frame_id = "camera_frame";
    img_msg.height = frame.rows;
    img_msg.width = frame.cols;
    img_msg.encoding = "bgr8";
    img_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.cols * frame.elemSize());
    img_msg.data.assign(frame.datastart, frame.dataend);
    image_pub_->publish(img_msg);

    // CameraInfo msg (dummy values)
    sensor_msgs::msg::CameraInfo info_msg;
    info_msg.header.stamp = now;
    info_msg.header.frame_id = "camera_frame";
    info_msg.height = frame.rows;
    info_msg.width = frame.cols;
    info_msg.k = {500.0, 0.0, frame.cols / 2.0,
                  0.0, 500.0, frame.rows / 2.0,
                  0.0, 0.0, 1.0};
    info_msg.p = {500.0, 0.0, frame.cols / 2.0, 0.0,
                  0.0, 500.0, frame.rows / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0};
    info_pub_->publish(info_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Publishing camera frame: %dx%d", frame.cols, frame.rows);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}

