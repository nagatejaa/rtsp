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
      : Node("rtsp_camera_publisher")
  {
   
    rtsp_url_ = "rtsp://Tapo_camera:trinnovate@172.20.10.2:554/stream1?buffer_size=0&max_delay=0&rtsp_transport=udp";

    camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
    camera_timer_ = this->create_wall_timer(100ms, std::bind(&CameraPublisher::timer_callback, this));

   
    cap.open(rtsp_url_, cv::CAP_FFMPEG);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open RTSP stream.");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "RTSP stream opened successfully (UDP).");
    }
  }

private:
  void timer_callback()
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
    RCLCPP_INFO(this->get_logger(), "Publishing camera frame.");
  }

  rclcpp::TimerBase::SharedPtr camera_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;

  cv::VideoCapture cap;
  std::string rtsp_url_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();

  return 0;
}


