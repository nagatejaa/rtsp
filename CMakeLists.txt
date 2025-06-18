cmake_minimum_required(VERSION 3.8)
project(nodes)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(camera_publisher src/image.cpp)
ament_target_dependencies(camera_publisher rclcpp sensor_msgs)
target_link_libraries(camera_publisher ${OpenCV_LIBS})

add_executable(rtsp_camera_publisher src/image_rtsp.cpp)
ament_target_dependencies(rtsp_camera_publisher rclcpp sensor_msgs)
target_link_libraries(rtsp_camera_publisher ${OpenCV_LIBS})

add_executable(apriltag_camera_publisher src/rtsp_apriltag.cpp)
ament_target_dependencies(apriltag_camera_publisher rclcpp sensor_msgs)
target_link_libraries(apriltag_camera_publisher ${OpenCV_LIBS})


add_executable(sub_node src/sub.cpp)
ament_target_dependencies(sub_node rclcpp sensor_msgs std_msgs)
target_link_libraries(sub_node ${OpenCV_LIBS})

add_executable(pub_node src/pub.cpp)
ament_target_dependencies(pub_node rclcpp sensor_msgs std_msgs)
target_link_libraries(pub_node ${OpenCV_LIBS})


install(TARGETS 
  sub_node
  pub_node
  camera_publisher
  rtsp_camera_publisher
  apriltag_camera_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
