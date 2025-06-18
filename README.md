to run pub or sub node 
ros2 run nodes sub_node
ros2 run nodes pub_node
_________________________________________________________________

to see live laptop camera feed as a ros2 topic  run
ros2 run nodes camera_publisher
_________________________________________________________________

to run any ip camera feed as a ros2 topic then we need to run 
ros2 run nodes rtsp_camera_publisher

but change the rtsp url for the ip camera its 
username:passward@ipaddress/stream
_________________________________________________________________

to run april tag tracking we need to run 
in Terminal 1: ros2 run nodes apriltag_camera_publisher
in Terminal 2: ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/camera -r camera_info:=/camera_info -p approx_sync:=true --log-level debug


__________________________________________________________________________
bghgthgfghghyghjnb