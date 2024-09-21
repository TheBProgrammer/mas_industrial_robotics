ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30
ros2 run mir_ppt_detection ppt_detector
ros2 run lifecycle_controller lifecycle_controller --ros-args -p lc_name:=ppt_detector -p command:=CA
ros2 run tf2_ros static_transform_publisher 1 2 3 0.5 0.1 -1.0 camera_color_optical_frame base_link
ros2 action send_goal ppt_detection mir_interfaces/action/ObjectDetection "{obj_category: "work"}"