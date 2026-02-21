1. CPU / RAM

# find process pid
ps aux | grep dual_laser_merger
top -p <pid>
# or use htop

2. ROS bag
ros2 bag record -o dual_merge_test /front_scan /rear_scan /merged_scan /tf


https://docs.ros.org/en/humble/p/dual_laser_merger/