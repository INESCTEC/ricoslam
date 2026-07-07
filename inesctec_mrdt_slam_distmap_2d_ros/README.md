# inesctec_mrdt_slam_distmap_2d_ros
## Stage
### Launch
**ROS 1**
```sh
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/ros/srrg_software/setup.bash
source /home/ros1${ROS_DISTRO}/ros_ws/devel_release/setup.bash

sudo ldconfig --verbose /opt/stage/lib/

roslaunch stage_ros stageros.launch
roslaunch stage_ros teleop_twist_keyboard.launch
```
**ROS 2**
```sh
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/ros/srrg_software/setup.bash
source /home/ros2${ROS_DISTRO}/ros_ws/install_release/setup.bash

sudo ldconfig --verbose /opt/stage/lib/

ros2 launch stage_ros stageros2.launch.xml
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Record
**ROS 1**
```sh
# rosbag record --repeat-latched --buffsize=0 --split --duration=5m --size=100 --bz2 --output-prefix /home/datasets/stage/ros1${ROS_DISTRO}/log_slam /base_pose_ground_truth /base_scan /clock /odom /tf /tf_static

rosbag record                                                     \
  --repeat-latched                                                \
  --buffsize=0                                                    \
  --split --duration=5m --size=100                                \
  --bz2                                                           \
  --output-prefix /home/datasets/stage/ros1${ROS_DISTRO}/log_slam \
  /base_pose_ground_truth /base_scan /clock /odom /tf /tf_static
```
**ROS 2**
```sh
# ros2 bag record --polling-interval 100 --compression-mode none --storage mcap --max-bag-size 10000000 --output /home/datasets/stage/ros2${ROS_DISTRO}/log_slam_`date +%Y-%m-%d-%H-%M-%S` /base_pose_ground_truth /base_scan /clock /odom /tf /tf_static

ros2 bag record                                                                       \
  --polling-interval 100                                                              \
  --compression-mode none                                                             \
  --storage mcap                                                                      \
  --max-bag-size 10000000                                                             \
  --output /home/datasets/stage/ros2${ROS_DISTRO}/log_slam_`date +%Y-%m-%d-%H-%M-%S`  \
  /base_pose_ground_truth /base_scan /clock /odom /tf /tf_static
```
