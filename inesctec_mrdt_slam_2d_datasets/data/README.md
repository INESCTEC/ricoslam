# inesctec_mrdt_slam_2d_datasets/data

All the datasets present in this folder were retrieved from the
[slam benchmarking](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php)
website.

## Datasets

- ACES Building (Austin) _(Patrick Beeson)_
  [\[log\]](aces.clf) ,
  [\[relations\]](aces.relations)
- Intel Research Lab (Seattle) _(Dirk Hähnel)_
  [\[log\]](intel.clf) ,
  [\[relations\]](intel.relations)
- 2 MIT Killian Court (2D) _(Mike Bosse and John Leonard)_
  [\[log\]](mit-killian.clf) ,
  [\[relations\]](mit-killian.relations)
- MIT CSAIL Building _(Cyrill Stachniss)_
  [\[log\]](mit-csail.clf) ,
  [\[relations\]](mit-csail.relations)
- Freiburg Indoor Building 079 _(Cyrill Stachniss)_
  [\[log\]](fr079.clf) ,
  [\[relations\]](fr079.relations)
- Freiburg University Hospital (3D)
  _(Bastian Steder, Rainer Kümmerle, Christian Dornhege, Michael Ruhnke, Cyrill_
  _Stachniss, Giorgio Grisetti, and Alexander Kleiner)_
  [\[log\]](fr-clinic_robotlaser.clf) ,
  [\[relations\]](fr-clinic.relations)

All the files referenced here are available in the
[slam benchmarking](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php)
website.

## Documentation

- CARMEN
  ([website](https://carmen.sourceforge.net/),
  [source](https://sourceforge.net/projects/carmen/)) log files
  - `ODOM x y theta tv rv accel ipc_timestamp ipc_hostname logger_timestamp`
    - (x,y,theta): robot pose at the center of the robot
    - (tv, tx): translational (m/s) and rotational (rad/s) velocities,
      respectively
    - (accel): robot acceleration (m/s^2)
  - `FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta ipc_timestamp ipc_hostname logger_timestamp`
    (older format, if no information is given, assume 1.0º resolution)
  - `RAWLASER laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values] ipc_timestamp ipc_hostname logger_timestamp`
  - `ROBOTLASER1 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] num_remissions [remission values] laser_pose_x laser_pose_y laser_pose_theta robot_pose_x robot_pose_y robot_pose_theta laser_tv laser_rv forward_safety_dist side_safty_dist turn_axis ipc_timestamp ipc_hostname logger_timestamp`
  - Additional observations:
    - [CARMEN Logging Data and Playing It Back Documentation](https://carmen.sourceforge.net/logger_playback.html)
    - [carmen_publisher](https://github.com/artivis/carmen_publisher)
    - [carmen_player_ros](https://github.com/NaokiAkai/carmen_player_ros)
    - [SLAM Evaluation CARMEN Log Files Documentation](http://ais.informatik.uni-freiburg.de/slamevaluation/software.php)
- Relations files
  ```
  # timestamp1 timestamp2 x y z roll pitch yaw
  1232658478.161285 1232658644.149293 1.031273 -0.003531 0.000000 0.000000 -0.000000 -0.009755
  1232658478.161285 1232658647.058497 2.039538 -0.014451 0.000000 0.000000 -0.000000 -0.016492
  ```
  - Each line describes a relation given by x, y, z, roll, pitch and yaw from
    timestamp1 to timestamp2. The timestamps have to match those in the logfile
    that is to be evaluated

## Stats

**aces**
- parser stats (#msgs):
  - odom: 7375
  - flaser: 7374
  - robotlaser: 0
  - laser total: 7374
  - total: 14749
- laser range stats (#msgs):
  - max_range: 50.00 m
  - 2nd max_range: 49.91 m
  - min_range: 0.31 m
  - 2nd min_range: 0.34 m

**fr-clinic**
- parser stats (#msgs):
  - odom: 6917
  - flaser: 0
  - robotlaser: 6917
  - laser total: 6917
  - total: 13834
- laser range stats (#msgs):
  - max_range: 81.91 m
  - 2nd max_range: 81.83 m
  - min_range: 0.54 m
  - 2nd min_range: 0.55 m

**fr079**
- parser stats (#msgs):
  - odom: 8951
  - flaser: 4934
  - robotlaser: 0
  - laser total: 4934
  - total: 13885
- laser range stats (#msgs):
  - max_range: 81.91 m
  - 2nd max_range: 36.62 m
  - min_range: 0.12 m
  - 2nd min_range: 0.13 m

**intel**
- parser stats (#msgs):
  - odom: 26915
  - flaser: 13631
  - robotlaser: 0
  - laser total: 13631
  - total: 40546
- laser range stats (#msgs):
  - max_range: 81.83 m
  - 2nd max_range: 26.37 m
  - min_range: 0.20 m
  - 2nd min_range: 0.21 m

**mit-csail**
- parser stats (#msgs):
  - odom: 1988
  - flaser: 1988
  - robotlaser: 0
  - laser total: 1988
  - total: 3976
- laser range stats (#msgs):
  - max_range: 81.91 m
  - 2nd max_range: 42.40 m
  - min_range: 0.27 m
  - 2nd min_range: 0.28 m

**mit-killian**
- parser stats (#msgs):
  - odom: 69924
  - flaser: 17480
  - robotlaser: 0
  - laser total: 17480
  - total: 87404
- laser range stats (#msgs):
  - max_range: 51.22 m
  - 2nd max_range: 51.16 m
  - min_range: 0.24 m
  - 2nd min_range: 0.26 m
