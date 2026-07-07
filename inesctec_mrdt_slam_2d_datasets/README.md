# inesctec_mrdt_slam_2d_datasets

This repository contains an executable to convert CARMEN log data to the ROSbag
format. See [data/README.md](data/README.md) for more information about the
CARMEN datasets retrieved from the
[slam benchmarking](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php)
website.
Additionally, two datasets provided by Prof. Dr. Giorgio Grisetti are also
available in the [bag](bag/) folder.

## ROS

**ROS 1**

- [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://index.ros.org/p/roscpp/#noetic)
- [geometry_msgs](https://index.ros.org/p/geometry_msgs/#noetic)
- [nav_msgs](https://index.ros.org/p/nav_msgs/#noetic)
- [rosbag](https://index.ros.org/p/rosbag/#noetic)
- [sensor_msgs](https://index.ros.org/p/sensor_msgs/#noetic)
- [tf](https://index.ros.org/p/tf/#noetic)
- [tf2_msgs](https://index.ros.org/p/tf2_msgs/#noetic)
- [Boost::program_options](https://www.boost.org/doc/libs/1_85_0/doc/html/program_options.html)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp/)

## Usage

### Convert CARMEN to ROS

```sh
cd ~/ros1_ws
rosrun inesctec_mrdt_slam_2d_datasets carmen_to_rosbag                          \
    -d src/inesctec_mrdt_slam_distmap_2d/inesctec_mrdt_slam_2d_datasets/data/   \
    -c src/inesctec_mrdt_slam_distmap_2d/inesctec_mrdt_slam_2d_datasets/config/carmen_to_rosbag_<aces|fr-clinic|fr079|intel|mit-csail|mit-killian>.yaml
```

This exexcutable generates two ROSbag: `<CARMEN filename>.orig.bag` and
`<CARMEN filename>.sort.bag`. The former has the ROS messages written in the
original order in the ROSbag, also putting the original message sequence in the
ROS messages' `std_msgs::Header::seq` field. The latter fixes the order and
recomputes the sequence accordingly to the CARMEN IPC timestamp.

### Play ROSbags

**giorgio datasets**

```sh
roslaunch inesctec_mrdt_slam_2d_datasets play_giorgio.launch          \
    rviz:=true                                                        \
    rate:=1.0                                                         \
    start:=0.0                                                        \
    dataset:=<cappero|kuka>
```

**slam benchmarking datasets**

```sh
roslaunch inesctec_mrdt_slam_2d_datasets play_slam_benchmark.launch   \
    rviz:=true                                                        \
    rate:=1.0                                                         \
    start:=0.0                                                        \
    dataset:=<aces|fr-clinic_robotlaser|fr079|intel|mit-csail|mit-killian>
```

## License

Distributed under the _GNU Lesser General Public License v3.0_.
See [LICENSE](LICENSE) for more information.

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- António Paulo Moreira
  ([github](https://github.com/apaulomoreira),
  [mail](mailto:amoreira@fe.up.pt))
- Giorgio Grisetti
  ([github](https://github.com/grisetti),
  [gitlab](https://gitlab.com/grisetti),
  [mail](mailto:grisetti@diag.uniroma1.it))
- Héber Miguel Sobreira
  ([github](https://github.com/HeberSobreira),
  [gitlab](https://gitlab.com/heber.m.sobreira),
  [gitlab:inesctec](https://gitlab.inesctec.pt/heber.m.sobreira),
  [mail](mailto:heber.m.sobreira@inesctec.pt))
- Manuel F. Silva
  ([mail](mss@isep.ipp.pt))
- Ricardo B. Sousa
  ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb),
  [gitlab:inesctec](https://gitlab.inesctec.pt/ricardo.b.sousa),
  [mail:inesctec](mailto:ricardo.b.sousa@inesctec.pt),
  [mail:personal](mailto:sousa.ricardob@outlook.com),
  [mail:student](mailto:rbs@fe.up.pt))

Project Link:
https://github.com/sousarbarb/inesctec_mrdt_slam_distmap_2d/.

## Acknowledgements

- [CRIIS - Centre for Robotics in Industry and Intelligent Systems](https://www.inesctec.pt/en/centres/criis/) from
  [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [Robots Vision and Perception Group](https://rvp-group.net/) from the
  [Department of Computer, Control, and Management Engineering](https://www.diag.uniroma1.it/en/)
  at [Sapienza University of Rome](https://www.uniroma1.it/en/)
