# inesctec_mrdt_slam_2d_datasets

ROS package bundling 2D SLAM datasets and a converter that turns
[CARMEN](https://carmen.sourceforge.net/) log files (`.clf`) into ROSbags.

Most datasets come from the
[SLAM benchmarking](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php)
framework by Kümmerle *et al.* (see [Reference](#reference)). Two additional
datasets provided by Prof. Dr. Giorgio Grisetti (`cappero` and `kuka`) are also
shipped as ready-to-play ROSbags in [`bag/`](bag/). See
[`data/README.md`](data/README.md) for details on the CARMEN datasets.

## Contents

- [Repository layout](#repository-layout)
- [Datasets](#datasets)
- [ROS](#ros)
  - [Dependencies](#dependencies)
  - [Build](#build)
- [Usage](#usage)
  - [Convert CARMEN to ROSbag](#convert-carmen-to-rosbag)
  - [Play ROSbags](#play-rosbags)
- [Reference](#reference)
- [License](#license)
- [Contacts](#contacts)
- [Acknowledgements](#acknowledgements)
- [Funding](#funding)

## Repository layout

```
inesctec_mrdt_slam_2d_datasets
├── bag/            # output/ready-to-play ROSbags (*.orig.bag, *.sort.bag, giorgio bags)
├── config/         # per-dataset YAML parametrization for the converter
├── data/           # CARMEN logs (*.clf), ground-truth relations (*.relations)
├── include/        # C++ headers (parser, math utils)
├── launch/         # conversion + playback launch files
├── rviz/           # RViz configurations
├── src/            # converter source (carmen_to_rosbag)
├── CMakeLists.txt
├── package.xml
├── LICENSE
└── README.md
```

## Datasets

All lasers are configured with a 180° field of view, published on `/scan` with
frame `laser`; odometry is published on `/odom` with frame `odom`. The laser
pose (`tf`) is `[x (m), y (m), θ (deg)]` w.r.t. `base_link`. `save_odom`
controls whether the odometry embedded in the laser message is also written to
`/odom`.

| Dataset      | CARMEN log                   | Laser `tf` (x, y, θ) | `range_min` | `range_max` | `save_odom` |
|--------------|------------------------------|----------------------|-------------|-------------|-------------|
| `aces`       | `aces.clf`                   | `0.00, 0, 0`         | 0.30 m      | 50.0 m      | disabled    |
| `fr-clinic`  | `fr-clinic_robotlaser.clf`   | `0.22, 0, 0`         | 0.10 m      | 80.0 m      | enabled     |
| `fr079`      | `fr079.clf`                  | `0.04, 0, 0`         | 0.10 m      | 80.0 m      | disabled    |
| `intel`      | `intel.clf`                  | `0.00, 0, 0`         | 0.10 m      | 80.0 m      | disabled    |
| `mit-csail`  | `mit-csail.clf`              | `0.00, 0, 0`         | 0.10 m      | 80.0 m      | enabled     |
| `mit-killian`| `mit-killian.clf`            | `0.00, 0, 0`         | 0.10 m      | 80.0 m      | disabled    |

The converter supports the CARMEN `ODOM`, `FLASER`, and `ROBOTLASER1` messages.
Unlike `ROBOTLASER1`, `FLASER` records carry neither field of view nor angular
resolution, so the resolution is derived from the sample count assuming the
configured 180° span.

Two extra datasets are shipped only as ROSbags (no CARMEN log / converter step):

| Dataset   | ROSbag                                                    | Fixed frame        |
|-----------|-----------------------------------------------------------|--------------------|
| `cappero` | `cappero_laser_odom_diag_2020-05-06-16-26-03_fixed.bag`   | `odom`             |
| `kuka`    | `kuka-halle7-log1.frontlaser.bag`                         | `odom_robot_link`  |

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

### Build

Clone the package into the `src/` folder of your catkin workspace, then build
and source it:

```sh
cd ~/ros1_ws
catkin build inesctec_mrdt_slam_2d_datasets   # or: catkin_make
source devel/setup.bash
```

## Usage

### Convert CARMEN to ROSbag

The converter reads a `.clf` log plus a YAML config and writes **two** BZ2-
compressed ROSbags next to each other: `<stem>.orig.bag` and `<stem>.sort.bag`,
where `<stem>` is the log filename without its extension.

- `<stem>.orig.bag` — messages in the original CARMEN order; the original
  message index is preserved in each message's `std_msgs::Header::seq`.
- `<stem>.sort.bag` — messages reordered by the CARMEN IPC timestamp, with
  `Header::seq` recomputed per topic accordingly.

**Via `roslaunch` (recommended):**

```sh
roslaunch inesctec_mrdt_slam_2d_datasets convert_carmen_to_rosbag.launch \
    dataset:=<aces|fr-clinic|fr079|intel|mit-csail|mit-killian>
```

Optional arguments:

- `clf_folder` — input `.clf` directory (default:
  `$(find inesctec_mrdt_slam_2d_datasets)/data/`)
- `bag_folder` — output `.bag` directory (default:
  `$(find inesctec_mrdt_slam_2d_datasets)/bag/`)

**Via `rosrun`:**

```sh
rosrun inesctec_mrdt_slam_2d_datasets carmen_to_rosbag         \
    -c $(rospack find inesctec_mrdt_slam_2d_datasets)/config/carmen_to_rosbag_<dataset>.yaml \
    -d $(rospack find inesctec_mrdt_slam_2d_datasets)/data/    \
    -o $(rospack find inesctec_mrdt_slam_2d_datasets)/bag/
```

| Option        | Required | Description                                            |
|---------------|----------|--------------------------------------------------------|
| `-c, --config`| yes      | YAML configuration file for the dataset                |
| `-d, --data`  | yes      | Input folder containing the `.clf` log                 |
| `-o, --output`| no       | Output folder for the ROSbags (defaults to the data folder) |

### Play ROSbags

Playback uses simulated time (`/use_sim_time`) and optionally opens RViz.
Common arguments across the launch files: `rate` (default `1.0`), `start`
(seconds, default `0.0`), and `rviz` (default `false`).

**SLAM benchmarking datasets** (converted `*.sort.bag`):

```sh
roslaunch inesctec_mrdt_slam_2d_datasets play_slam_benchmark.launch \
    dataset:=<aces|fr-clinic_robotlaser|fr079|intel|mit-csail|mit-killian> \
    rviz:=true rate:=1.0 start:=0.0
```

To play the original-order bag instead, use `play.launch` (which points at
`bag/<dataset>.bag`) or pass the bag path directly to `rosbag play`.

**Giorgio datasets:**

```sh
roslaunch inesctec_mrdt_slam_2d_datasets play_giorgio.launch \
    dataset:=<cappero|kuka> \
    rviz:=true rate:=1.0 start:=0.0
```

## Reference

If you use the SLAM benchmarking datasets, please cite:

> Rainer Kümmerle, Bastian Steder, Christian Dornhege, Michael Ruhnke,
> Giorgio Grisetti, Cyrill Stachniss, and Alexander Kleiner.
> "On measuring the accuracy of SLAM algorithms."
> *Autonomous Robots*, Volume 27, pages 397–407 (2009).
> DOI: [10.1007/s10514-009-9155-6](https://doi.org/10.1007/s10514-009-9155-6)

## License

Distributed under the GNU AGPLv3 license. See `LICENSE` for more information.

## Contacts

If you have any questions or you want to know more about this work, please
contact:
- **Ricardo B. Sousa**
  ([ricardo.b.sousa@inesctec.pt](mailto:ricardo.b.sousa@inesctec.pt))
  _(Corresponding Author)_
  ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [orcid](https://orcid.org/0000-0003-4537-5095),
  [website](https://sousarbarb.github.io/),
  [youtube](https://www.youtube.com/channel/UCXTR8mMlG0VOC_06PKg5KBQ))
- **Héber Miguel Sobreira, PhD**
  ([heber.m.sobreira@inesctec.pt](mailto:heber.m.sobreira@inesctec.pt))
  ([orcid](https://orcid.org/0000-0002-8055-1093))
- **Prof. Dr. António Paulo Moreira**
  ([amoreira@fe.up.pt](mailto:amoreira@fe.up.pt))
  ([orcid](https://orcid.org/0000-0001-8573-3147))
- **Prof. Dr. Giorgio Grisetti**
  ([grisetti@diag.uniroma1.it](mailto:grisetti@diag.uniroma1.it))
  ([github](https://github.com/grisetti),
  [gitlab](https://gitlab.com/grisetti),
  [google-scholar](https://scholar.google.com/citations?user=yD-SFG4AAAAJ))

## Acknowledgements

- [5dpo Robotics Team](https://5dpo.github.io/)
- [CRIIS - Centre for Robotics in Industry and Intelligent Systems](https://www.inesctec.pt/en/centres/criis),
  [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)
- [Flowbotic Mobile Systems, Lda](https://www.flowbotic.eu/)
- [Robots Vision and Perception (RVP)](https://rvp-group.net/) research group
  from the
  [Department of Computer, Control, and Management Engineering Antonio Ruberti (DIAG)](https://www.diag.uniroma1.it/en)
  of [Sapienza University of Rome](https://www.uniroma1.it/en)

## Funding

**GreenAuto: Green innovation for the Automotive Industry**

This work was co-financed by Component 5 - Capitalization and Business
Innovation, integrated in the Resilience Dimension of the Recovery and
Resilience Plan within the scope of the Recovery and Resilience Mechanism
(MRR) of the European Union (EU), framed in the Next Generation EU, for the
period 2021-2026, within project GreenAuto, with reference 54.

- **Operation Code:** 02/C05-i01.02/2022.PC644867037-00000013
- **Beneficiary:** Peugeot Citröen Automóveis Portugal, S.A.
- **Work Package:** WP10 - Automated logistics for the automotive industry
- **Product, Processes, or Services (PPS):**
  PPS18 - 3D navigation system for mobile robotic equipment
- **Consortium Partners:**
    - [Flowbotic Mobile Systems, Lda](https://www.flowbotic.eu/) _(leader)_
    - [Faculty of Engineering, University of Porto (FEUP)](https://www.up.pt/feup/en/)
    - [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
    - [STAR](https://starinstitute.pt/)
    - [Kaizen](https://kaizen.com/pt-pt/)
    - [Institute for Systems and Robotics (ISR)-Coimbra](https://www.isr.uc.pt/)
- **Timeline:** October 2021 - December 2025
- **Duration:** 51 months
- **URL:**
  [https://transparencia.gov.pt/en/fundos-europeus/prr/beneficiarios-projetos/projeto/02/C05-i01.02/2022.PC644867037-00000013/](https://transparencia.gov.pt/en/fundos-europeus/prr/beneficiarios-projetos/projeto/02/C05-i01.02/2022.PC644867037-00000013/)
