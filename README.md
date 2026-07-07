# RicoSLAM: Localization and Mapping in Dynamic Environments

A Simultaneous Localization and Mapping (SLAM) system for
dynamic industrial environments built around a distance map-based tracking
front-end, a factor graph optimization back-end using the
[srrg2_solver](https://gitlab.com/srrg-software/srrg2_solver) framework,
online and offline dynamics filtering through ray tracing, and a novel distance
map-based global bundle adjustment formulation.
The front-end tracker supports the distance map-based registration formulations
proposed in the PhD Thesis and also Iterative Closest Point (ICP) with
distance map-guided data association.
RicoSLAM has been deployed on-site across six industrial companies in the
scope of the GreenAuto and related projects.

## Setup
### Repository
```sh
mkdir ${HOME}/ros_ws/src -p
cd ${HOME}/ros_ws/src

git clone --recurse-submodules -j8 github.com:sousarbarb/inesctec_mrdt_slam_distmap_2d.git
```

### SRRG Software
```sh
sudo apt update
sudo apt dist-upgrade -y
sudo apt install -y git libsuitesparse-dev freeglut3-dev libqglviewer-dev-qt5

# SRRG Workspace
mkdir ${HOME}/srrg_ws/src -p

cd ${HOME}/srrg_ws/src/
git clone --recurse-submodules -j8 git@gitlab.com:sousarbarb_srrg-software/srrg_system.git
```
**ROS 1 (Noetic)**
```sh
source /opt/ros/noetic/setup.bash

cd ${HOME}/srrg_ws/
catkin_make --force-cmake -DCMAKE_BUILD_TYPE=Release
sudo catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/srrg_software

source /opt/ros/srrg_software/setup.bash
```
**ROS 2 (Foxy | Humble | Jazzy | Rolling)**
```sh
source /opt/ros/<ROS DISTRO>/setup.bash

cd ${HOME}/srrg_ws/
sudo colcon build --merge-install --install-base /opt/ros/srrg_software --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers status+ console_cohesion+ console_direct+ console_start_end+ console_stderr+

source /opt/ros/srrg_software/setup.bash
```
See more information in
[https://gitlab.com/sousarbarb_srrg-software/srrg_system](https://gitlab.com/sousarbarb_srrg-software/srrg_system).

### ROS 1 (other SLAM algorithms)
```sh
sudo apt install -y ros-noetic-slam-gmapping ros-noetic-grid-map ros-noetic-hector-slam* ros-noetic-slam-karto* ros-noetic-slam-toolbox*
```
### YAML-CPP
```sh
# apt repository
sudo apt install -y libyaml-cpp-dev

# Source installation
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp/
mkdir build
cd build/
cmake .. -DYAML_BUILD_SHARED_LIBS=on -DYAML_CPP_INSTALL=true
make
make install
```

## Usage
### Build
**ROS 1 (Noetic)**
```sh
source /opt/ros/noetic/setup.bash
source /opt/ros/srrg_software/setup.bash

cd ${HOME}/ros_ws
catkin_make --source src --build build_release --force-cmake -DCATKIN_DEVEL_PREFIX=devel_release -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install_release
# catkin_make --source src --build build_reldebinfo --force-cmake -DCATKIN_DEVEL_PREFIX=devel_reldebinfo -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=install_reldebinfo

source devel_release/setup.bash

# catkin_make --source src --build build_<debug|release|reldebinfo|minsizerel> --force-cmake -DCATKIN_DEVEL_PREFIX=devel_<debug|release|reldebinfo|minsizerel> -DCMAKE_BUILD_TYPE=<Debug|Release|RelWithDebInfo|MinSizeRel> -DCMAKE_INSTALL_PREFIX=install_<debug|release|reldebinfo|minsizerel>
# source devel_<debug|release|reldebinfo|minsizerel>/setup.bash
```

### Launch
- tracker
- tracker_offline
- slam_online
- slam_offline

See further information on the launch files in the
[inesctec_mrdt_slam_distmap_2d_ros/launch](/inesctec_mrdt_slam_distmap_2d_ros/launch/)
folder.

**How to enable debug with GDB?**
- build with `Debug` or `RelWithDebInfo` CMake configurations
- uncomment `launch-prefix="gdb -ex run --args"` for the node executable
- launch nodes (GDB should start automatically in the same terminal)

**How to know the options for the offline executables?**
```sh
rosrun inesctec_mrdt_slam_distmap_2d_ros tracker_offline --help
rosrun inesctec_mrdt_slam_distmap_2d_ros slam_offline --help
```

## License

Distributed under the GNU AGPLv3 license. See `LICENSE` for more information.

## Citation

If you use material from this thesis in a work that leads to a scientific
publication, please consider citing it as follows.

**Plain Text**

R.B. Sousa,
"RicoSLAM: Localization and Mapping in Dynamic Environments,"
PhD Thesis, Faculty of Engineering, University of Porto, Porto, Portugal,
2026.

**BibTeX**

```bibtex
@PHDTHESIS{sousa2026thesis,
  author  = {Ricardo B. Sousa},
  title   = {{RicoSLAM}: Localization and Mapping in Dynamic Environments},
  school  = {Faculty of Engineering, University of Porto},
  address = {Porto, Portugal},
  year    = {2026},
  type    = {{PhD} Thesis}}
```

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
