# Docker
## Setup
### Distrobox
```sh
## NVIDIA Container Toolkit
sudo apt update && sudo apt install -y --no-install-recommends curl gnupg2
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list |        \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update

sudo apt install -y nvidia-container-toolkit      \
                    nvidia-container-toolkit-base \
                    libnvidia-container-tools     \
                    libnvidia-container1

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# Rootless mode
# nvidia-ctk runtime configure --runtime=docker --config=$HOME/.config/docker/daemon.json
# systemctl --user restart docker
# sudo nvidia-ctk config --set nvidia-container-cli.no-cgroups --in-place

## Distrobox
sudo apt install -y distrobox

## Visual Studio Code (VSCode) Integration with Distrobox
mkdir -p ~/.local/bin
curl -s https://raw.githubusercontent.com/89luca89/distrobox/main/extras/docker-host -o ~/.local/bin/docker-host
chmod +x ~/.local/bin/docker-host
# then, change in VScode settings
# "dev.containers.dockerSocketPath": "/home/<username>/.local/bin/docker-host"
```
**.bashrc**
```sh
# Replace the following in your .bashrc...
# ...

# if [ "$color_prompt" = yes ]; then
#     PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
# else
#     PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
# fi
# unset color_prompt force_color_prompt

if [ "$color_prompt" = yes ]; then
    if [[ "$HOSTNAME" == "revelation" ]]; then  # green txt (32)
        HOST_STYLE="01;32"
    elif [[ "$HOSTNAME" == "noetic" ]]; then    # white txt (37) magenta bg (45)
        HOST_STYLE="1;37;45"
    elif [[ "$HOSTNAME" == "foxy" ]]; then      # white txt (37) red bg (41)
        HOST_STYLE="1;37;41"
    elif [[ "$HOSTNAME" == "humble" ]]; then    # white txt (37) blue bg (44)
        HOST_STYLE="1;37;44"
    elif [[ "$HOSTNAME" == "jazzy" ]]; then     # white txt (37) yellow bg (43)
        HOST_STYLE="1;37;43"
    elif [[ "$HOSTNAME" == "rolling" ]]; then   # white txt (37) green bg (42)
        HOST_STYLE="1;37;42"
    else                                        # white txt (37) black bg (40)
        HOST_STYLE="1;37;40"
    fi

    PS1='${debian_chroot:+($debian_chroot)}\[\033[${HOST_STYLE}m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else

# ...

## DISTROBOX
export DBX_CONTAINER_MANAGER=docker

## ROBOT OPERATING SYSTEM (ROS)
alias wsros='source /opt/ros/${ROS_DISTRO}/setup.bash'
alias wssrrg='source /opt/ros/srrg_software/setup.bash'

wsdir() {
  if [ -z "$1" ]; then
    echo "Provide a workspace folder (e.g., devel, devel_release, install, install_release)"
    return -1
  fi

  WORKSPACE_DIR=$1

  if [ -f "$WORKSPACE_DIR/setup.bash" ]; then
    source "$WORKSPACE_DIR/setup.bash"
  else
    echo "$WORKSPACE_DIR/setup.bash file not found!"
    return -1
  fi
}
```

## Usage
### Build
```sh
docker build -f Dockerfile.ros1noetic -t ricoslam-rosnoetic:latest .
docker build -f Dockerfile.ros2foxy -t ricoslam-rosfoxy:latest .
docker build -f Dockerfile.ros2humble -t ricoslam-roshumble:latest .
docker build -f Dockerfile.ros2jazzy -t ricoslam-rosjazzy:latest .
docker build -f Dockerfile.ros2rolling -t ricoslam-rosrolling:latest .
```
### Run
```sh
distrobox create                                          \
  --name ricoslam-ros<noetic|foxy|humble|jazzy|rolling>   \
  --image ricoslam-ros<noetic|foxy|humble|jazzy|rolling>  \
  --hostname <noetic|foxy|humble|jazzy|rolling>           \
  --volume "/mnt/data/datasets:/home/datasets"            \
  --nvidia --yes

distrobox enter ricoslam-ros<noetic|foxy|humble|jazzy|rolling>

# Required for Wayland-based hosts, given that ROS 1 Noetic not compatible with
export QT_QPA_PLATFORM=xcb

sudo ldconfig --verbose /opt/stage/lib/

cd /home/ros$ROS_DISTRO/ros_ws/src
ln -s /home/sousarbarb97/dev/phd/ricoslam .

cd ..
wsros
wssrrg

# ROS 1 (noetic)
catkin_make                                         \
  --source src                                      \
  --build build_release                             \
  --force-cmake -DCATKIN_DEVEL_PREFIX=devel_release \
  -DCMAKE_BUILD_TYPE=Release                        \
  -DCMAKE_INSTALL_PREFIX=install_release

# ROS 2 (foxy|humble|jazzy|rolling)
colcon build                              \
  --build-base build_release              \
  --install-base install_release          \
  --merge-install --symlink-install       \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers status+ console_cohesion+ console_direct+ console_start_end+ console_stderr+
```
