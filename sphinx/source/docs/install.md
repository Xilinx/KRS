# Install KRS

KRS `1.0` enables various robotic development paths:
- **Yocto/PetaLinux**: An embedded cross-compilation-oriented path for creating production-grade OSs for robots. Leverages the Yocto project.
- **Ubuntu 22.04**: A developer-friendly path leveraging a pre-built rootfs that allows to reproduce Desktop environments to build binaries on target, or cross-compile.

Below detail the installation and setup process of each one of the development paths. Before that, some considerations worth noting that apply to all of these development paths:

```eval_rst
.. important::
    **KRS 1.0 has only been tested in Ubuntu 22.04**. It assumes the following is installed in your workstation:

    - `Ubuntu 22.04` Jammy Jellyfish operating system.
    - the Vitis `2022.1` suite (Vitis, Vivado, Vitis HLS) (`install instructions <https://www.xilinx.com/support/download/index.html>`_)
    - the ROS 2 Humble Hawksbill  distribution (`install instructions <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>`_)
    - Gazebo Classic 11.0 (`install instructions <https://classic.gazebosim.org/tutorials?tut=install_ubuntu>`_)

    For KRS documentation source code, refer to https://github.com/Xilinx/KRS.


.. admonition:: Upstream integration

    KRS is served as a group of ROS 2 packages that you can install from .deb files or from sources in any arbitrary ROS 2 workspace, enhancing it with hardware acceleration capabilities. Some of these packages have been integrated upstream to simplify the development flow into the ROS buildfarm and are available as part of ROS 2 starting from ROS 2 Humble. Some others, including *firmware* for selected boards and *examples*, need to be fetched manually at desire.

    The following demonstrates how to create a new ROS 2 overlay workspace, fetch the KRS packages, build them from source and run some acceleration examples:

.. admonition:: Ignition Gazebo vs Gazebo Classic

    ROS 2 Humble ships with Ignition Gazebo (renamed to "Gazebo"). Installing Gazebo Classic (Gazebo 11.0) is still possible but requires some manual work. Some of the examples below were developed with Gazebo Classic. In turn, examples might be rewritten with Ignition Gazebo ("Gazebo") to facilitate the flows.
```


## Yocto/PetaLinux

```eval_rst
.. admonition:: Yocto/PetaLinux firmware artifacts download

    Pre-built firmware artifacts for creating robot OSs using Yocto/PetaLinux are bigger than 2GB, which is the maximum size allowed by GitHub. The firmware artifacts have temporarily been uploaded to https://drive.google.com/file/d/1gzrGHB-J_fKNBmcGYhClXdWo6wGw8k43/view?usp=sharing and need to be manually downloaded and deployed into the workspace src directory.


.. admonition:: Yocto (Honister)

    KRS 1.0 Yocto/PetaLinux development paths builds artifacts based on Yocto Honister.

```

[![asciicast](https://asciinema.org/a/434953.svg)](https://asciinema.org/a/434953)

<!--
###################################################
# 3. Install KRS packages from the ROS buildfarm
###################################################
sudo apt-get install -y \
  ros-humble-ament-acceleration \
  ros-humble-ament-vitis \
  ros-humble-vitis-common \
  ros-humble-tracetools-acceleration \
  ros-humble-ros2acceleration \
  python3-colcon-hardware-acceleration

-->

```shell
###################################################
# 0. install Vitis 2022.1 https://www.xilinx.com/support/download.html       
#   and ROS 2 Rolling https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
#    we recommend the Desktop-Full flavour (ros-humble-desktop-full)
###################################################

###################################################
# 1. install some dependencies you might be missing
###################################################
sudo apt-get -y install curl build-essential libssl-dev git wget \
                          ocl-icd-* opencl-headers python3-vcstool \
                          python3-colcon-common-extensions python3-colcon-mixin \
                          kpartx u-boot-tools pv

###################################################
# 2. create a new ROS 2 workspace with examples and
#    firmware for KV260
###################################################
mkdir -p ~/krs_ws/src; cd ~/krs_ws

###################################################
# 3. Create file with KRS 1.0 additional repos
###################################################
cat << 'EOF' > krs_humble.repos
repositories:  
  perception/image_pipeline:
    type: git
    url: https://github.com/ros-acceleration/image_pipeline
    version: ros2

  tracing/tracetools_acceleration:
    type: git
    url: https://github.com/ros-acceleration/tracetools_acceleration
    version: humble

  firmware/acceleration_firmware_kv260:
    type: zip
    url: https://drive.google.com/file/d/1gzrGHB-J_fKNBmcGYhClXdWo6wGw8k43

  acceleration/adaptive_component:
    type: git
    url: https://github.com/ros-acceleration/adaptive_component
    version: humble
  acceleration/ament_acceleration:
    type: git
    url: https://github.com/ros-acceleration/ament_acceleration
    version: humble
  acceleration/ament_vitis:
    type: git
    url: https://github.com/ros-acceleration/ament_vitis
    version: humble
  acceleration/colcon-hardware-acceleration:
    type: git
    url: https://github.com/colcon/colcon-hardware-acceleration
    version: main
  acceleration/ros2_kria:
    type: git
    url: https://github.com/ros-acceleration/ros2_kria
    version: main
  acceleration/ros2acceleration:
    type: git
    url: https://github.com/ros-acceleration/ros2acceleration
    version: humble
  acceleration/vitis_common:
    type: git
    url: https://github.com/ros-acceleration/vitis_common
    version: humble
  acceleration/acceleration_examples:
    type: git
    url: https://github.com/ros-acceleration/acceleration_examples
    version: main
EOF

###################################################
# 4. import repos of KRS beta release
###################################################
vcs import src --recursive < krs_humble.repos  # about 3 mins in an AMD Ryzen 5 PRO 4650G

###################################################
# 5. build the workspace and deploy firmware for hardware acceleration
###################################################
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/rolling/setup.bash  # Sources system ROS 2 installation.

# Note: The path above is valid if one installs ROS 2 from a pre-built debian
# packages. If one builds ROS 2 from the source the directory might
# vary (e.g. ~/ros2_humble/ros2-linux).
export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+
colcon build --merge-install  # about 4 mins in an AMD Ryzen 5 PRO 4650G

###################################################
# 6. source the overlay to enable all features
###################################################
source install/setup.bash
```

That's pretty much it, you've got now KRS installed in the `krs_ws` ROS overlay workspace.  You could also reproduce the same steps over an existing ROS 2 workspace if you'd like to avoid creating a new, or simply reusing the source code elsewhere.

Now's time to build and run some [examples](https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/0_ros2_publisher.html).


## Ubuntu 22.04