# Install KRS

```eval_rst
.. important::

    KRS is currently only available for Linux and **has only been tested in Ubuntu 20.04**. KRS assumes the following is installed in your workstation:

    - `Ubuntu 20.04` Focal Fossa operating system (`download <https://releases.ubuntu.com/20.04/ubuntu-20.04.3-desktop-amd64.iso>`_).
    - the Vitis `2020.2.2` suite (Vitis, Vivado, Vitis HLS) (`install instructions <https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis/2020-2.html>`_)
    - the ROS 2 Foxy distribution (`install instructions <https://docs.ros.org/en/foxy/Installation.html>`_)

.. admonition:: Dependencies

  KRS is served as a group of ROS 2 packages that you can install in any arbitrary ROS 2 workspace, enhancing it with hardware acceleration capabilities. The following demonstrates how to create a new ROS 2 overlay workspace and fetch the KRS packages, including some acceleration examples:
```

[![asciicast](https://asciinema.org/a/434953.svg)](https://asciinema.org/a/434953)

```shell
###################################################
# 0. install Vitis 2020.2.2, and ROS 2 Foxy,
      #  see above
###################################################

###################################################
# 1. install some dependencies you might be missing
###################################################
sudo apt-get -y install curl build-essential libssl-dev git wget \
                          ocl-icd-* opencl-headers python3-vcstool \
                          python3-colcon-common-extensions python3-colcon-mixin \
                          kpartx u-boot-tools pv

###################################################
# 2. create a new ROS 2 workspace
###################################################
mkdir -p ~/krs_ws/src; cd ~/krs_ws

###################################################
# 3. Create file with KRS alpha release
# TODO: this file should be fetched with wget once
#       KRS repository is publicly available
###################################################
cat << 'EOF' > krs_alpha.repos
repositories:
  acceleration/acceleration_firmware:
    type: git
    url: https://github.com/ros-acceleration/acceleration_firmware
    version: 0.4.0
  acceleration/acceleration_firmware_kv260:
    type: zip
    url: https://github.com/ros-acceleration/acceleration_firmware_kv260/releases/download/v0.6.0/acceleration_firmware_kv260.zip
  acceleration/colcon-acceleration:
    type: git
    url: https://github.com/ros-acceleration/colcon-acceleration
    version: 0.2.0
  acceleration/ros2acceleration:
    type: git
    url: https://github.com/ros-acceleration/ros2acceleration
    version: 0.2.0
  acceleration/ament_vitis:
    type: git
    url: https://github.com/ros-acceleration/ament_vitis
    version: 0.5.0
  acceleration/vitis_common:
    type: git
    url: https://github.com/ros-acceleration/vitis_common
    version: 0.1.0
EOF

###################################################
# 4. import repos of KRS alpha release
###################################################
vcs import src --recursive < krs_alpha.repos  # about 3 mins

###################################################
# 5. build the workspace and deploy firmware for hardware acceleration
###################################################
source /tools/Xilinx/Vitis/2020.2/settings64.sh  # source Xilinx tools
source /opt/ros/foxy/setup.bash  # Sources system ROS 2 installation.
# Note: The path above is valid if one installs ROS 2 from a pre-built
# package. If one builds ROS 2 from the source the directory might
# vary (e.g. ~/ros2_foxy/ros2-linux).
export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+
colcon build --merge-install  # about 2 mins

###################################################
# 6. source the overlay to enable all features
###################################################
source install/setup.bash
```

That's pretty much it, you've got now KRS installed in the `krs_ws` ROS overlay workspace.  You could also reproduce the same steps over an existing ROS 2 workspace if you'd like to avoid creating a new, or simply reusing the source code elsewhere.

Now's time to build and run some [examples](features/ros2centric).
