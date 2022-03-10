# Install KRS

```eval_rst
.. important::
    **KRS "alpha" release**

    KRS alpha **has only been tested in Ubuntu 20.04**. It assumes the following is installed in your workstation:

    - `Ubuntu 20.04` Focal Fossa operating system (`download <https://releases.ubuntu.com/20.04/ubuntu-20.04.3-desktop-amd64.iso>`_).
    - the Vitis `2020.2.2` suite (Vitis, Vivado, Vitis HLS) (`install instructions <https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis/2020-2.html>`_)
    - the ROS 2 Foxy distribution (`install instructions <https://docs.ros.org/en/foxy/Installation.html>`_)

    For KRS alpha documentation, refer to https://github.com/vmayoral/KRS/tree/krs-alpha.

.. important::

    **KRS "beta" release**

    KRS beta **has only been tested in Ubuntu 20.04**. It assumes the following is installed in your workstation:

    - `Ubuntu 20.04` Focal Fossa operating system (`download <https://releases.ubuntu.com/20.04/ubuntu-20.04.3-desktop-amd64.iso>`_).
    - the Vitis `2021.2` suite (Vitis, Vivado, Vitis HLS) (`install instructions <https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis.html>`_)
    - the ROS 2 Rolling distribution (`install instructions <https://docs.ros.org/en/rolling/Installation.html>`_)

.. admonition:: Dependencies

  KRS is served as a group of ROS 2 packages that you can install in any arbitrary ROS 2 workspace, enhancing it with hardware acceleration capabilities. The following demonstrates how to create a new ROS 2 overlay workspace and fetch the KRS packages, including some acceleration examples:
```

[![asciicast](https://asciinema.org/a/434953.svg)](https://asciinema.org/a/434953)

```shell
###################################################
# 0. install Vitis 2021.2, and ROS 2 Rolling,
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
# 3. Create file with KRS beta release
# TODO: this file should be fetched with wget once
#       KRS repository is publicly available
###################################################
cat << 'EOF' > krs_rolling.repos
repositories:
  ros2/ament_lint:
    type: git
    url: https://github.com/ament/ament_lint
    version: master
  ros2/launch:
    type: git
    url: https://github.com/ros2/launch
    version: master
  ros2/gazebo_ros_pkgs:
    type: git
    url: https://github.com/ros-simulation/gazebo_ros_pkgs
    version: da14a69e79502cd08064ccd261366ff023a9162a
  
  perception/image_common:
    type: git
    url: https://github.com/ros-perception/image_common
    version: 9729de81f7dff6156f644d6152b200f687360f1f
  perception/image_pipeline:
    type: git
    url: https://github.com/ros-acceleration/image_pipeline
    version: beta
  perception/vision_opencv:
    type: git
    url: https://github.com/ros-perception/vision_opencv
    version: 7bbc5ecc232e8faf36b45efaa2b6bc979b04157f

  tracing/ros2_tracing:
    type: git
    url: https://gitlab.com/ros-tracing/ros2_tracing.git
    version: master
  tracing/tracetools_acceleration:
    type: git
    url: https://github.com/ros-acceleration/tracetools_acceleration
    version: main

  acceleration/acceleration_firmware:
    type: git
    url: https://github.com/ros-acceleration/acceleration_firmware
    version: main
  acceleration/acceleration_firmware_kv260:
    type: zip
    url: https://github.com/ros-acceleration/acceleration_firmware_kv260/releases/download/v0.9.0/acceleration_firmware_kv260.zip
  acceleration/adaptive_component:
    type: git
    url: https://github.com/ros-acceleration/adaptive_component
    version: main
  acceleration/ament_acceleration:
    type: git
    url: https://github.com/ros-acceleration/ament_acceleration
    version: main
  acceleration/ament_vitis:
    type: git
    url: https://github.com/ros-acceleration/ament_vitis
    version: beta
  acceleration/colcon-acceleration:
    type: git
    url: https://github.com/ros-acceleration/colcon-acceleration
    version: main
  acceleration/ros2_kria:
    type: git
    url: https://github.com/ros-acceleration/ros2_kria
    version: main
  acceleration/ros2acceleration:
    type: git
    url: https://github.com/ros-acceleration/ros2acceleration
    version: main
  acceleration/vitis_common:
    type: git
    url: https://github.com/ros-acceleration/vitis_common
    version: master
  acceleration/acceleration_examples:
    type: git
    url: https://github.com/ros-acceleration/acceleration_examples
    version: beta

EOF

###################################################
# 4. import repos of KRS beta release
###################################################
vcs import src --recursive < krs_rolling.repos  # about 3 mins in an AMD Ryzen 5 PRO 4650G

###################################################
# 5. build the workspace and deploy firmware for hardware acceleration
###################################################
source /tools/Xilinx/Vitis/2021.2/settings64.sh  # source Xilinx tools
source /opt/ros/rolling/setup.bash  # Sources system ROS 2 installation.
# Note: The path above is valid if one installs ROS 2 from a pre-built
# package. If one builds ROS 2 from the source the directory might
# vary (e.g. ~/ros2_foxy/ros2-linux).
export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+
colcon build --merge-install  # about 4 mins in an AMD Ryzen 5 PRO 4650G

###################################################
# 6. source the overlay to enable all features
###################################################
source install/setup.bash
```

That's pretty much it, you've got now KRS installed in the `krs_ws` ROS overlay workspace.  You could also reproduce the same steps over an existing ROS 2 workspace if you'd like to avoid creating a new, or simply reusing the source code elsewhere.

Now's time to build and run some [examples](https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/0_ros2_publisher.html).
