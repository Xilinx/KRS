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
    url: https://www.xilinx.com/bin/public/openDownload?filename=acceleration_firmware_kv260.zip

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
# 4. import repos of KRS 1.0 release
###################################################
vcs import src --recursive < krs_humble.repos  # about 3 mins in an AMD Ryzen 5 PRO 4650G

###################################################
# 5. build the workspace and deploy firmware for hardware acceleration
###################################################
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.

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

### Cross-compilation development

Cross-compilation of ROS 2 workspaces allows to build both CPU binaries as well as accelerators thanks to KRS packages. This capabilities is demonstrated below for the KR260 using Ubuntu 22.04 OS:

```shell
###################################################
# 0. install Vitis 2022.1 https://www.xilinx.com/support/download.html       
#   and ROS 2 Rolling https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
#    we recommend the Desktop-Full flavour (ros-humble-desktop-full)
###################################################

###################################################
# 1. install some dependencies you might be missing
#
# NOTE: gcc-multilib conflicts with Yocto/PetaLinux 2022.1 dependencies
# so you can't have both paths simultaneously enabled in a single
# development machine
###################################################
sudo apt-get -y install curl build-essential libssl-dev git wget \
                          ocl-icd-* opencl-headers python3-vcstool \
                          python3-colcon-common-extensions python3-colcon-mixin \
                          kpartx u-boot-tools pv gcc-multilib gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

###################################################
# 2. create a new ROS 2 workspace with examples and
#    firmware for KR260
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

  firmware/acceleration_firmware_kr260:
    type: zip
    url: https://github.com/ros-acceleration/acceleration_firmware_kr260/releases/download/v1.0.0/acceleration_firmware_kr260.zip

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
# 4. import repos of KRS 1.0 release
###################################################
vcs import src --recursive < krs_humble.repos  # about 3 mins in an AMD Ryzen 5 PRO 4650G

###################################################
# 5. build the workspace and deploy firmware for hardware acceleration
###################################################
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.

# Note: The path above is valid if one installs ROS 2 from a pre-built debian
# packages. If one builds ROS 2 from the source the directory might
# vary (e.g. ~/ros2_humble/ros2-linux).
export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

```

```eval_rst
.. warning:: 

    Next step is going to ask for the *sudo password*, make sure to enter it appropriately, otherwise the build
    will go on indefinitely.
```


```shell


colcon build --merge-install  # about 18 mins in an AMD Ryzen 5 PRO 4650G

###################################################
# 6. source the overlay to enable all features
###################################################
source install/setup.bash
```

```eval_rst
.. warning:: 

    Ubuntu 22.04 sysroot is not fully prepared for cross-compilation (but for native builds instead) and thereby, while invoking FindPython, it's just picking the host resources, instead of the target/sysroot ones, which leads to ROS 2 packages relying on Python 3 getting a dependency against the host (which doesn't exist), instead of against the sysroot.

    A **workaround** for this is symlinking the Python3 library of the host to the sysroot one, so that it gets picked while cross-compiling against the Ubuntu 22.04 sysroot. The following should do: `sudo ln -s ~/krs_ws/src/install/../acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/usr/lib/aarch64-linux-gnu/libpython3.10.so.1.0 /usr/lib/aarch64-linux-gnu/libpython3.10.so`)
```

``` shell
###################################################
# 7.A cross-compile and generate ONLY CPU binaries
###################################################
colcon build --build-base=build-kr260-ubuntu --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --cmake-args -DNOKERNELS=true

###################################################
# 7.B cross-compile and generate CPU binaries and accelerators
###################################################
colcon build --build-base=build-kr260-ubuntu --install-base=install-kr260-ubuntu --merge-install --mixin kr260
```

Now that we've built binaries and accelerators, next's to run some of them in hardware. See [examples](https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/0_ros2_publisher.html) but **note that Ubuntu 22.04 is targeting KR260 (and thereby the `--mixin kr260` should be used instead)**.

### Native (on target) development

```eval_rst
.. warning:: No accelerators produced with native (on-target) compilation

    **This path is helpful only for creating CPU binaries. It's not possible to create accelerators on target** (from within the KR/KV260 boards) because Vivado and Vitis tools have only x86 support and no aarch64 support is planned. Refer to the cross-compilation path for jointly creating binaries and accelerators.

```

Native CPU compilation (*on target*, in the KR260 or KV260) is pretty straightforward and can be performed by:

1. Create an SD card with [Ubuntu 22.04 official image for KR260](https://ubuntu.com/download/amd-xilinx)
2. Install ROS 2 Humble from .deb file inside KR260's Ubuntu 22.04 as indicated at https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
3. `scp` your ROS workspace into the embedded board and build it with colcon as if it was your development machine.

Now that we've built binaries, next's to run them in hardware. See [examples](https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/0_ros2_publisher.html).


### QEMU (emulation) development

```eval_rst
.. warning:: No accelerators produced with native (on-target) compilation

    **This path is helpful only for creating CPU binaries. It's not possible to create accelerators on QEMU** (from within emulated rootfs') because Vivado and Vitis tools have only x86 support and no aarch64 support is planned. Refer to the cross-compilation path for jointly creating binaries and accelerators.

```

CPU binaries can also be built (and tested) using hardware emulation through QEMU. In particular, the following provides a walkthrough on how to leverage Ubuntu 22.04 pre-built sysroot for KR260 to build the local development workspace:

```shell
###################################################
# 0. install Vitis 2022.1 https://www.xilinx.com/support/download.html       
#   and ROS 2 Rolling https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
#    we recommend the Desktop-Full flavour (ros-humble-desktop-full)
###################################################

###################################################
# 1. install some dependencies you might be missing
#
# NOTE: gcc-multilib conflicts with Yocto/PetaLinux 2022.1 dependencies
# so you can't have both paths simultaneously enabled in a single
# development machine
###################################################
sudo apt-get -y install curl build-essential libssl-dev git wget \
                          ocl-icd-* opencl-headers python3-vcstool \
                          python3-colcon-common-extensions python3-colcon-mixin \
                          kpartx u-boot-tools pv gcc-multilib

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

  firmware/acceleration_firmware_kr260:
    type: zip
    url: https://github.com/ros-acceleration/acceleration_firmware_kr260/releases/download/v1.0.0/acceleration_firmware_kr260.zip

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
# 4. import repos of KRS 1.0 release
###################################################
vcs import src --recursive < krs_humble.repos  # about 3 mins in an AMD Ryzen 5 PRO 4650G

###################################################
# 5. build the workspace and deploy firmware for hardware acceleration
###################################################
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.

# Note: The path above is valid if one installs ROS 2 from a pre-built debian
# packages. If one builds ROS 2 from the source the directory might
# vary (e.g. ~/ros2_humble/ros2-linux).
export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+
```

```eval_rst
.. warning:: 

    Next step is going to ask for the *sudo password*, make sure to enter it appropriately, otherwise the build
    will go on indefinitely.
```


```shell
colcon build --merge-install  # about 20 mins in an AMD Ryzen 5 PRO 4650G,
                              # mostly spent installing ROS 2 and deps. into
                              # the sysroot

###################################################
# 6. Enter Ubuntu 22.04 jail while mounting ROS 2 overlay workspace sources for native builds
#
# NOTE: assumes to be executed from the root of the ROS 2 overlay workspace
# (e.g. ~/krs_ws/)
###################################################
sudo mount --rbind --make-rslave /dev ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/dev
mkdir -p ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/ros2_ws/src; sudo mount --bind ~/krs_ws/src ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/ros2_ws/src
sudo mount -t proc none ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/proc
sudo mount -t sysfs none ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/sys
sudo mount -t tmpfs none ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/tmp
sudo mount -t tmpfs none ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/var/lib/apt
sudo mount -t tmpfs none ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/var/cache/apt
sudo mount -t tmpfs none ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/var/cache/apt
sudo cp /etc/resolv.conf ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/etc/resolv.conf

# enter chroot
sudo chroot ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux

###################################################
# 7. Build (in emulation) natively CPU binaries
###################################################
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.

# add open robotics repo to .deb sources
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ROS build tools (colcon et al.)
apt update && apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-docstrings \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

# build overlay workspace
cd /ros2_ws
colcon build --merge-install --packages-ignore acceleration_firmware_kr260 perception_3nodes

###################################################
# 8. Run one of the packages
###################################################
source /ros2_ws/install/local_setup.bash
ros2 run publisher_xilinx member_function_publisher

###################################################
# 8. Exit chroot and unmount things
###################################################
exit  # inside of the emulation

# back, in your development station
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/proc
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/sys
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/tmp
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/var/lib/apt
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/dev/mqueue
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/dev/hugepages
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/dev/shm
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/dev/pts
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/dev
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/ros2_ws/src
sudo umount ~/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/var/cache/apt

```

Now that we've built binaries, next's to run them in hardware. See [examples](https://xilinx.github.io/KRS/sphinx/build/html/docs/examples/0_ros2_publisher.html).
