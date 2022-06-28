# 3. Offloading ROS 2 publisher

|   | Source code |
|---|----------|
| [`offloaded_doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/nodes/offloaded_doublevadd_publisher) | |
| kernel | [`vadd.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/offloaded_doublevadd_publisher/src/vadd.cpp) |
| publisher | [`accelerated_doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/offloaded_doublevadd_publisher/src/offloaded_doublevadd_publisher.cpp) |


```eval_rst
.. sidebar:: Before you begin
   
    This example builds on top of a prior one:

    - `0. ROS 2 publisher - doublevadd_publisher <0_ros2_publisher.html>`_, which runs completely on the scalar quad-core Cortex-A53 Application Processing Units (APUs) of the KV260 and is only able to publish at :code:`2.2 Hz`.

```

This example leverages KRS to offload the `vadd` function operations to the FPGA, showing how easy it is for ROS package maintainers to extend their packages to include hardware acceleration, and create deterministic kernels. The objective is to publish the resulting vector at 10 Hz.  

The source code is taken from `doublevadd_publisher` purposely *as is*, and HLS transforms the C++ code directly to RTL, creating a dedicated hardware circuit in the form of a kernel that offloads the CPU from the heavy vadd computations and provides deterministic responses.

Though deterministic, the resulting kernel computation is slower than its CPUs counterpart. The reason behind this is that the code is taken *as is*, and the kernel doesn't really exploit any parallelism, nor optimizes the computation flow. Given that the kernel clock (`4 ns`) is slower much than the one of the Arm CPUs, this leads altogether to an actual worse performance than the only-CPU case previously studied at [0. ROS 2 publisher](0_ros2_publisher/).

The ultimate objective of this example is to illustrate roboticists how **more deterministic (connected to real-time) does not necessarily lead to faster (lower latency) or better performance, quite the opposite**. Future examples will demonstrate how to achieve both, determinism and low latency.

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to `install <../install.html>`_.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## `offloaded_doublevadd_publisher`

The kernel is identitical to the one presented in [0. ROS 2 publisher](0_ros2_publisher/).

```cpp 
/*
        ____  ____
       /   /\/   /
      /___/  \  /   Copyright (c) 2021, Xilinx®.
      \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
       \   \
       /   /
      /___/   /\
      \   \  /  \
       \___\/\___\

Inspired by the Vector-Add example.
See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis

*/

#define DATA_SIZE 4096
// TRIPCOUNT identifier
const int c_size = DATA_SIZE;

extern "C" {
    void vadd(
            const unsigned int *in1,  // Read-Only Vector 1
            const unsigned int *in2,  // Read-Only Vector 2
            unsigned int *out,        // Output Result
            int size                  // Size in integer
            )
    {
        for (int j = 0; j < size; ++j) {  // stupidly iterate over
                                          // it to generate load
        #pragma HLS loop_tripcount min = c_size max = c_size
            for (int i = 0; i < size; ++i) {
            #pragma HLS loop_tripcount min = c_size max = c_size
              out[i] = in1[i] + in2[i];
            }
        }
    }
}
```

The only difference in this package is that it declares a kernel on its CMakeLists.txt file using the `vitis_acceleration_kernel` CMake macro:

```cmake 
# vadd kernel
vitis_acceleration_kernel(
  NAME vadd
  FILE src/vadd.cpp
  CONFIG src/kv260.cfg
  INCLUDE
    include
  TYPE
    hw
  PACKAGE
)
```

Let's build it:
```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
$ source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

# build the workspace to deploy KRS components
$ colcon build --merge-install  # about 2 mins

# source the workspace as an overlay
$ source install/setup.bash

# select kv260 firmware (in case you've been experimenting with something else)
$ colcon acceleration select kv260

# build offloaded_doublevadd_publisher
$ colcon build --build-base=build-kv260 --install-base=install-kv260 --merge-install --mixin kv260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration offloaded_doublevadd_publisher

# copy to KV260 rootfs, e.g.
$ scp -r install-kv260/* petalinux@192.168.1.86:/ros2_ws/
```

Since this package contains a kernel and we're using the Vitis `hw` build target (*more on Vitis build targets in future tutorials*), it'll take a bit longer to build the package. In an *AMD Ryzen 5 PRO 4650G* andit took **14 minutes**.

Note also the process is *slightly different* this time since we have an acceleration kernel. Before launching the binary in the CPUs, we need to load the kernel in the FPGA. For that, we'll be using some of the extensions KRS provides to the ROS 2 CLI tooling, particularly the `ros2 acceleration` suite:

!!! warning
    While you can re-arrange permissions and execute the following with the `petalinux` user, the simplest way forward is to execute as root.

```bash
$ sudo su
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation

$ . /ros2_ws/local_setup.bash     # source the ROS 2 overlay workspace we just 
                                  # created. Note it has been copied to the SD 
                                  # card image while being created.

# restart the daemon that manages the acceleration kernels
$ ros2 acceleration stop; ros2 acceleration start

# list the accelerators
$ ros2 acceleration list
                                       Accelerator           Type    Active
                                          kv260-dp       XRT_FLAT         1
                                              base       XRT_FLAT         0
                    offloaded_doublevadd_publisher       XRT_FLAT         0

# select the offloaded_doublevadd_publisher
$ ros2 acceleration select offloaded_doublevadd_publisher

# launch binary 
$ cd /ros2_ws/lib/offloaded_doublevadd_publisher
$ ros2 topic hz /vector_acceleration --window 10 &
$ ros2 run offloaded_doublevadd_publisher offloaded_doublevadd_publisher

[INFO] [1629663768.633315230] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 6'
[INFO] [1629663769.150109773] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 7'
average rate: 1.935
	min: 0.517s max: 0.517s std dev: 0.00010s window: 7
[INFO] [1629663769.666922955] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 8'
[INFO] [1629663770.183640105] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 9'
average rate: 1.935
	min: 0.517s max: 0.517s std dev: 0.00010s window: 9
[INFO] [1629663770.700318913] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 10'
[INFO] [1629663771.217068001] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 11'
average rate: 1.935
	min: 0.517s max: 0.517s std dev: 0.00010s window: 10
[INFO] [1629663771.733872538] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 12'
[INFO] [1629663772.250599612] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 13'
...
```

The publishing rate is `1.935 Hz`, which is lower than the `2.2 Hz` obtained in [0. ROS 2 publisher](0_ros2_publisher/). As introduced before and also in example [2. HLS in ROS 2](2_hls_ros2/), the rationale behind this is a combination of two aspects:
- First, the CPU clock is generally faster than the FPGA one, which means that pure offloading of operations (unless dataflow is optimized) are deterministic, but most of the time subject to be coherent with the slower clock.
- Second, the computation needs to be adapted to the dataflow and parallelism exploited (if available).