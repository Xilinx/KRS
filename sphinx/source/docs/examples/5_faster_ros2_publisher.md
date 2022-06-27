# 5. Faster ROS 2 publisher

|   | Source code |
|---|----------|
| [`faster_doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/nodes/faster_doublevadd_publisher) | |
| kernel | [`vadd.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/faster_doublevadd_publisher/src/vadd.cpp) |
| publisher | [`faster_doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/faster_doublevadd_publisher/src/faster_doublevadd_publisher.cpp) |

```eval_rst
.. sidebar:: Before you begin
   
    Past examples of this series include: 

    - `4. Accelerated ROS 2 publisher - offloaded_doublevadd_publisher <4_accelerated_ros2_publisher.html>`_, which offloads and accelerates the `vadd` operation to the FPGA, optimizing the dataflow and leading to a deterministic vadd operation with an improved publishing rate of :code:`6.3 Hz`.
    - `3. Offloading ROS 2 publisher - offloaded_doublevadd_publisher <3_offloading_ros2_publisher.html>`_, which offloads the `vadd` operation to the FPGA and leads to a deterministic vadd operation, yet insuficient overall publishing rate of :code:`1.935 Hz`.
    - `0. ROS 2 publisher - doublevadd_publisher <0_ros2_publisher.html>`_, which runs completely on the scalar quad-core Cortex-A53 Application Processing Units (APUs) of the KV260 and is only able to publish at :code:`2.2 Hz`.

```


This example is the last one of the *ROS 2 publisher series*. It features a trivial vector-add ROS 2 publisher, which adds two vector inputs in a loop, and tries to publish the result at 10 Hz. This example will leverage KRS to produce an acceleration kernel that a) optimizes the dataflow and b) leverages parallelism via loop unrolling to meet the initial goal established by the `doublevadd_publisher`.

- [4. Accelerated ROS 2 publisher - `offloaded_doublevadd_publisher`](4_accelerated_ros2_publisher/), which offloads and accelerates the `vadd` operation to the FPGA, optimizing the dataflow and leading to a deterministic vadd operation with an improved publishing rate of `6.3 Hz`.
- [3. Offloading ROS 2 publisher - `offloaded_doublevadd_publisher`](3_offloading_ros2_publisher/), which offloads the `vadd` operation to the FPGA and leads to a deterministic vadd operation, yet insuficient overall publishing rate of `1.935 Hz`.
- [0. ROS 2 publisher - `doublevadd_publisher`](0_ros2_publisher/), which runs completely on the scalar quad-core Cortex-A53 Application Processing Units (APUs) of the KV260 and is only able to publish at `2.2 Hz`.


```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to `install <../install.html>`_.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## `accelerated_doublevadd_publisher`

Let's take a look at the [kernel source code](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/faster_doublevadd_publisher/src/vadd.cpp) first:

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
#pragma HLS INTERFACE m_axi port=in1 bundle=aximm1
#pragma HLS INTERFACE m_axi port=in2 bundle=aximm2
#pragma HLS INTERFACE m_axi port=out bundle=aximm1

        for (int j = 0; j <size; ++j) {  // stupidly iterate over
                                          // it to generate load
        #pragma HLS loop_tripcount min = c_size max = c_size
            for (int i = 0; i<(size/16)*16; ++i) {
            #pragma HLS UNROLL factor=16
            #pragma HLS loop_tripcount min = c_size max = c_size
              out[i] = in1[i] + in2[i];
            }
        }
    }
}
```

Besides the dataflow optimizations between input and output arguments in the PL-PS interaction, *line 37* utilizes a pragma to unroll the inner `for` loop by a factor of `16`, executing `16` sums *in parallel*, within the same clock cycle. The value of `16` is not arbitrary, but selected specifically to consume the whole bandwidth (`512-bits`) of the `m_axi` ports at each clock cycle available after previous dataflow optimizations (see [4. Accelerated ROS 2 publisher](4_accelerated_ros2_publisher/) to understand more about the dataflow optimizations). To fill in `512` bits, we pack together `16 unsigned int` inputs, each of `4 bytes`:

```eval_rst
.. math::

   \texttt{16 unsigned int}  \cdot 4 \frac{bytes}{\texttt{unsigned int}} \cdot 8 \frac{bits}{byte} = 512 \texttt{ bits}
```

Altogether, this leads to the most optimized form of the `vadd` kernel, *delivering both dataflow optimizations and code parallelism*, which is **able to successfully meet the publishing target of 10 Hz**. Overall, when compared to the initial example [0. ROS 2 publisher - `doublevadd_publisher`](0_ros2_publisher/), the one presented in here obtains a **speedup of 5x** (*In fact, the speedup is higher than 5x however the ROS 2 `WallRate` instance is set to `100ms`, so the kernel is idle waiting for new data to arrive, discarding further acceleration opportunities.*).


Let's build it:
```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
$ source /opt/ros/rolling/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

# build the workspace to deploy KRS components
$ colcon build --merge-install  # about 2 mins

# source the workspace as an overlay
$ source install/setup.bash

# select kv260 firmware (in case you've been experimenting with something else)
$ colcon acceleration select kv260

# build faster_doublevadd_publisher
$ colcon build --build-base=build-kv260 --install-base=install-kv260 --merge-install --mixin kv260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration faster_doublevadd_publisher

# copy to KV260 rootfs, e.g.
$ scp -r install-kv260/* petalinux@192.168.1.86:/ros2_ws/
```

and run it:

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
                  accelerated_doublevadd_publisher       XRT_FLAT         0
                                          kv260-dp       XRT_FLAT         1
                                              base       XRT_FLAT         0
                    offloaded_doublevadd_publisher       XRT_FLAT         0
                       faster_doublevadd_publisher       XRT_FLAT         0                    

# select the faster_doublevadd_publisher
$ ros2 acceleration select faster_doublevadd_publisher

# launch binary 
$ cd /ros2_ws/lib/faster_doublevadd_publisher
$ ros2 topic hz /vector_acceleration --window 10 &
$ ros2 run faster_doublevadd_publisher faster_doublevadd_publisher
INFO: Found Xilinx Platform
INFO: Loading 'vadd.xclbin'
[INFO] [1629669100.348149277] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 0'
[INFO] [1629669100.437331164] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 1'
[INFO] [1629669100.626349680] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 2'
[INFO] [1629669100.737320080] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 3'
[INFO] [1629669100.837296068] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 4'
[INFO] [1629669100.937279027] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 5'
[INFO] [1629669101.037308705] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 6'
[INFO] [1629669101.137309244] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 7'
[INFO] [1629669101.237301062] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 8'
[INFO] [1629669101.337311561] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 9'
[INFO] [1629669101.437294539] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 10'
[INFO] [1629669101.537323708] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 11'
average rate: 9.091
	min: 0.100s max: 0.189s std dev: 0.02659s window: 10
[INFO] [1629669101.637296386] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 12'
[INFO] [1629669101.737290495] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 13'
[INFO] [1629669101.837324683] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 14'
[INFO] [1629669101.937318152] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 15'
[INFO] [1629669102.037294040] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 16'
[INFO] [1629669102.137285269] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 17'
[INFO] [1629669102.237289977] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 18'
[INFO] [1629669102.337286815] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 19'
[INFO] [1629669102.437316744] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 20'
[INFO] [1629669102.537339583] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 21'
[INFO] [1629669102.637276821] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 22'
average rate: 10.001
	min: 0.100s max: 0.100s std dev: 0.00007s window: 10
[INFO] [1629669102.737308289] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 23'
[INFO] [1629669102.837287528] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 24'
[INFO] [1629669102.937298656] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 25'
[INFO] [1629669103.037285145] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 26'
[INFO] [1629669103.137287133] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 27'
[INFO] [1629669103.237286742] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 28'
[INFO] [1629669103.337307070] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 29'
[INFO] [1629669103.437327789] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 30'
[INFO] [1629669103.537345337] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 31'
[INFO] [1629669103.637311146] [faster_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 32'
average rate: 10.000
	min: 0.100s max: 0.100s std dev: 0.00004s window: 10
...
```
