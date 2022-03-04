# 4. Accelerated ROS 2 publisher

|   | Source code |
|---|----------|
| [`accelerated_doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/nodes/accelerated_doublevadd_publisher) | |
| kernel | [`vadd.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/accelerated_doublevadd_publisher/src/vadd.cpp) |
| publisher | [`accelerated_doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/accelerated_doublevadd_publisher/src/accelerated_doublevadd_publisher.cpp) |


```eval_rst
.. sidebar:: Before you begin
   
    This example builds on top of two prior ones: 

    - `3. Offloading ROS 2 publisher - offloaded_doublevadd_publisher <3_offloading_ros2_publisher.html>`_, which offloads the `vadd` operation to the FPGA and leads to a deterministic vadd operation, yet insuficient overall publishing rate of :code:`1.935 Hz`.
    - `0. ROS 2 publisher - doublevadd_publisher <0_ros2_publisher.html>`_, which runs completely on the scalar quad-core Cortex-A53 Application Processing Units (APUs) of the KV260 and is only able to publish at :code:`2.2 Hz`.

```


This example leverages KRS to offload and accelerate the `vadd` function operations to the FPGA, showing how easy it is for ROS package maintainers to extend their packages, include hardware acceleration and create deterministic kernels. The objective is to publish the resulting vector at 10 Hz. 

This example upgrades the previous offloading operation at [3. Offloading ROS 2 publishe - `offloaded_doublevadd_publisher`](3_offloading_ros2_publisher/), and includes an optimization for the dataflow. This allows the publisher to improve its publishing rate from 2 Hz, up to 6 Hz. For that, the HLS `INTERFACE` pragma is used. The HLS INTERFACE specifies how RTL ports are created from the function definition during interface synthesis. Sharing ports helps save FPGA resources by eliminating AXI interfaces, but it can limit the performance of the kernel because all the memory transfers have to go through a single port. The `m_axi` port has  independent READ and WRITE channels, so with a single m_axi port, we can do reads and writes simultaneously but since the kernel (`vadd`) has two vectors from where its reading (simultaneously), we can optimize the dataflows by simply asking for an extra AXI interface.
    
After this dataflow optimization in the kernel, the `accelerated_doublevadd_publisher` ROS 2 package is able to publish at 6 Hz. *For a faster kernel that meets the 10 Hz goal, refer to [5. Faster ROS 2 publisher](5_faster_ros2_publisher/)*.

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to `install <../install.html>`_.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## `accelerated_doublevadd_publisher`

Let's take a look at the kernel source code this time:

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

Note the aforementioned `INTERFACE` pragma as the only relevant change introduced, which specifies how RTL ports are created from the function definition during interface synthesis.

```eval_rst

.. admonition:: Short explanation of the `HLS INTERFACE` pragma

    The parameters of the software functions defined in a HLS design are synthesized into ports in the RTL code [1]_ that group multiple signals to encapsulate the communication protocol between the HLS design and things external to the design.  The :code:`HLS INTERFACE` specifies how RTL ports are created from the function definition during interface synthesis [2]_. Sharing ports helps save FPGA resources by eliminating AXI interfaces, but it can limit the performance of the kernel because all the memory transfers have to go through a single port. The :code:`m_axi` port has independent READ and WRITE channels, so with a single :code:`m_axi` port, we can do reads and writes simultaneously but since we have two vectors from where we're reading (simultaneously), we can optimize the dataflows by simply asking for an extra AXI interface. 
    
    Note the bandwidth and throughput of the kernel can be increased by creating multiple  ports, using different bundle names, to connect multiple memory banks but this cames at the cost of resources in the PL fabric.
    
    
    To understand this better, it's important to also understand that in RTL design, input and output operations must be performed through a port in the design interface and typically operate using a specific I/O (input-output) protocol. The implementation of a function-level protocol is indicated in :code:`<mode>` after the :code:`#pragma HLS INTERFACE <mode>`. In this case, the pragma is using the :code:`m_axi` mode which corresponds with all ports as an AXI4 interface. The complete syntax of the pragma is as follows:

    :code:`#pragma HLS interface <mode> port=<name> bundle=<string>`

    where:

    - `<mode>` corresponds with the function-level protocol for the input/output operations through the RTL port
    - `<port>` specifies the name of the function argument, return value or global variable the pragma applies to
    - `<bundle>` groups function arguments into AXI interface ports. By default, HLS groups all function arguments specified as an AXI4 (:code:`m_axi` mode) interface into a single AXI4 port. This option explicitly groups all interface ports with the same :code:`bundle=<string>` into the same AXI interface port and names the RTL port the value specified by :code:`<string>`.

    In other words, the pragmas below define the :code:`INTERFACE` standards for the RTL ports of the :code:`vadd` function.
```

Let's build it:
```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2021.2/settings64.sh  # source Xilinx tools
$ source /opt/ros/rolling/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

# build the workspace to deploy KRS components
$ colcon build --merge-install  # about 2 mins

# source the workspace as an overlay
$ source install/setup.bash

# select kv260 firmware (in case you've been experimenting with something else)
$ colcon acceleration select kv260

# build accelerated_doublevadd_publisher
$ colcon build --build-base=build-kv260 --install-base=install-kv260 --merge-install --mixin kv260 --packages-select ament_acceleration ament_vitis vitis_common ros2acceleration accelerated_doublevadd_publisher

# copy to KV260 rootfs, e.g.
$ scp -r install-kv260/* petalinux@192.168.1.86:/ros2_ws/
```

and run it:

```eval_rst
.. warning:: Before you begin
    
    Due to a bug in the daemon client used underneath the ROS 2 CLI extensions, the following is likely to happen:

    .. code:: bash

        xilinx-k26-starterkit-2020_2:/lib/firmware/xilinx# ros2 acceleration select accelerated_doublevadd_publisher
        Removing accel /lib/firmware/xilinx/kv260-dp
        *** buffer overflow detected ***: dfx-mgr-client terminated
    

    This overflow is caused by a fixed buffer which is getting overflowed due to an accelerator name that's longer than expected. **This should be addressed in future firmware releases**.

    For the time being, a quick patch involves changing the name of the accelerator manually and loading it with that different name. E.g.:
    
    .. code:: bash

        cd /lib/firmware/xilinx/
        mv accelerated_doublevadd_publisher shorter
        ros2 acceleration stop; ros2 acceleration start
        ros2 acceleration select shorter
        # and then launch the ROS 2 package as usual

```

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

# select the offloaded_doublevadd_publisher
# NOTE: see troubleshooting note above, implement
#       countermeasure if necessary
$ ros2 acceleration select accelerated_doublevadd_publisher

# launch binary 
$ cd /ros2_ws/lib/accelerated_doublevadd_publisher
$ ros2 topic hz /vector_acceleration --window 10 &
$ ros2 run accelerated_doublevadd_publisher accelerated_doublevadd_publisher

ros2 run faster_doublevadd_publisher faster_doublevadd_publisher

...
[INFO] [1629667329.650547207] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 9'
[INFO] [1629667329.850627179] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 10'
[INFO] [1629667329.988200464] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 11'
[INFO] [1629667330.125859198] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 12'
[INFO] [1629667330.263443133] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 13'
[INFO] [1629667330.463512045] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 14'
[INFO] [1629667330.601060629] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 15'
average rate: 6.396
	min: 0.137s max: 0.200s std dev: 0.02870s window: 10
[INFO] [1629667330.738635863] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 16'
[INFO] [1629667330.876191868] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 17'
[INFO] [1629667331.076263260] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 18'
[INFO] [1629667331.213853284] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 19'
[INFO] [1629667331.351435218] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 20'
[INFO] [1629667331.488997143] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 21'
[INFO] [1629667331.689110265] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 22'
average rate: 6.397
	min: 0.137s max: 0.200s std dev: 0.02871s window: 10
[INFO] [1629667331.826752610] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 23'
[INFO] [1629667331.964359824] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 24'
[INFO] [1629667332.101934778] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 25'
[INFO] [1629667332.302034131] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 26'
[INFO] [1629667332.439673035] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 27'
[INFO] [1629667332.577249279] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 28'
[INFO] [1629667332.714848544] [accelerated_doublevadd_publisher]: Publishing: 'vadd finished, iteration: 29'
average rate: 6.662
	min: 0.138s max: 0.200s std dev: 0.02507s window: 10
...
```

The optimizations in the dataflow introduced in the kernel via the use of the HLS `INTERFACE` pragma lead to a `6.3 Hz` publishing rate, which is about 3 times better than what was obtained before, but still insuficient to meet the `10 Hz` goal.

Next, in [5. Faster ROS 2 publisher](5_faster_ros2_publisher/), we'll review an even faster kernel that meets the 10 Hz publishing goal, by optimizing both the dataflow (as in this example) and exploiting `vadd` parallelism (via loop unrolling).

<!-- references -->
[^1]: 
[^2]: 


```eval_rst

.. [1] Managing Interface Synthesis. Retrieved from https://www.xilinx.com/html_docs/xilinx2020_2/vitis_doc/managing_interface_synthesis.html#jro1585107736856

.. [2] pragma HLS interface. Retrieved from https://www.xilinx.com/html_docs/xilinx2021_1/vitis_doc/hls_pragmas.html#jit1504034365862.

```
