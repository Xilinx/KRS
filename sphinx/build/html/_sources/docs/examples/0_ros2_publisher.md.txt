# 0. ROS 2 publisher

|   | Source code |
|---|----------|
| [`vadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/nodes/vadd_publisher) | |
| publisher | [`vadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/vadd_publisher/src/vadd_publisher.cpp) |
| [`doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/nodes/doublevadd_publisher) | |
| publisher | [`doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/doublevadd_publisher/src/doublevadd_publisher.cpp) |


This first example presents a trivial vector-add ROS 2 publisher, which adds two vector inputs in a loop, and tries to publish the result at 10 Hz. The ROS 2 package runs in the scalar processors (the CPUs). Step-by-step, the process is documented, walking through the different actions required to run the ROS 2 package in hardware, leveraging KRS capabilities. Afterwards, a slighly modified version of the publisher is presented which has additional computation demands. With these modifications, it becomes clear how the publisher isn't able to meet the publication goal anymore, which motivates the use of hardware acceleration.

The ultimate objective of this example is to generate a simple ROS 2-centric example that creates a CPU baseline to understand the value of hardware acceleration and how KRS facilitates it. Next examples will build upon this one.

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to `install <../install.html>`_.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```

## `vadd_publisher`

<!-- KRS aims to provide a ROS 2-centric experience (see [here](/features/ros2centric/) for more) and instead of using external tools to build ROS 2 workspaces, extensions to ROS 2 build system (`ament`) and ROS build tools (`colcon`) are implemented which facilitate the process. -->
### Prepare the environment and fetch the example

We start by preparing the environment and fetching the source code of the example into our KRS workspace:
```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
$ source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

# build the workspace
$ colcon build --merge-install  # about 4 mins

# source the workspace as an overlay
$ source install/setup.bash
```

### Inspecting the ROS 2 publisher

The publisher is a CPU-based average one. The source code of the publisher has been split between the `vadd` function ([vadd.cpp](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/vadd_publisher/src/vadd.cpp)) and the rest ([vadd_publisher.cpp](https://github.com/ros-acceleration/acceleration_examples/blob/main/nodes/vadd_publisher/src/vadd_publisher.cpp)) for simplicity.
The `vadd` (vector-add) function is as follows:

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
        for (int i = 0; i < size; ++i) {
        #pragma HLS loop_tripcount min = c_size max = c_size
            out[i] = in1[i] + in2[i];
        }
    }
}

```

The [`loop_tripcount`](https://www.xilinx.com/html_docs/xilinx2021_1/vitis_doc/hls_pragmas.html#sty1504034367099) is for analysis only and the pragma doesn't impact the function logic in any way. Instead, it allows HLS to identify how many iterations are expected in the loop to make time estimations. This will come handy if we want to run synthesis tests to estimate the timing it'll take for this function to run on a dedicated circuit in the FPGA.

### Building, creating the raw image and running in hardware

Let's build the example, create a raw SD card image and run it in the [`KV260`](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit.html) reference development platform. First, let's select the firmware for the target hardware, KV260:

```bash
$ colcon acceleration select kv260
```

To verify that we indeed that the right firmware selected, look for the one marked with a "*" at the end of its name:
```bash
$ colcon acceleration list
kv260*
```

Let's now build the package targeting the KV260:

```bash
$ colcon build --build-base=build-kv260 --install-base=install-kv260 --merge-install --mixin kv260 --packages-select ament_acceleration ament_vitis vadd_publisher
```

Let's now create a raw disk image for the SD card with PetaLinux's rootfs, a vanilla Linux 5.4.0 kernel and the ROS 2 overlay workspace we've just created for the KV260:
```bash
$ colcon acceleration linux vanilla --install-dir install-kv260
```

We're now ready to run it on hardware. For that, we need to flash the `~/krs_ws/acceleration/firmware/select/sd_card.img` file into the SD card. One quick way to do it is as follows:

```bash
# first, find out where your SD card has been mapped, in my case, /dev/rdisk2
$ sudo diskutil umount /dev/rdisk2s1  # umount mounted partition
$ pv <your-path-to>/krs_ws/acceleration/firmware/select/sd_card.img | sudo dd of=/dev/rdisk2 bs=4M  # dd the image
```

There are other methods. If you need help doing so, check out [these instructions](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit/kv260-getting-started/setting-up-the-sd-card-image.html) for different OSs. **Make sure to flash `~/krs_ws/acceleration/firmware/select/sd_card.img` we just generated, and not some other image**.


Once flashed, connect the board to the computer via its USB/UART/JTAG FTDI adapter and power it on. Then, launch your favority serial console (e.g. `sudo tio /dev/ttyUSB1`). You should get to the following prompt where you can use the `petalinux` username and define your own password:
```bash
PetaLinux 2020.2.2 xilinx-k26-starterkit-2020_2.2 ttyPS0

xilinx-k26-starterkit-2020_2 login:
```

Then, we can launch the example (feel free to split the actions below in various terminals, for convenience `byobu` allows to run it all in one):



```bash
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation

$ . /ros2_ws/local_setup.bash     # source the ROS 2 overlay workspace we just
                                  # created. Note it has been copied to the SD
                                  # card image while being created.

$ ros2 topic hz /vector --window 10 &
$ ros2 run vadd_publisher vadd_publisher
...
average rate: 10.000
	min: 0.100s max: 0.100s std dev: 0.00007s window: 10
[INFO] [1629649453.734865115] [vadd_publisher]: Publishing: 'vadd finished, iteration: 37'
[INFO] [1629649453.834855650] [vadd_publisher]: Publishing: 'vadd finished, iteration: 38'
[INFO] [1629649453.934853073] [vadd_publisher]: Publishing: 'vadd finished, iteration: 39'
[INFO] [1629649454.034854160] [vadd_publisher]: Publishing: 'vadd finished, iteration: 40'
[INFO] [1629649454.134853854] [vadd_publisher]: Publishing: 'vadd finished, iteration: 41'
[INFO] [1629649454.234855027] [vadd_publisher]: Publishing: 'vadd finished, iteration: 42'
[INFO] [1629649454.334856580] [vadd_publisher]: Publishing: 'vadd finished, iteration: 43'
[INFO] [1629649454.434858073] [vadd_publisher]: Publishing: 'vadd finished, iteration: 44'
[INFO] [1629649454.534854066] [vadd_publisher]: Publishing: 'vadd finished, iteration: 45'
[INFO] [1629649454.634857869] [vadd_publisher]: Publishing: 'vadd finished, iteration: 46'
[INFO] [1629649454.734853412] [vadd_publisher]: Publishing: 'vadd finished, iteration: 47'
average rate: 10.001
	min: 0.100s max: 0.100s std dev: 0.00006s window: 10
[INFO] [1629649454.834858295] [vadd_publisher]: Publishing: 'vadd finished, iteration: 48'
[INFO] [1629649454.934855449] [vadd_publisher]: Publishing: 'vadd finished, iteration: 49'
[INFO] [1629649455.034850672] [vadd_publisher]: Publishing: 'vadd finished, iteration: 50'
[INFO] [1629649455.134852075] [vadd_publisher]: Publishing: 'vadd finished, iteration: 51'
[INFO] [1629649455.234859537] [vadd_publisher]: Publishing: 'vadd finished, iteration: 52'
[INFO] [1629649455.334861150] [vadd_publisher]: Publishing: 'vadd finished, iteration: 53'
[INFO] [1629649455.434863893] [vadd_publisher]: Publishing: 'vadd finished, iteration: 54'
[INFO] [1629649455.534855537] [vadd_publisher]: Publishing: 'vadd finished, iteration: 55'
[INFO] [1629649455.634844142] [vadd_publisher]: Publishing: 'vadd finished, iteration: 56'
[INFO] [1629649455.734844195] [vadd_publisher]: Publishing: 'vadd finished, iteration: 57'
[INFO] [1629649455.834858327] [vadd_publisher]: Publishing: 'vadd finished, iteration: 58'
...
```

You should see that the ROS 2 publisher successfully publishes at approximately 10 Hz.

<!--

With byobu, instead:

```bash
cat << EOF > vadd_publisher.conf
new-session 'vadd_publisher'
bind-key -n C-b send-prefix
new-window -n vadd_publisher
send-keys 'source /usr/bin/ros_setup.bash; . /ros2_ws/local_setup.bash; ros2 run vadd_publisher vadd_publisher' Enter
split-window -h
send-keys 'source /usr/bin/ros_setup.bash; . /ros2_ws/local_setup.bash; ros2 topic hz /vector --window 10' Enter
EOF
byobu -f vadd_publisher.conf attach
```
-->


### Bonus: Getting a time intuition of a `vadd` acceleration kernel
What if instead of running the `vadd` in the scalar CPUs, we create a specific hardware circuit with the FPGA that runs this function? What'd would be the associated timing? In other words, a *robot chip* for the `vadd` computation.

Though going all the way down to implementing the harware cirtuit for `vadd` is beyond the scope of this example, we can get a quick intuition with HLS capabilities integrated in KRS. Let's get such intuition:

```bash
$ colcon acceleration hls --run --synthesis-report vadd_publisher
Found Tcl script "project_vadd_publisher.tcl" for package: vadd_publisher
Executing /home/xilinx/krs_ws/build-kv260/vadd_publisher/project_vadd_publisher.tcl
 Project:  project_vadd_publisher
 Path:  /home/xilinx/krs_ws/build-kv260/vadd_publisher/project_vadd_publisher
 	- Solution:  solution_4ns
 		- C Simulation:               Pass
 		- C Synthesis:                Run
 		- C/RTL Co-simulation:        Not Run
		- Export:
 			- IP Catalog:         Not Run
 			- System Generator:   Not Run
 			- Export Evaluation:  Not Run
		- Synthesis report: /home/xilinx/krs_ws/build-kv260/vadd_publisher/project_vadd_publisher/solution_4ns/syn/report/vadd_csynth.rpt


  			================================================================
  			== Vitis HLS Report for 'vadd'
  			================================================================
  			* Date:           Sun Aug 22 18:24:43 2021

  			* Version:        2020.2.2 (Build 3118627 on Tue Feb  9 05:13:49 MST 2021)
  			* Project:        project_vadd_publisher
  			* Solution:       solution_4ns (Vitis Kernel Flow Target)
  			* Product family: zynquplus
  			* Target device:  xck26-sfvc784-2LV-c


  			================================================================
  			== Performance Estimates
  			================================================================
  			+ Timing:
  			    * Summary:
  			    +--------+---------+----------+------------+
  			    |  Clock |  Target | Estimated| Uncertainty|
  			    +--------+---------+----------+------------+
  			    |ap_clk  |  4.00 ns|  2.920 ns|     1.08 ns|
  			    +--------+---------+----------+------------+

  			+ Latency:
  			    * Summary:
  			    +---------+---------+----------+-----------+-----+------+---------+
  			    |  Latency (cycles) |  Latency (absolute)  |  Interval  | Pipeline|
  			    |   min   |   max   |    min   |    max    | min |  max |   Type  |
  			    +---------+---------+----------+-----------+-----+------+---------+
  			    |        2|     8334|  8.000 ns|  33.336 us|    3|  8335|     none|
  			    +---------+---------+----------+-----------+-----+------+---------+

  			    + Detail:
  			        * Instance:
  			        N/A

  			        * Loop:
  			        +-------------------+---------+---------+----------+-----------+-----------+------+----------+
  			        |                   |  Latency (cycles) | Iteration|  Initiation Interval  | Trip |          |
  			        |     Loop Name     |   min   |   max   |  Latency |  achieved |   target  | Count| Pipelined|
  			        +-------------------+---------+---------+----------+-----------+-----------+------+----------+
  			        |- VITIS_LOOP_30_1  |     8264|     8264|        75|          2|          1|  4096|       yes|
  			        +-------------------+---------+---------+----------+-----------+-----------+------+----------+



  			================================================================
  			== Utilization Estimates
  			================================================================
  			* Summary:
  			+-----------------+---------+------+--------+--------+-----+
  			|       Name      | BRAM_18K|  DSP |   FF   |   LUT  | URAM|
  			+-----------------+---------+------+--------+--------+-----+
  			|DSP              |        -|     -|       -|       -|    -|
  			|Expression       |        -|     -|       0|     280|    -|
  			|FIFO             |        -|     -|       -|       -|    -|
  			|Instance         |        2|     -|     796|    1068|    -|
  			|Memory           |        -|     -|       -|       -|    -|
  			|Multiplexer      |        -|     -|       -|     472|    -|
  			|Register         |        -|     -|     623|      32|    -|
  			+-----------------+---------+------+--------+--------+-----+
  			|Total            |        2|     0|    1419|    1852|    0|
  			+-----------------+---------+------+--------+--------+-----+
  			|Available        |      288|  1248|  234240|  117120|   64|
  			+-----------------+---------+------+--------+--------+-----+
  			|Utilization (%)  |       ~0|     0|      ~0|       1|    0|
  			+-----------------+---------+------+--------+--------+-----+

...

```

The output will show you that the `vadd` function with all its complexity can run with a `4 ns` clock (we could ask for other clocks) in a total of `33.336 us`. This result is **completely deterministic** from the vadd viewpoint, after all, the FPGA would create a specialized hardware cirtuit for the `vadd` computations. For what concerns vadd, there's no more deterministic execution than this. This is just a brief introduction, *if you wish to learn more about HLS and KRS, head to the second example: [1. Hello Xilinx](1_hello_xilinx/)*.

## `doublevadd_publisher`

We've seen that as a simple ROS 2 package, `vadd_publisher` runs perfectly fine and meets the 10 Hz publishing objective. But what if the `vadd` has bigger vectors, or has operations that involve many more iterations? This second section explores this with a more computationally expensive publisher, the `doublevadd_publisher`. The source code of the new `vadd` function is presented below:

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

Note how instead of one `for` loop, we now have two, simulating a more complex computation. Let's try this out in hardware. Provided that the image was previously created, we just need to replace the ROS 2 workspace, the rest should be identical. You can do this in various ways (physically mounting the raw image in the SD card, `scp`-ing the ROS 2 workspace, etc.).

Briefly, in the workstation:
```bash
# generate the workspace with doublevadd_publisher (if exists already, add to it)
$ colcon build --build-base=build-kv260 --install-base=install-kv260 --merge-install --mixin kv260 --packages-select ament_vitis doublevadd_publisher

# copy to rootfs in SD card, e.g.
$ scp -r install-kv260/* petalinux@192.168.1.86:/ros2_ws/

# Launch doublevadd_publisher
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation

$ . /ros2_ws/local_setup.bash     # source the ROS 2 overlay workspace we just
                                # created. Note it has been copied to the SD
                                # card image while being created.

$ ros2 topic hz /vector --window 10 &
$ ros2 run doublevadd_publisher doublevadd_publisher

...
[INFO] [1629656740.225647854] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 11'
[INFO] [1629656740.675023646] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 12'
[INFO] [1629656741.124260679] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 13'
average rate: 2.226
	min: 0.449s max: 0.449s std dev: 0.00011s window: 10
[INFO] [1629656741.573462323] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 14'
[INFO] [1629656742.022713366] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 15'
[INFO] [1629656742.471917079] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 16'
average rate: 2.226
	min: 0.449s max: 0.449s std dev: 0.00011s window: 10
[INFO] [1629656742.921175382] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 17'
[INFO] [1629656743.370423695] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 18'
[INFO] [1629656743.819651828] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 19'
average rate: 2.226
	min: 0.449s max: 0.449s std dev: 0.00010s window: 10
[INFO] [1629656744.268874211] [doublevadd_publisher]: Publishing: 'vadd finished, iteration: 20'
...
```

This new publisher achieves only `2.2 Hz`, quite far from the `10 Hz` targeted. Using hardware acceleration, future examples will demonstrate how to build a custom compute pipeline that offloads computations to a kernel. *If you wish to jump directly into hardware acceleration with `doublevadd_publisher`, head to: [3. Offloading ROS 2 publisher](3_offloading_ros2_publisher)*.


## Troubleshooting

### gzip: stdin: unexpected end of file

See https://github.com/ros-acceleration/colcon-acceleration/issues/9.