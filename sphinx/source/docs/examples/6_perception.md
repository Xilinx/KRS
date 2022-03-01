# 6. Acceleration a perception computational graph

|   | Source code |
|---|----------|
| [`faster_doublevadd_publisher`](https://github.com/ros-acceleration/acceleration_examples/tree/main/faster_doublevadd_publisher) | |
| kernel | [`vadd.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/faster_doublevadd_publisher/src/vadd.cpp) |
| publisher | [`faster_doublevadd_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/faster_doublevadd_publisher/src/faster_doublevadd_publisher.cpp) |

```eval_rst
.. sidebar:: Before you begin
   
    Past examples of this series include: 

    - `4. Accelerated ROS 2 publisher - offloaded_doublevadd_publisher <4_accelerated_ros2_publisher.html>`_, which offloads and accelerates the `vadd` operation to the FPGA, optimizing the dataflow and leading to a deterministic vadd operation with an improved publishing rate of :code:`6.3 Hz`.
    - `3. Offloading ROS 2 publisher - offloaded_doublevadd_publisher <3_offloading_ros2_publisher.html>`_, which offloads the `vadd` operation to the FPGA and leads to a deterministic vadd operation, yet insuficient overall publishing rate of :code:`1.935 Hz`.
    - `0. ROS 2 publisher - doublevadd_publisher <0_ros2_publisher.html>`_, which runs completely on the scalar quad-core Cortex-A53 Application Processing Units (APUs) of the KV260 and is only able to publish at :code:`2.2 Hz`.

```
