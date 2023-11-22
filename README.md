<h1 align="center">KRIA ROBOTICS STACK (KRS) DOCUMENTATION</h1>

## Introduction
This repository contains the source code of KRS documentation which gets rendered to sphinx pages. To view complete documentation click on the link below.

[KRS Landing Page](https://xilinx.github.io/KRS)

## About KRS
- **Introduction to KRS**
  - [Kria Robotics Stack (KRS)](#) (this page)
  - [Install KRS](sphinx/source/docs/install.md)
  - [Hardware supported](sphinx/source/docs/hardware.md)
- **Features**
  - [ROS 2-centric](sphinx/source/docs/features/ros2centric.md)
  - [Real-time ROS 2](sphinx/source/docs/features/realtime_ros2.md)
  - [ROS 2 Accelerated Apps](sphinx/source/docs/features/accelerated_apps_ros2.md)
  - [Contributing back to ROS 2](sphinx/source/docs/features/contributing_ros2.md)
- **Examples**
  - [0. ROS 2 publisher](sphinx/source/docs/examples/0_ros2_publisher.md)
  - [1. Hello Xilinx](sphinx/source/docs/examples/1_hello_xilinx.md)
  - [2. HLS in ROS 2](sphinx/source/docs/examples/2_hls_ros2.md)
  - [3. Offloading ROS 2 publisher](sphinx/source/docs/examples/3_offloading_ros2_publisher.md)
  - [4. Accelerated ROS 2 publisher](sphinx/source/docs/examples/4_accelerated_ros2_publisher.md)
  - [5. Faster ROS 2 publisher](sphinx/source/docs/examples/5_faster_ros2_publisher.md)
- **Troubleshooting and definitions**
  - [Troubleshooting](sphinx/source/docs/howto.md)
  - [Features](sphinx/source/docs/other/definitions.md)

The **Kria Robotics Stack (KRS)** is a ROS 2 superset for industry, an integrated set of robot libraries and utilities to accelerate the development, maintenance and commercialization of industrial-grade robotic solutions while using [adaptive computing](https://www.xilinx.com/applications/adaptive-computing.html). KRS provides to ROS 2 users an easy and robust path to hardware acceleration. It allow ROS 2 roboticists to create custom secure compute architectures with higher productivity. KRS leverages Xilinx technology targeting the [Kria SOM portfolio](https://www.xilinx.com/products/som/kria.html) to deliver low latency (real-fast), determinism (predictable), real-time (on-time), security and high throughput to robotics.

It does so by tightly integrating itself with ROS (lingua franca amongst roboticists) and by leveraging a combination of modern C++ and High-Level Synthesis (HLS), together with reference development boards and design architectures roboticists can use to kick-start their projects. Altogether, KRS supports Kria SOMs with an accelerated path to production in robotics.


----

:warning: *KRS builds on ROS 2. If you're not familiar with it, before continuing, learn more about ROS 2 from its [official documentation](https://docs.ros.org/)*.

----


![](sphinx/source/docs/imgs/krs.svg)


</br>
</br>

## Build documentation and contribute
### Install dependencies

```bash
python3 -m pip install --upgrade pip

pip3 install rst2pdf==0.99 sphinx==5.1.1 recommonmark==0.7.1 sphinx-markdown-tables==0.0.17 docutils==0.19
```
### Build html locally

```bash
cd sphinx; make html
```
