# ROS 2 Accelerated Apps

The ROS ecosystem brings together a worldwide community of thousands of roboticists developing robot applications while using ROS 2 abstractions. In a way, ROS is the common API roboticists use when building robot behaviours. The reference Software Development Kit (SDK) in robotics. With the advent of mixed source technology ecosystems in robotics, in the ROS world, there are already various examples of companies providing value around open source packages, while contributing back to the community. Xilinx attempts to bring those capabilities to selected partners by introducing a mixed source ecosystem powered by KRS:

![](../imgs/mixed.svg)

With KRS, we take a step further and propose a layer for commercialization of ROS 2 robot accelerated applications. By leveraging Xilinx experience using encryption and authentication to secure FPGA bitstreams, the [Xilinx App Store](https://www.xilinx.com/products/app-store.html) proposes a managed, easy to use and secure infrastructure for digital rights management (DRM) whereto commercialize and discover global customers. Through the Xilinx App Store connection, KRS containerizes ROS 2 overlay workspaces into robot accelerated applications allowing selected partners to protect and monetize their accelerated ROS 2 packages.


![](../imgs/apps.png)

Beyond extending the ROS 2 build system and tools, to faciliate the process of monetizing ROS 2 packages, KRS also facilitates additional tools and extensions to simplify the process of packaging ROS 2 overlay workspaces and shipping them into the Xilinx App Store. `colcon acceleration package` subverb above shows one of such tools.