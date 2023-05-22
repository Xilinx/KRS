# Hardware supported

KRS focuses on the Kria SOM portfolio. KRS full support is provided for the following boards:

<style>
table th:first-of-type {
    width: 15%;
}
table th:nth-of-type(2) {
    width: 10%;
}
table th:nth-of-type(3) {
    width: 25%;
}
table th:nth-of-type(4) {
    width: 50%;
}

.wy-table-responsive table td,
.wy-table-responsive table th {
    white-space: normal;
}
</style>


| Board | Picture |  Firmware | Description |
|------------|-------|-------------|-------|
| [Kria `K26` Adaptive System-on-Module](https://www.xilinx.com/products/som/kria/k26c-commercial.html) | ![](https://www.xilinx.com/content/dam/xilinx/imgs/products/som/som-k26-main.png) | [acceleration_firmware_kv260](https://github.com/ros-acceleration/acceleration_firmware_kv260) | The Kria™ K26 SOM is the first adaptive Single Board Computer, a production-grade and tiny System-on-Module for edge vision and robotics applications. Available in Commercial and Industrial Grade variants.  | 
| [Kria `KV260` Vision AI Starter Kit](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit.html) | ![](https://www.xilinx.com/content/dam/xilinx/imgs/products/som/som-kv260-4.png) | [acceleration_firmware_kv260](https://github.com/ros-acceleration/acceleration_firmware_kv260) | The Kria™ KV260 starter kit is a development platform for the K26, the first adaptive Single Board Computer. KV260 offers a compact board for edge vision and robotics applications.  |
| [Kria `KR260` Robotics Starter Kit](https://www.xilinx.com/products/som/kria/kr260-robotics-starter-kit.html) | ![](https://www.xilinx.com/content/dam/xilinx/imgs/products/som/kr260-angel-1.png) | [acceleration_firmware_kr260](https://github.com/ros-acceleration/acceleration_firmware_kr260) | The Kria™ KR260 is a development platform for Kria K26 SOMs, the KR260 is built for robotics and industrial applications, complete with high performance interfaces and native ROS 2 support for ease of development by roboticists and software developers.  |

## Unofficial support

Unofficial support is also available for the following:

| Board | Picture | Firmware | Description | 
|------------|-------|----------|----------|
| [Xilinx Zynq UltraScale+ MPSoC `ZCU102` Evaluation Kit](https://www.xilinx.com/products/boards-and-kits/ek-u1-zcu102-g.html) | ![](https://www.xilinx.com/content/dam/xilinx/imgs/kits/whats-inside/zcu102-evaluation-board-w.jpg) | [acceleration_firmware_zcu102](https://github.com/ros-acceleration/acceleration_firmware_zcu102) | The ZCU102 Evaluation Kit enables designers to jumpstart designs for automotive, industrial, video, and communications applications. This kit features a Zynq® UltraScale+™ MPSoC with a quad-core Arm® Cortex®-A53, dual-core Cortex-R5F real-time processors, and a Mali™-400 MP2 graphics processing unit  | 
| [Zynq UltraScale+ MPSoC ZCU104 Evaluation Kit `ZCU104`](https://www.xilinx.com/products/boards-and-kits/zcu104.html) | ![](https://www.xilinx.com/content/dam/xilinx/imgs/kits/whats-inside/zcu104-evaluation-board-w.jpg) | [Contact us](https://www.xilinx.com/about/contact.html) |  The ZCU104 Evaluation Kit enables designers to jumpstart designs for embedded vision applications such as surveillance, Advanced Driver Assisted Systems (ADAS), machine vision, Augmented Reality (AR), drones and medical imaging. This kit features a Zynq® UltraScale+™ MPSoC EV device with video codec and supports many common peripherals and interfaces for embedded vision use case. |
| [AVNET Ultra96-V2](https://www.avnet.com/wps/portal/us/products/new-product-introductions/npi/aes-ultra96-v2/) | ![](https://www.avnet.com/wps/wcm/connect/onesite/c163c966-18fa-44fc-81c0-e05425c52b5d/ultra96-v2-front-view.jpg?MOD=AJPERES&CACHEID=ROOTWORKSPACE.Z18_NA5A1I41L0ICD0ABNDMDDG0000-c163c966-18fa-44fc-81c0-e05425c52b5d-mBKcori) | [community port effort](https://github.com/ros-acceleration/community/issues/1) | The Ultra96-V2 is an Arm-based, Xilinx Zynq UltraScale+™ MPSoC development board based on the Linaro 96Boards Consumer Edition (CE) specification and designed with a certified radio module from Microchip providing Wi-Fi and Bluetooth. All components are updated to allow industrial temperature grade options. Additional power control and monitoring will be possible with the included Infineon PMICs.|