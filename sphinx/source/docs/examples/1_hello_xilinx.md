# 1. Hello Xilinx

|   | Source code |
|---|----------|
| [`publisher_xilinx`](https://github.com/ros-acceleration/acceleration_examples/tree/main/publisher_xilinx) | |
| publisher | [`member_function_publisher.cpp`](https://github.com/ros-acceleration/acceleration_examples/blob/main/publisher_xilinx/member_function_publisher.cpp) |

This example lets you experience KRS further, walking you through the process of building and launching a ROS 2 package across different targets: in the workstation, in the real hardware and in an emulation.

```eval_rst

.. important::
    The examples assume you've already installed KRS. If not, refer to `install <../install.html>`_.

.. note::
    `Learn ROS 2 <https://docs.ros.org/>`_ before trying this out first.
```


```bash
$ cd ~/krs_ws  # head to your KRS workspace

# prepare the environment
$ source /tools/Xilinx/Vitis/2021.2/settings64.sh  # source Xilinx tools
$ source /opt/ros/foxy/setup.bash  # Sources system ROS 2 installation
$ export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+

# fetch the source code of examples
$ git clone https://github.com/ros-acceleration/acceleration_examples src/acceleration_examples

# build the workspace
$ colcon build --merge-install  # about 2 mins

# source the workspace as an overlay
$ source install/setup.bash
```


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

*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
MinimalPublisher()
: Node("minimal_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
}

private:
void timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, Xilinx! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
size_t count_;
};

int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<MinimalPublisher>());
rclcpp::shutdown();
return 0;
}

```


## Launch `hello_xilinx` example in the workstation
```bash
$ source install/setup.bash  # source the overlay workspace
$ ros2 run publisher_xilinx member_function_publisher
[INFO] [1618407842.800443167] [minimal_publisher]: Publishing: 'Hello, Xilinx! 0'
[INFO] [1618407843.300407127] [minimal_publisher]: Publishing: 'Hello, Xilinx! 1'
[INFO] [1618407843.800389187] [minimal_publisher]: Publishing: 'Hello, Xilinx! 2'
...
```


## Launch `hello_xilinx` example in the KV260 hardware

First, let's select the firmware for the target hardware, [`KV260`](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit.html):

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
$ colcon build --build-base=build-kv260 --install-base=install-kv260 --merge-install --mixin kv260 --packages-select publisher_xilinx
```

Let's now create a raw disk image for the SD card with PetaLinux's rootfs, a vanilla Linux 5.4.0 kernel and the ROS 2 overlay workspace we've just created for the KV260:
```bash
$ colcon acceleration linux vanilla --install-dir install-kv260
```

We're now ready to run it on hardware. For that, we need to flash the `~/krs_ws/acceleration/firmware/select/sd_card.img` file into the SD card. If you need help doing so, check out [these instructions](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit/kv260-getting-started/setting-up-the-sd-card-image.html) for different OSs. **Make sure to flash `~/krs_ws/acceleration/firmware/select/sd_card.img` we just generated, and not some other image**. 


Once flashed, connect the board to the computer via its USB/UART/JTAG FTDI adapter and power it on. Then, launch your favority serial console (e.g. `sudo tio /dev/ttyUSB1`). You should get to the following prompt where you can use the `petalinux` username and define your own password:
```bash
PetaLinux 2020.2.2 xilinx-k26-starterkit-2020_2.2 ttyPS0

xilinx-k26-starterkit-2020_2 login:
```

To launch the `hello_xilinx` package:

```bash
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation
$ . /ros2_ws/local_setup.bash   # source the ROS 2 overlay workspace
$ ros2 run publisher_xilinx member_function_publisher  # launch the hello_xilinx example

[INFO] [1618478074.116887661] [minimal_publisher]: Publishing: 'Hello, Xilinx! 0'
[INFO] [1618478074.611878275] [minimal_publisher]: Publishing: 'Hello, Xilinx! 1'
[INFO] [1618478075.112175121] [minimal_publisher]: Publishing: 'Hello, Xilinx! 2'
...
```


## Launch `hello_xilinx` example in an emulated KV260

```eval_rst

.. admonition:: Install K26 and KV260 Vivado files manually
    
    Unfortunately, Vitis :code:`2020.2` (and :code:`2020.2.2`) do not ship Vivado with K26-related files, which is required for emulation. You need to install this manually. See https://github.com/ros-acceleration/acceleration_examples/issues/1.
```



First, let's select the firmware for the target hardware, [`KV260`](https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit.html):

```bash
$ colcon acceleration select kv260
```

Build the package for the KV260:

```bash
$ colcon build --build-base=build-kv260 --install-base=install-kv260 --merge-install --mixin kv260 --packages-select publisher_xilinx
```

Create a raw disk image for the SD card with PetaLinux's rootfs and a Linux 5.4.0 vanilla kernel:
```bash
$ colcon acceleration linux vanilla
```

Then use the `acceleration` extensions to the ROS 2 `colcon` meta-build system to launch an emulation:
```bash
$ colcon acceleration emulation sw_emu install-kv260
```

This will trigger the emulation with a prompt like what follows:

!!! warning
    **To be fixed in future releases**.

    Due to some issues with KV260, the current emulation capabilities only brings you to ramfs. To finalize booting the board it's necessary to manually introduce the following in ramfs:

    ```bash
    mkdir /configfs
    mount -t configfs configfs /configfs
    cd /configfs/device-tree/overlays/
    mkdir full
    mkdir /lib/firmware/
    cp /boot/devicetree/zynqmp-sck-kv-g-qemu.dtbo /lib/firmware/.
    echo -n "zynqmp-sck-kv-g-qemu.dtbo" > full/path
    exec /init
    ```


```bash
$ colcon acceleration emulation sw_emu install-kv260
SECURITY WARNING: This class invokes explicitly a shell via the shell=True argument of the Python subprocess library, and uses admin privileges to manage raw disk images. It is the user's responsibility to ensure that all whitespace and metacharacters passed are quoted appropriately to avoid shell injection vulnerabilities.
- Verified that install/ is available in the current ROS 2 workspace
- Confirmed availability of raw image file at: /home/xilinx/krs_ws/acceleration/firmware/select/sd_card.img
- Finished inspecting raw image, obtained UNITS and STARTSECTOR P1/P2
- Image mounted successfully at: /tmp/sdcard_img_p2
- Successfully cleaned up prior overlay ROS 2 workspace at: /tmp/sdcard_img_p2/ros2_ws
- Copied 'install-kv260' directory as a ROS 2 overlay workspace in the raw image.
- Umounted the raw image.
- Generated PMU and QEMU files.
- Launching emulation...
cd /home/xilinx/krs_ws/acceleration/firmware/select/emulation && /tools/Xilinx/Vitis/2020.2/bin/launch_emulator -device-family ultrascale -target sw_emu -qemu-args-file /home/xilinx/krs_ws/acceleration/firmware/select/emulation/qemu_args.txt -pmc-args-file /home/xilinx/krs_ws/acceleration/firmware/select/emulation/pmu_args.txt -sd-card-image /home/xilinx/krs_ws/acceleration/firmware/select/sd_card.img -enable-prep-target $*
Running SW Emulation
INFO :  [LAUNCH_EMULATOR] pl_sim_dir option is not provided.
Starting QEMU
 - Press <Ctrl-a h> for help
Waiting for QEMU to start. qemu_port 9720
Qemu_pids 601788 601789
qemu-system-aarch64: -chardev socket,path=./qemu-rport-_pmu@0,server,id=pmu-apu-rp: info: QEMU waiting for connection on: disconnected:unix:./qemu-rport-_pmu@0,server
QEMU started. qemu_pid=601788.
Waiting for PMU to start. Qemu_pids 601792 601793
qemu-system-aarch64: -chardev socket,id=pl-rp,host=127.0.0.1,port=8500,server: info: QEMU waiting for connection on: disconnected:tcp:127.0.0.1:8500,server
PMC started. pmc_pid=601792
Starting PL simulation.Generating PLLauncher commandline
running PLL Launcher
PMU Firmware 2020.2	Jun 10 2021   22:45:10
PMU_ROM Version: xpbr-v8.1.0-0
NOTICE:  ATF running on XCZUUNKN/QEMU v4/RTL0.0 at 0xfffea000
NOTICE:  BL31: v2.2(release):xilinx-v2020.2.2-k26
NOTICE:  BL31: Built : 22:39:14, Jun 10 2021


U-Boot 2020.01 (Jun 11 2021 - 08:39:52 +0000)

Model: ZynqMP SMK-K26 Rev1/B/A
Board: Xilinx ZynqMP
DRAM:  4 GiB
PMUFW:	v1.1
EL Level:	EL2
Chip ID:	unknown
NAND:  0 MiB
MMC:   mmc@ff170000: 1
In:    serial@ff010000
Out:   serial@ff010000
Err:   serial@ff010000
Bootmode: JTAG_MODE
Reset reason:
Net:   No ethernet found.
Hit any key to stop autoboot:  0
switch to partitions #0, OK
mmc1 is current device
Scanning mmc 1:1...
Found U-Boot script /boot.scr
2145 bytes read in 17 ms (123 KiB/s)
## Executing script at 20000000
17932800 bytes read in 3251 ms (5.3 MiB/s)
38045 bytes read in 26 ms (1.4 MiB/s)
25150797 bytes read in 4541 ms (5.3 MiB/s)
## Loading init Ramdisk from Legacy Image at 04000000 ...
   Image Name:   petalinux-initramfs-image-zynqmp
   Image Type:   AArch64 Linux RAMDisk Image (uncompressed)
   Data Size:    25150733 Bytes = 24 MiB
   Load Address: 00000000
   Entry Point:  00000000
   Verifying Checksum ... OK
## Flattened Device Tree blob at 00100000
   Booting using the fdt blob at 0x100000
   Loading Ramdisk to 77803000, end 78fff50d ... OK
   Loading Device Tree to 000000000fff3000, end 000000000ffff49c ... OK

Starting kernel ...

[    0.000000] Booting Linux on physical CPU 0x0000000000 [0x410fd034]
[    0.000000] Linux version 5.4.0-xilinx-v2020.2 (oe-user@oe-host) (gcc version 9.2.0 (GCC)) #1 SMP Thu Jun 10 22:03:38 UTC 2021
[    0.000000] Machine model: ZynqMP SMK-K26 Rev1/B/A
[    0.000000] earlycon: cdns0 at MMIO 0x00000000ff010000 (options '115200n8')
[    0.000000] printk: bootconsole [cdns0] enabled
[    0.000000] efi: Getting EFI parameters from FDT:
[    0.000000] efi: UEFI not found.
[    0.000000] cma: Reserved 1000 MiB at 0x0000000039000000
[    0.000000] psci: probing for conduit method from DT.
[    0.000000] psci: PSCIv1.1 detected in firmware.
[    0.000000] psci: Using standard PSCI v0.2 function IDs
[    0.000000] psci: MIGRATE_INFO_TYPE not supported.
[    0.000000] psci: SMC Calling Convention v1.1
[    0.000000] percpu: Embedded 22 pages/cpu s49880 r8192 d32040 u90112
[    0.000000] Detected VIPT I-cache on CPU0
[    0.000000] CPU features: detected: ARM erratum 845719
[    0.000000] CPU features: detected: ARM erratum 843419

...

ccpath is incorrect: /sys/bus/i2c/devices/*51/eeprom
[   11.106002] random: python3: uninitialized urandom read (24 bytes read)

ccpath is incorrect: /sys/bus/i2c/devices/*51/eeprom
SOM: CARRIER_CARD: REVISION:
NO CARRIER DTBO FOUND, PLEASE CHECK /boot/devicetree/
Waiting for /dev/mmcblk0p2 to pop up (attempt 1)
Waiting for /dev/mmcblk0p2 to pop up (attempt 2)
Waiting for /dev/mmcblk0p2 to pop up (attempt 3)
Waiting for /dev/mmcblk0p2 to pop up (attempt 4)
Waiting for /dev/mmcblk0p2 to pop up (attempt 5)
Waiting for /dev/mmcblk0p2 to pop up (attempt 6)
Waiting for /dev/mmcblk0p2 to pop up (attempt 7)
Waiting for /dev/mmcblk0p2 to pop up (attempt 8)
Waiting for /dev/mmcblk0p2 to pop up (attempt 9)
Waiting for /dev/mmcblk0p2 to pop up (attempt 10)
Device /dev/mmcblk0p2 not found
ERROR: There's no '/dev' on rootfs.

sh: can't access tty; job control turned off
/ #
```

To finalize the boot on rootfs, introduce the following:

```bash
mkdir /configfs
mount -t configfs configfs /configfs
cd /configfs/device-tree/overlays/
mkdir full
mkdir /lib/firmware/
cp /boot/devicetree/zynqmp-sck-kv-g-qemu.dtbo /lib/firmware/.
echo -n "zynqmp-sck-kv-g-qemu.dtbo" > full/path
exec /init
```

This should bring you all the way down to the prompt:

```bash
PetaLinux 2020.2.2 xilinx-k26-starterkit-2020_2.2 ttyPS0

xilinx-k26-starterkit-2020_2 login:
```

Use the `petalinux` username and pick you own password. Then, inside of the emulation, launch `hello_xilinx`:

```bash
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation
$ . /ros2_ws/local_setup.bash   # source the ROS 2 overlay workspace
$ ros2 run publisher_xilinx member_function_publisher  # launch the hello_xilinx example
[INFO] [1618478074.116887661] [minimal_publisher]: Publishing: 'Hello, Xilinx! 0'
[INFO] [1618478074.611878275] [minimal_publisher]: Publishing: 'Hello, Xilinx! 1'
[INFO] [1618478075.112175121] [minimal_publisher]: Publishing: 'Hello, Xilinx! 2'
...
```

To exit the emulation, type `Ctrl-A x`.


## **Bonus**: Launch `hello_xilinx` example in emulation with another firmware

As introduced in [Features](../features/ros2centric), KRS is designed to be easily portable across boards. To demonstrate it, we'll now fetch the firmware for another board capable of hardware acceleration and launch `hello_xilinx` in this board using emulation.
Let's start fetching the firmware:
```bash
$ cd src && wget https://github.com/ros-acceleration/acceleration_firmware_zcu102/releases/download/0.6.0/acceleration_firmware_zcu102.zip && unzip acceleration_firmware_zcu102.zip && rm acceleration_firmware_zcu102.zip && cd ..
```

We then build the workspace to deploy the firmware:
```bash
# build the workspace
$ colcon build --merge-install  # about 2 mins
```

Then, we select the firmware and build the workspace for the ZCU102:
```bash
$ colcon acceleration select zcu102
$ colcon build --build-base=build-zcu102 --install-base=install-zcu102 --merge-install --mixin zcu102 --packages-select publisher_xilinx
```

We create new raw disk image for the SD card (using ZCU102's firmware) with PetaLinux's rootfs and a Linux 5.4.0 vanilla kernel:
```bash
$ colcon acceleration linux vanilla --install-dir install-zcu102
```

Then use the `acceleration` extensions to the ROS 2 `colcon` meta-build system to launch an emulation:
```bash
$ colcon acceleration emulation sw_emu --no-install
```


The ZCU102 rootfs should login automatically with the `root` username. 
Inside of the emulation, launch `hello_xilinx`:

```bash
$ source /usr/bin/ros_setup.bash  # source the ROS 2 installation
$ . /ros2_ws/local_setup.bash   # source the ROS 2 overlay workspace
$ ros2 run publisher_xilinx member_function_publisher  # launch the hello_xilinx example
[INFO] [1618478074.116887661] [minimal_publisher]: Publishing: 'Hello, Xilinx! 0'
[INFO] [1618478074.611878275] [minimal_publisher]: Publishing: 'Hello, Xilinx! 1'
[INFO] [1618478075.112175121] [minimal_publisher]: Publishing: 'Hello, Xilinx! 2'
...
```

To exit the emulation, type `Ctrl-A x`.

```eval_rst
.. note::
    You can also try the `hw_emu` target in the ZCU102.
```
