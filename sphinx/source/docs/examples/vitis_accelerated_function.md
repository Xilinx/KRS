#  How to use Vitis Accelerated Functions

## Introduction

AMD offers a wide variety of libraries named as [Vitis™ Accelerated Libraries](https://github.com/Xilinx/Vitis_Libraries) which offers out-of-the-box acceleration for many applications. These libraries are pre optimized and open-source which are offered with [Vitis™ Unified Software Platform](https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vitis.html). The functions in the libraries include:
<br>

- Common Vitis accelerated-libraries for Math, Statistics, Linear Algebra, and DSP offer a set of core functionality for a wide range of diverse applications.
  
- Domain-specific Vitis accelerated libraries offer out-of-the-box acceleration for workloads like Vision and Image Processing, Quantitative Finance, Database, and Data Analytics, Data Compression and more.
<br>

These examples here will showcase how you can utilize a wide variety of pre-optimized / accelerated functions from Vitis™ Accelerated Libraries on FPGA with ROS2 using KRS. 

## Using Vitis Vision Library Functions

The Vitis Vision library is a set of 90+ kernels, optimized for AMD™ FPGAs, AI Engine™, and SoCs, based on the OpenCV computer vision library. See [here](https://github.com/Xilinx/Vitis_Libraries/tree/2022.1/vision) for more details. In KRS examples we will showcase how to integrate few of the functions and based on same workflow you can try to use any of the functions from the library based on your needs.



### Understand the Vitis Vision Library structure

The implementation of Vitis Vision functions is organized in three levels:

- **Level 1**: Here the APIs are presented as HLS C++ classes and functions. These APIs mostly match their OpenCV counterparts. Level 1 functions and API are included by default in "vitis_common" package [here](https://github.com/ros-acceleration/vitis_common/tree/humble/include).
  
- **Level 2**: This contains the OpenCL/[XRT](https://xilinx.github.io/XRT/master/html/xrt_native_apis.html#) host-callable kernels and engines for various Vitis Vision functions. These functions are important, to understand how to call accelerated functions from host code. We will dive deeper in this later.

- **Level 3**: This directory contains whole applications formed by stitching a pipeline of Vitis Vision functions. The host code shows how to call this multiple functions in OpenCL/XRT.


First, visit the [Vitis Vision Documentation](https://docs.amd.com/r/en-US/Vitis_Libraries/vision/api-reference.html_1_0) page and check if the function you want to accelerate is available in the list and meets your performance requirements as all the details are given in the documentation. If available, follow the steps similar to how we followed for below functions:

## Resize


Resize method is used to resize the source image to the size of the destination image. Different types of interpolation techniques can be used in resize function, namely: Nearest-neighbor, Bilinear, and Area interpolation. The type of interpolation can be passed as a template parameter to the API. The following enumeration types can be used to specify the interpolation type:

- XF_INTERPOLATION_NN - For Nearest-neighbor interpolation
- XF_INTERPOLATION_BILINEAR - For Bilinear interpolation
- XF_INTERPOLATION_AREA - For Area interpolation 

Here we have used one of the simplest example because the intent is to make you understand the flow of how you can utilize the accelerated functions with ROS2 using KRS and not getting into the complexities of Vitis Vision functions algorithms.

### Understand the source code of Vitis Vision "Resize" function

As mentioned earlier, Level 2 is important for us to understand on how to make use of HLS kernels in host code. So, let's have a look at the L2 code of resize function [here](https://github.com/Xilinx/Vitis_Libraries/tree/2022.1/vision/L2/examples/resize).


There are a few important files in the resize example folder. Let's discuss it one by one

- "**xf_resize_config.h**" & "**build/xf_config_params.h**" are the configuration files which allows user to configure the HLS kernel at compile time. The possible configrations are like what kind of interpolation required, performance levels like 1 Pixel per clock (PPC/NPC) or 8 PPC and many more configurations. So, it is important for developers to review these configuration and align it according to the application requirements.
  
- "**xf_resize_accel.cpp**" contains the HLS kernel code and this internally calls the L1 resize function. Check the function nomenclature becuase this function will be called from the host code. So, it is important to know the input and output parameters. In case of this resize function the API syntax is:
```bash
    void resize_accel(ap_uint<INPUT_PTR_WIDTH>* img_inp,
                  ap_uint<OUTPUT_PTR_WIDTH>* img_out,
                  int rows_in,
                  int cols_in,
                  int rows_out,
                  int cols_out)

```
- "**xf_resize_tb.cpp**", is testbench which contains the host code and the entrypoint main function. This file contains the implementation of how to call the accelerated kernel function, resize_accel in our current example. User can choose OpenCL or XRT APIs for the execution and buffer management for HLS kernel.


Once we have good understanding of the above files and the dataflow, we can stitch it together with ROS2 and we can use KRS for the workflow.


### Understand the vitis_accelerated_resize example

For easy understanding, we have created a simple example with ROS2 packages

The architecture of the example is as below:

![image](https://)


It has two nodes, one of the node named "opencv_image_publisher" is publishing a "random_image" topic, which is an input to "AcceleratedResize" node which runs this resize algorithm on FPGA and spits out a "resize" image topic. Below is the folder structure of the  [accelerated resize example](https://github.com/ros-acceleration/acceleration_examples/tree/main/vitis_accelerated_examples/vitis_accelerated_resize)

```bash
├── cfg
│   └── kr260.cfg
├── CMakeLists.txt
├── include
│   ├── minimalimagepublisher.hpp
│   ├── resize_fpga.hpp
│   └── xf_resize_config.h
├── package.xml
└── src
    ├── main.cpp
    ├── resize_fpga.cpp
    └── xf_resize_accel.cpp

```

Let's understand role of each file one by one:

- "**minimalimagepublisher.hpp**", this contains the implementation of "opencv_image_publisher" node and it generated a random RGB image. Code snipped of the implementation is below:

```bash
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalImagePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    // Create a new 640x480 image
    cv::Mat my_image(cv::Size(640, 480), CV_8UC3);

    // Generate an image where each pixel is a random color
    cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image)
               .toImageMsg();

    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
};

```

- "**xf_resize_config.h**", this file is a combination of **xf_resize_config.h**" & "**build/xf_config_params.h**" from the vitis vision L2 example.

- "**xf_resize_accel.cpp**", this file contains the implementation of resize HLS kernel and is copied from the L2 resize example from vitis vision library. No change is done in this file.

- "**resize_fpga.hpp**" & "**resize_fpga.cpp**", contains the implementation of "AcceleratedResize" node and showcases how the HLS kernel (resize_accel) is called from the host application. You can refer this file for developing your own application.

- "**main.cpp**", entrypoint of the appplicaiton and it spawns both the nodes.


### How to build the example

If the KRS is built for Ubuntu workflow, use below commands on the host machine to build this example

```bash

source install/setup.bash
colcon acceleration select kr260

###################################################
# 7.A cross-compile and generate ONLY CPU binaries
###################################################
colcon build --build-base=build-kr260-ubuntu --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --cmake-args -DNOKERNELS=true --packages-select vitis_accelerated_resize

###################################################
# 7.B cross-compile and generate CPU binaries and accelerators.
###################################################
colcon build --executor sequential --event-handlers console_direct+ --build-base=build-kr260-ubuntu --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --cmake-args -DNOKERNELS=false --packages-select vitis_accelerated_resize

```

### Execute Vitis Accelerated Resize example

Once the above command successfully build the example, the artifacts will be generated at "*.../krs_ws/install-kr260-ubuntu/lib/vitis_accelerated_resize*". 

If you have KRS installed on the Kria board, you can copy ".../krs_ws/install-kr260-ubuntu/lib/vitis_accelerated_resize" folder from x86 host machine to Kria board at "/home/ubuntu" directory or else you may tar .../krs_ws/install-kr260-ubuntu and copy it to the home directory of Kria board.

Run below commands on **Kria board**:

```bash
source /opt/ros/humble/setup.bash
source ~/install-kr260-ubuntu/local_setup.bash # Or source ~/krs_ws/install/setup.bash if krs is installed on the baord

cp -rf "~/install-kr260-ubuntu/lib/vitis_accelerated_resize" /lib/firmware/xilinx # Copy the required bitstream in firmware folder

sudo xmutil listapps # This will show the available bitstreams/overlays in firmware

sudo xmutil unloadapp # to unload existing overlay

sudo xmutil loadapp vitis_accelerated_resize

cd ~/install-kr260-ubuntu/lib/vitis_accelerated_resize

./vitis_accelerated_resize

```

This will run the example and you can see the behaviour of applicaiton in rqt application



## Stereo Block Matching
TBD


