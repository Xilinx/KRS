# Definitions

- **Block RAM (BRAM)**: hardened RAM resource. More efficient memories than using LUTs for more than a few elements.
- **Compute Unit (CU)**: An OpenCL device has one or more compute units. A work-group executes on a single compute unit. A compute unit is composed of one or more processing elements and local memory. A compute unit may also include dedicated texture filter units that can be accessed by its processing elements[^3].
- **Digital Signal Processor (DSP)**: performs multiplication and other arithmetic in the FPGA
- **Field-Programmable Gate Array (FPGA)**: semiconductor devices that are based around a matrix of configurable logic blocks (CLBs) connected via programmable interconnects. FPGAs can be reprogrammed to desired application or functionality requirements after manufacturing.
- **Flip Flops (FF)**: control the flow of data with the clock pulse. Used to build the pipeline and achieve high throughput
- **Hardware Description Language (HDL)**: a specialized computer language used to describe the structure and behavior of electronic circuits, and most commonly, digital logic circuits[^2]. 
- **High Level Synthesis (HLS)**: compiler for C, C++, SystemC into FPGA IP cores
- **Kernel**: An OpenCL kernel is a function declared in a program and executed on an OpenCL device. A kernel is identified by the kernel or kernel qualifier applied to any function defined in a program[^3].
- **Look Up Table (LUT)**: generic functions on small bitwidth inputs (the logic itself). Combine many to build the algorithm.
- **Register Transfer Level (RTL)**: a design abstraction (typically graphical) which models a synchronous digital circuit in terms of the flow of digital signals (data) between hardware registers, and the logical operations performed on those signals. The very low level description of the function and connection of logic gates: *Register-transfer-level abstraction is used in hardware description languages (HDLs) like Verilog and VHDL to create high-level representations of a circuit, from which lower-level representations and ultimately actual wiring can be derived. Design at the RTL level is typical practice in modern digital design*.
- **Robot Operating System (ROS)**: a framework for robot application development. Includes tools, libraries, conventions and a community. The de-facto standard in robotics.
- **VHSIC Hardware Description Language (VHDL)**: a hardware description language (HDL) that can model the behavior and structure of digital systems.
- **Verilog**: standardized as IEEE 1364, is a hardware description language (HDL) used to model electronic systems.


<!-- references -->

[^1]: Hardware description language. Wikipedia. Retrieved from https://en.wikipedia.org/wiki/Hardware_description_language
[^2]: Register-transfer level. Wikipedia. Retrieved from https://en.wikipedia.org/wiki/Register-transfer_level
[^3]: The OpenCL™ Specification Version V2.2-11. Retrieved from https://www.khronos.org/registry/OpenCL/specs/2.2/html/OpenCL_API.html.