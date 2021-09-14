# Kria Robotics Stack (KRS)

:warning: *KRS builds on ROS 2. If you're not familiar with it, before continuing, learn more about ROS 2 from its [official documentation](https://docs.ros.org/)*.

The **Kria Robotics Stack (KRS)** is a ROS 2 superset for industry, an integrated set of robot libraries and utilities to accelerate the development, maintenance and commercialization of industrial-grade robotic solutions while using [adaptive computing](https://www.xilinx.com/applications/adaptive-computing.html). KRS provides to ROS 2 users an easy and robust path to hardware acceleration. It allow ROS 2 roboticists to create custom secure compute architectures with higher productivity. KRS leverages Xilinx technology targeting the [Kria SOM portfolio](https://www.xilinx.com/products/som/kria.html) to deliver low latency (real-fast), determinism (predictable), real-time (on-time), security and high throughput to robotics.

It does so by tightly integrating itself with ROS (lingua franca amongst roboticists) and by leveraging a combination of modern C++ and High-Level Synthesis (HLS), together with reference development boards and design architectures roboticists can use to kick-start their projects. Altogether, KRS supports Kria SOMs with an accelerated path to production in robotics.


----

`Alpha Release`. KRS is still on **alpha** release. Correspondingly, the documentation provided here is not intended for production environments and should be used only for evaluation purposes. *Stay tuned for upcoming official releases*.

----


![](sphinx/source/docs/imgs/krs.svg)


</br>
</br>

## KRS capabilities

| action | quick peek | description |
|--------|-------------|------------|
| install KRS | [![asciicast](https://asciinema.org/a/434953.svg)](https://asciinema.org/a/434953) | |
| `colcon acceleration select` | [![asciicast](https://asciinema.org/a/434781.svg)](https://asciinema.org/a/434781) | The `select` verb allows to easily select and configure a specific target firmware for hardware acceleration, and default to it while producing binaries and accelerators.  |
| `colcon acceleration list` | [![asciicast](https://asciinema.org/a/434781.svg)](https://asciinema.org/a/434781) | The `list` verb  allows to inspect the acceleration firmware available in the ROS workspace, marking with a `*` the currently selected option.  |
| `colcon acceleration linux` | [![asciicast](https://asciinema.org/a/scOognokU4wt0PW3E1N4F0jCe.svg)](https://asciinema.org/a/scOognokU4wt0PW3E1N4F0jCe) | The `linux` verb helps configure the Linux kernel in the raw SD card image produced by the firmware. E.g. `colcon acceleration linux vanilla` will produce a Linux vanilla kernel, whereas `colcon acceleration linux preempt_rt` will instead use a pre-built kernel and kernel modules for improved determinism (fully preemptible kernel). |

## Build documentation and contribute
### Install dependencies

```bash
pip3 install rst2pdf sphinx
```
### Build html locally

```bash
cd sphinx; make html
```
