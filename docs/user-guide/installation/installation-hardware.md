# Software Installation for Hardware

This guide is meant for users preparing to conduct hardware flight experiments, not just simulation.
For the simulation guide, see [Software Installation for Sim](./installation-sim.md).
Note that these two installation guides have minor differences.

The first step to get started with ROSflight will be to set up and install the necessary dependencies and software.
On hardware, we need to set up two devices: the **companion computer** and the **flight controller**.
For more information on the difference between these two devices, see the [overview page](../overview.md).

!!! tip "Starting in sim"

    If you haven't done so yet, we recommend setting up the simulation environment first before getting started with hardware.
    The simulation environment makes it easier to get familiar with the ROSflight ecosystem without needing to debug any potential hardware issues.


## Overview

We will need to install these packages on the companion computer:

- ROS2
- ROSflightIO
- ROScopter
- ROSplane

We will install just the firmware on the flight controller.

!!! note

    You probably don't need both ROSplane and ROScopter.
    If that is the case, just install the one that makes sense for your application.

## Installing ROS2

Unsuprisingly, [ROS2](https://docs.ros.org/en/humble/index.html) is a required dependency for ROSflight.
You can do this with a native installation of ROS2 or by following the [ROSflight Docker guide](./using-docker-with-rosflight.md).
To install ROS2 natively, check out the official [ROS2 Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) page for details.

If you don't plan to run simulations or GUI applications on your companion computer, `ros-<ros-distro>-ros-base` can be used instead of `ros-<ros-distro>-desktop`.

!!! note
    ROSflight currently officially supports only LTS versions of ROS2, so **ROS2 Humble** running on Ubuntu 22.04 or **ROS2 Jazzy** running on Ubuntu 24.04.
    If you want to run a different version of ROS2, some of the below instructions may not work.
    [ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) is not fixed-release and is therefore not officially supported. 


## Companion Computer Setup

Do the following on your companion computer.

1. Create your ROSflight workspace:
    ```bash
    mkdir -p /path/to/rosflight_ws/src
    ```

1. Clone the `rosflight_ros_pkgs` repository, as well as `roscopter` and `rosplane`:
    ```bash
    cd /path/to/rosflight_ws/src
    git clone https://github.com/rosflight/rosflight_ros_pkgs --recursive
    git clone https://github.com/rosflight/roscopter
    git clone https://github.com/rosflight/rosplane
    ```

    !!! success "File structure"
        Your ROSflight workspace file structure should now look like
        ```bash
        rosflight_ws/
          └── src/
              ├── roscopter/
              ├── rosflight_ros_pkgs/
              └── rosplane/
        ```

1. Install dependencies using `rosdep`:

    !!! warning
        Make sure you have properly sourced [the ROS2 environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) in the terminal you are working in, or the `rosdep` and `colcon` commands will fail.
        ```bash
        source /opt/ros/<ros-distro>/setup.bash
        ```

    !!! tip

        If you are not planning on using the simulation environment on your companion computer, you can delete the `rosflight_sim` package safely.
        This will speed up build times.
        ```bash
        cd /path/to/rosflight_ws/src/rosflight_ros_pkgs
        rm -rf rosflight_sim
        ```

    ```bash
    cd /path/to/rosflight_ws
    sudo rosdep init
    rosdep update
    rosdep install --from-path . -y --ignore-src
    ```

    If you have already done `sudo rosdep init` previously, it will return an error.
    In most cases, you do not need to reinitialize.

1. Build using the [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) build tool:
```bash
cd /path/to/rosflight_ws
colcon build
```

    !!! warning "Resource Usage"
        Building the whole repository at once uses a lot of memory.
        If you get build errors, try running the build command with the following argument:
        ```bash
        colcon build --executor sequential
        ```

    !!! success
        Your ROSflight workspace file structure should now look like
        ```bash
        rosflight_ws/
          ├── build/
          ├── install/
          ├── log/
          └── src/
              ├── roscopter/
              ├── rosflight_ros_pkgs/
              └── rosplane/
        ```

## Flight controller setup

ROS2 will not be installed on the flight controller.
We will only need to build and flash the flight controller with the `rosflight_firmware`.

See the [flight controller guide](../concepts/flight-controller-setup.md) for instructions on how to do this.

## Next Steps

At this point, you have successfully installed ROS2 and set up the ROSflight simulation environment, including all required dependencies.
Your workspace is built and ready to run simulations with ROSflight, ROScopter, or ROSplane.
The next step is to launch a simulation scenario or integrate your own simulated vehicle.
<!--For guidance on running simulations, see the [simulation quickstart guide](./simulation-quickstart.md).-->
