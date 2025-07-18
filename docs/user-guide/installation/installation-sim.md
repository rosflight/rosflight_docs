# Software Installation for Sim

The first step to get started with ROSflight will be to set up and install the necessary dependencies and software.
This guide will detail how to install these packages as if preparing to run a simulation.
This means we will cover setting up the following packages on either the **companion computer** or any other Linux computer:

- ROS2
- ROSflight Sim (and other software from `rosflight_ros_pkgs`)
- ROScopter
- ROSplane

We will not cover flashing firmware or other hardware-specific instructions.
For instructions on setting up the software on real hardware, see [the hardware installation guide](./installation-hardware.md).

!!! note

    You probably don't need both ROSplane and ROScopter.
    If that is the case, just install the one that makes sense for your application.

## Installing ROS2

Unsuprisingly, [ROS2](https://docs.ros.org/en/humble/index.html) is a required dependency for ROSflight.
You can do this with a native installation of ROS2 or by following the [ROSflight Docker guide](./using-docker-with-rosflight.md).
To install ROS2 natively, check out the official [ROS2 Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) page for details.

If you don't plan to run simulations or GUI applications on your companion computer, `ros-humble-ros-base` can be used instead of `ros-humble-desktop`.

!!! note
    ROSflight currently officially supports only **ROS2 Humble**, running on Ubuntu 22.04.
    If you want to run a different version of ROS2, some of the below instructions may not work.
    [ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) is not fixed-release and is therefore not officially supported. 

## Installing ROSflight, ROScopter, and ROSplane

In this section, when we refer to ROSflight we are referring to the `rosflight_ros_pkgs` repository, which includes the `rosflight_sim`, `rosflight_io`, and `rosflight_firmware` modules.

In the [hardware installation guide](./installation-hardware.md), each of these packages will be installed separately, or not at all.

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
        source /opt/ros/humble/setup.bash
        ```
    
    ```bash
    cd /path/to/rosflight_ws
    sudo rosdep init
    rosdep update
    rosdep install --from-path . -y --ignore-src
    ```

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

1. Add the source files to your `.bashrc` (so you don't have to source the files every time you open a terminal):
    ```bash
    # add the sourcing commands to your .bashrc. Replace bash with zsh if using zsh.
    echo "source /opt/ros/humble/setup.bash >> $HOME/.bashrc"
    echo "source /path/to/rosflight_ws/install/setup.bash >> $HOME/.bashrc"
    ```

## Next Steps

At this point, you have successfully installed ROS2 and set up the ROSflight simulation environment, including all required dependencies.
Your workspace is built and ready to run simulations with ROSflight, ROScopter, or ROSplane.
The next step is to launch a simulation scenario or integrate your own simulated vehicle.
<!--For guidance on running simulations, see the [simulation quickstart guide](./simulation-quickstart.md).-->
