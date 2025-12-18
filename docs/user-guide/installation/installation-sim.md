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

If you don't plan to run simulations or GUI applications on your companion computer, `ros-<ros-distro>-ros-base` can be used instead of `ros-<ros-distro>-desktop`.

!!! note
    ROSflight currently officially supports only LTS versions of ROS2, so **ROS2 Humble** running on Ubuntu 22.04 or **ROS2 Jazzy** running on Ubuntu 24.04.
    If you want to run a different version of ROS2, some of the below instructions may not work.
    [ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) is not fixed-release and is therefore not officially supported. 

## Installing ROSflight, ROScopter, and ROSplane

In this section, when we refer to ROSflight we are referring to the `rosflight_ros_pkgs` repository, which includes the `rosflight_sim`, `rosflight_io`, and `rosflight_firmware` modules.

In the [hardware installation guide](./installation-hardware.md), each of these packages will be installed separately, or not at all.

1. Create your ROSflight workspace:
    ```bash
    mkdir -p ~/rosflight_ws/src
    ```

1. Clone the `rosflight_ros_pkgs` repository, as well as `roscopter` and `rosplane`:
    ```bash
    cd ~/rosflight_ws/src
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
        # Replace <ros-distro> with your ROS2 distro (probably 'humble' or 'jazzy')
        source /opt/ros/<ros-distro>/setup.bash
        ```

    ```bash
    cd ~/rosflight_ws
    sudo rosdep init
    rosdep update
    rosdep install --from-path . -y --ignore-src
    ```

    If you have already done `sudo rosdep init` previously, it will return an error.
    In most cases, you do not need to reinitialize.

1. Build using the [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) build tool:
    ```bash
    cd ~/rosflight_ws
    colcon build
    ```

    After the build, you should see output similar to:
    ```bash
    ---
    Finished <<< roscopter [1min 34s]
    Finished <<< rosflight_sim [1min 35s]                                
    Starting >>> rosflight_pkgs
    Finished <<< rosflight_pkgs [0.42s]                        

    Summary: 18 packages finished [1min 44s]
    4 packages had stderr output: roscopter rosflight_io rosplane rosplane_extra
    ```
    There may be warnings or `stderr` output, but as long as each package says `Finished`, the build was successful.

    !!! warning "Resource Usage"
        Building the whole repository at once uses a lot of memory.
        If you get build errors, try running the build command with the following argument:
        ```bash
        colcon build --executor sequential
        ```

    !!! success
        The `colcon` build tool will create `build`, `log`, and `install` folders, even if the build fails.
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
    echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc
    echo "source $HOME/rosflight_ws/install/setup.bash" >> $HOME/.bashrc

    source $HOME/.bashrc
    ```
    If you are using a different ROS 2 distribution, replace `humble` with your distro name.

    Make sure to reopen current terminals after adding these commands to your `.bashrc` file (or run `source $HOME/.bashrc` file in every open terminal).

## Next Steps

At this point, you have successfully installed ROS2 and set up the ROSflight simulation environment, including all required dependencies.
Your workspace is built and ready to run simulations with ROSflight, ROScopter, or ROSplane.
The next step is to launch a simulation scenario or integrate your own simulated vehicle.
<!--For guidance on running simulations, see the [simulation quickstart guide](./simulation-quickstart.md).-->
