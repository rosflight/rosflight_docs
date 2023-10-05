# Installing/Setting up ROS2

You will need to install ROS2 on both the companion computer and the base station laptop. Check out the official [ROS2 Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) page for details on how to do this. Make sure to install both the `ros-humble-desktop` and `ros-dev-tools` packages, or the equivalent packages for your version of ROS2. `ros-humble-ros-base` can be used instead of `ros-humble-desktop` if you don't need GUI tools or the simulation.

We support all fixed-release ROS2 versions that are not EOL, which currently includes [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) and [ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html). [ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) is not fixed-release and is therefore not officially supported. 

## Installing ROSflight

You will need to install the ROSflight packages on both the companion computer and the base station computer. The companion computer will run the node that actually communicates with the flight controller over a serial connection, while the base station needs the message and service definitions to be able to call services or subscribe and publish to topics.

### From Source

First, set up a ROS2 workspace:

```bash
mkdir -p ~/rosflight_ws/src
cd ~/rosflight_ws/src
```

Next, download the source code into your workspace (include the `--recursive` argument to download the necessary submodules):
```bash
git clone --recursive https://github.com/rosflight/rosflight_ros_pkgs.git
```
Install dependencies:
```bash
sudo rosdep init
rosdep update
rosdep install -i --from-path ./ -y --ignore-src
```
Build the packages:
```bash
cd ~/rosflight_ws
colcon build
```
Source the setup file and set it to be sourced automatically:
```bash
source ~/rosflight_ws/install/setup.bash
echo "source ~/rosflight_ws/install/setup.bash" >> ~/.bashrc
```

!!! note
    You'll also need to source the file at `/usr/share/gazebo/setup.sh` if you plan to use the Gazebo simulator.

## Running rosflight_io

The `rosflight_io` node is the bridge between ROS2 and the MAVLink communication with the flight controller. This node must be run on the computer that has the physical serial connection to your flight controller. To run this node, use something like the following command:
```bash
ros2 run rosflight rosflight_io --ros-args -p port:=/dev/ttyUSB0
```
Replace `/dev/ttyUSB0` with the port your flight controller is connected to.

## Using a Docker Container to run ROS2

!!! note
    This guide was written for using Linux as the host machine, but theoretically you should be able to use Docker to do the same thing on Mac or Windows. However, the specifics of the commands may be different. Please refer to the [Docker documentation](https://docs.docker.com/) for information on how to use Docker on a non-Linux system.
!!! tip
    This guide was written using ROS2 Humble Docker images, but any distribution of ROS can be used. Just replace `humble` with the name of the distribution you want to use.

If you aren't running a compatible version of Ubuntu for ROS2, don't want to make changes to your system, want to be able to easily switch between ROS verions, or just want to containerize your applications then you can use Docker containers. To get started, install [Docker Engine](https://docs.docker.com/engine/install/), sometimes referred to as Docker server.

Docker works by running self-contained systems called containers, which act kind of like a separate computer system but without all the overhead of a full virtual machine. Docker containers are based on Docker images, which provide the initial operating system, files, and programs for the Docker container. Fortunately, the developers of ROS provide Docker images for nearly all versions of ROS, which makes it very easy to get any version of ROS up and running on your system very quickly.

* To start a Docker container with ROS run this command:
```bash
docker run -it ros:humble
```

Once completed, you should enter a bash terminal in a fresh installation of Ubuntu and ROS! However, the container does not have access to any USB devices, source files on your system, or your network to be able to communicate with different computers. To do these things we'll need to add more arguments to the `docker run` command.

* To give access to source file on your system, mount a folder on your system to the docker container with the `-v` argument:
```bash
docker run -it -v /folder_to_mount_from_host:/location_to_mount_folder_in_container ros:humble
```

* To give access to a USB device, you need to mount the location of the USB device and give the container sudo access to your host system with `--privileged`:
```bash
docker run -it -v /dev/usb_to_mount:/dev/USB --privileged ros:humble
```

* To give access to your network, specify the network to use as the host's network with `--network host`.
```bash
docker run -it --network host ros:humble
```

It is worth noting that every time you use the `docker run` command, a new container with those specific arguments are created. So, if you want a container with access to all those things, you need to use all the arguments at once.

* As an example, if I wanted to make a ROS2 Humble container with access to a USB device at `/dev/ttyUSB0`, access to my host network, access to the source files at `~/rosflight_ws`, and give the container a convenient name of `rosflight` with the `--name` argument, I would use this command:
```bash
docker run --name rosflight -it -v /dev/ttyUSB0:/dev/USB --privileged --network host -v ~/rosflight_ws:/rosflight_ws ros:humble
```

Something the previous commands don't include is the ability to render GUI tools (like rqt_graph and Gazebo). Being able to render GUI tools can be done a number of ways, but an easy way is to give the container access to your system's X11 windowing system. Note that this isn't the most secure method and doesn't include GPU acceleration. (For more information about including GPU acceleration, refer to the guide found [here](https://roboticseabass.com/2021/04/21/docker-and-ros/).)

* To give access to the host's windowing system, use the following commands. Note that the base Docker image is `osrf/ros:humble-desktop-full`, as `ros:humble` doesn't include GUI tools in the image. Also, `xhost +` needs to be run once per login session and is not persistent.
```bash
xhost +
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" osrf/ros:humble-desktop-full
```

* To create a GUI enabled ROS container named `rosflight` with access to the host network, source files found at `~/rosflight_ws`, and all USB devices, use this command:
```bash
docker run --name rosflight -it -v /dev:/dev --privileged --network host -v ~/rosflight_ws:/rosflight_ws --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" osrf/ros:humble-desktop-full
```

!!! warning
    During testing, we found some strange behavior with ROS when running a GUI enabled container on a system with ROS already installed. If you need a GUI enabled system, try to do so on a system without ROS installed (or at the very least avoid sourcing/using ROS on your system). Also avoid having multiple GUI enabled containers running at once.

Some other useful Docker commands:

* To see all containers that exist on your system and their names:
```bash
docker ps -a
```

* To start a container named `rosflight` that was previously created with the `docker run` command:
```bash
docker start -i rosflight
```

* To open another bash terminal in a container named `rosflight` that is already running:
```bash
docker exec -it rosflight bash
```

* To delete a container named `rosflight`:
```bash
docker rm rosflight
```

You should now be able to create Docker containers with everything you need to run ROSflight! Docker is a very powerful tool and there is much more you can do with Docker to improve your development workflow. However, we're not interested in making an exhaustive Docker guide so please refer to the [Docker documentation](https://docs.docker.com/) or other [online guides](https://roboticseabass.com/2023/07/09/updated-guide-docker-and-ros2/) to learn more.
