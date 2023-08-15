# Installing/Setting up ROS

You will need to install ROS2 on both the companion computer and the base station laptop. Check out the official [ROS2 Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) page for details on how to do this. Make sure to install both the `ros-humble-desktop` (`ros-humble-ros-base` can be used instead of `ros-humble-desktop` if you don't need GUI tools or the simulation) and `ros-dev-tools` packages, or the equivalent packages for your version of ROS2.

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
git clone --recursive https://github.com/rosflight/rosflight2.git
```
Install dependencies:
```bash
sudo rosdep init
rosdep update
rosdep install -i --from-path ./ -y --ignore-src
```
Finally, build the packages:
```bash
cd ~/rosflight_ws
colcon build
```

!!! tip
    In order to ensure that new terminal windows are configured to use this workspace, you can add the line `source ~/rosflight_ws/install/setup.bash` to your `~/.bashrc` file or its equivalent on other systems. Change the path if your workspace is located somewhere other than  `~/rosflight_ws`. You'll also need to add `source /usr/share/gazebo/setup.sh` if you plan to use the Gazebo simulator.

## Running rosflight_io

The `rosflight_io` node is the bridge between ROS2 and the MAVLink communication with the flight controller. This node must be run on the computer that has the physical serial connection to your flight controller. To run this node, use something like the following command:
```bash
ros2 run rosflight rosflight_io --ros-args -p port:=/dev/ttyUSB0
```
Replace `/dev/ttyUSB0` with the port your flight controller is connected to.
