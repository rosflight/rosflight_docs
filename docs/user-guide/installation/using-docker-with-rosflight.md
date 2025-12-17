# Using Docker with ROSflight

If you aren't running a compatible version of Linux for ROS2, don't want to make changes to your system, want to be able to easily switch between ROS verions, or just want to containerize your applications, then you can use Docker containers.

Docker works by running self-contained systems called containers, which act kind of like a separate computer system but without all the overhead of a full virtual machine.
Docker containers are based on Docker images, which provide the initial operating system, files, and programs for the Docker container.
Fortunately, the developers of ROS provide Docker images for nearly all versions of ROS, which makes it very easy to get any version of ROS up and running on your system very quickly.

!!! tip "Docker Tutorials"

    This guide is not a Docker tutorial.
    There are a lot of great tutorials and resources for learning what Docker is and how to use it.
    For more information on Docker, see [the Docker docs](https://docs.docker.com/).

    In this section, we will focus explicitly on how we use our Docker containers to set up a simulation environment, with some explanation of what we are doing.

!!! warning "Docker on Windows"

    If you are using Docker on Windows natively, be aware that GUI tools may not work.
    We have not tested GUI applications while using a Docker container in WSL.
    GUI applications in WSL (without Docker) work fine with a "native" installation of ROS2.

!!! note "Using Docker instead of a native installation"

    If you choose to use Docker, you will need to be attached to your Docker container any time you run ROS2 commands in the following tutorials.

    Note also that we have set up the container so that the build, install, log, and source files are mounted **from the host system to the container**, which enables you to have changes and builds that are persistent across Docker sessions.

### Installing Docker

To get started, install [Docker Engine](https://docs.docker.com/engine/install/), sometimes referred to as Docker server.
You can install the desktop version (Docker Desktop), but just the engine works fine.
Make sure you also have the `docker compose` utility installed.

You can check by running `docker compose` in a terminal.

### Building

We have provided the Docker files that we use [here](https://github.com/rosflight/rosflight_ros_pkgs/tree/main/docker).

1. Follow the steps 1-2 [from the sim installation guide](./installation-sim.md#installing-rosflight-roscopter-and-rosplane) to set up the file structure.
    It should look like:
    ```bash
    rosflight_ws/
        └── src/
            ├── roscopter/
            ├── rosflight_ros_pkgs/
            └── rosplane/
    ```

1. Navigate to the `rosflight_ws` directory:
    ```bash
    cd ~/rosflight_ws
    ```

1. Build the Docker container:
    ```bash
    docker compose -f src/rosflight_ros_pkgs/docker/compose.yaml build
    ```

    !!! tip "Filepaths"

        Here we use the build and other `docker compose` commands from the `rosflight_ws` directory.
        You can run all these commands from the `src/rosflight_ros_pkgs/docker` directory (or any other directory), but make sure to update the relative paths appropriately.


### Running the Docker container

To run the Docker container, we will first start the container in the background with:
```bash
docker compose -f src/rosflight_ros_pkgs/docker/compose.yaml up -d
```

Attach to the container with:
```bash
docker compose -f src/rosflight_ros_pkgs/docker/compose.yaml exec rosflight zsh
```

!!! tip
    You can attach as many times as you want from different terminals!

### Building the ROSflight workspace

1. Attach to the container using the above commands.

1. Build the packages in `rosflight_ws` using the [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) build tool:
    ```bash
    cd ~/rosflight_ws
    colcon build
    ```

    After the build, you should see output similar to:
    ```bash
    ---
    Finished <<< roscopter [1min 34s]
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

!!! warning
    During testing, we found some strange behavior with ROS when running a GUI enabled container on a system with ROS already installed.
    If you need a GUI enabled system, try to do so on a system without ROS installed (or at the very least avoid sourcing/using ROS on your system).
    Also avoid having multiple GUI enabled containers running at once.
