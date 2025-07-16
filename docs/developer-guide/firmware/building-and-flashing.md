# Building and Flashing the Firmware


This guide assumes you are running Ubuntu 22.04 LTS, which is the currently supported development environment.

## Installing the ARM Embedded Toolchain

``` bash
sudo apt install gcc-arm-none-eabi
```

You can test the installation and check which version is installed by running `arm-none-eabi-gcc --version`.

## Building the Firmware from Source

Now that we have the compiler installed, simply clone the ROSflight firmware repository, pull down the submodules, and build:

``` bash
git clone --recursive https://github.com/rosflight/rosflight_firmware
cd rosflight_firmware
mkdir build 
cd build 
cmake .. -DBUILD_VARMINT=TRUE
make
```

## Flashing Newly-Built Firmware

!!! danger "TODO"
    Update this when hardware support is finalized.

First, make sure you have configured your computer as described in the [Serial Port Configuration](../../user-guide/concepts/flight-controller-setup.md#serial-port-configuration) section of the user guide.

### F4

Flash the firmware to the board by running `make BOARD=REVO flash`.
If necessary, specify the serial port with `make BOARD=REVO SERIAL_DEVICE=/dev/ttyACM0 flash`.
