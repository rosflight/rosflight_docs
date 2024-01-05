# Building and Flashing the Firmware

!!! warning "TODO"
    Update this when hardware support is finalized.

This guide assumes you are running Ubuntu 18.04, which is the currently supported development environment.

## Installing the ARM Embedded Toolchain

``` bash
sudo apt install gcc-arm-none-eabi
```

You can test the installation and check which version is installed by running `arm-none-eabi-gcc --version`.

## Building the Firmware from Source

Now that we have the compiler installed, simply clone the ROSflight firmware repository, pull down the submodules, and build:

``` bash
git clone https://github.com/rosflight/rosflight_firmware
cd firmware
git submodule update --init --recursive
make
```

To build only the F4 firmware, use `make BOARD=REVO`. To build only the F1 firmware, use `make BOARD=NAZE`.

## Flashing Newly-Built Firmware

First, make sure you have configured your computer as described in the [Serial Port Configuration](../../user-guide/flight-controller-setup.md#serial-port-configuration) section of the user guide.

### F4

Flash the firmware to the board by running `make BOARD=REVO flash`.
If necessary, specify the serial port with `make BOARD=REVO SERIAL_DEVICE=/dev/ttyACM0 flash`.

### F1

Flash the firmware to the board by running `make BOARD=NAZE flash`
If necessary, specify the serial port with `make BOARD=REVO SERIAL_DEVICE=/dev/ttyUSB0 flash`.
