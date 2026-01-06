# Using an In-Circuit Debugger

!!! warning "TODO"
    Update this when hardware support is finalized.

Debugging an STM32-based board is accomplished with an ST-LINK/V2 in-circuit debugger and programmer. We have had the best luck with the official version from STMicroelectronics. These devices are reasonably priced, and are available [directly from STMicroelectronics](https://www.st.com/en/development-tools/st-link-v2.html) or from vendors such as [Digi-Key](https://www.digikey.com/product-detail/en/stmicroelectronics/ST-LINK-V2/497-10484-ND/2214535), [Mouser](https://www.mouser.com/ProductDetail/STMicroelectronics/ST-LINK-V2?qs=H4BOwPtf9MC1sDQ8j3cy4w%3D%3D), and [Newark](https://www.newark.com/stmicroelectronics/st-link-v2/icd-programmer-usb-2-0-jtag-for/dp/46T6935).

The following guide will show you how to get the in-circuit debugger running with either the Visual Studio Code or QtCreator IDE. Start with the steps in the [General Setup](#general-setup) section, then move on to either the [VS Code](#vs-code) or [STM32cubeIDE](#stm32cubeide) sections depending on your choice of IDE.

This guide assumes you are running Ubuntu 22.04 LTS, which is the currently supported development environment.


## General Setup

Follow the guide in [Building and Flashing](../../user-guide/rosflight-firmware/building-and-flashing.md) to install the compiler toolchain.

Also make sure you have configured your computer as described in the [Serial Port Configuration](../../user-guide/hardware-and-rosflight/flight-controller-setup.md#serial-port-configuration) section of the user guide.

### Connect debugger to flight controller

The ST-LINK/V2 connects to the microcontroller using the Serial Wire Debug (SWD) interface. You will need to connect the `GND`, `NRST`, `SWDIO`, and `SWCLK` lines of the ST-LINK/V2 to your flight controller. On many F4 boards, these lines are pinned out through a 4-position JST SH connector, although that connector is not always populated. Refer to the documentation for your specific board for details.

The official ST-LINK/V2 also needs a target voltage reference on pin 1 or 2, which for the F4 boards is 3.3V. However, there is no externally accessible 3.3V pinout on the F4 boards. An easy solution to this is to connect pin 19 (VDD 3.3V) of the ST-LINK/V2 to pin 1 or 2 of the ST-LINK/V2 (Target VCC) to provide the voltage reference. You will also need to power the board from another source, either through the USB port or over the servo rail. Note that this connection is not required for the cheap clone versions of the ST-LINK/V2.

## VS Code

You can install Visual Studio Code by downloading the latest version from [their website](https://code.visualstudio.com). Follow the steps below to configure debugging with the in-circuit debugger.

You should open the root firmware directory for editing and debugging, e.g. `code /path/to/rosflight_firmware`.

### Install OpenOCD

OpenOCD (On-Chip Debugger) is the software that will control the debugger. Install from the `apt` repositories:

``` bash
sudo apt install openocd
```

### Install Cortex-Debug extension

The embedded debugging functionality is provided by the `Cortex-Debug` extension. Install using the VS Code GUI, or from VS Code press `Ctrl+P` then type `ext install marus25.cortex-debug`.

Steps for configuring this extension are described next.

### Download SVD file

A System View Description (SVD) file describes the configuration (CPU, peripherals, registers, etc.) of the microcontroller. The Cortex-Debug extension can make use of an SVD file to provide more detailed debugging information, such as the ability to inspect register values.

SVD files can be downloaded from STMicroelectronics. The files for the F4 are contained in the ZIP file that can be downloaded [here](https://www.st.com/resource/en/svd/stm32f4_svd.zip), and the relevant file is `STM32F405.svd`. The files for the F1 are contained in the ZIP file that can be downloaded [here](https://www.st.com/resource/en/svd/stm32f1_svd.zip), and the relevant file is `STM32F103.svd`. Put those files in a convenient location.

### Configure build step

You can configure VS Code to run `make` for you when you press `Ctrl+Shift+B`. To do this, put the following in `.vscode/tasks.json` inside your firmware working directory:

``` json
{
  // See https://go.microsoft.com/fwlink/?LinkId=733558
  // for the documentation about the tasks.json format
  "version": "2.0.0",
  "tasks": [
    {
      "type": "shell",
      "label": "make",
      "command": "make",
      "args": ["DEBUG=GDB"],
      "group": {
        "kind": "build",
        "isDefault": true
      }
    }
  ]
}
```

Note that by default, this will only build the F4 (Revo) firmware. To build the F1 firmware, you will need to edit this to add the argument `BOARD=NAZE`.

### Configure debugging

To configure in-circuit debugging of F4 and F1 targets, put something like the following in `.vscode/launch.json` inside your firmware working repository:

``` json
{
  // Use IntelliSense to learn about possible attributes.
  // Hover to view descriptions of existing attributes.
  // For more information, visit: https://go.microsoft.com/fwlink/?linkid=830387
  "version": "0.2.0",
  "configurations": [
    {
      "name": "STM32F405",
      "type": "cortex-debug",
      "request": "launch",
      "servertype": "openocd",
      "cwd": "${workspaceRoot}",
      "executable": "${workspaceRoot}/boards/airbourne/build/rosflight_REVO_Debug.elf",
      "device": "STM32F405",
      "svdFile": "/path/to/STM32F405.svd",
      "configFiles": [
        "interface/stlink-v2.cfg",
        "target/stm32f4x.cfg"
      ],
      "runToMain": true
    },
    {
      "name": "STM32F103",
      "type": "cortex-debug",
      "request": "launch",
      "servertype": "openocd",
      "cwd": "${workspaceRoot}",
      "executable": "${workspaceRoot}/boards/breezy/build/rosflight_NAZE_Debug.elf",
      "device": "STM32F103",
      "svdFile": "/path/to/STM32F103.svd",
      "configFiles": [
        "interface/stlink-v2.cfg",
        "target/stm32f1x.cfg"
      ],
      "runToMain": true
    }
  ]
}
```

Be sure to edit the values of `"svdFile"` to point to the respective SVD files you downloaded earlier.

To start debugging, enter the debug pane in VS Code, select the desired configuration, then click the green arrow to start debugging. The shortcut key `F5` will also launch the last-selected debug configuration.

More details on the configuration and use of the `Cortex-Debug` extension can be found [here](https://marcelball.ca/projects/cortex-debug/cortex-debug-launch-configurations/) and [here](https://github.com/Marus/cortex-debug).

## STM32cubeIDE
