# Flight Controller Setup

!!! Note
    This page contains instructions for flashing pre-built firmware binaries.
    For instructions on building and flashing from source, see [Building and Flashing](../../developer-guide/firmware/building-and-flashing.md) in the Developer Guide.

## Compatible Hardware

Currently, the ROSflight firmware supports:

1. An in-development board from AeroVironment, the **Varmint**.
This board has an integrated Jetson Orin, but is not yet commercially available.
1. **[MRO's Pixracer Pro](https://store.3dr.com/pixracer-pro/)**, which has the same H7 processor.

!!! note "Supporting your own board"

    It is possible to write your own board support package (BSP).
    If you do create your own support package for the ROSflight firmware, please contribute back!

## Serial Port Configuration

!!! tip
    You can see which groups you are a member of by running `groups $USER` on the command line.

The following bullet point is necessary:

* Be sure your user is in the `dialout` and `plugdev` groups so you have access to the serial ports. You will need to log out and back in for these changes to take effect.
``` bash
sudo usermod -aG dialout,plugdev $USER
```

If you experience issues, you may need one or both of the next two bullet points:

* Temporarily stop the modem-manager (Sometimes, Linux thinks the device is a modem -- this command will be effective until next boot, or until you run the command again with `start` in place of `stop`)
``` bash
sudo systemctl stop ModemManager.service
```

* Add the custom udev rule so Linux handles the flight controller properly (copy the following as `/etc/udev/rules.d/45-stm32dfu.rules`)
``` bash
# DFU (Internal bootloader for STM32 MCUs)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0664", GROUP="plugdev"
```

!!! Tip
    You can permanently disable the ModemManager if you do not need it, then you won't have to disable it every time you reboot:
    ```
    sudo systemctl disable ModemManager.service
    ```
    Replace `disable` with `enable` to revert (i.e. if you find some other program you use needs access to it).
    Or you can uninstall it entirely from your system:
    ```
    sudo apt purge modemmanager
    ```

## Building and Flashing the Firmware

Follow the [building and flashing firmware guide](../../developer-guide/firmware/building-and-flashing.md) to build the ROSflight firmware and flash it to your flight controller.

## LEDs

The meaning of the various LEDs is summarized in the following table. The colors of the LEDs may change depending on your specific board:

| LED           | On            | Off              | Slow Blink       | Fast Blink       |
|---------------|---------------|------------------|------------------|------------------|
| Power (Blue)  | Board powered | -                | -                | -                |
| Info (Green)  | RC control    | Offboard control | -                | -                |
| Warning (Red) | Armed         | Disarmed         | Error (disarmed) | Failsafe (armed) |
