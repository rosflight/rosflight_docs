# Flight Controller Setup

!!! Note
    This page contains instructions for flashing pre-built firmware binaries.
    For instructions on building and flashing from source, see [Building and Flashing](../developer-guide/firmware/building-and-flashing.md) in the Developer Guide.

## Compatible Hardware

Currently, the ROSflight firmware only supports an in-development board from AeroVironment, the Varmint. This board is not yet commercially available, but plans are for it to start being sold early 2024.

We also plan to add support for the [CubePilot Orange](https://docs.cubepilot.org/user-guides/autopilot/the-cube-module-overview), which uses the same H7 processor as the Varmint.

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

## Flashing Firmware

!!! warning "TODO"
    Update flashing instructions.

### F4 Boards

* Install the dfu-util utility

``` bash
sudo apt install dfu-util
```

* Download the latest rosflight-F4.bin file, [found here](https://github.com/rosflight/rosflight_firmware/releases)
* Put the board in bootloader mode (short the boot pins while restarting the board by cycling power)

!!! tip
    dfu-util auto-detects F4-based boards. Try `dfu-util -l` to make sure your board is in bootloader mode

* Flash the firmware to the device
``` bash
    dfu-util -a 0 -s 0x08000000 -D rosflight-F4.bin
```
