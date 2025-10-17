# Building and Flashing the Firmware


This guide assumes you are running Ubuntu 22.04 LTS, which is the currently supported development environment.

## Installing the ARM Embedded Toolchain

``` bash
sudo apt install gcc-arm-none-eabi
```

You can test the installation and check which version is installed by running `arm-none-eabi-gcc --version`.

## Building the Firmware from Source

### Building the firmware

1. To build the firmware, first clone the firmware repository:
    ```bash
    git clone https://github.com/rosflight/rosflight_firmware
    ```
1. Create build directory:
    ```bash
    cd rosflight_firmware && mkdir build && cd build
    ```
1. Build using: (`board_name` should be either `varmint` or `pixracer_pro`)
    ```bash
    cmake .. -DBOARD_TO_BUILD=<board_name> && make -j
    ```

### Install STM32CubeProgrammer

We use the STM32CubeProgrammer to flash the flight controller.

1. Download the programmer [here](https://www.st.com/en/development-tools/stm32cubeprog.html#get-software).
    You may have to enter your email to download the software.
1. Install the software by following the instructions in the downloaded package.

### Flashing the Varmint

!!! warning "Needed tools"

    You will need an ST-Link STM programmer to flash the firmware.
    You can find the one we use [on Mouser](https://www.mouser.com/ProductDetail/STMicroelectronics/ST-LINK-V2?qs=H4BOwPtf9MC1sDQ8j3cy4w%3D%3D&mgh=1&utm_id=22314976717&utm_source=google&utm_medium=cpc&utm_marketing_tactic=amercorp&gad_source=1&gad_campaignid=22304734959).

1. Plug the end of the ribbon cable into the 6-pin slot on the Varmint.
    You may have to make your own cable that connects the ST-Link to the 6-pin connector.

    !!! danger

        The Varmint has 2 6-pin connectors.
        **Do not** connect the ribbon cable to the port closest to the power wires.

    ![Varmint 6-pin cable](images/varmint_flash_instructions.png)

1. Power on the Varmint by connecting a battery to the board.

1. Open STM32CubeProgrammer.

1. Plug in the USB connector from the ST-Link to the computer. Select "Connect" in the STM Programmer. This should detect the STLink and connect automatically.
    ![Select "Connect"](images/stm_programmer_connect.png)

1. Navigate to the programming page.
    ![Navigate to programming page](images/stm32_programming_page.png)

1. Select the hex file that was just built and click "Open".
    ```
    /path/to/rosflight_firmware/build/boards/varmint_h7/varmint_10X/varmint10X.hex
    ```
    ![Select the previously built hex file](images/stm32_select_hex.png)

1. Select the appropriate options and press "Start Programming"
    ![Select options and start programming](images/stm32_select_options.png)

### Flashing the Pixracer Pro

Flashing the Pixracer Pro is a very similar process to flashing the Varmint.

1. The PixRacer Pro does not use the same 6-pin connector to connect to the ST-Link.
    Instead, it uses a TC2030 connector with retaining clips.

1. Power on the Pixracer Pro using a USB-C port.

2. Follow steps 3 through 7 of the [Varmint flashing guide](#flashing-the-varmint).
