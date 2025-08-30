# Crazyflie 2.0 nRF51 Bootloader [![CI](https://github.com/bitcraze/crazyflie2-nrf-bootloader/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie2-nrf-bootloader/actions?query=workflow%3ACI)


Crazflie 2.0 bootloader firmware that runs in the nRF51. See readme.md in
[crazyflie2-nrf-firmware repo](https://github.com/bitcraze/crazyflie2-nrf-firmware) for more information about flash and boot
architecture.

Just after cloning the repository:
``` bash
./tools/build/fetch_dependencies
```

This will download and patch the Nordic's nrf5 bootloader.

Working with the bootloader
---------------------------

In order to work on this bootloader, you must have a debug probe and a Crazyflie fitted with the nRF debug port from the debug adapter kit.
This is because the bootloader is part of the safe boot sequence of the Crazyflie, and flashing a non-functional bootloader will require a debug probe to restore the Crazyflie.


Once a stable version of the bootloader has been produced, you can create an update binary that flashes both the bootloader and the Bluetooth softdevice.
This update binary can be flashed over the radio like normal firmware.

Compiling
---------

To compile, you must have the `arm-none-eabi-` tools in your `PATH`, as well as Python 3 and `git`.

Flashing requires a **J-Link** debug probe and `nrfjprog`.
``` bash
make
make flash
```

Architecture
--------

Check out the readme of the [crazyflie2-stm-bootloader repository](https://github.com/bitcraze/crazyflie2-stm-bootloader) to understand the interplay between the STM and nRF bootloaders.
