# Crazyflie 2.0 nRF51 Bootloader [![CI](https://github.com/bitcraze/crazyflie2-nrf-bootloader/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie2-nrf-bootloader/actions?query=workflow%3ACI)


Crazflie 2.0 bootloader firmware that runs in the nRF51. See readme.md in
crazyflie-nrf-firmware repos for more information about flash and boot
architecture.

Just after clonning the repository:
``` bash
./tools/build/fetch_dependencies
```

This will download and patch the Nordic's nrf5 bootloader

Working with the bootloader
---------------------------

In order to work on this bootloader you must have a debug probe and your Crazyflie fited with the nRF debug port from the debug adapter kit.
This is because, the bootloader is part of the safe boot sequence of the Crazyflie and so flashing a non-functional bootloader will require a debug probe to get the Crazyflie back to work.


Once a stable version of the bootloader has been produced, it is possible to make an update binary that will flash both the bootloader and the bluetooth softdevice.
This update binary can be flashed over radio like a normal firmware.

Compiling
---------

To compile you must hase arm-none-eabi- tools in the path, Python 3 and git.

Flashing requires a jlink debug probe and nrfjprog.
``` bash
make
make flash
```

Architecture
--------

Check out the readme of the [crazyflie-stm-bootloader repository](https://github.com/bitcraze/crazyflie2-stm-bootloader) to understand the interplay between the stm and nrf bootloaders.
