# Crazyflie 2.0 nRF51 Bootloader [![CI](https://github.com/bitcraze/crazyflie2-nrf-bootloader/workflows/CI/badge.svg)](https://github.com/bitcraze/crazyflie2-nrf-bootloader/actions?query=workflow%3ACI)


Crazflie 2.0 bootloader firmware that runs in the nRF51. See readme.md in
crazyflie-nrf-firmware repos for more information about flash and boot
architecture.

Compiling requires the nRF51_SDK and S110 packages.

        ./tools/build/download_deps

will download the zips and unpack them.
If you want to download manually from the Nordic semiconductor website, you
will find the details in nrf51_sdk/readme and s110/readme.

License
-------

Most of the code is licensed under LGPL-3.0.

Some files under src/ble/ are modified from Nordic semiconductor examples.

Downloading the nrf51_sdk and nordic S110 softdevice require to aquire one
of the Nordic Semiconductor development kit. Discussion is in progress
with Nordic to solve this situation.

Compiling
---------

To compile arm-none-eabi- tools from https://launchpad.net/gcc-arm-embedded
should be in the path.

Architecture
--------

Check out the readme of the [crazyflie-stm-bootloader repository](https://github.com/bitcraze/crazyflie2-stm-bootloader) to understand the interplay between the stm and nrf bootloaders.
