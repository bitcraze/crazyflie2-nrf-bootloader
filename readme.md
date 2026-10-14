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

### Creating and flashing an update binary

``` bash
make
./tools/generate_update_binary.py
```

This writes `_build/sd130_bootloader.bin`, which contains the bootloader, the S130 softdevice and a flag page describing where each of them goes, with their CRC32s.
The script renames `_build/nrf51422_xxaa.bin` to `_build/nrf_bootloader.bin`, so run `make` again before generating a new update binary.

The update binary is flashed with the running bootloader so that it ends just below the bootloader, overwriting the nRF51 firmware.
On the next restart the MBS checks the flag page and the CRC32s, copies the softdevice and the bootloader into place and only then erases the flag page, so an interrupted copy is simply done again on the following restart.
The nRF51 firmware has to be flashed again afterwards.

Release archives carry the update binary as the `bootloader+softdevice` file, which the Crazyflie clients flash before the nRF51 firmware when the Crazyflie needs it.

Protocol
--------

The protocol of both Crazyflie bootloaders, including the commands added in protocol version 0x11 for flashing several Crazyflies at once, is described in the readme of the [crazyflie2-stm-bootloader repository](https://github.com/bitcraze/crazyflie2-stm-bootloader).

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
