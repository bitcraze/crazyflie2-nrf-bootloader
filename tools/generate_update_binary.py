#!/usr/bin/env python3

import struct
from zlib import crc32
from os import system

system("arm-none-eabi-objcopy -I ihex -O binary vendor/nrf5sdk/components/softdevice/s130/hex/s130_nrf51_2.0.1_softdevice.hex _build/s130.bin")
system("mv _build/nrf51422_xxaa.bin _build/bootloader_only_for_swd.bin")

bl = open('_build/bootloader_only_for_swd.bin', 'rb').read()
sd = open('_build/s130.bin', 'rb').read()

# Strip the 4K MBR from the begining of the softdevice
sd = sd[4096:]

BOOTLOADER_PAGE = (256 - 20 - 4)
BOOTLOADER_ADDRESS =  BOOTLOADER_PAGE * 1024

STRUCT_PAGE = BOOTLOADER_PAGE - 1
STRUCT_ADDRESS = STRUCT_PAGE * 1024

SD_PAGE_LENGTH = ((len(sd)-1) // 1024) + 1
SD_PAGE = STRUCT_PAGE - SD_PAGE_LENGTH
SD_ADDRESS = SD_PAGE * 1024

BL_PAGE_LENGTH = ((len(bl)-1) // 1024) + 1
BL_PAGE = SD_PAGE - BL_PAGE_LENGTH
BL_ADDRESS = BL_PAGE * 1024

SD_DEST_PAGE = 4

print('Bootloader: {} bytes, {} pages, page {} length {:x}'.format(len(bl), BL_PAGE_LENGTH, BL_PAGE, len(bl)))
print('SD: {} bytes, {} pages, page {} length {:x}'.format(len(sd), SD_PAGE_LENGTH, SD_PAGE, len(sd)))

config_struct = struct.pack('<BHHLLHHLL', 0xCF, SD_PAGE, SD_DEST_PAGE, len(sd), crc32(sd), BL_PAGE, BOOTLOADER_PAGE, len(bl), crc32(bl))
crc = crc32(config_struct)
config_struct += struct.pack('<L', crc)

print('Config struct: {} bytes, page {} at address {} CRC: {:08X}'.format(len(config_struct), STRUCT_PAGE, STRUCT_ADDRESS, crc))

# Generate the out file, softdevice first, then bootloader, then config struct. All aligned on 1024 bytes page
out = open('_build/sd130_bootloader.bin', 'wb')
out.write(bl)
out.write(b'\xFF' * (1024 - (len(bl) % 1024)))
out.write(sd)
out.write(b'\xFF' * (1024 - (len(sd) % 1024)))
out.write(config_struct)
out.write(b'\xFF' * (1024 - (len(config_struct) % 1024)))
out.close()

print("Update binary saved in _build/sd130_bootloader.bin")