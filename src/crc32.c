/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 nRF51 Bootloader
 * Copyright (c) 2026, Bitcraze AB
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * crc32.c - Standard CRC-32 (ISO 3309), same result as crcSlow() but fast
 * enough to checksum the whole flash
 */
#include "crc32.h"

// Reflected polynomial 0xEDB88320, one entry per nibble
static const uint32_t crcNibbleTable[16] = {
  0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
  0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
  0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
  0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C,
};

uint32_t crc32Calculate(const void *buffer, uint32_t size)
{
  const uint8_t *data = buffer;
  uint32_t crc = 0xFFFFFFFFUL;

  for (uint32_t i = 0; i < size; i++) {
    crc ^= data[i];
    crc = (crc >> 4) ^ crcNibbleTable[crc & 0x0F];
    crc = (crc >> 4) ^ crcNibbleTable[crc & 0x0F];
  }

  return crc ^ 0xFFFFFFFFUL;
}
