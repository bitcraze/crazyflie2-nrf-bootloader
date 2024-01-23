/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 nRF51 Bootloader
 * Copyright (c) 2014, Bitcraze AB
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
 */
#ifndef __CRTP_H__
#define __CRTP_H__

#include <stdint.h>

#define CRTP_MAX_DATA_SIZE 31

typedef struct crtpPacket_s {
  union {
    struct {
      union {
        uint8_t header;
        struct {
          uint8_t channel:2;
          uint8_t reserved:2;
          uint8_t port:4;
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE];
    } __attribute__((packed));
    char raw[CRTP_MAX_DATA_SIZE +1];
  };
  uint8_t datalen;
} CrtpPacket;

#endif //__CRTP_H__
