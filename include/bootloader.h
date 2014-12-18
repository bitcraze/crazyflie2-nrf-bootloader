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
#ifndef __BOOTLOADER_H__
#define __BOOTLOADER_H__

#include <stdbool.h>
#include <ble.h>
#include "crtp.h"

#define BUFFER_PAGES 1
#define FLASH_PAGES  232
#define FLASH_START  88

#define PAGE_SIZE 1024

#define FLASH_BASE 0x00000000
#define FLASH_SIZE (256*1024)

#define MBR_SIZE (4*1024)
#define MBS_SIZE (4*1024)

//Public functions
bool bootloaderProcess(CrtpPacket *packet);
void bootloaderOnSdEvt(uint32_t evt);

/* XXX: Protocol version has to be increased each time a command is
 * added or modified!
 */
#define PROTOCOL_VERSION 0x10

#define CPUID_LEN 12

/******* Reset (to fw) ******/
#define CMD_RESET_INIT 0xFF
#define CMD_RESET 0xF0

/******* GetInfo ******/
#define CMD_GET_INFO 0x10
//Parameters ... void
//Returns:
typedef struct {
  short pageSize;
  short nBuffPages;
  short nFlashPages;
  short flashStart;
  char  cpuId[CPUID_LEN];
  char version;
} __attribute__((__packed__)) GetInfoReturns_t;

/****** SetAddress ****/
#define CMD_SET_ADDRESS 0x11
//Parameters:
typedef struct {
  char address[5];
} __attribute__((__packed__)) SetAddressParameters_t;
//Returns ... void

/****** LoadBuffer ****/
#define CMD_LOAD_BUFFER 0x14
//Parameters:
typedef struct {
  unsigned short page;
  unsigned short address;
} __attribute__((__packed__)) LoadBufferParameters_t;
//Returns ... void

/****** ReadBuffer ****/
#define CMD_READ_BUFFER 0x15
//Parameters:
typedef struct {
  unsigned short page;
  unsigned short address;
} __attribute__((__packed__)) ReadBufferParameters_t;
//Returns ... Same as parameters but with data

/****** CopyFlash init ****/
#define CMD_COPY_FLASH_INIT 0x16
//Parameters ... void
//Returns:
typedef struct {
  char  willdo;
  unsigned char error;
} __attribute__((__packed__)) CopyFlashReturns_t;

/****** CopyFlash ****/
#define CMD_COPY_FLASH 0x17
//Parameters ... void
//Returns ... Resets just after the command

/****** WriteFlash ****/
#define CMD_WRITE_FLASH 0x18
//Parameters:
typedef struct {
  unsigned short bufferPage;
  unsigned short flashPage;
  unsigned short nPages;
} __attribute__((__packed__)) WriteFlashParameters_t;
//Returns ... Same as parameters but with data
typedef struct {
  char  done;
  unsigned char error;
} __attribute__((__packed__)) WriteFlashReturns_t;

#define CMD_FLASH_STATUS 0x19
//Parameters ... void
//Returns:
typedef struct {
  unsigned char  done;
  unsigned char error;
} __attribute__((__packed__)) FlashStatusReturns_t;

/****** ReadBuffer ****/
#define CMD_READ_FLASH 0x1C
//Parameters:
typedef struct {
  unsigned short page;
  unsigned short address;
} __attribute__((__packed__)) ReadFlashParameters_t;
//Returns ... Same as parameters but with data


/************** Flag page for copy-flash operation *********/
//This structure shall be written in the flash page just before the bootloader
//It will be erased by the MBS when the flash operation is completed
typedef struct {
  uint8_t header;        //Should be 0xCF

  uint16_t sdSrcPage;
  uint16_t sdDestPage;
  uint32_t sdLength;
  uint32_t sdCrc32;

  uint16_t blSrcPage;
  uint16_t blDestPage;
  uint32_t blLength;
  uint32_t blCrc32;

  uint32_t crc32;      //CRC32 of the complete structure except crc32
} __attribute__((__packed__)) CopyFlashFlags_t;


#endif //__BOOTLOADER_H__
