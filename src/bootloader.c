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
 *
 * Implementation of nRF51 CRTP Bootloader
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <strings.h>

#include "crtp.h"
#include "bootloader.h"
#include "crc.h"
#include "systick.h"

#include <nrf.h>
#include <ble.h>
#include <nrf_soc.h>

#include "version.h"


static char buffer[PAGE_SIZE*BUFFER_PAGES] = {0x42};

static enum {bootloaderIdle,
             bootloaderErasing,
             bootloaderFlashing,
             bootloaderFlashOk,
             bootloaderFlashFail} bootloaderState = bootloaderIdle;
static int currentFlashPage;
static int currentBufferPage;
static int flashError;
static WriteFlashParameters_t flashParams = {0};


static int verifyCopyFlashFlags() {
  CopyFlashFlags_t *flashFlags = (void*)FLASH_BASE+((FLASH_PAGES-1)*PAGE_SIZE);
  int error = 0;
  uint32_t crc = crcSlow(flashFlags, sizeof(CopyFlashFlags_t)-4);

  // Check that the structure has a good checksum
  if ( (flashFlags->header != 0xcf) || (crc != flashFlags->crc32) ) {
    error = 1;
    goto end;
  }

  // Check memory boundaries
  if (flashFlags->sdLength != 0) {
    if ( (((flashFlags->sdDestPage*PAGE_SIZE)+flashFlags->sdLength) > (FLASH_SIZE-MBS_SIZE)) ||
         ((flashFlags->sdDestPage*PAGE_SIZE) < MBR_SIZE) ) {
      error = 2;
      goto end;
    }
  }
  if (flashFlags->blLength != 0) {
    if ( (((flashFlags->blDestPage*PAGE_SIZE)+flashFlags->blLength) > (FLASH_SIZE-MBS_SIZE)) ||
         ((flashFlags->blDestPage*PAGE_SIZE) < MBR_SIZE) ) {
      error = 3;
      goto end;
    }
  }

  // Check images checksum
  if (flashFlags->sdLength != 0) {
    crc = crcSlow((void*)(FLASH_BASE+(flashFlags->sdSrcPage*PAGE_SIZE)), flashFlags->sdLength);
    if (crc != flashFlags->sdCrc32) {
      error = 4;
      goto end;
    }
  }
  if (flashFlags->blLength != 0) {
    crc = crcSlow((void*)(FLASH_BASE+(flashFlags->blSrcPage*PAGE_SIZE)), flashFlags->blLength);
    if (crc != flashFlags->blCrc32) {
      error = 5;
      goto end;
    }
  }

  end:
  return error;
}


// Pass a packet.
// Return true and replaces the packet content if has data to communicate back
bool bootloaderProcess(CrtpPacket *packet) {
  bool tx = false;

  if ((packet->datalen != 0xFFU) &&
         (packet->datalen >= 2) &&
         (packet->header == 0xFF) &&
         (packet->data[0] == 0xFE)) {

    if (packet->data[1] == CMD_GET_INFO) {
      GetInfoReturns_t * info = (GetInfoReturns_t *)&packet->data[2];

      info->pageSize = PAGE_SIZE;
      info->nBuffPages = BUFFER_PAGES;
      info->nFlashPages = FLASH_PAGES;
      info->flashStart = FLASH_START;
      //memcpy(info->cpuId, cpuidGetId(), CPUID_LEN);
      bzero(info->cpuId, CPUID_LEN);
      info->version = PROTOCOL_VERSION;
      info->version_major = VERSION_MAJOR | (VERSION_DIRTY)?0x8000U:0x000;
      info->version_minor = VERSION_MINOR;
      info->version_patch = VERSION_PATCH;

      packet->datalen = 2+sizeof(GetInfoReturns_t);

      tx = true;
    }
/*    else if (packet->data[1] == CMD_SET_ADDRESS)
    {
      SetAddressParameters_t * addressPk;
      addressPk = (SetAddressParameters_t *)&packet.data[1];

      esbSetAddress(addressPk->address);
    }*/
    else if (packet->data[1] == CMD_LOAD_BUFFER) {
      int i = 0;
      LoadBufferParameters_t *params = (LoadBufferParameters_t *)&packet->data[2];
      char *data = (char*)&packet->data[2 + sizeof(LoadBufferParameters_t)];

      //Fill the buffer with the given datas
      for (i = 0; i < (packet->datalen - (2 + sizeof(LoadBufferParameters_t)))
        && (i + (params->page * PAGE_SIZE) + params->address) < (BUFFER_PAGES * PAGE_SIZE); i++)
        buffer[(i+(params->page*PAGE_SIZE)+params->address)] = data[i];

    }
    else if (packet->data[1] == CMD_READ_BUFFER) {
      int i = 0;
      ReadBufferParameters_t *params = (ReadBufferParameters_t *)&packet->data[2];
      char *data = (char*)&packet->data[2 + sizeof(ReadBufferParameters_t)];

      //Return the datas required
      for (i = 0; i < 25 && (i + (params->page * PAGE_SIZE) + params->address) < (BUFFER_PAGES*PAGE_SIZE); i++)
        data[i] = buffer[(i + (params->page * PAGE_SIZE) + params->address)];

      packet->datalen += i;

      tx = true;
    } else if (packet->data[1] == CMD_READ_FLASH) {
      int i = 0;
      ReadFlashParameters_t *params = (ReadFlashParameters_t *)&packet->data[2];
      char *data = (char*)&packet->data[2 + sizeof(ReadFlashParameters_t)];
      char *flash = (char*)FLASH_BASE;

      //Return the datas required
      for (i = 0; i < 25 && (i + (params->page * PAGE_SIZE) + params->address) < (FLASH_PAGES * PAGE_SIZE); i++) {
        //data[i] = flash[(i+(params->page*PAGE_SIZE)+params->address)];
        //data[i] = *((char*)(FLASH_BASE+i+(params->page*PAGE_SIZE)+params->address));
        data[i] = flash[(i + (params->page * PAGE_SIZE) + params->address)];
      }

      packet->datalen += i;

      tx = true;
    } else if (packet->data[1] == CMD_WRITE_FLASH) {
      //int i;
      unsigned int error = 0xFF;
      //int flashAddress;
      //uint32_t *bufferToFlash;
      WriteFlashParameters_t *params = (WriteFlashParameters_t *)&packet->data[2];
      WriteFlashReturns_t *returns = (WriteFlashReturns_t *)&packet->data[2];

      if (bootloaderState != bootloaderIdle)
        goto finally;

      //Test if it is an acceptable write request
      if ((params->flashPage<FLASH_START) || (params->flashPage >= FLASH_PAGES) ||
           ((params->flashPage + params->nPages) > FLASH_PAGES) || (params->bufferPage >= BUFFER_PAGES)
         ) {
        //Return a failure answer
        returns->done = 0;
        returns->error = 1;
        packet->datalen = 2+sizeof(WriteFlashReturns_t);
        tx = 1;
      }
      // Else, if everything is OK, flash the page(s)
      else {

        flashParams = *params;
        currentFlashPage = flashParams.flashPage;
        currentBufferPage = flashParams.bufferPage;
        bootloaderState = bootloaderErasing;

        // Start the flashing process. The rest is handled by the event handler state machine
        if (sd_flash_page_erase(currentFlashPage) != NRF_SUCCESS) {
          error = 1;
          goto failure;
        }

        goto finally;

        failure:
        //If the write procedure failed, send the error packet
        //TODO: see if it is necessary or wanted to send the reason as well
        returns->done = 0;
        returns->error = error;
        packet->datalen = 2+sizeof(WriteFlashReturns_t);
        tx = true;

        finally:
        ; //None...
      }
    } else if (packet->data[1] == CMD_RESET_INIT) {
      packet->raw[0] = 0xff;
      packet->raw[1] = 0xfe;
      packet->raw[2] = CMD_RESET_INIT;

      memcpy(&(packet->raw[3]), (uint32_t*)NRF_FICR->DEVICEADDR, 6);

      packet->datalen = 8;

      tx = true;
    } else if (packet->data[1] == CMD_RESET) {
      int start = systickGetTick();
      while ((systickGetTick()-start) < 100);
      if ((packet->datalen >= 3) && (packet->data[2] == 0)) {
        //Set bit 0x40 forces boot to bootloader
        NRF_POWER->GPREGRET |= 0x40U;
      } else {
        //Set bit 0x20 forces boot to firmware
        NRF_POWER->GPREGRET |= 0x20U;
      }
      NVIC_SystemReset();
    } else if (packet->data[1] == CMD_COPY_FLASH_INIT) {
      CopyFlashReturns_t *returns = (void*)&packet->data[2];

      returns->error = verifyCopyFlashFlags();
      returns->willdo = (returns->error) ? 0 : 1;
      packet->datalen = 2 + sizeof(CopyFlashReturns_t);
      tx = true;
    }
  }

  /* Flashing asynchronous work, make sure to run them when they can return
   * data to send back
   */
  if ((tx == false) && (packet->datalen != 0xFFU)) {
    WriteFlashReturns_t *returns = (WriteFlashReturns_t *)&packet->data[2];
    switch (bootloaderState) {
      case bootloaderFlashOk:
        packet->data[1] = CMD_WRITE_FLASH;
        returns->done = 1;
        returns->error = 0;
        packet->datalen = 2+sizeof(WriteFlashReturns_t);
        tx = true;

        bootloaderState = bootloaderIdle;
        break;
      case bootloaderFlashFail:
        packet->data[1] = CMD_WRITE_FLASH;
        returns->done = 0;
        returns->error = flashError;
        packet->datalen = 2 + sizeof(WriteFlashReturns_t);
        tx = true;

        bootloaderState = bootloaderIdle;
        break;
      default:
        break;
    }
  }

  return tx;
}

void bootloaderOnSdEvt(uint32_t evt) {
  int err;

  if (evt == NRF_EVT_FLASH_OPERATION_SUCCESS) {
    switch (bootloaderState) {
      case bootloaderErasing:
        currentFlashPage += 1;
        if (currentFlashPage < (flashParams.flashPage + flashParams.nPages)) {
          // Erase next page
          if (sd_flash_page_erase(currentFlashPage) != NRF_SUCCESS) {
            flashError = 2;
            bootloaderState = bootloaderFlashFail;
          }
        } else {
          // Start flashing
          currentFlashPage = flashParams.flashPage;
          currentBufferPage = flashParams.bufferPage;
          bootloaderState = bootloaderFlashing;

          err = sd_flash_write((uint32_t *)(FLASH_BASE + (currentFlashPage * PAGE_SIZE)),
                               (uint32_t const *)((uint32_t)buffer) + (currentBufferPage * PAGE_SIZE),
                               256);
          if (err != NRF_SUCCESS) {
            flashError = 3;
            bootloaderState = bootloaderFlashFail;
          }
        }
        break;
      case bootloaderFlashing:
        currentFlashPage += 1;
        currentBufferPage += 1;

        if (currentFlashPage < (flashParams.flashPage + flashParams.nPages)) {
          err = sd_flash_write((uint32_t *)(FLASH_BASE + (currentFlashPage * PAGE_SIZE)),
                               (uint32_t const *)((uint32_t)buffer) + (currentBufferPage * PAGE_SIZE),
                               256);
          if (err != NRF_SUCCESS) {
            flashError = 3;
            bootloaderState = bootloaderFlashFail;
          }
        } else {
          bootloaderState = bootloaderFlashOk;
        }
        break;
      default:
        break;
    }
  } else if (evt == NRF_EVT_FLASH_OPERATION_ERROR) {
    switch (bootloaderState) {
      case bootloaderErasing:
        // Start the flashing process. The rest is handled by the event handler state machine
        if (sd_flash_page_erase(currentFlashPage) != NRF_SUCCESS) {
          flashError = 2;
          bootloaderState = bootloaderFlashFail;
        }
        break;
      case bootloaderFlashing:
        err = sd_flash_write((uint32_t *)(FLASH_BASE + (currentFlashPage * PAGE_SIZE)),
                             (uint32_t const *)((uint32_t)buffer) + (currentBufferPage * PAGE_SIZE),
                             256);
        if (err != NRF_SUCCESS) {
          flashError = 3;
          bootloaderState = bootloaderFlashFail;
        }
        break;
      default:
        break;
    }
  }
}

