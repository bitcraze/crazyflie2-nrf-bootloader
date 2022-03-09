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
 * nRF51822 Cload bootloader for Crazyflie 2.0
 */
#include <string.h>

#include <nrf.h>
#include <nrf_mbr.h>
#include <nrf_sdm.h>
#include <nrf_gpio.h>
#include <nrf_gpiote.h>

#include <nrf_soc.h>

#include "systick.h"
#include "button.h"
#include "pinout.h"
#include "syslink.h"
#include "uart.h"

#include "app_timer.h"

#include "ble_gap.h"
#include "ble_crazyflies.h"

#include "esb.h"
#include "crtp.h"
#include "bootloader.h"

#define BOOTLOADER_ADDRESS 0x0003A000
#define FW_ADDRESS 0x00016000

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 4                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              5                                          /**< Size of timer operation queues. */

// Time for timers
#define MS  125UL
#define SEC (1000UL * MS)

static sd_mbr_command_t startSdCmd = {
  .command = SD_MBR_COMMAND_INIT_SD,
};

void ble_init(void);

void mainLoop(void);

void start_firmware() __attribute__ ((noreturn, naked));
void start_firmware() {
  void (*fw_start)(void) = *(void (**)(void))(FW_ADDRESS + 4);

  sd_softdevice_vector_table_base_set(FW_ADDRESS);
  __set_MSP(*(uint32_t*)FW_ADDRESS);
  fw_start();

  while (1);
}


// int main() __attribute__ ((noreturn));
int main() {
  static char address[5];

  sd_mbr_command(&startSdCmd);
  sd_softdevice_vector_table_base_set(BOOTLOADER_ADDRESS);

  // If the master boot switch has detected short or no click: boot the firmware
  if (((NRF_POWER->GPREGRET & 0x86U) != 0x82U) &&
      ((NRF_POWER->GPREGRET & 0x40U) != 0x40U) &&
      (*(uint32_t *)FW_ADDRESS != 0xFFFFFFFFU) ) {
    start_firmware();
  }

  if (NRF_POWER->GPREGRET & 0x40U) {
    address[4] = 0xb1;
    memcpy(&address[0], (char*)&NRF_FICR->DEVICEADDR[0], 4);
    esbSetAddress(address);
  }

  NRF_POWER->GPREGRET &= ~(0x60U);

#ifdef HAS_RFX2411N
  // Enable RF power amplifier
  nrf_gpio_cfg_output(RADIO_PA_RX_EN);
  nrf_gpio_cfg_output(RADIO_PA_MODE);
  nrf_gpio_cfg_output(RADIO_PA_ANT_SW);
  // Select antenna port A
  nrf_gpio_pin_set(RADIO_PA_ANT_SW);

  nrf_gpio_pin_set(RADIO_PA_RX_EN);
  nrf_gpio_pin_clear(RADIO_PA_MODE);
#else
  // Enable the radio LNA
  nrf_gpio_cfg_output(RADIO_PAEN_PIN);
  nrf_gpio_pin_set(RADIO_PAEN_PIN);
#endif

  // Initialize timer module.
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

  ble_init();
/*
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSTAT_SRC_Synth;

  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_LFCLKSTARTED);
*/
  systickInit();
  buttonInit(buttonIdle);

#ifndef DEBUG_TIMESLOT
  //sd_ppi_channel_assign(0, &(NRF_TIMER1->EVENTS_COMPARE[0]), &(NRF_GPIOTE->TASKS_OUT[0]));
  //sd_ppi_channel_enable_set(PPI_CHEN_CH0_Msk);

  //NRF_PPI->CH[0].EEP = &(NRF_TIMER1->EVENTS_COMPARE[0]);
  //NRF_PPI->CH[0].TEP = &(NRF_GPIOTE->TASKS_OUT[0]);
  //NRF_PPI->CHENSET = 1;
#endif

  // Start (or continue) to blink  the LED at 0.5Hz
  //NRF_TIMER1->TASKS_STOP = 1;

  //NRF_TIMER1->MODE      = TIMER_MODE_MODE_Timer;
  //NRF_TIMER1->PRESCALER = 7;
  //NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
  //NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk; // | TIMER_SHORTS_COMPARE1_CLEAR_Msk;

  //NRF_TIMER1->TASKS_CLEAR = 1;

  NRF_TIMER1->CC[0] = 1 * SEC; //0x1E84 ;
  NRF_TIMER1->CC[1] = 2 * SEC;


  nrf_gpio_cfg_output(LED_PIN);

  nrf_gpiote_task_config(0,
                         LED_PIN,
                         NRF_GPIOTE_POLARITY_TOGGLE,
                         NRF_GPIOTE_INITIAL_VALUE_LOW);
  NRF_TIMER1->TASKS_START = 1;


#ifdef HAS_TI_CHARGER
  // Enable 500mA USB input and enable battery charging
  nrf_gpio_cfg_output(PM_EN1);
  nrf_gpio_pin_set(PM_EN1);
  nrf_gpio_cfg_output(PM_EN2);
  nrf_gpio_pin_clear(PM_EN2);
  nrf_gpio_cfg_output(PM_CHG_EN);
  nrf_gpio_pin_clear(PM_CHG_EN);
#endif

  // Power STM32, hold reset
  nrf_gpio_cfg_output(PM_VCCEN_PIN);
  nrf_gpio_pin_set(PM_VCCEN_PIN);
  nrf_gpio_cfg_output(STM_NRST_PIN);
  nrf_gpio_pin_clear(STM_NRST_PIN);

  // Set flow control and activate pull-down on RX data pin
  nrf_gpio_cfg_output(UART_TX_PIN);
  nrf_gpio_pin_set(UART_TX_PIN);
  nrf_gpio_cfg_output(UART_RTS_PIN);
  nrf_gpio_pin_set(UART_RTS_PIN);
  nrf_gpio_cfg_input(UART_RX_PIN, NRF_GPIO_PIN_PULLDOWN);


  nrf_gpio_pin_set(STM_NRST_PIN);

  //systickInit();
  //syslinkInit();
  //buttonInit();

//  nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

  mainLoop();

  while (1);
  return 0;
}

static enum {connect_idle, connect_ble, connect_sb} cstate = connect_idle;

/*
typedef struct CRTPPacket_s {
  union {
    struct {
      union {
        uint8_t header;
        struct {
          uint8_t channel:4;
          uint8_t port:2;
        };
      };
      char data[31];
    };
    char raw[32];
  }
  uint8_t datalen;
} CRTPPacket;
*/

void mainLoop(void) {
  bool resetToFw = false;
  static CrtpPacket crtpPacket;
  static bool stmStarted = false;

  while (!resetToFw) {
    EsbPacket *packet;
    buttonProcess();

    if (buttonGetState() != buttonIdle) {
      resetToFw = true;
    }

    if ((stmStarted == false) && (nrf_gpio_pin_read(UART_RX_PIN))) {
      nrf_gpio_cfg_input(UART_RTS_PIN, NRF_GPIO_PIN_NOPULL);
      uartInit();
      stmStarted = true;
    }


    if (cstate != connect_ble) {
      packet = esbGetRxPacket();
      if (packet != NULL) {

        if ( ((packet->size >= 2) &&
              (packet->data[0]==0xff) &&
              (packet->data[1]==0xff)) ||
             ((packet->size >= 2) &&
              (packet->data[0]==0xff) &&
              (packet->data[1]==0xfe))
           ) {
          // Disable Bluetooth advertizing when receiving a bootloader SB packet
          if (cstate == connect_idle) {
            //sd_ble_gap_adv_stop();
            cstate = connect_sb;
          }
        }

        // If we are connected SB, the packet is read and used
        if (cstate == connect_sb) {
          memcpy(crtpPacket.raw, packet->data, packet->size);
          crtpPacket.datalen = packet->size-1;
        }
        esbReleaseRxPacket(packet);
      }
    }
    if (cstate != connect_sb) {
      if (bleCrazyfliesIsPacketReceived()) {
        cstate = connect_ble;

        packet = bleCrazyfliesGetRxPacket();
        memcpy(crtpPacket.raw, packet->data, packet->size);
        crtpPacket.datalen = packet->size-1;
        bleCrazyfliesReleaseRxPacket(packet);
      }
    }

    if (crtpPacket.datalen != 0xffu) {
      struct syslinkPacket slPacket;
      slPacket.type = SYSLINK_RADIO_RAW;
      memcpy(slPacket.data, crtpPacket.raw, crtpPacket.datalen+1);
      slPacket.length = crtpPacket.datalen+1;

      if (bootloaderProcess(&crtpPacket) == false) {
        // Send packet to stm32
        syslinkSend(&slPacket);

        crtpPacket.datalen = 0xFFU;
        // If packet received from stm32, send it back
        if (syslinkReceive(&slPacket)) {
          if (slPacket.type == SYSLINK_RADIO_RAW) {
            memcpy(crtpPacket.raw, slPacket.data, slPacket.length);
            crtpPacket.datalen = slPacket.length-1;
          }
        }
      }
    }
    if (crtpPacket.datalen != 0xFFU) {
      if (cstate == connect_sb) {
        EsbPacket *pk = esbGetTxPacket();
        if (pk) {
          memcpy(pk->data, crtpPacket.raw, crtpPacket.datalen+1);
          pk->size = crtpPacket.datalen+1;
          esbSendTxPacket(pk);
        }
      } else if (cstate == connect_ble) {
        static EsbPacket pk;
        memcpy(pk.data, crtpPacket.raw, crtpPacket.datalen+1);
        pk.size = crtpPacket.datalen+1;
        bleCrazyfliesSendPacket(&pk);
      }
    }

    crtpPacket.datalen = 0xFFU;

    // Blink the LED
    if (NRF_TIMER1->EVENTS_COMPARE[0]) {
      NRF_TIMER1->EVENTS_COMPARE[0] = 0;
#ifndef DEBUG_TIMESLOT
      NRF_GPIOTE->TASKS_OUT[0] = 1;
#endif
    }
  }

  //Set bit 0x20 forces boot to firmware
  NRF_POWER->GPREGRET |= 0x20U;
  sd_nvic_SystemReset();

  while(1);
}
