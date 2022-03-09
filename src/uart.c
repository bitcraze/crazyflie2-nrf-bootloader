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
#include <stdbool.h>

#include <nrf.h>

#include "pinout.h"

#include "uart.h"

static bool isInit = false;

#define Q_LENGTH 128

static char rxq[Q_LENGTH];
static int head = 0;
static int tail = 0;

static int dropped = 0;
static char dummy;

int error = 0;

void UART0_IRQHandler() {
  int nhead = head + 1;

  //if (NRF_UART0->ERRORSRC) {
  //  error = NRF_UART0->ERRORSRC;
  //  NRF_UART0->ERRORSRC = 0xFF;
  //  __BKPT(0);
 // }

  NRF_UART0->EVENTS_RXDRDY = 0;

  // Check if the queue is not full
  if (nhead >= Q_LENGTH) nhead = 0;
  if (nhead == tail) {
    dummy = NRF_UART0->RXD; //Read anyway to avoid hw overflow
    dropped++;
    return;
  }

  // Push data in queue
  rxq[head++] = NRF_UART0->RXD;
  if (head >= Q_LENGTH) head = 0;
}

void uartInit() {
  NRF_GPIO->PIN_CNF[UART_TX_PIN] = (NRF_GPIO->PIN_CNF[UART_TX_PIN] & (~GPIO_PIN_CNF_DRIVE_Msk)) | (GPIO_PIN_CNF_DRIVE_S0S1<<GPIO_PIN_CNF_DRIVE_Pos);

  NRF_GPIO->DIRSET = 1 << UART_TX_PIN;
  NRF_GPIO->OUTSET = 1 << UART_TX_PIN;
  NRF_UART0->PSELTXD = UART_TX_PIN;

  NRF_GPIO->DIRCLR = 1 << UART_RX_PIN;
  NRF_UART0->PSELRXD = UART_RX_PIN;

  NRF_GPIO->DIRSET = 1 << UART_RTS_PIN;
  NRF_GPIO->OUTSET = 1 << UART_RTS_PIN;
  NRF_UART0->PSELRTS = UART_RTS_PIN;

  NRF_UART0->CONFIG = UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos;

  NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud1M;

  NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled;

  NRF_UART0->TASKS_STARTTX = 1;

  // Enable interrupt on receive
  NVIC_EnableIRQ(UART0_IRQn);
  NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Msk;

  NRF_UART0->TASKS_STARTRX = 1;

  isInit = true;
}

void uartPuts(char* string) {
  if (!isInit)
    return;

  while (*string)
    uartPutc(*string++);
}

void uartSend(char* data, int len) {
  if (!isInit)
    return;

  while (len--)
    uartPutc(*data++);

}

void uartPutc(char c) {
  if (!isInit)
    return;

  NRF_UART0->TXD = c;
  while (!NRF_UART0->EVENTS_TXDRDY);
  NRF_UART0->EVENTS_TXDRDY = 0;
}

bool uartIsDataReceived() {
  if (!isInit)
    return false;

  return head!=tail;
}

char uartGetc() {
  char c = 0;

  if (!isInit)
    return c;

  if (head != tail) {
    c = rxq[tail++];
    if (tail >= Q_LENGTH) tail = 0;
  }

  return c;
}
