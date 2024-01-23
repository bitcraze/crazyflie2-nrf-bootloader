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
 * Implementation of the Nordic ESB protocol in PRX mode for nRF51822
 */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "esb.h"

#include <nrf.h>

#undef RSSI_ACK_PACKET

#define RXQ_LEN 4
#define TXQ_LEN 4

static char address[5] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};

static enum {doTx, doRx} rs;      //Radio state

static EsbPacket rxPackets[TXQ_LEN];
static int rxq_head = 0;
static int rxq_tail = 0;

static EsbPacket txPackets[TXQ_LEN];
static int txq_head = 0;
static int txq_tail = 0;

static EsbPacket ackPacket;

// Function that swaps the bits within each byte in a uint32. Used to convert from nRF24L type addressing to nRF51 type addressing
static uint32_t bytewise_bit_swap(uint32_t inp)
{
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    return (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
}

/* Radio protocol implementation */

static bool isRetry(EsbPacket *pk)
{
  static int prevPid;
  static int prevCrc;

  bool retry = false;

  if ((prevPid == pk->pid) && (prevCrc == pk->crc)) {
    retry = true;
  }

  prevPid = pk->pid;
  prevCrc = pk->crc;

  return retry;
}

// Handles the queue
static void setupTx(bool retry) {
  static EsbPacket * lastSentPacket;

  if (retry) {
    NRF_RADIO->PACKETPTR = (uint32_t)lastSentPacket;
  } else {
    if (lastSentPacket != &ackPacket) {
      //No retry, TX payload has been sent!
      if (txq_head != txq_tail) {
        txq_tail = ((txq_tail+1)%TXQ_LEN);
      }
    }

    if (txq_tail != txq_head) {
      // Send next TX packet
      NRF_RADIO->PACKETPTR = (uint32_t)&txPackets[txq_tail];
      lastSentPacket = &txPackets[txq_tail];
    } else {
      // Send empty ACK
#ifdef RSSI_ACK_PACKET
      ackPacket.size = 3;
      ackPacket.data[0] = 0xff;
      ackPacket.data[1] = 0x01;
      ackPacket.data[2] = NRF_RADIO->RSSISAMPLE;
#endif
      NRF_RADIO->PACKETPTR = (uint32_t)&ackPacket;
      lastSentPacket = &ackPacket;
    }
  }

  //After being disabled the radio will automatically send the ACK
  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_RXEN_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
  rs = doTx;
  NRF_RADIO->TASKS_DISABLE = 1UL;
}

static void setupRx() {
  NRF_RADIO->PACKETPTR = (uint32_t)&rxPackets[rxq_head];

  NRF_RADIO->SHORTS &= ~RADIO_SHORTS_DISABLED_TXEN_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk;
  rs = doRx;
  NRF_RADIO->TASKS_DISABLE = 1UL;
}

void esbInterruptHandler()
{
  EsbPacket *pk;

  if (NRF_RADIO->EVENTS_END) {
	  NRF_RADIO->EVENTS_END = 0UL;

    switch (rs){
    case doRx:
      //Wrong CRC packet are dropped
      if (!NRF_RADIO->CRCSTATUS) {
        NRF_RADIO->TASKS_START = 1UL;
        return;
      }

      pk = &rxPackets[rxq_head];
      pk->rssi = NRF_RADIO->RSSISAMPLE;
      pk->crc = NRF_RADIO->RXCRC;

      // If no more space available on RX queue, drop packet!
      if (((rxq_head+1)%RXQ_LEN) == rxq_tail) {
        NRF_RADIO->TASKS_START = 1UL;
        return;
      }

      // If this packet is a retry, send the same ACK again
      if (isRetry(pk)) {
        setupTx(true);
        return;
      }

      // Good packet received, yea!
      rxq_head = ((rxq_head+1)%RXQ_LEN);
      setupTx(false);

      break;
    case doTx:
      //Setup RX for next packet
      setupRx();
      break;
    }
  }
}


/* Public API */

// S1 is used for compatibility with NRF24L0+. These three bits are used
// to store the PID and NO_ACK.
#define PACKET0_S1_SIZE                  (3UL)
// S0 is not used
#define PACKET0_S0_SIZE                  (0UL)
// The size of the packet length field is 6 bits
#define PACKET0_PAYLOAD_SIZE             (6UL)
// The size of the base address field is 4 bytes
#define PACKET1_BASE_ADDRESS_LENGTH      (4UL)
// Don't use any extra added length besides the length field when sending
#define PACKET1_STATIC_LENGTH            (0UL)
// Max payload allowed in a packet
#define PACKET1_PAYLOAD_SIZE             (32UL)

void esbInit()
{
  NRF_RADIO->POWER = 1;

  // Enable Radio interrupts
  //NVIC_SetPriority(RADIO_IRQn, 1);
  NVIC_EnableIRQ(RADIO_IRQn);
  // Radio config
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
  esbSetChannel(0);
  esbSetDatarate(esbDatarate2M);

  // Radio address config
  // Using logical address 0 so only BASE0 and PREFIX0 & 0xFF are used
  NRF_RADIO->PREFIX0 = bytewise_bit_swap(0xC4C3C200UL | address[4]);  // Prefix byte of addresses 3 to 0
  NRF_RADIO->PREFIX1 = bytewise_bit_swap(0xC5C6C7C8UL);  // Prefix byte of addresses 7 to 4
  NRF_RADIO->BASE0   = bytewise_bit_swap(*(uint32_t*)address); //*(uint32_t*)&address[0];  // Base address for prefix 0
  NRF_RADIO->BASE1   = bytewise_bit_swap(0x00C2C2C2UL);  // Base address for prefix 1-7
  NRF_RADIO->TXADDRESS = 0x00UL;      // Set device address 0 to use when transmitting
  NRF_RADIO->RXADDRESSES = 0x01UL;    // Enable device address 0 to use which receiving

  // Packet configuration
  NRF_RADIO->PCNF0 = (PACKET0_S1_SIZE << RADIO_PCNF0_S1LEN_Pos) |
                     (PACKET0_S0_SIZE << RADIO_PCNF0_S0LEN_Pos) |
                     (PACKET0_PAYLOAD_SIZE << RADIO_PCNF0_LFLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

  // Packet configuration
   NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos)    |
                      (RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos)           |
                      (PACKET1_BASE_ADDRESS_LENGTH << RADIO_PCNF1_BALEN_Pos)       |
                      (PACKET1_STATIC_LENGTH << RADIO_PCNF1_STATLEN_Pos)           |
                      (PACKET1_PAYLOAD_SIZE << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

  // CRC Config
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
  NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
  NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1

  // Enable interrupt for end event
  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;

  // Set all shorts so that RSSI is measured and only END is required interrupt
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
  NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RSSISTOP_Enabled;

  // Set RX buffer and start RX
  rs = doRx;
	NRF_RADIO->PACKETPTR = (uint32_t)&rxPackets[rxq_head];
  NRF_RADIO->TASKS_RXEN = 1U;
}

void esbDeinit()
{
  NVIC_DisableIRQ(RADIO_IRQn);

  NRF_RADIO->INTENCLR = RADIO_INTENSET_END_Msk;
  NRF_RADIO->SHORTS = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  NRF_RADIO->POWER = 0;
}

bool esbIsRxPacket()
{
  return (rxq_head != rxq_tail);
}

EsbPacket * esbGetRxPacket()
{
  EsbPacket *pk = NULL;

  if (esbIsRxPacket()) {
    pk = &rxPackets[rxq_tail];
  }

  return pk;
}

void esbReleaseRxPacket()
{
  rxq_tail = (rxq_tail+1)%RXQ_LEN;
}

bool esbCanTxPacket()
{
  return ((txq_head+1)%TXQ_LEN)!=txq_tail;
}

EsbPacket * esbGetTxPacket()
{
  EsbPacket *pk = NULL;

  if (esbCanTxPacket()) {
    pk = &txPackets[txq_head];
  }

  return pk;
}

void esbSendTxPacket()
{
  txq_head = (txq_head+1)%TXQ_LEN;
}

void esbSetDatarate(EsbDatarate datarate)
{
  switch (datarate) {
	case esbDatarate250K:
		  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_250Kbit << RADIO_MODE_MODE_Pos);
		  break;
	case esbDatarate1M:
		  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);
		  break;
	case esbDatarate2M:
		  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos);
		  break;
	}
}

void esbSetChannel(unsigned int channel)
{
  if (channel < 126) {
	  NRF_RADIO->FREQUENCY = channel;
	}
}

void esbSetAddress(char *addr) {
  memcpy(address, addr, 5);
}

