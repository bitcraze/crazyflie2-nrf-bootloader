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
#include <nrf_gpio.h>


#include <nrf.h>
#include "sdk_common.h"
#include "nrf_nvic.h"
#include "boards.h"
#include "button.h"
#include "systick.h"

#include "custom_board.h"

#define BUTTON_READ() ((NRF_GPIO->IN >> BUTTON_1) & 1UL)
#define BUTTON_PRESSED 0UL
#define BUTTON_RELEASED 1UL
#define BUTTON_DEBOUNCE_TICK 1
#define BUTTON_LONGPRESS_TICK 300

static ButtonEvent state;

void buttonInit(ButtonEvent initialEvent) {
  nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
  NRF_GPIO->PIN_CNF[BUTTON_1] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

  state = initialEvent;
}

void buttonProcess() {
  static unsigned int lastTick;
  static unsigned int pressedTick;
  static bool pressed;

  if (lastTick != systickGetTick()) {
    lastTick = systickGetTick();

    if (pressed==false && BUTTON_READ() == BUTTON_PRESSED) {
      pressed = true;
      pressedTick = systickGetTick();
    } else if (pressed==true && BUTTON_READ() == BUTTON_RELEASED) {
      pressed = false;
      if (((systickGetTick() - pressedTick) < BUTTON_LONGPRESS_TICK) &&
          ((systickGetTick() - pressedTick) > BUTTON_DEBOUNCE_TICK) ) {
        state = buttonShortPress;
      } else {
        state = buttonLongPress;
      }
    }
  }
}

ButtonEvent buttonGetState() {
  ButtonEvent currentState = state;
  state = buttonIdle;

  return currentState;
}



