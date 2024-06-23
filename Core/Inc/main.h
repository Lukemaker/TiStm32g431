/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32g4xx_hal.h"

  void Error_Handler(void);

  /*
   * Initialize the controller
   */
  void initialize();

  /**
   * Initialize LED
   */
  void initializeLed();

  /**
   * Initialize pin interrupts PA0 (A0) and PA2 (A7)
   */
  void initializePinInterrupts();

  /**
   * Initialize timers 3 & 4
   */
  void initializeTimers();

  /**
   * Start timer 3 with a duration calculated with the number of pin touches since the last call of this function. Uses variable ammountPinA0Triggered
   */
  void startTimer3();

  /**
   * Stop timer 3.
   */
  void stopTimer3();

  /**
   * Start timer 4 in one-pulse-mode with a duration of 750ms.
   */
  void startTimer4();

  /**
   * Calculates the PSC value based on ammountPinA0Triggered.
   */
  int getPscValue();

  /**
   * Toggles the LED.
   * @param turnLedOn Turns Led on if true. Turns it off if false.
   */
  void toggleLed(bool turnLedOn);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
