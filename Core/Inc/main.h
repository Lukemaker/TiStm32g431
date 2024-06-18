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
#include "stm32g4xx_hal.h"

  void Error_Handler(void);

  /*
   * Initialize the controller
   */
  void initialize();

  /*
   * Initialize timer 2 with 0.35 sec
   */
  void initializeTimer2();

  /*
   * Initialize timer 3 with 1 sec
   */
  void initializeTimer3();

  /*
   * Initialize timer 4 with 3 sec
   */
  void initializeTimer4();

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
