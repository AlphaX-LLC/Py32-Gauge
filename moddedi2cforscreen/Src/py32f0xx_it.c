/**
  ******************************************************************************
  * @file    py32f0xx_it.c
  * @author  MCU Application Team
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>© Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"        // This includes py32f0xx_hal.h and extern declarations
#include "py32f0xx_it.h" // This file's own header

/* External variables --------------------------------------------------------*/
// Global variables like hi2c1, hadc1, g_last_valid_pulse_time_ms, etc.,
// are made visible by including main.h which has their extern declarations.

// This define should match the one in main.c for RPM debounce logic
// If main.c doesn't define it, this provides a default.
#ifndef MIN_PULSE_INTERVAL_MS
#define MIN_PULSE_INTERVAL_MS         2  // Default debounce/filter for RPM (milliseconds)
#endif

/******************************************************************************/
/*          Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* PY32F0xx Peripheral Interrupt Handlers                                     */
/******************************************************************************/
void I2C1_IRQHandler(void)
{
  HAL_I2C_IRQHandler(&hi2c1);
}

void EXTI0_1_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) // Check if interrupt is from PA1
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);     // Clear the EXTI line 1 pending bit
    
    uint32_t now_ms = HAL_GetTick();
    uint32_t interval_ms;

    // Calculate interval, handle HAL_GetTick() wraparound
    if (now_ms >= g_last_valid_pulse_time_ms) {
        interval_ms = now_ms - g_last_valid_pulse_time_ms;
    } else {
        // HAL_GetTick() has wrapped around
        interval_ms = (0xFFFFFFFFU - g_last_valid_pulse_time_ms) + now_ms + 1;
    }

    if (interval_ms > MIN_PULSE_INTERVAL_MS) // Debounce/filter
    {
      g_current_pulse_interval_ms = interval_ms;
      g_last_valid_pulse_time_ms = now_ms;
      g_new_rpm_data_available = 1; // Signal main loop (use 1 for true)
    }
  }
  // If PA0 was also an EXTI source, check for GPIO_PIN_0 here too
}
/************************ (C) COPYRIGHT Puya *****END OF FILE****/