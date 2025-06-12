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
// hi2c1, hadc1, g_rpm_pulse_count are made visible by including main.h

/******************************************************************************/
/*          Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* PY32F0xx Peripheral Interrupt Handlers                                     */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (e.g., startup_py32f002xx.s).            */
/******************************************************************************/

/**
  * @brief This function handles I2C1 event global interrupt.
  */
void I2C1_IRQHandler(void)
{
  HAL_I2C_IRQHandler(&hi2c1);
}

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  *        This will handle the interrupt from PA1 for RPM counting.
  */
void EXTI0_1_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) // Check if interrupt is from PA1
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);     // Clear the EXTI line 1 pending bit
    g_rpm_pulse_count++;                      // Increment global pulse counter
  }
  // If PA0 was also an EXTI source, check for GPIO_PIN_0 here too
}


/* USER CODE BEGIN 1 */
// Add other ISRs here if needed (e.g., ADC1_IRQHandler if using ADC IT mode)
/* USER CODE END 1 */
/************************ (C) COPYRIGHT Puya *****END OF FILE****/