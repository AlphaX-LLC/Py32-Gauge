/**
  ******************************************************************************
  * @file    py32f0xx_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
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
#include "main.h" // Ensures HAL types and externs (like hi2c1, hadc1) are visible

/**
  * @brief Initialize global MSP
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize I2C MSP for PA2 (SDA) and PA3 (SCL)
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(hi2c->Instance==I2C1) // Check if it's for I2C1
  {
    /* Peripheral clock enable */
    __HAL_RCC_I2C_CLK_ENABLE();    // Generic I2C clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE();  // For PA2/PA3

    /* I2C1 GPIO Configuration    
    PA2     ------> I2C1_SDA
    PA3     ------> I2C1_SCL 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // Or GPIO_SPEED_FREQ_HIGH
    GPIO_InitStruct.Alternate = GPIO_AF12_I2C;       // From your py32f0xx_hal_gpio_ex.h
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Optional: I2C1 Force Reset and Release (some examples do this) */
    // __HAL_RCC_I2C_FORCE_RESET();
    // __HAL_RCC_I2C_RELEASE_RESET();

    /* I2C1 interrupt Init (if using I2C interrupts) */
    // Your original MSP had this, keeping it:
    HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
  }
}

/**
  * @brief DeInitialize I2C MSP
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
    __HAL_RCC_I2C_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    HAL_NVIC_DisableIRQ(I2C1_IRQn); // Disable the IRQ if it was enabled
  }
}

/**
  * @brief ADC MSP Initialization for PA6
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
    __HAL_RCC_ADC_CLK_ENABLE();   // Generic ADC clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE(); // For PA6

    /**ADC1 GPIO Configuration
    PA6     ------> ADC1_IN6 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Optional: ADC1 interrupt Init (not used by current polling main.c) */
    // HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(ADC1_IRQn);
  }
}

/**
  * @brief ADC MSP De-Initialization for PA6
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    __HAL_RCC_ADC_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    // Optional: ADC1 interrupt DeInit
    // HAL_NVIC_DisableIRQ(ADC1_IRQn);
  }
}
/************************ (C) COPYRIGHT Puya *****END OF FILE****/