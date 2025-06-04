/** // (Original file header comment)
  ******************************************************************************
  * @file    py32f0xx_hal_msp.c
  * @author  MCU Application Team
  * @brief   This file provides code for the MSP Initialization
  *          and de-Initialization codes.
  ******************************************************************************
  * ... (rest of header comment) ...
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h" // <<<<<<<<<<<<<<<<<<<< MOVED TO THE VERY TOP (after file header comment)

/**
  * @brief Initialize global MSP
  */
void HAL_MspInit(void)
{
  // This function should already be here from your original file
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief Initialize I2C MSP
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  // This function should already be here from your original file
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_I2C_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_I2C;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_I2C_FORCE_RESET();
  __HAL_RCC_I2C_RELEASE_RESET();

  HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
}

// ADDED ADC MSP FUNCTIONS (can go after the I2C MSP function)

/**
  * @brief ADC MSP Initialization
  * This function configures the hardware resources needed for ADC:
  *  - Peripheral's clock enable
  *  - GPIO Pin configuration for PA5
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1) // Assuming ADC1 is the correct instance
  {
    __HAL_RCC_ADC_CLK_ENABLE();   // Generic ADC clock enable
    __HAL_RCC_GPIOA_CLK_ENABLE(); // GPIOA clock for PA5

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Optional: NVIC for ADC interrupts
    // HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
    // HAL_NVIC_EnableIRQ(ADC1_IRQn);
  }
}

/**
  * @brief ADC MSP De-Initialization
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    __HAL_RCC_ADC_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    // Optional: NVIC disable
    // HAL_NVIC_DisableIRQ(ADC1_IRQn);
  }
}

/************************ (C) COPYRIGHT Puya *****END OF FILE****/