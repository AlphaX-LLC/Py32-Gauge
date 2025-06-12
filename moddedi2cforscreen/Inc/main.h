/**
  ******************************************************************************
  * @file    main.h
  * @author  MCU Application Team / Your Name
  * @brief   Header for main.c file.
  *          This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "py32f0xx_hal.h"
// #include "py32f002xx_Start_Kit.h" // Only if you use specific BSP functions from it

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// Add custom typedefs here if needed
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// Add custom constants here if needed
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
// Add custom macros here if needed (example below was from your original)
// #define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void APP_ErrorHandler(void);
/* USER CODE BEGIN EFP */
// Add other globally visible function prototypes from main.c if needed
/* USER CODE END EFP */

/* Exported variables ------------------------------------------------------- */
/* USER CODE BEGIN EV */
extern I2C_HandleTypeDef hi2c1;             // I2C handle for SSD1306 display
extern ADC_HandleTypeDef hadc1;             // ADC handle for pressure sensor input (PA6)
extern volatile uint32_t g_rpm_pulse_count; // Pulse counter for RPM measurement (PA1)
/* USER CODE END EV */

/* Private defines -----------------------------------------------------------*/
// The defines below appear to be from a different example or for an I2C configuration
// (on PB6/PB7) not currently active in our main setup (which uses PA2/PA3 for I2C).
// They are commented out for clarity and to avoid potential confusion.
/*
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1) // aTxBuffer not defined globally
#define RXBUFFERSIZE                      TXBUFFERSIZE

#define I2Cx                            I2C
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

#define I2Cx_SCL_PIN                    GPIO_PIN_6
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_7
#define I2Cx_SDA_GPIO_PORT              GPIOB

#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn
#define I2Cx_EV_IRQHandler              I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHandler              I2C1_ER_IRQHandler
*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/