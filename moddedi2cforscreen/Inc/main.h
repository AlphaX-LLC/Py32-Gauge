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
// Add custom typedefs here if needed

/* Exported constants --------------------------------------------------------*/
// Add custom constants here if needed

/* Exported macro ------------------------------------------------------------*/
// #define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__))) // Example from your original

/* Exported functions prototypes ---------------------------------------------*/
void APP_ErrorHandler(void);
// Add other globally visible function prototypes from main.c if needed

/* Exported variables ------------------------------------------------------- */
extern I2C_HandleTypeDef hi2c1;             // I2C handle for SSD1306 display
extern ADC_HandleTypeDef hadc1;             // ADC handle for pressure sensor input (PA5)

// RPM related global variables for millisecond timing method
extern volatile uint32_t g_last_valid_pulse_time_ms;   // Timestamp of the last valid pulse (milliseconds)
extern volatile uint32_t g_current_pulse_interval_ms;  // Time between the last two valid pulses (milliseconds)
extern volatile uint8_t  g_new_rpm_data_available;     // Flag set by ISR for new RPM data (0 or 1)

/* Private defines -----------------------------------------------------------*/
// The defines below appear to be from a different example or for an I2C configuration
// (on PB6/PB7) not currently active in our main setup (which uses PA2/PA3 for I2C).
// They are commented out for clarity.
/*
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1) // aTxBuffer not defined globally
#define RXBUFFERSIZE                      TXBUFFERSIZE

#define I2Cx                            I2C
// ... (rest of old I2Cx defines commented out) ...
*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE****/