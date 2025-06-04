/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team / Your Name
  * @brief   Main program body for PY32F002A with SSD1306 (ADC to Pressure)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"      // CRITICAL: This MUST include "py32f0xx_hal.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>     // For sprintf
#include <string.h>    // For memset

/* Private define ------------------------------------------------------------*/
#define I2C_PERIPH_INSTANCE           I2C1
#define I2C_SPEEDCLOCK                100000U
#define I2C_DUTYCYCLE_SSD1306         I2C_DUTYCYCLE_2

// ADC Related Defines
#define ADC_MAX_RAW_VALUE             4095.0f   // For 12-bit ADC (2^12 - 1)
#define MAX_PRESSURE_KPA              300.0f    // Maximum pressure in kPa
#define MAX_BAR_CHARS                 16        // Number of characters for the bar graph

// LED Pin definitions
#define LED_PORT        GPIOB
#define LED_PIN         GPIO_PIN_0
#define LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;      // ADC Handle
char pressure_str[20];      // Buffer for Pressure string "PA6: XXX.XkPa"
char bar_graph_str[MAX_BAR_CHARS + 1]; // Buffer for bar graph string (+1 for null terminator)

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void APP_GPIO_Init(void); // For LED
static void MX_ADC1_Init(void);  // For ADC
void APP_ErrorHandler(void);

/**
  * @brief  Main program.
  */
int main(void)
{
  uint32_t adc_raw_value;
  float pressure_kpa;
  int num_equals;
  int i; // Loop counter

  HAL_Init();
  SystemClock_Config();
  APP_GPIO_Init();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // LED OFF

  /* Configure I2C1 */
  hi2c1.Instance             = I2C_PERIPH_INSTANCE;
  hi2c1.Init.ClockSpeed      = I2C_SPEEDCLOCK;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_SSD1306;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { APP_ErrorHandler(); }

  /* Initialize ADC1 for PA6 (ensure MX_ADC1_Init and MSP are set for PA6) */
  MX_ADC1_Init();

  /* Initialize SSD1306 OLED Display */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // LED ON during SSD1306 init
  if (ssd1306_Init(&hi2c1) != 0)
  {
    while(1) { HAL_GPIO_TogglePin(LED_PORT, LED_PIN); HAL_Delay(100); }
  }
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // LED OFF after successful init
  HAL_Delay(200);

  while (1)
  {
    // 1. Start ADC Conversion on PA6
    HAL_ADC_Start(&hadc1);
    // 2. Poll for ADC conversion completion
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) // Timeout 100ms
    {
      adc_raw_value = HAL_ADC_GetValue(&hadc1);
    }
    else
    {
      adc_raw_value = 0; // Error or timeout
    }
    HAL_ADC_Stop(&hadc1); // Stop ADC (for single conversion mode)

    // 3. Convert raw ADC value to Pressure
    pressure_kpa = (adc_raw_value / ADC_MAX_RAW_VALUE) * MAX_PRESSURE_KPA;

    // 4. Prepare pressure string (e.g., "PA6: 150.3kPa")
    sprintf(pressure_str, "PA6: %.1fkPa", pressure_kpa); // Displaying with 1 decimal place

    // 5. Prepare bar graph string (this logic remains the same)
    num_equals = (int)((adc_raw_value / ADC_MAX_RAW_VALUE) * MAX_BAR_CHARS);
    if (num_equals < 0) num_equals = 0;
    if (num_equals > MAX_BAR_CHARS) num_equals = MAX_BAR_CHARS;
    
    memset(bar_graph_str, ' ', MAX_BAR_CHARS); // Fill with spaces
    for (i = 0; i < num_equals; i++)
    {
      bar_graph_str[i] = '=';
    }
    bar_graph_str[MAX_BAR_CHARS] = '\0'; // Null-terminate

    // 6. Update display
    ssd1306_Fill(Black); // Clear screen

    ssd1306_SetCursor(2, 2);  // Line 1
    ssd1306_WriteString("AlphaX bGauge", Font_7x10, White); // Updated title

    ssd1306_SetCursor(2, 15); // Line 2 (Pressure Value)
    ssd1306_WriteString(pressure_str, Font_7x10, White);
    
    ssd1306_SetCursor(2, 30); // Line 3 (Bar Graph Label)
    ssd1306_WriteString("LVL:", Font_7x10, White);

    ssd1306_SetCursor(2 + (7*4) + 2, 30); // Position bar graph after "LVL:" + space
    ssd1306_WriteString(bar_graph_str, Font_7x10, White);
    
    // Optional: Display raw ADC value on Line 4
    // char raw_str_buf[20];
    // sprintf(raw_str_buf, "Raw: %4lu", adc_raw_value); // %lu for uint32_t
    // ssd1306_SetCursor(2, 45); 
    // ssd1306_WriteString(raw_str_buf, Font_7x10, White);

    ssd1306_UpdateScreen(&hi2c1);

    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    HAL_Delay(250); // Update rate
  }
}

/**
  * @brief System Clock Configuration
  * (Sets HSI to 24MHz, uses it as SYSCLK)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSI_FS, (4U << RCC_ICSCR_HSI_FS_Pos));

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { APP_ErrorHandler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { APP_ErrorHandler(); }
  SystemCoreClockUpdate();
}

/**
  * @brief ADC1 Initialization Function
  * Configured for PA6 (ADC_CHANNEL_6)
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;

  /* Configure ADC peripheral */
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = 0x00000000U;                 // Scan disable for single channel
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  // hadc1.Init.NbrOfConversion       = 1; // Not present in this HAL's ADC_InitTypeDef
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon    = ADC_SAMPLETIME_239CYCLES_5;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    APP_ErrorHandler();
  }

  /* Configure ADC Regular Channel for PA6 */
  sConfig.Channel      = ADC_CHANNEL_6;    // PA6
  sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
  
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    APP_ErrorHandler();
  }
}

/* LED GPIO Initialization */
static void APP_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  LED_GPIO_CLK_ENABLE();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/* Error Handler */
void APP_ErrorHandler(void) {
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
  while (1) { HAL_GPIO_TogglePin(LED_PORT, LED_PIN); HAL_Delay(50); }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { APP_ErrorHandler(); }
#endif