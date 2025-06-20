/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team / Your Name
  * @brief   Main program body for PY32F002A: OLED, ADC, RPM, PB2 Output Control
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/
#define I2C_PERIPH_INSTANCE           I2C1
#define I2C_SPEEDCLOCK                100000U
#define I2C_DUTYCYCLE_SSD1306         I2C_DUTYCYCLE_2

#define ADC_VREF_ACTUAL               5.0f
#define ADC_MAX_RAW_VALUE             4095.0f
#define MAX_PRESSURE_KPA              300.0f
#define MAX_BAR_CHARS                 16
#define R1_BATTERY_OHM                (51000.0f)
#define R2_BATTERY_OHM                (10000.0f)
#define BATTERY_VOLTAGE_MULTIPLIER    ((R1_BATTERY_OHM + R2_BATTERY_OHM) / R2_BATTERY_OHM)

#define PULSES_PER_REVOLUTION         2
#define PROCESSING_INTERVAL_MS        100
#define RPM_TIMEOUT_MS                1000
#define MIN_PULSE_INTERVAL_MS         2

#define LED_TOGGLE_INTERVAL_NO_RPM_MS 400
#define LED_TOGGLE_INTERVAL_RPM_MS    200
#define RPM_ACTIVE_THRESHOLD_FLOAT    0.5f
#define RPM_CONTROL_THRESHOLD_INT     200

// GPIO Pin definitions
#define LED_PORT        GPIOB
#define LED_PIN         GPIO_PIN_0
#define OUTPUT_PORT     GPIOB         // For MOSFET drive
#define OUTPUT_PIN      GPIO_PIN_2    // <<<< CHANGED TO PB2
#define GPIO_CLK_ENABLE() do { \
                            __HAL_RCC_GPIOA_CLK_ENABLE(); \
                            __HAL_RCC_GPIOB_CLK_ENABLE(); \
                          } while(0)

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;
char pressure_display_str[20];
char bar_graph_str[MAX_BAR_CHARS + 1];
char rpm_display_str[20];
char battery_display_str[20];

volatile uint32_t g_last_valid_pulse_time_ms = 0;
volatile uint32_t g_current_pulse_interval_ms = 0;
volatile uint8_t  g_new_rpm_data_available = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void APP_Peripherals_Init(void);
static void MX_ADC1_Init(void);
static void MX_EXTI_Init_PA1(void);
void APP_ErrorHandler(void);

/**
  * @brief  Main program.
  */
int main(void)
{
  uint16_t adc_raw_pa4, adc_raw_pa5;
  float    v_at_pa4, v_battery;
  uint16_t vbatt_display_x10;
  char     batt_alert_char;
  uint16_t pressure_val_x10;
  int num_equals, i;

  uint32_t last_processing_time = 0, current_tick;
  uint32_t local_pulse_interval_ms, local_last_pulse_time_ms;
  uint8_t  local_new_rpm_data;
  float    calculated_float_rpm = 0.0f;
  uint16_t rpm_val_display;

  uint32_t last_led_toggle_time = 0;
  uint32_t led_toggle_interval_ms = LED_TOGGLE_INTERVAL_NO_RPM_MS;

  HAL_Init();
  SystemClock_Config();
  APP_Peripherals_Init(); // Initializes LED (PB0) and Output (PB2)
  MX_EXTI_Init_PA1();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // LED OFF initially
  HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_RESET); // PB2 OFF initially

  hi2c1.Instance = I2C_PERIPH_INSTANCE;
  hi2c1.Init.ClockSpeed      = I2C_SPEEDCLOCK;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_SSD1306;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { APP_ErrorHandler(); }

  MX_ADC1_Init();

  if (ssd1306_Init(&hi2c1) != HAL_OK) { APP_ErrorHandler(); }

  //****************************************************
  //***** BOOT SCREEN LOGIC & PB2 Control *****
  //****************************************************
  HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_SET); // PB2 ON for boot screen

  char line1_boot_str[20], line2_boot_str[20], line3_boot_str[20];
  sprintf(line1_boot_str, "------AlphaX------"); 
  sprintf(line2_boot_str, "------Super------");
  sprintf(line3_boot_str, "------Gauge------");

  ssd1306_Fill(Black);
  ssd1306_SetCursor(1, 12); ssd1306_WriteString(line1_boot_str, Font_7x10, White);
  ssd1306_SetCursor(4, 27); ssd1306_WriteString(line2_boot_str, Font_7x10, White);
  ssd1306_SetCursor(4, 42); ssd1306_WriteString(line3_boot_str, Font_7x10, White);
  ssd1306_UpdateScreen(&hi2c1);
  HAL_Delay(2000);
  
  HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_RESET); // PB2 OFF after boot screen
  //****************************************************

  sprintf(rpm_display_str, "RPM: ---");
  sprintf(pressure_display_str, "PA5: ---kPa");
  sprintf(battery_display_str, "BATT:--.-V");
  last_processing_time = HAL_GetTick();
  last_led_toggle_time = HAL_GetTick();
  g_last_valid_pulse_time_ms = HAL_GetTick();

  while (1)
  {
    current_tick = HAL_GetTick();

    HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
    local_new_rpm_data = g_new_rpm_data_available;
    local_pulse_interval_ms = g_current_pulse_interval_ms;
    local_last_pulse_time_ms = g_last_valid_pulse_time_ms;
    if (local_new_rpm_data) { g_new_rpm_data_available = 0; }
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    if (local_new_rpm_data) {
      if (local_pulse_interval_ms > 0 && PULSES_PER_REVOLUTION > 0) {
        calculated_float_rpm = 60000.0f / ( (float)local_pulse_interval_ms * (float)PULSES_PER_REVOLUTION );
      } else { calculated_float_rpm = 0.0f; }
    }
    uint32_t time_since_last_pulse_ms;
    if (current_tick >= local_last_pulse_time_ms) {
        time_since_last_pulse_ms = current_tick - local_last_pulse_time_ms;
    } else { time_since_last_pulse_ms = (0xFFFFFFFFU - local_last_pulse_time_ms) + current_tick + 1; }
    if (time_since_last_pulse_ms > RPM_TIMEOUT_MS) { calculated_float_rpm = 0.0f; }
    rpm_val_display = (uint16_t)calculated_float_rpm;

    // --- PB2 Output Control based on RPM ---
    if (rpm_val_display > RPM_CONTROL_THRESHOLD_INT) {
        HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_SET); // PB2 ON
    } else {
        HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_RESET); // PB2 OFF
    }

    if (current_tick - last_processing_time >= PROCESSING_INTERVAL_MS)
    {
      HAL_ADC_Start(&hadc1); 
      if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) { adc_raw_pa4 = (uint16_t)HAL_ADC_GetValue(&hadc1); } 
      else { adc_raw_pa4 = 0; }
      if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) { adc_raw_pa5 = (uint16_t)HAL_ADC_GetValue(&hadc1); } 
      else { adc_raw_pa5 = 0; }
      HAL_ADC_Stop(&hadc1);

      v_at_pa4 = ((float)adc_raw_pa4 / ADC_MAX_RAW_VALUE) * ADC_VREF_ACTUAL;
      v_battery = v_at_pa4 * BATTERY_VOLTAGE_MULTIPLIER;
      vbatt_display_x10 = (uint16_t)(v_battery * 10.0f);
      batt_alert_char = (v_battery > 15.0f || v_battery < 11.0f) ? '*' : ' ';
      sprintf(battery_display_str, "BATT:%u.%1uV%c", vbatt_display_x10 / 10, vbatt_display_x10 % 10, batt_alert_char);
      
      pressure_val_x10 = (uint16_t)(((float)adc_raw_pa5 / ADC_MAX_RAW_VALUE) * MAX_PRESSURE_KPA * 10.0f);
      sprintf(pressure_display_str, "PA5:%u.%1ukPa", pressure_val_x10 / 10, pressure_val_x10 % 10);

      num_equals = (int)(((float)adc_raw_pa5 / ADC_MAX_RAW_VALUE) * MAX_BAR_CHARS);
      if (num_equals < 0) num_equals = 0; if (num_equals > MAX_BAR_CHARS) num_equals = MAX_BAR_CHARS;
      memset(bar_graph_str, ' ', MAX_BAR_CHARS);
      for (i = 0; i < num_equals; i++) { bar_graph_str[i] = '='; }
      bar_graph_str[MAX_BAR_CHARS] = '\0';
      
      sprintf(rpm_display_str, "RPM:%u", rpm_val_display);
      
      ssd1306_Fill(Black);
      ssd1306_SetCursor(2, 2);  ssd1306_WriteString(battery_display_str, Font_7x10, White);
      ssd1306_SetCursor(2, 15); ssd1306_WriteString(pressure_display_str, Font_7x10, White);
      ssd1306_SetCursor(2, 30); ssd1306_WriteString("LVL:", Font_7x10, White);
      ssd1306_SetCursor(2 + (7*4) + 2, 30); ssd1306_WriteString(bar_graph_str, Font_7x10, White);
      ssd1306_SetCursor(2, 45); ssd1306_WriteString(rpm_display_str, Font_7x10, White);

      ssd1306_UpdateScreen(&hi2c1);
      last_processing_time = current_tick;
    }

    if (calculated_float_rpm > RPM_ACTIVE_THRESHOLD_FLOAT) {
        led_toggle_interval_ms = LED_TOGGLE_INTERVAL_RPM_MS;
    } else {
        led_toggle_interval_ms = LED_TOGGLE_INTERVAL_NO_RPM_MS;
    }
    if (current_tick - last_led_toggle_time >= led_toggle_interval_ms) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        last_led_toggle_time = current_tick;
    }
  }
}

void SystemClock_Config(void) { /* ... same ... */ 
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

static void MX_ADC1_Init(void) { /* ... same, for PA4 and PA5 ... */ 
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = 0x00000000U; 
  hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV; 
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon    = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { APP_ErrorHandler(); }
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) { APP_ErrorHandler(); }
  sConfig.Channel      = ADC_CHANNEL_4;
  sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER; 
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { APP_ErrorHandler(); }
  sConfig.Channel      = ADC_CHANNEL_5;
  sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER; 
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { APP_ErrorHandler(); }
}

static void MX_EXTI_Init_PA1(void) { /* ... same ... */ 
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  // __HAL_RCC_GPIOA_CLK_ENABLE(); // Now called in APP_Peripherals_Init
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

/**
  * @brief Initializes common GPIOs: LED (PB0) and Output (PB2)
  */
static void APP_Peripherals_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  GPIO_CLK_ENABLE(); // Enables GPIOA and GPIOB

  /*Configure GPIO pin : LED_PIN (PB0) */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : OUTPUT_PIN (PB2) */ // <<<< CHANGED TO PB2
  HAL_GPIO_WritePin(OUTPUT_PORT, OUTPUT_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = OUTPUT_PIN; // This is now GPIO_PIN_2
  // Mode, Pull, Speed are the same as LED for a simple MOSFET drive output
  HAL_GPIO_Init(OUTPUT_PORT, &GPIO_InitStruct);
}

void APP_ErrorHandler(void) { /* ... same ... */ 
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
  while (1) { HAL_GPIO_TogglePin(LED_PORT, LED_PIN); HAL_Delay(50); }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { APP_ErrorHandler(); }
#endif