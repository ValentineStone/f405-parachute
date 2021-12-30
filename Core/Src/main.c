/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include "bmp280.h"
#include "icm20602.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ICM_I2C_ADDR 0x69
/* sensor ODR limit */
#define MIN_ODR_US        1000
#define MAX_ODR_US       20000
#define DEFAULT_ODR_US   20000

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RNG_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Minimal version, without OLED, builtin BEC and HC-12 module

char str_buffer[256] = "";

static void debug_printf(const char* format, ...) {
  static char buff[1024];
  va_list argptr;
  va_start(argptr, format);
  vsnprintf(buff, 1023, format, argptr);
  va_end(argptr);
  const size_t len = strlen(buff);
  HAL_UART_Transmit(&huart1, (uint8_t*)buff, len, 1000);
}

int8_t _icm20602_hal_rd(uint8_t id, uint8_t reg, uint8_t * data, uint16_t len) {
  reg |= 128; // set first bit to 1 to read from spi register
  HAL_GPIO_WritePin(GPIOC, SPI1_CS_MANUAL_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef s;
  s = HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
  s = s ? s : HAL_SPI_Receive(&hspi1, data, len, 1000);
  HAL_GPIO_WritePin(GPIOC, SPI1_CS_MANUAL_Pin, GPIO_PIN_SET);
  return s;
}

int8_t _icm20602_hal_wr(uint8_t id, uint8_t reg, uint8_t * data, uint16_t len) {
  HAL_GPIO_WritePin(GPIOC, SPI1_CS_MANUAL_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef s;
  s = HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
  s = s ? s : HAL_SPI_Transmit(&hspi1, data, len, 1000);
  HAL_GPIO_WritePin(GPIOC, SPI1_CS_MANUAL_Pin, GPIO_PIN_SET);
  return s;
}

float altitude_from_pressure_and_temp(float pressure, float temperature) {
  static float pressure0 = 101325;
  return ((pow(pressure0 / pressure, 1 / 5.257) - 1) * (temperature + 273.15)) / 0.0065;
}


float altitude_from_pressure(float pressure) {
  static float pressure0 = 101325;
  return  44330 * (1.0 - pow(pressure / pressure0, 0.1903));
}

float accel_abs(float ax, float ay, float az) {
  return sqrtf(powf(ax, 2) + pow(ay, 2) + pow(az, 2));
}

typedef struct {
  uint32_t interval;
  uint32_t last;
  uint32_t skipped;
} ticker_t;

ticker_t create_ticker(uint32_t interval) {
  return (ticker_t) { interval, 0, 0 };
}

bool tick(ticker_t* t) {
  uint32_t now = HAL_GetTick();
  if (now - t->last >= t->interval) {
    t->last = now;
    t->skipped = 0;
  } else
    t->skipped++;
  return !t->skipped;
}

typedef struct button_t {
  GPIO_TypeDef* GPIOx;
  uint16_t GPIO_Pin;
  uint32_t doubleclickInterval;
  uint32_t pressedAt;
  bool _reading1;
  bool _reading2;
  bool _reading3;
  bool pressed;
  bool changed;
  bool doubleclicked;
  bool toggleReleased;
  bool togglePressed;
  bool toggleDoubleclicked;
} button_t;

void update_button(button_t* button, uint32_t now) {
  button->_reading1 = button->_reading2;
  button->_reading2 = button->_reading3;
  button->_reading3 = !HAL_GPIO_ReadPin(button->GPIOx, button->GPIO_Pin);
  bool reading = button->_reading1 + button->_reading2 + button->_reading3 >= 2;
  button->doubleclicked = false;
  button->changed = false;
  if (button->pressed != reading) {
    button->pressed = reading;
    button->changed = true;
    if (button->pressed) {
      button->togglePressed = !button->togglePressed;
      if (now - button->pressedAt < button->doubleclickInterval) {
        button->doubleclicked = true;
        button->toggleDoubleclicked = !button->toggleDoubleclicked;
      }
      button->pressedAt = now;
    }
    else
      button->toggleReleased = !button->toggleReleased;
  } else
    button->changed = false;
}
void beep(uint8_t count, uint16_t duration, uint16_t silence) {
  for (uint8_t i = 0; i < count; i++) {
    HAL_GPIO_WritePin(GPIOC, Buzzer_Pin, GPIO_PIN_SET);
    HAL_Delay(duration);
    HAL_GPIO_WritePin(GPIOC, Buzzer_Pin, GPIO_PIN_RESET);
    HAL_Delay(silence);
  }
}

void on_armed() {
  HAL_GPIO_WritePin(GPIOB, LED_Blue_Pin, GPIO_PIN_RESET);
  beep(3, 50, 50);
}

void on_disarmed() {
  beep(3, 50, 50);
}

void on_alarm() {

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_RNG_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  srand(HAL_GetTick());

  struct icm20602_dev icm20602 = ICM20602_DEFAULT_INIT();
  icm20602.i2c_disable = true;
  icm20602.hal_sleep = HAL_Delay;
  icm20602.hal_rd = _icm20602_hal_rd;
  icm20602.hal_wr = _icm20602_hal_wr;
  icm20602.accel_dlpf = ICM20602_ACCEL_DLPF_BYPASS_1046_HZ;
  icm20602_init(&icm20602);

  ticker_t blink_ticker = create_ticker(1000);
  button_t button = { GPIOC, Button_Pin, 300 };

  ticker_t buzzer_alarm_ticker = create_ticker(200);

  bool armed = false;
  uint32_t arm_pressed = 0;
  uint32_t arm_timeout = 1000;

  bool alarm = false;

  bool low_g_detected = false;
  uint32_t low_g_detected_since = 0;
  float low_g_detection_range = 0.5;
  uint32_t low_g_detection_interval = 1000;

  float baro_fall_detection_speed = 10;
  uint32_t baro_fall_detection_interval = 1000;

  on_disarmed();

  const uint8_t alts_count = 20;
  float alts[alts_count];
  uint32_t alt_times[alts_count];
  uint8_t alts_curr = 0;

  const uint8_t accs_count = 5;
  float accs[accs_count];
  uint8_t accs_curr = 0;

  BMP280_HandleTypedef bmp280;
  bmp280_init_default_params(&bmp280.params);
  bmp280.params.standby = BMP280_STANDBY_62;
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  if (!bmp280_init(&bmp280, &bmp280.params))
    debug_printf("Baro init failed\n");

  // Initialize alt and accel

  HAL_Delay(100);

  float pressure, temperature, altitude, altitude_base;
  bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
  altitude_base = altitude_from_pressure(pressure);
  altitude = altitude_base;
  for (uint16_t i = 0; i < alts_count; i++)
    alts[i] = altitude_base;

  float ax, ay, az;
  icm20602_read_accel(&icm20602, &ax, &ay, &az);
  float acceleration = accel_abs(ax, ay, az);
  for (uint16_t i = 0; i < accs_count; i++)
    accs[i] = acceleration;

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /*
    int32_t DC = 0;
    while(DC < 65535)
    {
        TIM4->CCR3 = DC;
        DC += 70;
        HAL_Delay(1);
    }
    while(DC > 0)
    {
        TIM4->CCR3 = DC;
        DC -= 70;
        HAL_Delay(1);
    }
    */

    for(int x = 0; x < 65535; x += 10) {
      TIM4->CCR3 = x;
      //__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, x);
      HAL_Delay(3);
    }

    uint32_t now = HAL_GetTick();

    update_button(&button, now);

    if (tick(&blink_ticker) && !armed)
        HAL_GPIO_TogglePin(GPIOB, LED_Blue_Pin);

    if(alarm && tick(&buzzer_alarm_ticker))
      HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);

    if (button.pressed)
      altitude_base = altitude;

    if (button.pressed) {
      if (button.changed)
        arm_pressed = now;
    } else
      arm_pressed = 0;

    if (arm_pressed && now - arm_pressed > arm_timeout) {
      if (armed) {
        armed = false;
        arm_pressed = 0;
        alarm = false;
        on_disarmed();
      } else {
        armed = true;
        arm_pressed = 0;
        button.toggleDoubleclicked = true;
        on_armed();


        bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
        altitude_base = altitude_from_pressure(pressure);
        altitude = altitude_base;
        for (uint16_t i = 0; i < alts_count; i++) {
          alts[i] = altitude_base;
          alt_times[i] = 0;
        }

        icm20602_read_accel(&icm20602, &ax, &ay, &az);
        acceleration = accel_abs(ax, ay, az);
        for (uint16_t i = 0; i < accs_count; i++)
          accs[i] = acceleration;
      }
    }


    if (armed && !alarm) {
      float ax_prev = ax, ay_prev = ay, az_prev = az;
      icm20602_read_accel(&icm20602, &ax, &ay, &az);
      if (ax != ax_prev || ay != ay_prev || az != az_prev) {

        float acceleration_now = accel_abs(ax, ay, az);
        accs[accs_curr] = acceleration_now;

        acceleration = 0;
        for (uint16_t i = 0; i < accs_count; i++)
          acceleration += accs[i];
        acceleration /= accs_count;

        if (acceleration_now < low_g_detection_range) {
          if (low_g_detected) {
            if (now - low_g_detected_since > low_g_detection_interval) {
              alarm = true;
              on_alarm();
            }
          } else {
            low_g_detected = true;
            low_g_detected_since = now;
          }
        } else {
          low_g_detected = false;
        }

        float low_g_time = !low_g_detected ? 0 : ((now - low_g_detected_since) / (float)low_g_detection_interval);
        low_g_time = low_g_time > 3 ? 3 : low_g_time;

        if (button.toggleDoubleclicked){
          //debug_printf("min:-3 max:3 x:%f y:%f z:%f cur:%f avg:%f t:%f\n", ax, ay, az, acceleration_now, acceleration, low_g_time);
        }
        accs_curr = (accs_curr + 1) % accs_count;
      }

      float pressure_prev = pressure;
      float temperature_prev = temperature;
      bmp280_read_float(&bmp280, &temperature, &pressure, NULL);

      if (pressure != pressure_prev || temperature != temperature_prev) {
        float altitude_now = altitude_from_pressure(pressure);
        alts[alts_curr] = altitude_now;
        alt_times[alts_curr] = now;
        altitude = 0;
        for (uint16_t i = 0; i < alts_count; i++)
          altitude += alts[i] / alts_count;

        uint8_t start_index = (alts_curr + 1) % alts_count;
        float alt_diff = 0;
        for (uint16_t i = 0; i < alts_count - 2; i++) {
          uint16_t i0 = (alts_count + alts_curr + i - 1) % alts_count;
          uint16_t i1 = (alts_count + alts_curr + i) % alts_count;
          bool timely = now - alt_times[i0] < baro_fall_detection_interval;
          if (timely) {
            alt_diff += alts[i0] - alts[i1];
            if (alt_diff <= -baro_fall_detection_speed) {
              alarm = true;
              on_alarm();
              break;
            }
          } else break;
        }

        alts_curr = (alts_curr + 1) % alts_count;

        if (button.toggleDoubleclicked) {
          debug_printf("cur:%f avg:%f dif:%f\n", altitude_now - altitude_base, altitude - altitude_base, alt_diff);
        }
      }

    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Buzzer_Pin|SPI1_CS_MANUAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_MANUAL_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_MANUAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_MANUAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Blue_Pin */
  GPIO_InitStruct.Pin = LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Blue_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//point_t scale(point_t p, point_t s);

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

