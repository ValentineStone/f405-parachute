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
#include "ssd1306.h"
#include "ssd1306_fonts.h"
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

float altitude_from_pressure(float pressure, float temperature) {
  static float pressure0 = 101325;
  return ((pow(pressure0 / pressure, 1 / 5.257) - 1) * (temperature + 273.15)) / 0.0065;
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

void on_armed() {
  HAL_GPIO_WritePin(GPIOB, LED_Blue_Pin, GPIO_PIN_RESET);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  snprintf(str_buffer, sizeof(str_buffer) - 1, "ARMED");
  ssd1306_WriteString(str_buffer, Font_16x26, White);
  ssd1306_UpdateScreen();

  for (uint8_t i = 0; i < 6 - 1; i++) {
    HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
    HAL_Delay(50);
  }
  HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
}

void on_disarmed() {
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  snprintf(str_buffer, sizeof(str_buffer) - 1, "DISARMED");
  ssd1306_WriteString(str_buffer, Font_16x26, White);
  ssd1306_UpdateScreen();

  for (uint8_t i = 0; i < 6 - 1; i++) {
    HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
    HAL_Delay(50);
  }
  HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
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
  /* USER CODE BEGIN 2 */

  srand(HAL_GetTick());
  ssd1306_Init();

  struct icm20602_dev icm20602 = ICM20602_DEFAULT_INIT();
  icm20602.i2c_disable = true;
  icm20602.hal_sleep = HAL_Delay;
  icm20602.hal_rd = _icm20602_hal_rd;
  icm20602.hal_wr = _icm20602_hal_wr;
  icm20602_init(&icm20602);

  ticker_t blink_ticker = create_ticker(1000);
  button_t button = { GPIOC, Button_Pin, 300 };

  ticker_t buzzer_alarm_ticker = create_ticker(1000);

  bool armed = false;
  uint32_t arm_pressed = 0;
  uint32_t arm_timeout = 1000;

  bool alarm = false;
  bool low_g_detected = false;
  uint32_t low_g_detected_since = 0;

  uint32_t arm_togglePressed_at = 0;
  uint32_t arm_doubleclick_interval = 300;
  bool arm_doubleclicked = 0;

  on_disarmed();

  const uint8_t alts_count = 100;
  float alts[alts_count];
  uint8_t alts_curr = 0;
  float alt_base = 0;

  const uint8_t accs_count = 5;
  float accs[accs_count];
  uint8_t accs_curr = 0;

  BMP280_HandleTypedef bmp280;
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  if (!bmp280_init(&bmp280, &bmp280.params))
    debug_printf("Baro init failed\n");

  // Initialize alt and accel

  HAL_Delay(100);

  float pressure, temperature, altitude;
  bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
  alt_base = altitude_from_pressure(pressure, temperature);
  altitude = alt_base;
  for (uint16_t i = 0; i < alts_count; i++)
    alts[i] = alt_base;

  float ax, ay, az;
  icm20602_read_accel(&icm20602, &ax, &ay, &az);
  float acceleration = accel_abs(ax, ay, az);
  for (uint16_t i = 0; i < accs_count; i++)
    accs[i] = acceleration;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint32_t now = HAL_GetTick();

    update_button(&button, now);

    if (tick(&blink_ticker) && !armed)
        HAL_GPIO_TogglePin(GPIOB, LED_Blue_Pin);

    if (alarm && tick(&buzzer_alarm_ticker))
      HAL_GPIO_TogglePin(GPIOC, Buzzer_Pin);
    else if (button.changed)
      HAL_GPIO_WritePin(GPIOC, Buzzer_Pin, button.pressed);

    if (button.pressed)
      alt_base = altitude;

    if (button.pressed) {
      if (button.changed)
        arm_pressed = now;
    } else
      arm_pressed = 0;

    if (arm_pressed && now - arm_pressed > arm_timeout) {
      if (armed) {
        armed = false;
        arm_pressed = 0;
        on_disarmed();
      } else {
        armed = true;
        arm_pressed = 0;
        on_armed();
      }
    }

    /*
    bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
    alts[alts_curr] = altitude_from_pressure(pressure, temperature);
    altitude = 0;
    for (uint16_t i = 0; i < alts_count; i++)
      altitude += alts[i] / alts_count;
    alts_curr = (alts_curr + 1) % alts_count;
    */

    if (armed) {
      float ax_prev = ax, ay_prev = ay, az_prev = az;
      icm20602_read_accel(&icm20602, &ax, &ay, &az);
      if (ax != ax_prev || ay != ay_prev || az != az_prev) {

      accs[accs_curr] = accel_abs(ax, ay, az);

      acceleration = 0;
      for (uint16_t i = 0; i < accs_count; i++)
        acceleration += accs[i];
      acceleration /= accs_count;
      if (button.toggleDoubleclicked)
        debug_printf("min:-30 max:30 x:%f y:%f z:%f cur:%f avg:%f\n", 10*ax, 10*ay, 10*az, 10*accs[accs_curr], 10*acceleration);
      accs_curr = (accs_curr + 1) % accs_count;
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);

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

