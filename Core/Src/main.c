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

static void debug_printf(const char* format, ...);
static void debug_log(const char* format, ...);

int8_t _icm20602_hal_wr(uint8_t id, uint8_t reg, uint8_t * data, uint16_t len);
int8_t _icm20602_hal_rd(uint8_t id, uint8_t reg, uint8_t * data, uint16_t len);
//void   _icm20602_mutex_lock(uint8_t id);
//void   _icm20602_mutex_unlock(uint8_t id);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int8_t rand_sign(void) {
	return (rand() % 100 > 50) ? 1 : -1;
}

uint8_t rand_bool(void) {
	return (rand() % 100 > 50) ? 1 : 0;
}

float altitude_from_pressure(float pressure, float temperature) {
	static float pressure0 = 101325;
	return ((pow(pressure0 / pressure, 1 / 5.257) - 1) * (temperature + 273.15)) / 0.0065;
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

  debug_log("I2C Devices:");
  for(uint8_t i = 1, ret; i < 128; i++)
    if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5))
    	debug_log("0x%X", i);


  debug_log("ICM-20602 setup");
  struct icm20602_dev icm20602 = ICM20602_DEFAULT_INIT();
  icm20602.i2c_disable = true;
  icm20602.hal_sleep = HAL_Delay;
  icm20602.hal_rd = _icm20602_hal_rd;
  icm20602.hal_wr = _icm20602_hal_wr;
  icm20602_init(&icm20602);

  debug_log("OLED setup");
  char str_buffer[256] = "Boop zero";
  struct { float x, y, dx, dy; uint8_t r; } pos;
  uint32_t last_boop = HAL_GetTick();
  uint32_t boop_counter = 0;
  uint32_t beep_counter = 0;
  uint8_t button_presed_prev = 0;

  const uint8_t alts_count = 50;
  float alts[alts_count];
  uint8_t alts_curr = 0;

  pos.r = 10;
  pos.x = (128 - pos.r) / 2;
  pos.y = (64 - pos.r) / 2;
  pos.dx = rand_sign() * (rand() % 10 / 2. + 1);
  pos.dy = rand_sign() * (rand() % 10 / 2. + 1);

  BMP280_HandleTypedef bmp280;
  float pressure, temperature, altitude, altitude0;
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;

  if (!bmp280_init(&bmp280, &bmp280.params)) {
	  const char* baro_error = "Baro init failed";
	  debug_log(baro_error);
  }

  HAL_Delay(100);

  bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
  altitude0 = altitude_from_pressure(pressure, temperature);

  for (uint8_t i = 0; i < alts_count; i++) {
	  bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
	  alts[i] = altitude_from_pressure(pressure, temperature);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  pos.x += pos.dx;
	  pos.y += pos.dy;

	  if (pos.x < 0) {
		  pos.x = 0;
		  pos.dx = -pos.dx;
	  } else if (pos.x + pos.r >= 128) {
		  pos.x = 128 - pos.r;
		  pos.dx = -pos.dx;
	  }

	  if (pos.y < 50) {
		  pos.y = 50;
		  pos.dy = -pos.dy;
	  } else if (pos.y + pos.r >= 64) {
		  pos.y = 64 - pos.r;
		  pos.dy = -pos.dy;
	  }

	  uint32_t now = HAL_GetTick();
	  if (now - last_boop > 1000) {
		  last_boop = now;
		  boop_counter++;
		  HAL_GPIO_TogglePin(GPIOB, LED_Blue_Pin);
	  }

	  bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
	  alts[alts_curr] = altitude_from_pressure(pressure, temperature);
	  alts_curr = (alts_curr + 1) % alts_count;
	  altitude = 0;
	  for (uint8_t i = 0; i < alts_count; i++)
		  altitude += alts[i];
	  altitude /= alts_count;

	  uint8_t button_presed = !HAL_GPIO_ReadPin(GPIOC, Button_Pin);

	  if (button_presed != button_presed_prev) {
		  HAL_GPIO_WritePin(GPIOC, Buzzer_Pin, button_presed);
		  if (button_presed)
			  beep_counter++;
		  button_presed_prev = button_presed;
	  }

	  if (button_presed) {
		  altitude0 = altitude;
	  }

	  ssd1306_Fill(Black);

	  snprintf(str_buffer, sizeof(str_buffer) - 1, "#%u (%d, %d), %u", boop_counter, (uint8_t)pos.x, (uint8_t)pos.y, beep_counter);
	  ssd1306_SetCursor(0, 0);
	  ssd1306_WriteString(str_buffer, Font_6x8, White);

	  snprintf(str_buffer, sizeof(str_buffer) - 1, "Pressure: %.2f Pa", pressure);
	  ssd1306_SetCursor(0, 10);
	  ssd1306_WriteString(str_buffer, Font_6x8, White);

	  snprintf(str_buffer, sizeof(str_buffer) - 1, "Temperature: %.2f C", temperature);
	  ssd1306_SetCursor(0, 20);
	  ssd1306_WriteString(str_buffer, Font_6x8, White);

	  snprintf(str_buffer, sizeof(str_buffer) - 1, "Alt_relative: %.2f m", altitude - altitude0);
	  ssd1306_SetCursor(0, 30);
	  ssd1306_WriteString(str_buffer, Font_6x8, White);

	  float ax, ay, az;
	  float gx, gy, gz;

	  icm20602_read_accel(&icm20602, &ax, &ay, &az);
	  icm20602_read_gyro(&icm20602, &gx, &gy, &gz);

	  snprintf(
        str_buffer,
		sizeof(str_buffer) - 1,
		"G %f %f %f A %f %f %f",
		ax, ay, az, gx, gy, gz
	  );
	  ssd1306_SetCursor(0, 40);
	  ssd1306_WriteString(str_buffer, Font_6x8, White);

	  ssd1306_DrawRectangle(pos.x, pos.y, pos.x + pos.r - 1, pos.y + pos.r - 1, White);
	  ssd1306_UpdateScreen();

	  HAL_Delay(33);
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

static void debug_printf(const char* format, ...) {
  static char buff[1024];
  va_list argptr;
  va_start(argptr, format);
  vsnprintf(buff, 1023, format, argptr);
  va_end(argptr);
  const size_t len = strlen(buff);
  HAL_UART_Transmit(&huart1, (uint8_t*)buff, len, 1000);
}


static void debug_log(const char* format, ...) {
  static char buff[1024];
  va_list argptr;
  va_start(argptr, format);
  vsnprintf(buff, 1023, format, argptr);
  va_end(argptr);
  const size_t len = strlen(buff);
  HAL_UART_Transmit(&huart1, (uint8_t*)buff, len, 1000);
  HAL_UART_Transmit(&huart1, "\n", 1, 1000);
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

