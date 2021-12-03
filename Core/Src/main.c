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
#include "Invn/Devices/Drivers/Icm20602/Icm20602.h"
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

static inv_icm20602_t icm_device;
static uint8_t chip_info[3];
static uint32_t period_us = DEFAULT_ODR_US /* 50Hz by default */;
static int32_t cfg_acc_fsr = 4000; /* +/- 4g */
static int32_t cfg_gyr_fsr = 2000; /* +/- 2000dps */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RNG_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

static void debug_log(const char* format, ...);

static int icm20602_sensor_setup(void);
static int icm20602_sensor_configuration(void);

static int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static void check_rc(int rc, const char * context_str);

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

  struct inv_icm20602_serif icm20602_serif;
  icm20602_serif.context   = 0; // no need
  icm20602_serif.read_reg  = idd_io_hal_read_reg;
  icm20602_serif.write_reg = idd_io_hal_write_reg;
  icm20602_serif.max_read  = 1024*32; // maximum number of bytes allowed per serial read
  icm20602_serif.max_write = 1024*32; // maximum number of bytes allowed per serial write
  icm20602_serif.is_spi    = 1;
  inv_icm20602_reset_states(&icm_device, &icm20602_serif);
  icm20602_sensor_setup();
  icm20602_sensor_configuration();

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

	  /*snprintf(
        str_buffer,
		sizeof(str_buffer) - 1,
		"G %u %u %u %u %u %u",
		icm_device.base_state.gyro_averaging,
		icm_device.base_state.accel_bw,
		icm_device.base_state.accel_div,
		icm_device.base_state.accel_div,
		icm_device.base_state.accel_fullscale,
		icm_device.base_state.accel_ois_fullscale
	  );
	  ssd1306_SetCursor(0, 40);
	  ssd1306_WriteString(str_buffer, Font_6x8, White);*/

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

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

static const uint8_t EXPECTED_WHOAMI[] = { 0x12, 0x11 };

int icm20602_sensor_setup(void)
{
	int rc;
	uint8_t i, whoami = 0xff;

	/*
	 * Just get the whoami
	 */
	rc = inv_icm20602_get_whoami(&icm_device, &whoami);
	debug_log("ICM20602 WHOAMI=0x%02x", whoami);
	check_rc(rc, "Error reading WHOAMI");

	/*
	 * Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
	 */
	for(i = 0; i < sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0]); ++i) {
		if(whoami == EXPECTED_WHOAMI[i])
			break;
	}

	if(i == sizeof(EXPECTED_WHOAMI)/sizeof(EXPECTED_WHOAMI[0])) {
		debug_log("Bad WHOAMI value. Got 0x%02x. Expected 0x12, 0x11.", whoami);
		//check_rc(-1, "");
	}

	rc = inv_icm20602_get_chip_info(&icm_device, chip_info);
	check_rc(rc, "Could not obtain chip info");

	/*
	 * Configure and initialize the ICM20602 for normal use
	 */
	debug_log("Booting up icm20602...");

	/* set default power mode */
	if (!inv_icm20602_is_sensor_enabled(&icm_device, INV_ICM20602_SENSOR_GYRO) &&
		!inv_icm20602_is_sensor_enabled(&icm_device, INV_ICM20602_SENSOR_ACCEL)) {
		debug_log("Putting icm20602 in sleep mode...");
		rc = inv_icm20602_initialize(&icm_device);
		check_rc(rc, "Error %d while setting-up icm20602 device");
	}

	/* set default ODR = 50Hz */
	rc = inv_icm20602_set_sensor_period(&icm_device, INV_ICM20602_SENSOR_ACCEL, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up icm20602 device");

	rc = inv_icm20602_set_sensor_period(&icm_device, INV_ICM20602_SENSOR_GYRO, DEFAULT_ODR_US/1000 /*ms*/);
	check_rc(rc, "Error %d while setting-up icm20602 device");

	period_us = DEFAULT_ODR_US;

	/* we should be good to go ! */
	debug_log("We're good to go !");

	return 0;
}

int icm20602_sensor_configuration(void)
{
	int rc;

	debug_log("Configuring accelerometer FSR");
	rc = inv_icm20602_set_accel_fullscale(&icm_device, inv_icm20602_accel_fsr_2_reg(cfg_acc_fsr));
	check_rc(rc, "Error configuring ACC sensor");

	debug_log("Configuring gyroscope FSR");
	rc = inv_icm20602_set_gyro_fullscale(&icm_device, inv_icm20602_gyro_fsr_2_reg(cfg_gyr_fsr));
	check_rc(rc, "Error configuring GYR sensor");

	return rc;
}


static int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	HAL_StatusTypeDef s;
	uint8_t zero;
	uint8_t zero_buff[rlen];
	memset(zero_buff, 0, rlen);
	debug_log("Read @%u %u bytes", reg, rlen);
	s = HAL_SPI_TransmitReceive(&hspi1, &reg, &zero, 1, 1000);
	if (s) return s;
	s = HAL_SPI_TransmitReceive(&hspi1, zero_buff, rbuffer, rlen, 1000);
	if (s) return s;
	debug_log("Result %u", s);
	return s;
	//int rc = HAL_I2C_Mem_Read(&hi2c1, ICM_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, rbuffer, rlen, 1000);
	//return HAL_I2C_Master_Receive(&hi2c1, ICM_I2C_ADDR, rbuffer, rlen, 1000);
	//return i2c_master_read_register(ICM_I2C_ADDR, reg, rlen, rbuffer);
}

static int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	HAL_StatusTypeDef s;
	uint8_t zero;
	uint8_t zero_buff[wlen];
	memset(zero_buff, 0, wlen);
	debug_log("Writ @%u %u bytes", reg, wlen);
	s = HAL_SPI_TransmitReceive(&hspi1, &reg, &zero, 1, 1000);
	if (s) return s;
	s = HAL_SPI_TransmitReceive(&hspi1, wbuffer, zero_buff, wlen, 1000);
	if (s) return s;
	debug_log("Result %u", s);
	return s;
	//int rc = HAL_I2C_Mem_Write(&hi2c1, ICM_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, wbuffer, wlen, 1000);
	//return HAL_I2C_Master_Transmit(&hi2c1, ICM_I2C_ADDR, wbuffer, wlen, 1000);
	//return i2c_master_write_register(ICM_I2C_ADDR, reg, wlen, wbuffer);
}

/*
 * Sleep implementation for ICM20602
 */
void inv_icm20602_sleep(int ms)
{
	HAL_Delay(ms);
}

void inv_icm20602_sleep_us(int us)
{
	HAL_Delay(us / 1000);
}

static void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		debug_log("%s: error %d (%s)", msg_context, rc, inv_error_str(rc));
		while(1);
	}
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

