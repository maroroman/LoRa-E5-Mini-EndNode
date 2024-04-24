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
#include "dma.h"
#include "i2c.h"
#include "subghz.h"
#include "app_lorawan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <time.h>
#include "sys_app.h"
#include "sys_conf.h"
#include "lora_app.h"
#ifdef USE_AHT20_SENSOR
  // i2c.h inclusion is controlled by device configuration
  #include "aht20.h"
#endif
#ifdef USE_PMS_SENSOR
  // usart.h inclusion is controlled by device configuration
#endif
#include "mpu6050.h"
#include <bme280.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // init random var generator for fake sensor readings
  srand((unsigned) time(NULL));
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
  MX_DMA_Init();
  MX_SUBGHZ_Init();
  MX_LoRaWAN_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
#ifdef USE_PMS_SENSOR
  // init particle sensor
  if (initPMS()) {
	  APP_LOG(TS_OFF, VLEVEL_M, "\r\n UART: INIT OK\r\n");
  } else {
	  APP_LOG(TS_OFF, VLEVEL_M, "\r\n UART: INIT FAILED\r\n");
  }
#endif

#ifdef USE_AHT20_SENSOR
  // tell aht20 lib which i2c to use
  setI2CHandle(&hi2c2);
#endif
  APP_LOG(TS_OFF, VLEVEL_M, "\r\n STARTING SENSORS...\r\n");
  //HAL_GPIO_WritePin(DBG4_GPIO_Port, DBG4_Pin, GPIO_PIN_SET);
  APP_LOG(TS_OFF, VLEVEL_M, "\r\n SENSORS STARTED!\r\n");
  APP_LOG(TS_OFF, VLEVEL_M, "\r\n STARTING GNSS...\r\n");
  //HAL_GPIO_WritePin(DBG3_GPIO_Port, DBG3_Pin, GPIO_PIN_RESET);
  APP_LOG(TS_OFF, VLEVEL_M, "\r\n GNSS STARTED!\r\n");

  loraSetI2CHandle(&hi2c2);
  loraSetUARTHandle(&huart2);
  APP_LOG(TS_OFF, VLEVEL_M, "\r\n INITIALISING GYROSCOPE...\r\n");
  //while (MPU6050_Init(&hi2c2) == 1);
  APP_LOG(TS_OFF, VLEVEL_M, "\r\n GYROSCOPE INITIALISED!\r\n");

  uint8_t devices = 0u;

    APP_LOG(TS_OFF, VLEVEL_M,"Searching for I2C devices on the bus...\n");
    /* Values outside 0x03 and 0x77 are invalid. */
    for (uint8_t i = 0x03u; i < 0x78u; i++)
    {
      uint8_t address = i << 1u ;
      /* In case there is a positive feedback, print it out. */
      if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c2, address, 3u, 10u))
      {
    	APP_LOG(TS_OFF, VLEVEL_M,"Device found: 0x%02X\n", address);
        devices++;
      }
    }
    /* Feedback of the total number of devices. */
    if (0u == devices)
    {
      APP_LOG(TS_OFF, VLEVEL_M,"No device found.\n");
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_M,"Total found devices: %d\n", devices);
    }

//  APP_LOG(TS_OFF, VLEVEL_M, "\r\n INITIALISING BAROMETER...\r\n");
//  struct bme280_t bme280;
//  bme280.bus_write = BME280_I2C_bus_write;
//  bme280.bus_read = BME280_I2C_bus_read;
//  bme280.dev_addr = BME280_I2C_ADDRESS2;
//  bme280.delay_msec = BME280_delay_msek;
//  dev.i2c = &hi2c2;
//  bmp280_init_default_params(&params);
//  while (bmp280_init(&dev, &params) == 1);
//  int32_t bmp280_temp = 0;
//  uint32_t bmp280_pres = 0;
//  uint32_t bmp280_hum = 0;
//  bmp280_is_measuring(dev)
//  bmp280_read_fixed(&dev, &bmp280_temp, &bmp280_pres, &bmp280_hum);
//  HAL_Delay(500);
//  APP_LOG(TS_OFF, VLEVEL_M, "BMP280 OUTPUT: TEMP: %d PRES: %d HUM: %d\r\n", bmp280_temp, bmp280_pres, bmp280_hum);
//  APP_LOG(TS_OFF, VLEVEL_M, "\r\n BAROMETER INITIALISED!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_LoRaWAN_Process();

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
