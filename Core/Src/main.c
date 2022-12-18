/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int a = 0;
uint8_t rece_buf[4] = { 0 };
uint8_t type = 1;

//定时器回调函�?
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//	a++;
//	if (a > 999) {
//		a = 0;
//	}
//	if (a < 500) {
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 999);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 999);
//	} else {
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 999);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 999);
//	}

//	HAL_GPIO_WritePin(pwm1_GPIO_Port, pwm1_Pin, GPIO_PIN_SET);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, a);

//	if ( rece_buf[0] != 0) {
//		type = rece_buf[0];
//	}
//	if (type == 1) {
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 999);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//	} else {
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 999);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 999);
//	}

}

// 串口接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		HAL_UART_Transmit(&huart2, (uint8_t*) rece_buf, 4, 100);
		memset(rece_buf, 0, 4);
		// 重新使能接收中断
		HAL_UART_Receive_IT(&huart2, (uint8_t*) rece_buf, 1);
	}
}

// 匿名上位机数据格�?
void nimingModelData(uint8_t * buf, int16_t rol, int16_t pit, int16_t yaw) {
	uint8_t cnt = 0;

	buf[cnt++] = 0xAA;
	buf[cnt++] = 0xFF;
	buf[cnt++] = 0x03;
	buf[cnt++] = 7;

	buf[cnt++] = BYTE0(rol);
	buf[cnt++] = BYTE1(rol);

	buf[cnt++] = BYTE0(pit);
	buf[cnt++] = BYTE1(pit);

	buf[cnt++] = BYTE0(yaw);
	buf[cnt++] = BYTE1(yaw);

	buf[cnt++] = 2;

	uint8_t sc = 0;
	uint8_t ac = 0;
	for (int i = 0; i < cnt; i++) {
		sc += buf[i];
		ac += sc;
	}

	buf[cnt++] = sc;
	buf[cnt++] = ac;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	while (MPU6050_Init(&hi2c1) == 1);
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


	HAL_UART_Receive_IT(&huart2, (uint8_t*) rece_buf, 1);
//  HAL_UART_Transmit_IT(&huart2, (uint8_t*)rece_buf, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		MPU6050_Read_All(&hi2c1, &MPU6050);

		int16_t KalmanAngleX = (int16_t)(MPU6050.KalmanAngleX * 100);
		int16_t KalmanAngleY = (int16_t)(MPU6050.KalmanAngleY * 100);
		int16_t Gz = (int16_t)(0);

		uint8_t buf[13]={0};

		nimingModelData(buf, KalmanAngleX, KalmanAngleY, Gz);

//		sprintf(buffer, "%.12f", MPU6050.KalmanAngleX * 1000);	//9���������?
		HAL_UART_Transmit(&huart2, (uint8_t *)buf, 15, 300);
		HAL_Delay (100);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
	while (1) {
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
