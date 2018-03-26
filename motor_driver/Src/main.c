/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#define SERVO_MID_BACK  1750
#define SERVO_MID_FRONT 1790
#define MPU6050reg 0xD0

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/

#define dataSize 5
#define rxSize 6
uint8_t data[dataSize] = { 0xAA, 0x00, 0x35, 0x19, 0x56 };
uint8_t rxBuf[rxSize] = { 0 };
uint8_t GyroAccConfig[6] = { 0x1B, 0x08, 0x1C, 0x08, 0x6B, 0x00 }; // {Gyro Sense 500/s , Acc Sense 4g, Wake MPU6050}

typedef struct

{
	uint16_t AccX;
	uint16_t AccY;
	uint16_t AccZ;
	uint16_t Temp;
	uint16_t GyX;
	uint16_t GyY;
	uint16_t GyZ;
} GyroAcc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/
void setMotors(int8_t Spd, int8_t Bal);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM1_Init();
	MX_SPI2_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	htim2.Instance->CCR4 = SERVO_MID_BACK;
	htim4.Instance->CCR1 = SERVO_MID_FRONT;
	htim1.Instance->CCR1 = 100;
	htim1.Instance->CCR2 = 100;
	htim1.Instance->CCR3 = 100;
	htim1.Instance->CCR4 = 100;
	HAL_GPIO_WritePin(M0_REF_GPIO_Port, M0_REF_Pin, SET);
	HAL_GPIO_WritePin(M1_REF_GPIO_Port, M1_REF_Pin, SET);
	HAL_GPIO_WritePin(M2_REF_GPIO_Port, M2_REF_Pin, SET);
	HAL_GPIO_WritePin(M3_REF_GPIO_Port, M3_REF_Pin, SET);
	HAL_GPIO_WritePin(M0_DIR_GPIO_Port, M0_DIR_Pin, RESET);
	HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, RESET);
	HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, RESET);
	HAL_GPIO_WritePin(M3_DIR_GPIO_Port, M3_DIR_Pin, RESET);

	HAL_SPI_Init(&hspi2);

	//I2C INITIALIZATION


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/*

		 htim1.Instance->CCR1 = 1200;

		 htim1.Instance->CCR2 = 1200;

		 htim1.Instance->CCR3 = 1200;

		 htim1.Instance->CCR4 = 1200;

		 */
		/*
		 for (int i = -25; i < 25; i++) {
		 htim1.Instance->CCR1 = (i + 30) * 40;
		 htim1.Instance->CCR2 = (i + 30) * 40;
		 htim1.Instance->CCR3 = (i + 30) * 40;
		 htim1.Instance->CCR4 = (i + 30) * 40;

		 htim2.Instance->CCR4 = SERVO_MID_BACK+i*10;
		 htim4.Instance->CCR1 = SERVO_MID_FRONT-i*10;

		 HAL_Delay(5);

		 }
		 */
		/*
		 for (int i = 25; i > -25; i--) {
		 htim1.Instance->CCR1 = (i + 30) * 40;
		 htim1.Instance->CCR2 = (i + 30) * 40;
		 htim1.Instance->CCR3 = (i + 30) * 40;
		 htim1.Instance->CCR4 = (i + 30) * 40;

		 htim2.Instance->CCR4 = SERVO_MID_BACK+i*10;
		 htim4.Instance->CCR1 = SERVO_MID_FRONT-i*10;

		 HAL_Delay(5);

		 }
		 */

		HAL_Delay(20);

		for (int i = 0; i < rxSize; i++) {
			rxBuf[i] = 0;

		}

		HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, RESET);

		HAL_SPI_DeInit(&hspi2);
		hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
		HAL_SPI_Init(&hspi2);

		HAL_SPI_Transmit(&hspi2, data, 2, 800);

		HAL_SPI_DeInit(&hspi2);
		hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
		HAL_SPI_Init(&hspi2);

		HAL_SPI_Receive(&hspi2, rxBuf, rxSize, 800);

		HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, SET);

		for (int i = 0; i < rxSize - 1; i++) {
			rxBuf[i] = (rxBuf[i] << 1) | ((rxBuf[i + 1] & 0x80) >> 7);
		}

		int dist1 = rxBuf[1] | (rxBuf[2] << 8);
		int dist2 = rxBuf[3] | (rxBuf[4] << 8);

		HAL_I2C_Master_Transmit(&hi2c2, MPU6050reg, &GyroAccConfig[0], 2, 1000); // CONFIG GYRO SENSITIVITY to 500 - default 250 degrees/second
		HAL_I2C_Master_Transmit(&hi2c2, MPU6050reg, &GyroAccConfig[2], 2, 1000); // CONFIG ACC SENSE to 4g - default 2g
		HAL_I2C_Master_Transmit(&hi2c2, MPU6050reg, &GyroAccConfig[4], 2, 1000); // Wake AccGyro

		dist2=(dist2>1000)?1000:dist2;

		dist2 = (dist2-500)/5;

		setMotors(0,(int8_t)dist2);
/*
		for (int8_t i = 0; i < 120; i++) {

			setMotors(i, 0);

			HAL_Delay(50);
		}
*/
		/* USER CODE END 3 */

	}

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void setMotors(int8_t Spd, int8_t Bal) {
	//uint8_t spd_abs = (Spd > 0 ? Spd : -Spd);
	htim1.Instance->CCR1 = Spd * 8;
	htim1.Instance->CCR2 = Spd * 8;
	htim1.Instance->CCR3 = Spd * 8;
	htim1.Instance->CCR4 = Spd * 8;

	htim2.Instance->CCR4 = SERVO_MID_BACK + Bal * 2;
	htim4.Instance->CCR1 = SERVO_MID_FRONT - Bal * 2;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */

	/* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */

	/* User can add his own implementation to report the file name and line number,

	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
