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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "string.h"
#include "fonts.h"
#include "ssd1306.h"
#include <stdio.h>
#include <stdbool.h>

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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern bool MAX31865_Sensor_Error;
uint8_t RxDat[6];
uint8_t TxDat[6];
uint8_t temp [6];
char yazi[25];


void sendData (uint8_t *data)//gönderme işlem fonksiyonu
{
	// Pull DE high to enable TX operation
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, 1);
	HAL_UART_Transmit(&huart1, data, strlen(data) , 1000);
	// Pull RE Low to enable RX operation
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, 0);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  SSD1306_Init();
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxDat, 64);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	//  HAL_UART_Receive(&huart1, &temp, 8, 100);
	  	  	  	  	  	if(RxDat [1]>= 51)
	  	  	  	  	  		{
	  	  	  	  	  		sprintf(yazi, RxDat);
	  	  	  	  	  		TxDat[0] = 'H';
	  	  	  	  	  		TxDat[1] = 'I';
	  	  	  	  	  		TxDat[2] = 'G';
	  	  	  	  	  		TxDat[3] = 'H';
	  	  	  	  	  		TxDat[4] = ' ';
	  	  	  	  	  		TxDat[5] = '!';
	  	  	  	  	  		sendData(TxDat);
	  	  	  	  			SSD1306_Clear();
	  	  	  	  	  		SSD1306_GotoXY (50,5);
	  	  	  	  	  		SSD1306_Puts(yazi, &Font_11x18, 1);
  	  	  		  	  		SSD1306_GotoXY(25,25);
  	  	  		  	   		SSD1306_Puts("HIGH VALUE!!", &Font_7x10, 1);
	  	  	  	  	  		SSD1306_UpdateScreen();
	  	  	  	  	  		HAL_Delay(500);

	  	  	  	  	  		HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxDat, 64);
	  	  	  	  	  		}
	  	  	  	  	  else if(RxDat [1]< 51)
	  	  	  	  	  	  	{
	  	  	  	  	  	  		sprintf(yazi, RxDat);
	  	  	  	  	  	  		TxDat[0] = 'N';
	  	  	  	  	  	  		TxDat[1] = 'O';
	  	  	  	  	  	  		TxDat[2] = 'R';
	  	  	  	  	  	  		TxDat[3] = 'M';
	  	  	  	  	  	  		TxDat[4] = 'A';
	  	  	  	  	  	  		TxDat[5] = 'L';
	  	  	  		  	  		sendData(TxDat);
	  	  	  		  			SSD1306_Clear();
	  	  	  		  	  		SSD1306_GotoXY(50,5);
	  	  	  		  	   		SSD1306_Puts(yazi, &Font_11x18, 1);
	  	  	  		  	  		SSD1306_GotoXY(25,25);
	  	  	  		  	   		SSD1306_Puts("NORMAL VALUE", &Font_7x10, 1);
	  	  	  		  	   		SSD1306_UpdateScreen();
	  	  	  		  	   		HAL_Delay(500);
	  	  	  	  	  	  		HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxDat, 64);
	  	  	  	  	  	  		//send NORMAL
	  	  	  	  	  	  	}
	  	  	  	  /*int temp = TX_EN_GPIO_Port;// burada temp değişkeninen RS-485den alınan değer atanması amaçlanıyor.
	  	  	  	  	if(temp > 35)
	  	  	  	  		{
	  	  	  	  		sprintf(yazi, "%d.%d °C HIGH VALUE!!", (uint16_t) (temp),((uint16_t) (temp * 100) - ((uint16_t) temp) * 100) / 10);
	  	  	  	  		RxData[0] = 'R';
	  	  	  	  		RxData[1] = 'E';
	  	  	  	  		RxData[2] = 'F';
	  	  	  	  		RxData[3] = 'R';
	  	  	  	  		RxData[4] = 'I';
	  	  	  	  		RxData[5] = 'G';
	  	  	  	  		RxData[6] = 'E';
	  	  	  	  		RxData[7] = 'R';
	  	  	  	  		RxData[8] = 'A';
	  	  	  	  		RxData[9] = 'T';
	  	  	  	  		RxData[10] = 'E';
	  	  	  	  		RxData[11] = ' ';
	  	  	  	  		RxData[12] = '!';
	  	  	  	  		sendData(RxData);
	  	  	  	  		HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 64);//send REFRIGERATE !
	  	  	  	  		}
	  	  	  	  	else if(temp < 30)
	  	  	  	  		{
	  	  	  	  		sprintf(yazi, "%d.%d °C LOW VALUE!!", (uint16_t) (temp),((uint16_t) (temp * 100) - ((uint16_t) temp) * 100) / 10);
	  	  	  	  		RxData[0] = 'H';
	  	  	  	  		RxData[1] = 'E';
	  	  	  	  		RxData[2] = 'A';
	  	  	  	  		RxData[3] = 'T';
	  	  	  	  		RxData[4] = ' ';
	  	  	  	  		RxData[5] = '!';
	  	  	  	  		sendData(RxData);
	  	  	  	  		HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 64);
	  	  	  	  		//send HEAT !
	  	  	  	  		}
	  	  	  	  	else if(temp >30 && temp<35)
	  	  	  	  	{
	  	  	  	  		sprintf(yazi, "%d.%d °C NORMAL VALUE!!", (uint16_t) (temp),((uint16_t) (temp * 100) - ((uint16_t) temp) * 100) / 10);
	  	  	  	  		RxData[0] = 'N';
	  	  	  	  		RxData[1] = 'O';
	  	  	  	  		RxData[2] = 'R';
	  	  	  	  		RxData[3] = 'M';
	  	  	  	  		RxData[4] = 'A';
	  	  	  	  		RxData[5] = 'L';
	  	  	  	  		HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 64);
	  	  	  	  		//send NORMAL
	  	  	  	  	}*/


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TX_EN_Pin */
  GPIO_InitStruct.Pin = TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TX_EN_GPIO_Port, &GPIO_InitStruct);

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
