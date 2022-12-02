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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int val=0;
uint8_t p1=0;
uint8_t p2=0;
uint8_t p3=0;
int val2=0;
float trawsc = 0;
float prawsc = 0;
float pcomp=0;
int iter = 0;
float initial = 0;
float altitude = 0;
float dif = 0;

void getTwosComplement(int32_t *raw, uint8_t length)
{
	if (*raw & ((uint32_t)1 << (length - 1)))
	{
		*raw -= (uint32_t)1 << length;
	}
}

int main(void)
{
	int k = 253952;

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

//READ FROM reg 0x10 to 0x21
  uint8_t buffer[18]={0};
  HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0x10, 1, buffer, 18, 1000);
  uint32_t c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
  getTwosComplement(&c00, 20);
  int dc00 = (int) c00;

  uint32_t c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
  getTwosComplement(&c10, 20);
  int dc10 = (int) c10;

  uint32_t c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
  getTwosComplement(&c01, 16);
  int dc01 = (int) c01;

  uint32_t c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
  getTwosComplement(&c11, 16);
  int dc11 = (int) c11;


  uint32_t c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
  getTwosComplement(&c20, 16);
  int dc20 = (int) c20;


  uint32_t c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
  getTwosComplement(&c21, 16);
  int dc21 = (int) c21;


  uint32_t c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
  getTwosComplement(&c30, 16);
  int dc30 = (int) c30;

//WRITE TO REG 0x06
  uint8_t prscfg[1];
  prscfg[0] = 0x04;
  HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0x06, 1, prscfg, 1, 1000);


//WRITE TO REG 0x07
  uint8_t tmpcfg[1];
  tmpcfg[0] = 0x04;
  HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0x07, 1, tmpcfg, 1, 1000);


//WRITE TO REG 0x09
  uint8_t cfgreg[1];
  cfgreg[0] = 0x0C;
  HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0x09, 1, cfgreg, 1, 1000);

//WRITE 0x07 TO REG 0x08
  uint8_t meascfg[1];
  meascfg[0] = 0x07;
  HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0x08, 1, meascfg, 1, 1000);


  while (1)
  {
/*
//WRITE 0x02 TO REG 0x08
	  uint8_t meascfg[1];
	  meascfg[0] = 0x02;
	  HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0x08, 1, &meascfg, 1, 1000);
	  uint8_t test[1]={0};
	  	  HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0x08, 1, test, 1, 1000);
	  	  val = test[0];
*/

//READ REG 0x03 TO REG 0x05
	  uint8_t tmp[3]={0};
	  HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0x03, 1, tmp, 3, 1000);
	  uint32_t tmpraw =((tmp[0] << 16) |(tmp[1] << 8) | (tmp[2]));
	  getTwosComplement(&tmpraw, 24);
	  int traw = (int) tmpraw;
	  val=traw;
	  //val2=traw;

//CALC TEMP
//NOT NEEDED

/*
//WRITE 0x01 TO REG 0x08
	  uint8_t meascfg2[1];
	  meascfg2[0] = 0x01;
	  HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0x08, 1, &meascfg2, 1, 1000);
	  uint8_t test2[1]={0};
	  HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0x08, 1, test2, 1, 1000);
	  val2 = test2[0];
*/

//READ REG 0x00 TO REG 0x02
	  uint8_t prs[3]={0};
	  HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0x00, 1, prs, 3, 1000);
	  uint32_t prsraw = prs[0] << 16 | prs[1] << 8 | prs[2];
	  getTwosComplement(&prsraw, 24);
	  int praw = (int) prsraw;
	  val2=praw;

//CALC PRESSURE
	  prawsc = (float) val2/k;
	  trawsc = (float) val/k;
	  pcomp = dc00+prawsc*(dc10+prawsc*(dc20+prawsc*dc30))+trawsc*dc01+trawsc*prawsc*(dc11+prawsc*dc21);
	  pcomp = pcomp/100;
	  altitude = (float) 44330 * (1-pow(pcomp/1015,1/5.255));
	  if (iter == 0)
	  {
		  initial = altitude;
		  //HAL_UART_Transmit(&huart2, "Starting altitude\n", strlen("Starting altitude\n"), HAL_MAX_DELAY);
	  }
	  else
	  {
		  dif = altitude - initial;
		  //HAL_UART_Transmit(&huart2, "Diff from starting point\n", strlen("Diff from starting point\n"), HAL_MAX_DELAY);
	  }
	  iter++;
  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

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
