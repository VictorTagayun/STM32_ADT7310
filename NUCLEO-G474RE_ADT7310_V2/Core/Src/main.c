/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

#include "VT_ADT7310.h"
#include <stdio.h>

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
UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

/* USER CODE BEGIN PV */

uint8_t ADT7310_reset[4] = {0xff, 0xff, 0xff, 0xff}, ADT7310_RX[10], ADT7310_TX[3], read = 0x40, write = 0x00, address, num_data, SPI_TXRX_stat = 0;
float Temperature = 0;
uint16_t Temperature_binary = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void VT_Reset_ADT7310(void);
void VT_Print_Read_ADT7310_address(uint8_t ADT7310_address);
void VT_Print_Read_ADT7310_TempVal_16bit(void);
void VT_Print_Read_ADT7310_TempVal_8bit(void);
void VT_Print_Write_ADT7310(uint8_t ADT7310_address, uint16_t data);
int VT_Read_ADT7310_address(uint8_t ADT7310_address);
uint16_t Conv_temp_Float_to_Bin(float temperature);
void VT_ADT7310_address_Error(void);

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
	MX_LPUART1_UART_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

	printf("====================================\n");
	printf("Starting >> NUCLEO-G474RE_ADT7310_V2 \n");

	VT_Reset_ADT7310();

	VT_Print_Read_ADT7310_address(Status);

	VT_Print_Read_ADT7310_address(Config);

	VT_Print_Read_ADT7310_address(Temp_val);

	VT_Print_Read_ADT7310_address(ID);

	VT_Print_Read_ADT7310_address(TCRIT_setpt);

	VT_Print_Read_ADT7310_address(THYST_setpt);

	VT_Print_Read_ADT7310_address(THIGH_setpt);

	VT_Print_Read_ADT7310_address(TLOW_setpt);

	for(uint8_t cntr = 0; cntr < 3; cntr++)
	{
		HAL_Delay(500);
		VT_Print_Read_ADT7310_address(Temp_val);
	}

	for(uint8_t cntr = 0; cntr < 3; cntr++)
	{
		HAL_Delay(500);
		VT_Print_Read_ADT7310_TempVal_8bit();
	}

	printf("Converting to 16bit Temperature data... \n");
	VT_Print_Write_ADT7310(Config, ADT7310_16bit);

	printf("Read config register if written correctly.. \n");
	VT_Print_Read_ADT7310_address(Config);

	for(uint8_t cntr = 0; cntr < 3; cntr++)
	{
		HAL_Delay(500);
		VT_Print_Read_ADT7310_TempVal_16bit();
	}

	printf("Setting THIGH_setpt to 65degC... \n");
	VT_Print_Write_ADT7310(THIGH_setpt, Conv_temp_Float_to_Bin(65.0));

	printf("Read THIGH_setpt register if written correctly.. \n");
	VT_Print_Read_ADT7310_address(THIGH_setpt);

	printf("Ending   >> NUCLEO-G474RE_ADT7310_V2 \n\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(50);
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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void)
{

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void VT_Reset_ADT7310(void)
{
	printf("VT_Reset_ADT7310 (Resetting ADT7310.......) \n");
	num_data = 4;
	switch (HAL_SPI_Transmit_DMA(&hspi2, ADT7310_reset, num_data))
	{
	case HAL_OK: 	  /* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT: /* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:   /* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}
	for(int cntr = 0; cntr < 400; cntr++) {}
}

void VT_Print_Read_ADT7310_address(uint8_t ADT7310_address)
{
	SPI_TXRX_stat = 0;
	ADT7310_TX[0] = ADT7310_read + (ADT7310_address << 3);
	if ((ADT7310_address == Status) || (ADT7310_address == Config) || (ADT7310_address == ID) || (ADT7310_address == THYST_setpt))
	{
		num_data = 2;
	}
	else if ((ADT7310_address == Temp_val) || (ADT7310_address == TCRIT_setpt) || (ADT7310_address == THIGH_setpt) || (ADT7310_address == TLOW_setpt))
	{
		num_data = 3;
	}
	else
		VT_ADT7310_address_Error();

	switch (HAL_SPI_TransmitReceive_DMA(&hspi2, ADT7310_TX, ADT7310_RX, num_data))
	{
	case HAL_OK: 	  /* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT: /* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:   /* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}

	while(!SPI_TXRX_stat){}
	printf("Read  Address = %x | ", ADT7310_address);
	if (num_data == 2)
		printf("ADT7310 data = x%02x \n", ADT7310_RX[1]);
	else
		printf("ADT7310 data = x%04x \n", ((ADT7310_RX[1] << 8) + ADT7310_RX[2]));

	ADT7310_RX[1] = 0;
	ADT7310_RX[2] = 0;
}

void VT_Print_Read_ADT7310_TempVal_16bit(void)
{
	SPI_TXRX_stat = 0;
	ADT7310_TX[0] = ADT7310_read + (Temp_val << 3);

	switch (HAL_SPI_TransmitReceive_DMA(&hspi2, ADT7310_TX, ADT7310_RX, 3))
	{
	case HAL_OK: 	  /* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT: /* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:   /* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}

	while(!SPI_TXRX_stat){}
	printf("Read ADT7310 TempVal 16bit = %.07f degC \n", (((ADT7310_RX[1] << 8) + ADT7310_RX[2]) * 0.0078125));
	ADT7310_RX[1] = 0;
	ADT7310_RX[2] = 0;
}

void VT_Print_Read_ADT7310_TempVal_8bit(void)
{
	SPI_TXRX_stat = 0;
	ADT7310_TX[0] = ADT7310_read + (Temp_val << 3);

	switch (HAL_SPI_TransmitReceive_DMA(&hspi2, ADT7310_TX, ADT7310_RX, 3))
	{
	case HAL_OK: 	  /* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT: /* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:   /* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}

	while(!SPI_TXRX_stat){}
	printf("Read ADT7310 TempVal 8bit = %.04f degC \n", ((((ADT7310_RX[1] << 8) + ADT7310_RX[2]) >> 3) *  0.0625));
	ADT7310_RX[1] = 0;
	ADT7310_RX[2] = 0;
}

void VT_Print_Write_ADT7310(uint8_t ADT7310_address, uint16_t data)
{
	SPI_TXRX_stat = 0;
	ADT7310_TX[0] = ADT7310_write + (ADT7310_address << 3);
	if ((ADT7310_address == Config) || (ADT7310_address == THYST_setpt))
	{
		num_data = 2;
		ADT7310_TX[1] = data;
	}
	else if ((ADT7310_address == TCRIT_setpt) || (ADT7310_address == THIGH_setpt) || (ADT7310_address == TLOW_setpt))
	{
		num_data = 3;
		ADT7310_TX[1] = data >> 8;
		ADT7310_TX[2] = data;
	}
	else
		VT_ADT7310_address_Error();

	switch (HAL_SPI_Transmit_DMA(&hspi2, ADT7310_TX, num_data))
	{
	case HAL_OK: 	  /* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT: /* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:   /* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}

	while(!SPI_TXRX_stat){}
	printf("Write Address = %x | ", ADT7310_address);
	if (num_data == 2)
		printf("ADT7310 data = x%02x \n", ADT7310_TX[1]);
	else
		printf("ADT7310 data = x%04x \n", ((ADT7310_TX[1] << 8) + ADT7310_TX[2]));

}

int VT_Read_ADT7310_address(uint8_t ADT7310_address)
{
	SPI_TXRX_stat = 0;
	int data;
	ADT7310_TX[0] = read + (ADT7310_address << 3);
	if ((ADT7310_address == Status) || (ADT7310_address == Config) || (ADT7310_address == ID) || (ADT7310_address == THYST_setpt))
	{
		num_data = 2;
	}
	else if ((ADT7310_address == Temp_val) || (ADT7310_address == TCRIT_setpt) || (ADT7310_address == THIGH_setpt) || (ADT7310_address == TLOW_setpt))
	{
		num_data = 3;
	}
	else
		VT_ADT7310_address_Error();

	switch (HAL_SPI_TransmitReceive_DMA(&hspi2, ADT7310_TX, ADT7310_RX, num_data))
	{
	case HAL_OK: 	  /* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT: /* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:   /* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}

	while(!SPI_TXRX_stat){}

	if (num_data == 2)
	{
		data = ADT7310_RX[1];
	}
	else // num_data == 3
	{
		data = (ADT7310_RX[2] << 8) + ADT7310_RX[1];
	}

	return data;

}

uint16_t Conv_temp_Float_to_Bin(float temperature)
{
	return (int16_t) (temperature / 0.0078125);
//	printf("Temperature_binary = x%04x \n", Temperature_binary);
//	ADT7310_TX[1] = 0;
//	ADT7310_TX[2] = 0;
//	ADT7310_TX[1] = (uint8_t) (Temperature_binary >> 8);
//	ADT7310_TX[2] = (uint8_t) Temperature_binary;
//	printf("ADT7310_TX[1] = x%02x \n", ADT7310_TX[1]);
//	printf("ADT7310_TX[2] = x%02x \n", ADT7310_TX[2]);
}

void VT_ADT7310_address_Error(void)
{
	printf("ADT7310 address Error, not in the specified address for reading and writing!! \n");
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxCpltCallback should be implemented in the user file
	 */

	//	for(int cntr = 0; cntr < num_data; cntr++)
	//	{
	//		printf("ADT7310_RX [%x] = %x \n", cntr, ADT7310_RX[cntr]);
	//	}

	SPI_TXRX_stat = 1;

}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxCpltCallback should be implemented in the user file
	 */

	//	for(int cntr = 0; cntr < num_data; cntr++)
	//	{
	//		printf("ADT7310_RX [%x] = %x \n", cntr, ADT7310_RX[cntr]);
	//	}

	SPI_TXRX_stat = 1;

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
	printf("Error_Handler \n");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
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
