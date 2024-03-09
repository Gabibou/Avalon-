/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "at25x041b.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t start_of_flash = 0;
uint8_t end_of_flash = 0;
uint8_t end_of_coordinate = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
AT25X041B_t external_flash;
coord_t coordinate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void JumpToApplication(uint32_t application_addr);
uint8_t Rx_Buffer_Processing(uint8_t RX_BUFFER[],coord_t *output);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t RX_BUFFER[100];
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
  MX_USB_Device_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* Init the external flash */
  if(AT25X041B_Init(&hspi1, &external_flash, SPI1_CS_GPIO_Port, SPI1_CS_Pin) != 0x00){
	  Error_Handler();
  }

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /* loop to ensure user is ready to send a data */
  uint8_t Flash_confirmation[] = "Flashing data please wait ...\r\n";
  uint8_t Print_time[] = "Waiting for USB interrupt !\r\n";
  uint8_t Flash_erase[] = "External flash erase please wait ... \r\n";
  uint8_t Jumping_to_app[] = "Jumping to application !\r\n";
  uint8_t Error1[] = "An error occurred while erasing external flash !\r\n";
  uint8_t Write_confirmation[] = "Flash successful\r\n";

  for(int i=0;i<TIME_TO_WAIT_S;i++){
	  if(start_of_flash){
		  CDC_Transmit_FS(Flash_erase, sizeof(Flash_erase));
		  if( AT25X041B_ChipErase(&hspi1, &external_flash) == 0x01){
			  CDC_Transmit_FS(Flash_confirmation, sizeof(Flash_confirmation));
		  }
		  else{
			  CDC_Transmit_FS(Error1, sizeof(Error1));
		  }

		  /* Wait until we received a end of flash sequence */
		  while(end_of_flash != 1){

			  /* Wait until a full coordinate has been send, it must end with \r\n */
			  if(end_of_coordinate == 0x01){
				  /* convert RX buffer into coordinate */
				  Rx_Buffer_Processing(RX_BUFFER,&coordinate);

				  /* Flash the external flash */
				  WriteCoordinateFlash(&hspi1, &external_flash,coordinate);
				  end_of_coordinate = 0;
				  memset(RX_BUFFER, 0, sizeof(RX_BUFFER));
				  CDC_Transmit_FS(Write_confirmation, sizeof(Write_confirmation));
			  }
		  }
		  break;
	  }
	  CDC_Transmit_FS(Print_time, sizeof(Print_time));
	  HAL_Delay(1000);
  }

  CDC_Transmit_FS(Jumping_to_app, sizeof(Jumping_to_app));
  HAL_Delay(1000);
  JumpToApplication(APP_ADDRESS);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_CS_Pin LED_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void JumpToApplication(uint32_t application_addr){

	  /*Init a function pointer*/
	  uint32_t go_address = *((volatile uint32_t*) (application_addr + 4));
	  void (*jump_to_app)(void) = (void *)go_address;

	  /*disable all IRQ*/
	  __disable_irq();

	  /*Uninit all peripheral use by the bootloader*/
	  HAL_GPIO_DeInit(LED_GPIO_Port, LED_Pin);

	  /* Disable SPI*/
	  HAL_SPI_DeInit(&hspi1);

	  /*Disable clock*/
	  HAL_RCC_DeInit();

	  /*Relocate vector table*/
	  SCB->VTOR = application_addr;

	  /*Set main stack pointer pas sure*/
	  __set_MSP(*(volatile uint32_t*) application_addr);

	  /*Reset systick timer*/
	  SysTick->CTRL = 0;
	  SysTick->LOAD = 0;
	  SysTick->VAL = 0;
		//#error "In order to jump correctly the app should be relocated at different @ - please allow more than (2048byte*4)=16384 byte -- probably around 40 or 42 kB "
	  /*Jump*/
	  jump_to_app();
}


/*
 * @brief Convert RX buffer into coordinate - Each coordinate should be send using the following method
 * @method latitude,longitude,altitude\r\n
 * @param RX_BUFFER any size buffer (should end with a \0)
 * @param coord is the output coordinate pointer
 * @output 0 if success 1 else
 */
uint8_t Rx_Buffer_Processing(uint8_t RX_BUFFER[],coord_t *output){

	uint8_t temp_string[30] = {0x0};
	uint8_t temp_char[2] = {0x0};
	uint16_t counter = 0;

	for(int i=0;i<3;i++){

		while((RX_BUFFER[counter] != ',')&&(RX_BUFFER[counter] != '\r')){
			temp_char[0] = RX_BUFFER[counter];
			temp_char[1] = 0;
			strcat(temp_string,temp_char);
			counter++;
		}

		switch (i) {
			case 0:
				output->latitude = atof(temp_string);
				break;
			case 1:
				output->longitude = atof(temp_string);
				break;
			case 2:
				output->altitude = atof(temp_string);
				break;
		}
		counter++;
		memset(temp_string, 0, sizeof(temp_string));
	}
	return 0;
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
