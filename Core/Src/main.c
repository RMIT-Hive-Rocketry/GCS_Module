/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
uint8_t pointerdata[LORA_MSG_LENGTH];

uint8_t Module_state = 0;

	static SX1272_t lora;
	static SPI_t spi_lora;
	static GPIOpin_t DIO_0;
	static GPIOpin_t LORA_CS;
	static SX1272_Packet LORA_DATA_PACKET_3;
	static SX1272_Packet LORA_DATA_PACKET_4;
	static SX1272_Packet GSE_DATA_PACKET_1;
	static SX1272_Packet GSE_DATA_PACKET_2;
	static LORA_Error lora_error_state;


	static GPIOpin_t LED1;
	static GPIOpin_t LED2;

	//**specific configurations for various pins and protocols**********************************
	SPI_Config spi_config = {
			SPI_CPHA_FIRST,
			SPI_CPOL0,
			SPI_MASTER,
			SPI_BR_PCLK8,
			SPI_DFF8,
			SPI_MSB_FIRST,
			};

	GPIO_Config LoRa_CS_config = {
			GPIO_OUTPUT_MODE,
			GPIO_TYPE_PUSHPULL,
			GPIO_VERYHIGH_SPEED,
			GPIO_PUPD_NONE, //recheck timing diagram for ISA
			GPIO_AF0 //no alternative function
	};


	GPIO_Config LoRA_DIO0_config = {
			GPIO_INPUT_MODE,
			GPIO_TYPE_PUSHPULL,
			GPIO_HIGH_SPEED,
			GPIO_PUPD_NONE, //recheck timing diagram for ISA
			GPIO_AF0 //no alternative function
	};

	GPIO_Config LED_config = {
			GPIO_OUTPUT_MODE,
			GPIO_TYPE_PUSHPULL,
			GPIO_MEDIUM_SPEED,
			GPIO_PUPD_NONE, //recheck timing diagram for ISA
			GPIO_AF0 //no alternative function
	};
	//**************************************************

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  RCC_START_PERIPHERAL(AHB1, GPIOA);
  RCC_START_PERIPHERAL(AHB1, GPIOB);
  RCC_START_PERIPHERAL(AHB1, GPIOC);
  RCC_START_PERIPHERAL(AHB1, GPIOD);
  RCC_START_PERIPHERAL(AHB1, GPIOE);
  RCC_START_PERIPHERAL(AHB1, GPIOF);
  RCC_START_PERIPHERAL(AHB1, GPIOG);
  RCC_START_PERIPHERAL(AHB1, GPIOH);
  RCC_START_PERIPHERAL(AHB1, GPIOI);
  RCC_START_PERIPHERAL(AHB1, GPIOJ);
  RCC_START_PERIPHERAL(AHB1, GPIOK);
  RCC_START_PERIPHERAL(AHB1, DMA1);

  RCC_START_PERIPHERAL(APB2, SPI1);
  RCC_START_PERIPHERAL(APB2, SYSCFG);
  RCC_START_PERIPHERAL(APB2, TIM1);
  RCC_START_PERIPHERAL(APB2, TIM1);


  RCC_START_PERIPHERAL(APB1, TIM4);
  RCC_START_PERIPHERAL(APB1, USART3); //UART 4 also (same pins)

  Delay_ms(100);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
   LED1 = GPIOpin_init(GPIOJ, 0, &LED_config);
   LED2 = GPIOpin_init(GPIOJ, 1, &LED_config);


  //LORA Configuration!*****************************************************************
  LORA_CS = GPIOpin_init(GPIOC, 5, &LoRa_CS_config); //PC5
  DIO_0 = GPIOpin_init(GPIOA, 3, &LoRA_DIO0_config); //PA3
 spi_lora = SPI_init(SPI1, &spi_config); //SPI1 config for LoRa
  SX1272_init(&lora, &spi_lora,LORA_CS, SX1272_BW500, SX1272_SF9, SX1272_CR5);
  SX1272_startReceive(&lora); //default state will be to receive -> interr



	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3_PA;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA;
	EXTI->FTSR &= ~EXTI_FTSR_TR3;
	EXTI->FTSR |= EXTI_RTSR_TR3;
	EXTI->RTSR &= ~EXTI_RTSR_TR3;
	EXTI->RTSR |= EXTI_FTSR_TR3;
	EXTI->IMR &= ~EXTI_IMR_MR3;
	EXTI->IMR |= EXTI_IMR_MR3;
	//begin IRQ handler separately!
// **************************************************************************************


	//ISA Bus Initialisation**************************************************************







	//************************************************************************************

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //have a switch case that is defined as either one of three states
	  	  /*1) Lora communication (LED1) can be on
	  	   * 1.1) LoRa state means Module receive RX packet, and then immediately transmits USART TX
	  	   *2) USART communication (LED2) can be on
	  	   *2.1) USART communication means it receives from the PI, from which
	  	   *3) IDLE -> waiting for either state 1 of 2
	  	   */


	  /*go into STATE 1 via DIO0 interrupt flag via EXTI3
	   *go into STATE 2 via RXNE interrupt flag via USART interrupt event handler
	   *
	   */

	  switch(Module_state)
	  {
	  case 1:
		  //LORA state

	  case 2:

		  //USART state
	  default:
		  //idle state -> do nothing really (or something with time)!

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI3_IRQHandler(void)
{
	EXTI->PR &= ~(0x01<<3); //resets interrupt flag
	LED1.port->ODR |= 0x01; //->PJ0/no bit shifting required!
	__NVIC_DisableIRQ(EXTI3_IRQn); //prevents race condition

	bool RX_result = SX1272_readReceive(&lora, pointerdata, LORA_MSG_LENGTH);
	uint8_t packet_id = pointerdata[0];
	uint8_t idx = 1;
	if(RX_result)
	{
	//have USART comms send packets to PI in each switch case!
		lora_error_state.LoRa_receive_failed = false;
		switch(packet_id) //this is packet IDs!
		{
			case 3: //NORMAL AV DATA
				memcpy(&LORA_DATA_PACKET_3.data, pointerdata[idx +=LORA_MSG_PAYLOAD_LENGTH], LORA_MSG_PAYLOAD_LENGTH);
				lora_error_state.ID_not_valid = false;

				break;
			case 4: //GPS DATA
				memcpy(&LORA_DATA_PACKET_4.data, pointerdata[idx +=LORA_MSG_PAYLOAD_LENGTH], LORA_MSG_PAYLOAD_LENGTH);
				lora_error_state.ID_not_valid = false;

				break;
			case 6: //GSE DATA 1
				memcpy(&GSE_DATA_PACKET_1.data, pointerdata[idx +=LORA_MSG_PAYLOAD_LENGTH], LORA_MSG_PAYLOAD_LENGTH);
				lora_error_state.ID_not_valid = false;

				break;
			case 7: //GSE DATA 2
				memcpy(&GSE_DATA_PACKET_2.data, pointerdata[idx +=LORA_MSG_PAYLOAD_LENGTH], LORA_MSG_PAYLOAD_LENGTH);
				lora_error_state.ID_not_valid = false;

				break;

			default:
				LED1.port->ODR &= ~0x01; //->PJ0/no bit shifting required!
				Delay_ms(20);
				LED1.port->ODR |= 0x01; //->PJ0/no bit shifting required!
				Delay_ms(15);
				LED1.port->ODR &= ~0x01; //->PJ0/no bit shifting required!
				Delay_ms(20);
				LED1.port->ODR |= 0x01; //->PJ0/no bit shifting required!

			lora_error_state.ID_not_valid = true;
			break;
		}
	}
	else
	{
		lora_error_state.LoRa_receive_failed = true;
	}
	LED1.port->ODR &= ~0x01; //->PJ0/no bit shifting required!
	__NVIC_EnableIRQ(EXTI3_IRQn); //disables current interrupt!
}

//EXTI15_10 a higher priority than EXTI3 ->
void EXTI15_10_IRQHandler(void)
{
	_
	//this interrupt handler is designed for the PC11 when RXNE is set!
	EXTI->PR &= ~(0x01<<10); //resets interrupt flag
	__NVIC_DisableIRQ(EXTI3_IRQn); //prevents race condition
	Module_state = 2;

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
