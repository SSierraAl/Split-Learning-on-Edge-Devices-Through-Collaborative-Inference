/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <prueba.h>

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
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////--ID--//////////////////////

//ID actual node
uint8_t ID_node = 3;

//ID next node
uint8_t ID_next_node = 3;

//////////////////////////////////

//ID First node
uint8_t ID_first_node = 1;

//ID Final node
uint8_t ID_final_node = 3;

//ID Result node
uint8_t ID_result_node=0;


//Check if data is receive
uint8_t activate_data = 1;

//Listen Message Send
uint8_t Listen_msg_s[7];

//Listen Message Receive
uint8_t Listen_msg_r[7];

//Receive List
uint8_t receiveLIST[10];

// Send List
uint8_t sendLIST[10];

//Weight
double sendVAR = 0;

//Receive Var
double receiveVAR;

//Receive Weight split
uint8_t receiveWeight[8];

//Counter
uint8_t counterToggle2=0;

//Communication_Step
uint8_t Com_Step=0; // 0- Send to first node, 1- Send to next node
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_RX);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
typedef enum{
	S0_HOME,

	S1_SEND_DATA_R,
	S1_SEND_DATA_R_Package,

	S2_RECE_DATA_R,
	S2_RECE_DATA_R_Package,
	S2_CHECK_Info,

	S3_END,

} STATES;

STATES NextState;

typedef union
{
    uint8_t split[8];
    double d_value;
} double_or_split;
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


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
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  /////////////////////////////////////////////////////
  // START COMUNICATION TIMER
  HAL_TIM_Base_Start_IT(&htim7);
  /////////////////////////////////////////////////////
  Listen_msg_s[0]=ID_first_node; //Next node
  Listen_msg_s[1]='s';
  Listen_msg_s[2]=ID_result_node; //Final to inference
  Listen_msg_s[3]=0; //Result
  Listen_msg_s[4]=0; //ValueResultA
  Listen_msg_s[5]=0; //ValueResultB
  Listen_msg_s[6]=0xAA;
  activate_data=0;

  double_or_split my_union= {0};

  NextState= S0_HOME;

  counterToggle2=0;

  sendVAR=Sebas();
  /////////////////////////////////////////////////////


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

	switch(NextState){

	/////////////////////////////////// HOME ////////////////////////////////////////////////////////
	case S0_HOME:
		Listen_msg_s[0]=ID_first_node;
		Listen_msg_s[2]=0;
		Listen_msg_s[3]=0;
		Listen_msg_s[4]=0;
		Listen_msg_s[5]=0;


		Listen_msg_r[0]=0;
		Listen_msg_r[1]=0;
		Listen_msg_r[2]=0;
		Listen_msg_r[3]=0;
		Listen_msg_r[4]=0;
		Listen_msg_r[5]=0;
		Listen_msg_r[6]=0;
		Listen_msg_r[7]=0;
		//Define Outputs
		HAL_GPIO_WritePin(Request_OUT_GPIO_Port, Request_OUT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Check_OUT_GPIO_Port, Check_OUT_Pin, GPIO_PIN_RESET);

		//Define Events------------------
		//Receive Request
		if(HAL_GPIO_ReadPin(Request_IN_GPIO_Port, Request_IN_Pin)== GPIO_PIN_SET){
			NextState=S2_RECE_DATA_R;
		}
		//Capture Data Request
		else if(HAL_GPIO_ReadPin(Pulsador_GPIO_Port, Pulsador_Pin)==GPIO_PIN_SET){
			Listen_msg_s[2]=ID_node;
			ID_result_node=ID_node;
			if(ID_node==1)
			{
			Listen_msg_s[0]=ID_next_node;
			}
			NextState=S1_SEND_DATA_R;
		}
		else{
			NextState=S0_HOME;
		}
		break;


	/////////////////////////////////// TRANSMIT REQUEST ////////////////////////////////////////////////
	case S1_SEND_DATA_R:
		//Define Outputs
		HAL_GPIO_WritePin(Request_OUT_GPIO_Port, Request_OUT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Check_OUT_GPIO_Port, Check_OUT_Pin, GPIO_PIN_RESET);


		if(ID_node==ID_final_node && Listen_msg_s[3]==1)
		{
			//acabe
			if(HAL_GPIO_ReadPin(Check_IN_GPIO_Port, Check_IN_Pin)==GPIO_PIN_SET){
				counterToggle2=0;
				NextState=S3_END;
			}

		}
		else{
			//Define Events------------------
			if(HAL_GPIO_ReadPin(Check_IN_GPIO_Port, Check_IN_Pin)==GPIO_PIN_SET){
				NextState=S1_SEND_DATA_R_Package;
			}
			else{
				//HAL_GPIO_WritePin(Transmit_GPIO_Port, Transmit_Pin, GPIO_PIN_SET);
				//HAL_UART_Transmit(&huart1, Listen_msg_s, sizeof(Listen_msg_s),2000);
				NextState=S1_SEND_DATA_R;
			}
		}



		break;


	/////////////////////////////////// TRANSMIT DATA ///////////////////////////////////////////////////
	case S1_SEND_DATA_R_Package:
		//Define Outputs
		HAL_GPIO_WritePin(Request_OUT_GPIO_Port, Request_OUT_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Check_OUT_GPIO_Port, Check_OUT_Pin, GPIO_PIN_RESET);

		//DO:
		sendVAR=1.98052;
		my_union.d_value = sendVAR;
		sendLIST[0]=0xAA;
		sendLIST[1]= my_union.split[0];
		sendLIST[2]= my_union.split[1];
		sendLIST[3]= my_union.split[2];
		sendLIST[4]= my_union.split[3];
		sendLIST[5]= my_union.split[4];
		sendLIST[6]= my_union.split[5];
		sendLIST[7]= my_union.split[6];
		sendLIST[8]= my_union.split[7];
		sendLIST[9]=0xAA;

		//HAL_GPIO_WritePin(Transmit_GPIO_Port, Transmit_Pin, GPIO_PIN_SET);
		//HAL_UART_Transmit(&huart1, sendLIST, sizeof(sendLIST),2000);

		//Define Events------------------
		if(HAL_GPIO_ReadPin(Check_IN_GPIO_Port, Check_IN_Pin)==GPIO_PIN_RESET){
			NextState=S0_HOME;
		}
		else{
			NextState=S1_SEND_DATA_R_Package;
		}
		break;


	/////////////////////////////////// RECEIVE REQUEST////////////////////////////////////////////////////////
	case S2_RECE_DATA_R:
		//Define Outputs
		HAL_GPIO_WritePin(Request_OUT_GPIO_Port, Request_OUT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Check_OUT_GPIO_Port, Check_OUT_Pin, GPIO_PIN_RESET);

		//NextState=S0_HOME;
		//Enable for Receive Info
		if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY)
		{
		if(Listen_msg_r[0]!=ID_node || Listen_msg_r[1]!='s'|| Listen_msg_r[6]!=0xAA){
		  //HAL_GPIO_WritePin(Enable_Msg_Transmit_GPIO_Port, Enable_Msg_Transmit_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Transmit_GPIO_Port, Transmit_Pin, GPIO_PIN_RESET);
		  HAL_UART_Receive_IT(&huart1, Listen_msg_r, sizeof(Listen_msg_r));
			}
		}

		//Check receive request msg
		if(Listen_msg_r[0]==ID_node){
				if(Listen_msg_r[1]=='s'){
					if(Listen_msg_r[2]!=0){
						if(Listen_msg_r[6]==0xAA){

						ID_result_node=Listen_msg_r[2];

						//Receive All Data
						//Is final request with inference result?
						if(Listen_msg_r[3]==1)
						{
							HAL_GPIO_WritePin(Check_OUT_GPIO_Port, Check_OUT_Pin,GPIO_PIN_SET);
							NextState=S3_END;
							counterToggle2=0;
							break;
						}
						//Just one step in Queue
						else{
							Listen_msg_r[0]=0;
							Listen_msg_r[1]=0;
							Listen_msg_r[3]=0;
							Listen_msg_r[6]=0;
							NextState=S2_RECE_DATA_R_Package;
						}
					}}}}


		if(HAL_GPIO_ReadPin(Check_IN_GPIO_Port, Check_IN_Pin)==GPIO_PIN_SET)
		{
			NextState=S0_HOME;
		}
		break;


	/////////////////////////////////// RECEIVE DATA ////////////////////////////////////////////////////////
	case S2_RECE_DATA_R_Package:
	  //Define Outputs
	  HAL_GPIO_WritePin(Request_OUT_GPIO_Port, Request_OUT_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(Check_OUT_GPIO_Port, Check_OUT_Pin, GPIO_PIN_SET);

	  NextState=S2_RECE_DATA_R_Package;

	  //Enable for Receive Info
	  if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY)
	  {
		if(receiveLIST[0]!=0xAA ||receiveLIST[9]!=0xAA ){
		  //HAL_GPIO_WritePin(Enable_Msg_Transmit_GPIO_Port, Enable_Msg_Transmit_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Transmit_GPIO_Port, Transmit_Pin, GPIO_PIN_RESET);
		  HAL_UART_Receive_IT(&huart1, receiveLIST, sizeof(receiveLIST));
			}
	  }

		//Check receive request msg
		if(receiveLIST[0]==0xAA){
				if(receiveLIST[9]==0xAA){

					NextState= S2_CHECK_Info;

					receiveWeight[0]=receiveLIST[1];
					receiveWeight[1]=receiveLIST[2];
					receiveWeight[2]=receiveLIST[3];
					receiveWeight[3]=receiveLIST[4];
					receiveWeight[4]=receiveLIST[5];
					receiveWeight[5]=receiveLIST[6];
					receiveWeight[6]=receiveLIST[7];
					receiveWeight[7]=receiveLIST[8];

					memcpy(&receiveVAR,&receiveWeight,sizeof(receiveWeight));

					break;
				}
		}
		break;

	/////////////////////////////////// RECEIVE DATA ////////////////////////////////////////////////////////
	case S2_CHECK_Info:
		//Define Outputs
		  HAL_GPIO_WritePin(Request_OUT_GPIO_Port, Request_OUT_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(Check_OUT_GPIO_Port, Check_OUT_Pin, GPIO_PIN_RESET);

		  //Last node in NN Queue
		  if(ID_node==ID_final_node)
		  {
			  //This is the inference node?
			  if(ID_node==ID_result_node)
			  {
				  NextState=S3_END;
				  counterToggle2=0;
			  }
			  else{
		  //Send Result to final Node
			  Listen_msg_s[0]=ID_result_node;
			  Listen_msg_s[3]=1;
			  Listen_msg_s[2]=ID_result_node;
			  NextState=S1_SEND_DATA_R;
			  }
		  }
		else{
			Listen_msg_s[0]=ID_next_node;
			Listen_msg_s[2]=ID_result_node;
			NextState=S1_SEND_DATA_R;
		}
		break;

		/////////////////////////////////// END PROCESS ////////////////////////////////////////////////////////
	case S3_END:
		//Define Outputs
		HAL_GPIO_WritePin(Request_OUT_GPIO_Port, Request_OUT_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Check_OUT_GPIO_Port, Check_OUT_Pin, GPIO_PIN_RESET);

		if(counterToggle2<20)
		{
			HAL_GPIO_TogglePin(Led_Board_GPIO_Port, Led_Board_Pin);
			NextState=S3_END;
			counterToggle2=counterToggle2+1;
		}
		else
		{
			NextState=S0_HOME;
		}
		break;

	}

	HAL_Delay(10);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 720;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Check_OUT_Pin|Request_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Led_Board_Pin|Transmit_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Check_OUT_Pin Request_OUT_Pin */
  GPIO_InitStruct.Pin = Check_OUT_Pin|Request_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Check_IN_Pin Request_IN_Pin */
  GPIO_InitStruct.Pin = Check_IN_Pin|Request_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Pulsador_Pin */
  GPIO_InitStruct.Pin = Pulsador_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pulsador_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_Board_Pin Transmit_Pin */
  GPIO_InitStruct.Pin = Led_Board_Pin|Transmit_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if (htim == &htim7 )
	  {
		  if(NextState==S1_SEND_DATA_R)
		  {
			  if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY)
			  {
				  HAL_GPIO_WritePin(Transmit_GPIO_Port, Transmit_Pin, GPIO_PIN_SET);
				  HAL_UART_Transmit_IT(&huart1, Listen_msg_s, sizeof(Listen_msg_s));
			  }
		  }
		  else if(NextState==S1_SEND_DATA_R_Package)
		  {
			  if(HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY)
			  {
				  //HAL_GPIO_WritePin(Enable_Msg_Transmit_GPIO_Port, Enable_Msg_Transmit_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(Transmit_GPIO_Port, Transmit_Pin, GPIO_PIN_SET);
				  HAL_UART_Transmit_IT(&huart1, sendLIST, sizeof(sendLIST));
			  }
		  }
	  }
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //__disable_irq();
  //while (1)
  //{
  //}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
