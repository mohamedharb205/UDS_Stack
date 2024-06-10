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
#include "cmsis_os.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "APP_UDS_Diag.h"
#include "string.h"
#include "CanTp.h"
#include "CanIf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CAN_TX 1
#define CAN_RX 0
#define CAN_MODE CAN_TX

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * TODO:
 * NRC Handling
 * Use flags in server callback
 * Implement said flags in UDS_MainFunction
 *
 *
 *
 *
 *
 *
 *
 *
 */

#if CAN_MODE == CAN_TX
/**********App *******************/

//For Menus
ClientMenu Menu = Main_Menu;
uint8_t Menu_Letter = {0};

//Array of Menu Massages
const uint8_t* Menu_Msg_Arr[] =
{(uint8_t*)"\r\nHello, Main Menu.\r\n",
		(uint8_t*) "\r\nplease Choose Your Service.\r\n",
		(uint8_t*) "\r\nA --> Control Session.\r\n",
		(uint8_t*) "\r\nB --> Read Data.\r\n",
		(uint8_t*) "\r\nC --> Write Data.\r\n",
		(uint8_t*) "\r\nD --> Security Access.\r\n",
		(uint8_t*) "\r\nE --> Tester Representer.\r\n",
		(uint8_t*) "\r\nChoose Your Session.\r\n",
		(uint8_t*) "\r\nF --> Default Session.\r\n",
		(uint8_t*) "\r\nG --> Extended Session.\r\n",
		(uint8_t*) "\r\nChoose Your Data.\r\n",
		(uint8_t*) "\r\nH --> Read Oil Temperature.\r\n",
		(uint8_t*) "\r\nI --> Read Oil Pressure.\r\n",
		(uint8_t*) "\r\nChoose Your Option.\r\n",
		(uint8_t*) "\r\nJ --> Seed.\r\n",
		(uint8_t*) "\r\nK --> Key.\r\n",
		(uint8_t*) "\r\nM --> Return to Main Menue.\r\n",
		(uint8_t*) "\r\n=========================================.\r\n",
		(uint8_t*) "\r\nO --> Write Oil Temperature.\r\n",
		(uint8_t*) "\r\nP --> Write Oil Pressure.\r\n"
};
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;


TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;
TaskHandle_t xTaskHandle3 = NULL;


#if CAN_MODE == CAN_RX


extern TIM_HandleTypeDef htim6;
volatile uint8_t count;
//extern volatile uint8_t global_sec_flag;
extern volatile uint8_t global_session;
#else
extern uint8_t* seed;
extern uint8_t* key;

#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */


#if CAN_MODE == CAN_TX

void Display_Menu(void);

#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if CAN_MODE == CAN_TX
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	//A --> Control Session.
	//B --> Read Data.
	//C --> Write Data.
	//D --> Security Access.
	//E --> Tester Representer.

	//F --> Default Session.
	//G --> Extended Session.

	//H --> Read Oil Temperature.
	//I --> Read Oil Pressure.

	//J --> Seed.
	//K --> Key.

	//M --> Return to Main Menue

	//O --> Write Oil Temperature.
	//P --> Write Oil Pressure.

	//Display Choosen Letter
	HAL_UART_Transmit(&huart2 ,&Menu_Letter , 1, HAL_MAX_DELAY);

	//Switch on User Menus
	switch(Menu_Letter)
	{
	case 'A':
		Menu = Control_Session_Menu;
		break;

	case 'B':
		Menu = Read_Data_Menu;
		break;

	case 'C':
		Menu = Write_Data_Menu;
		break;

	case 'D':
		Menu = Security_Access_Menu;
		break;

	case 'E':
		//Tester Presenter Function
		UDS_Tester_Presenter_Client();
		break;

	case 'F':
		//PduInfoType Glgl;
		//Glgl.Data[0] = 0x7F;
		//Glgl.Data[1] = 0xF1;
		//Glgl.Data[2] = 0x3D;
		//Glgl.Length = 3;
		//UDS_Client_Callback(&Glgl);
		UDS_Control_Session_Default();
		break;

	case 'G':
		UDS_Control_Session_Extended();
		break;


	case 'H':
		DID Read_Oil_Temp = Oil_Temp;
		UDS_Read_Data_Client(Read_Oil_Temp);
		break;


	case 'I':
		DID Read_Oil_Pressure = Oil_Pressure;
		UDS_Read_Data_Client(Read_Oil_Pressure);

		break;


	case 'O':
		DID Write_Oil_Temp = Oil_Temp;
		uint32_t Write_Oil_Temp_data = 0x5432;
		UDS_Write_Data_Client(Write_Oil_Temp, Write_Oil_Temp_data);
		break;


	case 'P':
		DID Write_Oil_Pressure = Oil_Pressure;
		uint32_t Write_Oil_Pressure_data = 0x54321044;
		UDS_Write_Data_Client(Write_Oil_Pressure, Write_Oil_Pressure_data);
		break;


	case 'J':
		Sub_Fun sub_fun_seed = Seed;
		UDS_Send_Security_Client(sub_fun_seed);
		break;


	case 'K':
		Sub_Fun sub_fun_key = Key;
		// A Temporary example for a seed (Accessed in CallBack)
		uint8_t data[Seed_Key_Lenght] = {0x42, 0x31, 0x00, 0xD0};
		seed = data;
		UDS_Send_Security_Client(sub_fun_key);
		break;


	case 'M':
		Menu = Main_Menu;
		break;


	default:
		Menu = Main_Menu;
	}

	Display_Menu();

	//Recieve Another Letter
	HAL_UART_Receive_IT(&huart2, &Menu_Letter, 1);
}


#endif

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
	MX_CAN1_Init();
	MX_USART2_UART_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
#if CAN_MODE == CAN_TX
	Display_Menu();
	HAL_UART_Receive_IT(&huart2, &Menu_Letter, 1);
	CanTp_setCallback(UDS_Client_Callback);
	xTaskCreate(UDS_MainFunction, "UDS_RX", configMINIMAL_STACK_SIZE,NULL, 2, &xTaskHandle3) ;
#else
	CanTp_setCallback(server_call_back);
	//	reset_timer();
	//	HAL_TIM_Base_Start_IT(&htim6);
	//	stop_timer();
#endif
	//HAL_UART_Receive_IT(&huart2,(uint8_t*)&rxData, 1);


	xTaskCreate(CanIf_Receive, "CANIf_RX", configMINIMAL_STACK_SIZE,NULL, 2, &xTaskHandle1) ;
	xTaskCreate(CanTp_MainFunction, "CANTp_RX", configMINIMAL_STACK_SIZE,NULL, 3, &xTaskHandle2) ;
	CanTp_Init();

	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
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




#if CAN_MODE == CAN_TX

void Display_Menu(void)
{

	switch(Menu)
	{
	case Main_Menu:

		//Default Session Menu
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[0], strlen((char*)Menu_Msg_Arr[0]), HAL_MAX_DELAY);    //{"\r\nHello, Main Menu.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[1], strlen((char*)Menu_Msg_Arr[1]), HAL_MAX_DELAY);    //"\r\n please Choose Your Service.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[2], strlen((char*)Menu_Msg_Arr[2]), HAL_MAX_DELAY);    //"\r\nA --> Control Session.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[3], strlen((char*)Menu_Msg_Arr[3]), HAL_MAX_DELAY);    //"\r\nB --> Read Data.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[4], strlen((char*)Menu_Msg_Arr[4]), HAL_MAX_DELAY);    //"\r\nC --> Write Data.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[5], strlen((char*)Menu_Msg_Arr[5]), HAL_MAX_DELAY);    //"\r\nD --> Security Access.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[6], strlen((char*)Menu_Msg_Arr[6]), HAL_MAX_DELAY);    //"\r\nE --> Tester Representer.\r\n"};
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[17], strlen((char*)Menu_Msg_Arr[17]), HAL_MAX_DELAY);    //"\r\n=========================================.\r\n"};
		break;

	case Control_Session_Menu:
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[7], strlen((char*)Menu_Msg_Arr[7]), HAL_MAX_DELAY);    //"\r\nChoose Your Session.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[8], strlen((char*)Menu_Msg_Arr[8]), HAL_MAX_DELAY);    //"\r\nF --> Default Session.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[9], strlen((char*)Menu_Msg_Arr[9]), HAL_MAX_DELAY);    //"\r\nG --> Extended Session.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[16], strlen((char*)Menu_Msg_Arr[16]), HAL_MAX_DELAY);    //"\r\nM --> Return to Main Menue.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[17], strlen((char*)Menu_Msg_Arr[17]), HAL_MAX_DELAY);    //"\r\n=========================================.\r\n"};
		break;


	case Read_Data_Menu:
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[10], strlen((char*)Menu_Msg_Arr[10]), HAL_MAX_DELAY);    //"\r\nChoose Your Data.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[11], strlen((char*)Menu_Msg_Arr[11]), HAL_MAX_DELAY);    //"\r\nH --> Read Oil Temperature.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[12], strlen((char*)Menu_Msg_Arr[12]), HAL_MAX_DELAY);    //"\r\nI --> Read Oil Pressure.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[16], strlen((char*)Menu_Msg_Arr[16]), HAL_MAX_DELAY);    //"\r\nM --> Return to Main Menue.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[17], strlen((char*)Menu_Msg_Arr[17]), HAL_MAX_DELAY);    //"\r\n=========================================.\r\n"};
		break;


	case Security_Access_Menu:
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[13], strlen((char*)Menu_Msg_Arr[13]), HAL_MAX_DELAY);    //"\r\nChoose Your Option.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[14], strlen((char*)Menu_Msg_Arr[14]), HAL_MAX_DELAY);    //"\r\nJ --> Seed.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[15], strlen((char*)Menu_Msg_Arr[15]), HAL_MAX_DELAY);    //"\r\nK --> Key.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[16], strlen((char*)Menu_Msg_Arr[16]), HAL_MAX_DELAY);    //"\r\nM --> Return to Main Menue.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[17], strlen((char*)Menu_Msg_Arr[17]), HAL_MAX_DELAY);    //"\r\n=========================================.\r\n"};
		break;

	case Write_Data_Menu:
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[10], strlen((char*)Menu_Msg_Arr[10]), HAL_MAX_DELAY);    //"\r\nChoose Your Data.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[18], strlen((char*)Menu_Msg_Arr[18]), HAL_MAX_DELAY);    //"\r\nO --> Write Oil Temperature.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[19], strlen((char*)Menu_Msg_Arr[19]), HAL_MAX_DELAY);    //"\r\nP --> Write Oil Pressure.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[16], strlen((char*)Menu_Msg_Arr[16]), HAL_MAX_DELAY);    //"\r\nM --> Return to Main Menue.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[17], strlen((char*)Menu_Msg_Arr[17]), HAL_MAX_DELAY);    //"\r\n=========================================.\r\n"};
		break;

	default:
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[0], strlen((char*)Menu_Msg_Arr[0]), HAL_MAX_DELAY);    //{"\r\nHello, Main Menu.\r\n"
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[1], strlen((char*)Menu_Msg_Arr[1]), HAL_MAX_DELAY);    //"\r\n please Choose Your Service.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[2], strlen((char*)Menu_Msg_Arr[2]), HAL_MAX_DELAY);    //"\r\nA --> Control Session.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[3], strlen((char*)Menu_Msg_Arr[3]), HAL_MAX_DELAY);    //"\r\nB --> Read Data.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[4], strlen((char*)Menu_Msg_Arr[4]), HAL_MAX_DELAY);    //"\r\nC --> Write Data.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[5], strlen((char*)Menu_Msg_Arr[5]), HAL_MAX_DELAY);    //"\r\nD --> Security Access.\r\n",
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[6], strlen((char*)Menu_Msg_Arr[6]), HAL_MAX_DELAY);    //"\r\nE --> Tester Representer.\r\n"};
		HAL_UART_Transmit(&huart2, Menu_Msg_Arr[17], strlen((char*)Menu_Msg_Arr[17]), HAL_MAX_DELAY);    //"\r\n=========================================.\r\n"};
		break;

		break;
		//Nothing
	}

}
#else


#endif
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */
#if CAN_MODE == CAN_RX
	else if(htim->Instance == TIM6){
		if(count == 0){
			count++;
			return;
		}

		//		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
		if (global_session != DefaultSession)
		{
			global_session = DefaultSession;
			//			global_sec_flag = Un_Secure;
		}
		else
		{
			// do nothing
		}
		//		stop_timer();
		reset_timer();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
#endif
	/* USER CODE END Callback 1 */
}

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
