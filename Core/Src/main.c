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
#include "usart.h"
#include "gpio.h"

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
#define RX_BUFFER_SIZE 100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;
// Define CAN Rx message structure
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8]; // Buffer to store received data
//uint8_t TxData[8]; // Data to be transmitted

uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile int32_t rxBufferIndex = 0;
volatile int32_t rxCurrentMaxIndex = 0;

volatile uint8_t rxData = 0;
volatile uint8_t rxComplete = 0;
TaskHandle_t xTaskHandle1 = NULL;

typedef struct
{
	uint8_t Data[8];
	uint32_t Length;
}PduInfoType;


typedef enum
{
	E_OK=0,
	E_NOK
}Std_ReturnType;


typedef enum
{
	Single_Frame,
	First_Frame,
	Consecutive_Frame,
	FlowControl_Frame,
	None
}Frame_Type;

typedef enum
{
	Any_State,
	Consecutive_Frame_State,
	FlowControl_Frame_State
}Frame_States;

Frame_States expectedFrameState = Any_State;

uint32_t numberOfConsecutiveFramesToSend = 0;
uint32_t numberOfConsecutiveFramesToReceive = 0;
uint32_t numberOfRemainingBytesToSend = 0;
uint32_t numberOfRemainingBytesToReceive = 0;
uint32_t availableBuffers = 100;
void (*App_Callback)(uint32_t RxPduId, PduInfoType* PduInfoPtr) = NULL;

PduInfoType EncodedPduInfo;
PduInfoType DecodedPduInfo;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void CAN_TX();
void CANTaskFunction(void *pvParameters);
void CanTp_Init();
Std_ReturnType CanTp_Transmit(uint32_t TxPduId, PduInfoType* PduInfoPtr);
Std_ReturnType CanTp_RxIndication (uint32_t RxPduId, PduInfoType* PduInfoPtr);
Frame_Type CanTp_GetFrameType(uint8_t PCI);
void CanTp_setCallback(void (*PTF)(uint32_t TxPduId, PduInfoType* PduInfoPtr));
void CanTp_encodeSingleFrame(uint32_t TxPduId,PduInfoType* PduInfoPtr);
void CanTp_encodeFirstFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr);
void CanTp_encodeConsecutiveFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr);
void CanTp_encodeFlowControlFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr);
void CanTp_decodeSingleFrame(PduInfoType* PduInfoPtr);
void CanTp_decodeFirstFrame(PduInfoType* PduInfoPtr);
void CanTp_decodeConsecutiveFrame(PduInfoType* PduInfoPtr);
void CanTp_decodeFlowControlFrame(PduInfoType* PduInfoPtr);
void CanIf_Transmit(uint32_t RxPduId, PduInfoType* PduInfoPtr);
//use this setcallback in the init so that the canIf calls our  CanTp_RxIndication
void CanIf_setCallback(void (*IF_Callback)(uint32_t RxPduId, PduInfoType* PduInfoPtr));


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
	MX_CAN1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_UART_Receive_IT(&huart2,&rxData, 1);

	xTaskCreate(CANTaskFunction, "CAN_TX", configMINIMAL_STACK_SIZE,NULL, 2, &xTaskHandle1) ;
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(!rxComplete){
		if (rxBufferIndex <= RX_BUFFER_SIZE - 1) {
			rxBuffer[rxBufferIndex++] = rxData;
		}
		else{

			// Buffer overflow handling
			//			rxBufferIndex = 0;
		}
		HAL_UART_Receive_IT(&huart2, &rxData, 1); // Start next reception
		//	HAL_UART_Transmit(&huart2, &rxData, 1, HAL_MAX_DELAY);
		if (rxData == 0x0D) { // Example: End of line delimiter
			rxComplete = 1;
			//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}
	}
}

void CAN_TX(){
	// Define CAN Tx message structure
	CAN_TxHeaderTypeDef TxHeader;
	// Define CAN handle


	// Configure CAN Tx Header
	TxHeader.StdId = 0x1; // Standard CAN ID
	TxHeader.ExtId = 0; // No extended ID used
	TxHeader.RTR = CAN_RTR_DATA; // Data frame
	TxHeader.IDE = CAN_ID_STD; // Standard ID
	TxHeader.DLC = 8; // Data length
	// Prepare data to be sent
	//	for(int i = 0; i < 8; i++) {
	//		TxData[i] = '0' + i; // Example data
	//	}
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
	// Add CAN Tx message
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &rxBuffer[rxCurrentMaxIndex - rxBufferIndex], (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) {
		// Error handling

		//		while(HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0));
		//		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	}


}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
		// Process received message
		//		for(uint8_t i = 0; i < 8; i++){
		//			if(RxData[i] != '0' + i){
		//				return;
		//			}
		//		}
		HAL_UART_Transmit(&huart2, RxData, 8, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		// RxHeader contains received message header
		// RxData contains received message data
	}
}


void CANTaskFunction(void *pvParameters) {
	// Task code goes here
	while(1){
		if(rxComplete){
			rxCurrentMaxIndex = rxBufferIndex;
			while(rxBufferIndex > 0){
				if(rxBufferIndex < 8){
					for(int8_t i = rxBufferIndex; i < 8; i++){
						rxBuffer[rxCurrentMaxIndex - rxBufferIndex + i] = 0;
					}
				}
				CAN_TX();
				rxBufferIndex -= 8;
				vTaskDelay(10);
			}
			rxBufferIndex = 0;
			rxComplete = 0;
		}
		vTaskDelay(10);
	}
}

void CanTp_Init(){


}

Std_ReturnType CanTp_Transmit(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	//Stop the program if the PduID doesn't equal 0 :)
	if(TxPduId != 0){
		while(1);
	}

	Frame_Type frame_type;
	numberOfRemainingBytesToSend = PduInfoPtr->Length;
	if(expectedFrameState == Any_State){
		if(PduInfoPtr->Length < 8){
			frame_type = Single_Frame;
		}
		else{
			frame_type = First_Frame;
		}
	}
	while(numberOfRemainingBytesToSend > 0){

		if(expectedFrameState == Consecutive_Frame_State){
			frame_type = Consecutive_Frame;
		}
		else if(expectedFrameState == FlowControl_Frame_State){
			frame_type = FlowControl_Frame;
		}

		//Call the right encoder function according to the frame type
		//Make sure to adjust the numberOfRemainingBytesToSend variable to know if all the data has been sent
		//Also make sure to call the CanIf_Transmit method at the end of these functions.
		switch(frame_type){
		case Single_Frame:
			CanTp_encodeSingleFrame(TxPduId, PduInfoPtr);
			break;
		case First_Frame:
			CanTp_encodeFirstFrame(TxPduId, PduInfoPtr);
			expectedFrameState = Consecutive_Frame;
			break;
		case Consecutive_Frame:
			if(numberOfConsecutiveFramesToSend > 0){
				CanTp_encodeConsecutiveFrame(TxPduId, PduInfoPtr);
			}
			else{
				//wait for flow control to reach CanTp_RxIndication in order to change numberOfConsecutiveFramesToSend variable
			}
			break;
		case FlowControl_Frame:
			//Check the availableBuffers variable (in our case it's the size of the receive array)
			CanTp_encodeFlowControlFrame(TxPduId, PduInfoPtr);
			break;
		default:
			break;
		}
	}
	//Reset the expected frame
	expectedFrameState = Any_State;
	return E_OK;
}

Std_ReturnType CanTp_RxIndication (uint32_t RxPduId, PduInfoType* PduInfoPtr){
	//Stop the program if the PduID doesn't equal 0 :)
	if(RxPduId != 0){
		while(1);
	}
	//Get the frame type from the
	Frame_Type frame_type = CanTp_GetFrameType(PduInfoPtr->Data[0]);

	//Call the correct decoder depending on the frame type
	//extract the length and save it in numberOfRemainingBytesToReceive and connect the data
	switch(frame_type){
	case Single_Frame:
		CanTp_decodeSingleFrame(PduInfoPtr);
		break;
	case First_Frame:
		CanTp_decodeFirstFrame(PduInfoPtr);
		expectedFrameState = FlowControl_Frame_State;
		CanTp_Transmit(RxPduId, PduInfoPtr);
		break;
	case Consecutive_Frame:
		CanTp_decodeConsecutiveFrame(PduInfoPtr);
		if(numberOfConsecutiveFramesToReceive == 0){
			expectedFrameState = FlowControl_Frame_State;
		}
		CanTp_Transmit(RxPduId, PduInfoPtr);
		break;
	case FlowControl_Frame:
		//adjust the numberOfConsecutiveFramesToSend variable inside a function
		//based on the number of empty buffers available in the other node
		//as indicated in the BS (block size) byte of the flow control frame
		expectedFrameState = Consecutive_Frame_State;
		break;
	default:
		break;
	}
	if(numberOfRemainingBytesToReceive == 0){
		if(App_Callback != NULL){
			App_Callback(RxPduId, &DecodedPduInfo);
		}
	}
	return E_OK;
}

Frame_Type CanTp_GetFrameType(uint8_t PCI){
	//Switch case on the PCI to determine the frame type

	return 0;
}

void CanTp_setCallback(void (*PTF)(uint32_t TxPduId, PduInfoType* PduInfoPtr)){

}


void CanIf_Transmit(uint32_t RxPduId, PduInfoType* PduInfoPtr){

}

void CanIf_setCallback(void (*IF_Callback)(uint32_t RxPduId, PduInfoType* PduInfoPtr)){

}

void CanTp_encodeSingleFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}
void CanTp_encodeFirstFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}
void CanTp_encodeConsecutiveFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}
void CanTp_encodeFlowControlFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}

void CanTp_decodeSingleFrame(PduInfoType* PduInfoPtr){

}
void CanTp_decodeFirstFrame(PduInfoType* PduInfoPtr){

}
void CanTp_decodeConsecutiveFrame(PduInfoType* PduInfoPtr){

}
void CanTp_decodeFlowControlFrame(PduInfoType* PduInfoPtr){

}


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
