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
#define CAN_FRAME_LENGTH 8
#define CAN_MAX_PAYLOAD_LENGTH 7
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
	int8_t Data[4096];
	uint32_t Length;
}PduInfoType;

typedef struct
{
	uint8_t Data[8];
	uint32_t Length;
}PduInfoTRx;

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

volatile uint32_t numberOfConsecutiveFramesToSend = 0;
volatile uint32_t numberOfConsecutiveFramesToReceive = 0;
volatile uint32_t numberOfRemainingBytesToSend = 0;
volatile uint32_t numberOfRemainingBytesToReceive = 0;
volatile uint32_t availableBuffers = 10;
void (*App_Callback)(uint32_t RxPduId, PduInfoType* PduInfoPtr) = NULL;

PduInfoTRx EncodedPduInfo;
PduInfoTRx DecodedPduInfo;
PduInfoType CompletePduInfo;

volatile uint8_t ConsecSN;
volatile uint16_t currentIndex;
volatile uint8_t SNFlag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void CAN_TX();
void CANTaskFunction(void *pvParameters);
void CanTp_Init();
Std_ReturnType CanTp_Transmit(uint32_t TxPduId, PduInfoType* PduInfoPtr);
Std_ReturnType CanTp_RxIndication (uint32_t RxPduId, PduInfoTRx* PduInfoPtr);

Frame_Type CanTp_GetFrameType(uint8_t PCI);
void CanTp_setCallback(void (*PTF)(uint32_t TxPduId, PduInfoType* PduInfoPtr));
void CanTp_encodeSingleFrame(uint32_t TxPduId,PduInfoType* PduInfoPtr);
void CanTp_encodeFirstFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr);
void CanTp_encodeConsecutiveFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr);
void CanTp_encodeFlowControlFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr);
void CanTp_decodeSingleFrame(uint32_t RxPduId, PduInfoTRx* PduInfoPtr);
void CanTp_decodeFirstFrame(uint32_t RxPduId, PduInfoTRx* PduInfoPtr);
void CanTp_decodeConsecutiveFrame(uint32_t RxPduId, PduInfoTRx* PduInfoPtr);
void CanTp_decodeFlowControlFrame(uint32_t RxPduId, PduInfoTRx* PduInfoPtr);
void CanTp_ConnectData(PduInfoTRx* PduInfoPtr);
void CanIf_Transmit(uint32_t RxPduId, PduInfoTRx* PduInfoPtr);
//use this setcallback in the init so that the canIf calls our  CanTp_RxIndication
void CanIf_setCallback(Std_ReturnType (*IF_Callback)(uint32_t RxPduId, PduInfoTRx* PduInfoPtr));


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
	HAL_UART_Receive_IT(&huart2,(uint8_t*)&rxData, 1);

	xTaskCreate(CANTaskFunction, "CAN_TX", configMINIMAL_STACK_SIZE,NULL, 2, &xTaskHandle1) ;
	//	PduInfoType PduInfoPtr ={.Data={0x10,0x09,0x01,0x02,0x03,0x04,0x05,0x06},.Length=0};
	//	CanTp_RxIndication(0, &PduInfoPtr);
	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();
	/* Start scheduler */
	//	osKernelStart();

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
		HAL_UART_Receive_IT(&huart2, (uint8_t*) &rxData, 1); // Start next reception
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
	CanIf_setCallback(CanTp_RxIndication);
}

Std_ReturnType CanTp_Transmit(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	//Stop the program if the PduID doesn't equal 0 :)
	if(TxPduId != 0){
		while(1);
	}

	Frame_Type frame_type;
	numberOfRemainingBytesToSend = PduInfoPtr->Length;
	CompletePduInfo.Length = numberOfRemainingBytesToSend;
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
		else
		{

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
			frame_type = None;
			break;
		case Consecutive_Frame:
			if(numberOfConsecutiveFramesToSend > 0){
				numberOfConsecutiveFramesToSend--;
				CanTp_encodeConsecutiveFrame(TxPduId, PduInfoPtr);
			}
			else{
				frame_type = None;
				SNFlag = 1;
				//wait for flow control to reach CanTp_RxIndication in order to change numberOfConsecutiveFramesToSend variable
			}
			break;
		case FlowControl_Frame:
			//Check the availableBuffers variable (in our case it's the size of the receive array)
			CanTp_encodeFlowControlFrame(TxPduId, PduInfoPtr);
			return E_OK;
		default:
			break;
		}
		HAL_Delay(10);
	}

	//Reset the expected frame
	expectedFrameState = Any_State;
	return E_OK;
}

Std_ReturnType CanTp_RxIndication (uint32_t RxPduId, PduInfoTRx* PduInfoPtr){
	//Stop the program if the PduID doesn't equal 0 :)
	if(RxPduId != 0){
		while(1);
	}
	//Get the frame type from the
	Frame_Type frame_type = CanTp_GetFrameType(PduInfoPtr->Data[0]);
	//	Frame_Type frame_type = First_Frame

	//Call the correct decoder depending on the frame type
	//extract the length and save it in numberOfRemainingBytesToReceive and connect the data
	switch(frame_type){
	case Single_Frame:
		CanTp_decodeSingleFrame(RxPduId, PduInfoPtr);
		break;
	case First_Frame:
		CanTp_decodeFirstFrame(RxPduId, PduInfoPtr);
		expectedFrameState = FlowControl_Frame_State;
		CanTp_Transmit(RxPduId, (PduInfoType*) PduInfoPtr);
		break;
	case Consecutive_Frame:
		CanTp_decodeConsecutiveFrame(RxPduId, PduInfoPtr);
		if(numberOfConsecutiveFramesToReceive == 0){
			expectedFrameState = FlowControl_Frame_State;
			CanTp_Transmit(RxPduId, (PduInfoType*) PduInfoPtr);
		}
		break;
	case FlowControl_Frame:
		//adjust the numberOfConsecutiveFramesToSend variable inside a function
		//based on the number of empty buffers available in the other node
		//as indicated in the BS (block size) byte of the flow control frame
		CanTp_decodeFlowControlFrame(RxPduId, PduInfoPtr);
		return E_OK;
	default:
		break;
	}
	if(numberOfRemainingBytesToReceive == 0){
		if(App_Callback != NULL){
			currentIndex = 0;
			App_Callback(RxPduId, &CompletePduInfo);
		}
	}
	return E_OK;
}

Frame_Type CanTp_GetFrameType(uint8_t PCI){
	//Switch case on the PCI to determine the frame type
	PCI >>= 4;
	if(PCI < 4){
		return (Frame_Type) PCI;
	}
	else{
		return None;
	}
}

void CanTp_setCallback(void (*PTF)(uint32_t TxPduId, PduInfoType* PduInfoPtr)){
	if(PTF != NULL){
		App_Callback = PTF;
	}
}


void CanIf_Transmit(uint32_t RxPduId, PduInfoTRx* PduInfoPtr){

}

void CanIf_setCallback(Std_ReturnType (*IF_Callback)(uint32_t RxPduId, PduInfoTRx* PduInfoPtr)){

}

void CanTp_encodeSingleFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	//	// Check for NULL pointers
	//	if (PduInfoPtr == NULL ) {
	//		return E_NOK; // Return E_NOK for NULL pointer
	//	}

	// Ensure the data length does not exceed the maximum payload length
	uint32_t dataLength = PduInfoPtr->Length;
	EncodedPduInfo.Length = PduInfoPtr->Length;
	//	if (dataLength > CAN_MAX_PAYLOAD_LENGTH) {
	//		return E_NOK; // Return E_NOK for data length exceeding CAN payload length
	//	}

	// The first byte of the CAN frame is reserved for PCI (Protocol Control Information)
	EncodedPduInfo.Data[0] = 0x00 | (dataLength & 0x0F); // PCI is 0x0N where N is the length of the data
	uint32_t i;
	// Copy the data from PduInfoType to the CAN frame manually, starting from the second byte
	for ( i = 0; i < dataLength; i++) {
		EncodedPduInfo.Data[i + 1] = PduInfoPtr->Data[i];
	}

	// Fill the rest of the frame with zeros if necessary
	for (i = dataLength + 1; i < CAN_FRAME_LENGTH; i++) {
		EncodedPduInfo.Data[i] = 0;
	}

	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}
void CanTp_encodeFirstFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	/*** Local Variables ****/
	uint8_t Counter=0;
	PduInfoTRx EncodedPduInfo ;
	/************/

	// assume that data is [0x1 0x2 0x3 0x4 0x5 0x6 0x7 0x8 0x9 0xA]
	EncodedPduInfo.Data[0] = (0x01 <<4 ) | ((PduInfoPtr->Length)>>8 & 0x0F); // First Frame Should be 10 A 0x1 0x2 0x3 0x4 0x5 0x6
	EncodedPduInfo.Data[1] = (PduInfoPtr->Length)& 0xFF;

	// Form First Frame
	for(Counter=2;Counter<8;Counter++)
	{
		EncodedPduInfo.Data[Counter]=PduInfoPtr->Data[Counter - 2];
	}

	/** Call CanIF_Transmit Function**/
	numberOfRemainingBytesToSend = (PduInfoPtr->Length - 6);
	ConsecSN = 1;
	SNFlag = 0;
	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}
void CanTp_encodeConsecutiveFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){

	uint8_t i = 0;
	EncodedPduInfo.Length = numberOfRemainingBytesToSend > 7 ? 7 : numberOfRemainingBytesToSend;
	EncodedPduInfo.Data[0]=(0x02 << 4) | ConsecSN;
	uint8_t offset = (ConsecSN == 1 && !SNFlag)? ConsecSN * 7 - 1: ConsecSN * 7;
	for(i=0 ; i < EncodedPduInfo.Length ; i++)
	{
		EncodedPduInfo.Data[i+1] = PduInfoPtr->Data[i + offset];
	}
	ConsecSN = ConsecSN + 1 > 0xF ? 0 : ConsecSN + 1;
	SNFlag = ConsecSN ? 1 : 0;
	numberOfRemainingBytesToSend -= EncodedPduInfo.Length;
	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}
void CanTp_encodeFlowControlFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	// Initialize the flow control frame parameters
	// Byte 0: Flow Status (0x30 for continue to send, 0x31 for wait, 0x32 for overflow/abort)
	// Byte 1: Block Size (0 for continuous sending without waiting for flow control)
	// Byte 2: Separation Time (ST, in milliseconds, 0-127, 241-249 are valid values)

	EncodedPduInfo.Data[0] = 0x30;  // Flow Status: Continue to send (CTS)
	EncodedPduInfo.Data[1] = availableBuffers;  // Block Size: 0 (no blocks)
	EncodedPduInfo.Data[2] = 0x00;  // Separation Time: 0 ms (no delay)

	// The remaining bytes can be set to 0
	for (uint8_t i = 3; i < 8; i++) {
		EncodedPduInfo.Data[i] = 0x00;
	}

	// Set the length of the flow control frame
	//    EncodedPduInfo.Length = 3;

	// Use CanIf_Transmit to send the flow control frame
	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}

void CanTp_decodeSingleFrame(uint32_t RxPduId, PduInfoTRx* PduInfoPtr){
	// Extract the data length from the first byte of the CAN frame
	uint32_t dataLength = PduInfoPtr->Data[0] & 0x0F;
	int i;
	// Allocate memory for the data in the PduInfoTRx struct
	for ( i = 0; i < dataLength; i++) {
		DecodedPduInfo.Data[i] = PduInfoPtr->Data[i+1];
	}

	// Check for memory allocation failure
	//	if (PduInfoPtr->Data == NULL) {
	//		DecodedPduInfo.Length = 0;
	//		return DecodedPduInfo;
	//	}

	// Set the length in the PduInfoType struct
	DecodedPduInfo.Length = dataLength;

	CanTp_ConnectData(&DecodedPduInfo);
}
void CanTp_decodeFirstFrame(uint32_t RxPduId, PduInfoTRx* PduInfoPtr){
	numberOfRemainingBytesToReceive = ((PduInfoPtr->Data[0] & 0x0F) << 8) | PduInfoPtr->Data[1];
	DecodedPduInfo.Length=6;
	uint8_t Counter=0;

	for(Counter=0;Counter<8;Counter++)
	{
		DecodedPduInfo.Data[Counter]=PduInfoPtr->Data[Counter+2];
	}
	ConsecSN = 1;
	CanTp_ConnectData(&DecodedPduInfo);
}
void CanTp_decodeConsecutiveFrame(uint32_t RxPduId, PduInfoTRx* PduInfoPtr){
	uint8_t i = 0;
	DecodedPduInfo.Length = numberOfRemainingBytesToReceive > 7 ? 7 : numberOfRemainingBytesToReceive;
	if(ConsecSN == (PduInfoPtr->Data[0] & 0x0F)){
		for(i=0 ; i < DecodedPduInfo.Length ; i++)
		{
			DecodedPduInfo.Data[i] = PduInfoPtr->Data[i+1];
		}
		ConsecSN = ConsecSN + 1 > 0xF ? 0 : ConsecSN + 1;
		CanTp_ConnectData(&DecodedPduInfo);
	}
}
void CanTp_decodeFlowControlFrame(uint32_t RxPduId, PduInfoTRx* PduInfoPtr){
	// Extract the Flow Status, Block Size, and Separation Time from the PDU
	uint8_t flowStatus = PduInfoPtr->Data[0];
	uint8_t blockSize = PduInfoPtr->Data[1];
//	uint8_t separationTime = PduInfoPtr->Data[2];

	// Update the number of consecutive frames to send based on the Block Size
	//	if (blockSize == 0) {
	// Continuous sending without waiting for further flow control
	numberOfConsecutiveFramesToSend = blockSize;
	//	} else {
	//		numberOfConsecutiveFramesToSend = blockSize;
	//	}

	// Handle different flow statuses
	switch (flowStatus) {
	case 0x30:  // Continue to send (CTS)
		// Update expected frame state to send consecutive frames
		expectedFrameState = Consecutive_Frame_State;
		break;

	case 0x31:  // Wait (WT)
		// Flow control indicates to wait
		expectedFrameState = FlowControl_Frame_State;
		break;

	case 0x32:  // Overflow/Abort (OVFLW/ABORT)
		// Handle overflow or abort condition
		//		expectedFrameState = Any_State;
		// Perform any additional actions needed for abort, such as notifying upper layers
		break;

	default:
		// Invalid flow status, handle as needed (e.g., set an error state)
		//		expectedFrameState = Any_State;
		break;
	}

}

void CanTp_ConnectData(PduInfoTRx* PduInfoPtr){
	//use CompletePduInfo struct to connect the data received from PduInfoTRx
	uint16_t tempCurrentIndex = currentIndex;
	while(currentIndex < PduInfoPtr->Length + tempCurrentIndex){
		CompletePduInfo.Data[currentIndex] = PduInfoPtr->Data[currentIndex - tempCurrentIndex];
		currentIndex++;
	}
	numberOfRemainingBytesToReceive -= PduInfoPtr->Length;
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
