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
#include "APP_UDS_Diag.h"
#include "string.h"
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
#define CAN_MODE CAN_RX
#define RX_BUFFER_SIZE 100
#define CAN_FRAME_LENGTH 8
#define CAN_MAX_PAYLOAD_LENGTH 7
// there are many ser. has sub,  this var indecate for which sub servise
#define sub_func_control  	1
#define Num_of_Services 	5


#if CAN_MODE == CAN_TX
/**********App *******************/

//For Menus
ClientMenu Menu = Main_Menu;
uint8_t Menu_Letter = {0};

uint8_t UDS_Tx_Confirm = 0;
PduInfoType* UDS_Struct;

uint8_t* seed = NULL;
uint8_t* key = NULL;
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
TaskHandle_t xTaskHandle2 = NULL;
TaskHandle_t xTaskHandle3 = NULL;
//
//typedef struct
//{
//	int8_t Data[4096];
//	uint32_t Length;
//}PduInfoType;
//
//typedef struct
//{
//	uint8_t Data[8];
//	uint32_t Length;
//}PduInfoTRx;

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



#if CAN_MODE == CAN_RX


typedef struct
{
	uint8_t* data;
	uint8_t length;
}ST_TP_pduID;

typedef enum
{
	NRC_WRITE_secuirty = 10,
	NRC_WRITE_defualt	=15,
	NRC_SID = 20,
	NRC_sub_fun = 30,
	NRC_sec_key_seed = 40


}NRC_VAR;


#endif


volatile uint32_t numberOfConsecutiveFramesToSend = 0;
volatile uint32_t currentConsecutiveFrames = 0;
volatile uint32_t numberOfConsecutiveFramesToReceive = 0;
volatile uint32_t numberOfRemainingBytesToSend = 0;
volatile uint32_t numberOfRemainingBytesToReceive = 0;
volatile uint32_t availableBuffers = 10;
void (*App_Callback)(uint32_t RxPduId, PduInfoType* PduInfoPtr) = NULL;
Std_ReturnType (*CanTp_Callback)(uint32_t RxPduId, PduInfoTRx* PduInfoPtr) = NULL;

Frame_States expectedFrameState = Any_State;
PduInfoTRx EncodedPduInfo;
PduInfoTRx DecodedPduInfo;
PduInfoType CompletePduInfo;
PduInfoTRx CanIfPduInfo;
CAN_RxHeaderTypeDef rxHeader;
PduInfoTRx* GlobalRxPduInfoPtr;
PduInfoType* GlobalTxPduInfoPtr;

volatile uint8_t ConsecSN;
volatile uint16_t currentIndex;
volatile int32_t currentOffset = -1;
volatile int32_t startOffset;
volatile int8_t CanIf_Rx;
volatile int8_t CanTp_Rx;
volatile int8_t CanTp_Tx;
volatile uint32_t GlobalRxPduId;
volatile uint32_t GlobalTxPduId;


#if CAN_MODE == CAN_RX
volatile uint8_t global_sec_flag = 0;
volatile uint8_t global_session = DefaultSession;
volatile uint8_t flag_sub_fun = 0;
volatile uint8_t Security_Service_Availability_Flag = Not_Available;
//uint8_t seed =50 ; // for example
/*Security Variables */
volatile uint32_t Sec_u32SeedValue = 0 ;
Security_Access_State Sec_State = Un_Secure ;
volatile uint32_t Oil_Pressure_var = 0x778899AA;
volatile uint32_t Oil_Temp_var = 0x5566;
ServiceInfo pos_Response;
PduInfoType *PduDataPTR;
ServiceInfo Control;
PduInfoType msg;
#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void delay_ms(uint32_t milliseconds);
void PseudoAppCallback(uint32_t TxPduId, PduInfoType* PduInfoPtr);
void CanTp_Init();
void CanTp_MainFunction();


#if CAN_MODE == CAN_TX


void UDS_MainFunction();
void Display_Menu(void);
void UDS_Client_Callback(uint32_t TxPduId,PduInfoType *PduInfoPtr);


#else


void server_call_back(uint32_t TxPduId, PduInfoType* ptr);
void send_ses_Def();
void send_ses_ext();
uint8_t check_DID(uint16_t DID, uint32_t data);
void UDS_Write_Data_Server(uint8_t* received_data, uint16_t received_length);
void UDS_Send_Pos_Res(ServiceInfo* Response);
void UDS_Send_Neg_Res(uint8_t SID, uint8_t NRC);
void Sec_u32GetSeed (void);
uint32_t Sec_u32GetAlgorithm(void);
uint32_t Sec_u32GetKey (void);
uint8_t Sec_uint32SecurityAccess (PduInfoType * Ptr);
void UDS_Read_Data_Server(uint8_t* data);
void UDS_Control_Session_Server(uint8_t *Received);


#endif


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
void CanIf_Receive();
//use this setcallback in the init so that the canIf calls our  CanTp_RxIndication
void CanIf_setCallback(Std_ReturnType (*IF_Callback)(uint32_t RxPduId, PduInfoTRx* PduInfoPtr));

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
	/* USER CODE BEGIN 2 */
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
#if CAN_MODE == CAN_TX
	Display_Menu();
	HAL_UART_Receive_IT(&huart2, &Menu_Letter, 1);
	CanTp_setCallback(UDS_Client_Callback);
	xTaskCreate(UDS_MainFunction, "UDS_RX", configMINIMAL_STACK_SIZE,NULL, 2, &xTaskHandle3) ;
#else
	CanTp_setCallback(server_call_back);
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

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	if(!rxComplete){
//		if (rxBufferIndex <= RX_BUFFER_SIZE - 1) {
//			rxBuffer[rxBufferIndex++] = rxData;
//		}
//		else{
//
//			// Buffer overflow handling
//			//			rxBufferIndex = 0;
//		}
//		HAL_UART_Receive_IT(&huart2, (uint8_t*) &rxData, 1); // Start next reception
//		//	HAL_UART_Transmit(&huart2, &rxData, 1, HAL_MAX_DELAY);
//		if (rxData == 0x0D) { // Example: End of line delimiter
//			rxComplete = 1;
//			//			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
//		}
//	}
//}


/**
 *  @brief CAN interface transmit function
 *  @param  PduInfoTRx*		: Pointer to message structure contain (Data, Length)
 *  @param  TxPduId	: PDU ID
 *  @return None
 */
void CanIf_Transmit(uint32_t TxPduId, PduInfoTRx* PduInfoPtr){
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox;
	if(TxPduId == 0){
		txHeader.StdId = 0x100;
	}

	txHeader.ExtId = 0x00;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 8;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, PduInfoPtr->Data, &txMailbox) != HAL_OK) {
		// Transmission error
		Error_Handler();

	}
#if CAN_MODE == CAN_TX
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
#endif
}

/**
 *  @brief CAN interface receive data
 *  @param  None
 *  @return None
 */
void CanIf_Receive(){
	uint32_t PDU_ID;
	while(1){
		if(CanIf_Rx){
			CanIf_Rx = 0;
			CanIfPduInfo.Length = rxHeader.DLC;
			switch(rxHeader.StdId)
			{
			case 0x100 :PDU_ID = 0;
			break;
			case 0x200 :PDU_ID = 1;
			break;
			case 0x300 :PDU_ID = 2;
			break;
			case 0x400 :PDU_ID = 3;
			break;
			case 0x500 :PDU_ID = 4;
			break;
			case 0x600 :PDU_ID = 5;
			break;
			case 0x700 :PDU_ID = 6;
			break;
			case 0x800 :PDU_ID = 7;
			break;
			}

			if(CanTp_Callback != NULL)
			{
				CanTp_Callback(PDU_ID, &CanIfPduInfo);
			}
		}
		vTaskDelay(10);
	}
}

void CanIf_setCallback(Std_ReturnType (*IF_Callback)(uint32_t RxPduId, PduInfoTRx* PduInfoPtr)){
	if(IF_Callback != NULL)
	{
		CanTp_Callback = IF_Callback ;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1){
	if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &rxHeader, CanIfPduInfo.Data) != HAL_OK) {
		// Reception error
		Error_Handler();
	}
	CanIf_Rx = 1;
}

void CanTp_Init(){
	CanIf_setCallback(CanTp_RxIndication);
}

void CanTp_MainFunction(){
	while(1){
		if(CanTp_Rx){
			CanTp_Rx = 0;
			//Stop the program if the PduID doesn't equal 0 :)
			if(GlobalRxPduId != 0){
				while(1);
			}
			//Get the frame type from the
			Frame_Type frame_type = CanTp_GetFrameType(GlobalRxPduInfoPtr->Data[0]);
			//	Frame_Type frame_type = First_Frame

			//Call the correct decoder depending on the frame type
			//extract the length and save it in numberOfRemainingBytesToReceive and connect the data
			switch(frame_type){
			case Single_Frame:
				CanTp_decodeSingleFrame(GlobalRxPduId, GlobalRxPduInfoPtr);
				break;
			case First_Frame:
				CanTp_decodeFirstFrame(GlobalRxPduId, GlobalRxPduInfoPtr);
				expectedFrameState = FlowControl_Frame_State;
				CanTp_Transmit(GlobalRxPduId, (PduInfoType*) GlobalRxPduInfoPtr);
				break;
			case Consecutive_Frame:
				CanTp_decodeConsecutiveFrame(GlobalRxPduId, GlobalRxPduInfoPtr);
				if(numberOfConsecutiveFramesToReceive == 0 && numberOfRemainingBytesToReceive > 0){
					expectedFrameState = FlowControl_Frame_State;
					CanTp_Transmit(GlobalRxPduId, (PduInfoType*) GlobalRxPduInfoPtr);
				}
				break;
			case FlowControl_Frame:
				//adjust the numberOfConsecutiveFramesToSend variable inside a function
				//based on the number of empty buffers available in the other node
				//as indicated in the BS (block size) byte of the flow control frame
				CanTp_decodeFlowControlFrame(GlobalRxPduId, GlobalRxPduInfoPtr);
				break;
			default:
				break;
			}

			if(frame_type == FlowControl_Frame){

			}
			else if(numberOfRemainingBytesToReceive == 0){
				if(App_Callback != NULL){
					currentIndex = 0;
					App_Callback(GlobalRxPduId, &CompletePduInfo);
				}
			}
		}
		else if(CanTp_Tx){
			//Stop the program if the PduID doesn't equal 0 :)
			if(GlobalTxPduId != 0){
				while(1);
			}

			Frame_Type frame_type = None;
			if(numberOfRemainingBytesToSend == 0 && expectedFrameState == Any_State){
				numberOfRemainingBytesToSend = GlobalTxPduInfoPtr->Length;
				CompletePduInfo.Length = numberOfRemainingBytesToSend;
				if(GlobalTxPduInfoPtr->Length < 8){
					frame_type = Single_Frame;
				}
				else{
					frame_type = First_Frame;
				}
			}


			if(numberOfRemainingBytesToSend > 0 || expectedFrameState == FlowControl_Frame_State){

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
					CanTp_encodeSingleFrame(GlobalTxPduId, GlobalTxPduInfoPtr);
					break;
				case First_Frame:
					CanTp_encodeFirstFrame(GlobalTxPduId, GlobalTxPduInfoPtr);
					frame_type = None;
					break;
				case Consecutive_Frame:
					if(numberOfConsecutiveFramesToSend > 0){
						numberOfConsecutiveFramesToSend--;
						CanTp_encodeConsecutiveFrame(GlobalTxPduId, GlobalTxPduInfoPtr);
					}
					else{
						frame_type = None;
						//wait for flow control to reach CanTp_RxIndication in order to change numberOfConsecutiveFramesToSend variable
					}
					break;
				case FlowControl_Frame:
					//Check the availableBuffers variable (in our case it's the size of the receive array)
					CanTp_encodeFlowControlFrame(GlobalTxPduId, GlobalTxPduInfoPtr);
					break;
				default:
					break;
				}
			}

			if(numberOfRemainingBytesToSend == 0){
				//Reset the expected frame
				expectedFrameState = Any_State;
				currentOffset = -1;
				CanTp_Tx = 0;
			}
		}
		vTaskDelay(100);
	}
}

Std_ReturnType CanTp_Transmit(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	GlobalTxPduInfoPtr = PduInfoPtr;
	GlobalTxPduId = TxPduId;
	CanTp_Tx = 1;
	return E_OK;
}

Std_ReturnType CanTp_RxIndication (uint32_t RxPduId, PduInfoTRx* PduInfoPtr){
	GlobalRxPduInfoPtr = PduInfoPtr;
	GlobalRxPduId = RxPduId;
	CanTp_Rx = 1;
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


void CanTp_encodeSingleFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	//	// Check for NULL pointers
	//	if (PduInfoPtr == NULL ) {
	//		return E_NOK; // Return E_NOK for NULL pointer
	//	}

	// Ensure the data length does not exceed the maximum payload length
	uint32_t dataLength = PduInfoPtr->Length;
	EncodedPduInfo.Length = PduInfoPtr->Length;
	numberOfRemainingBytesToSend -= dataLength;
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
	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}
void CanTp_encodeConsecutiveFrame(uint32_t TxPduId, PduInfoType* PduInfoPtr){
	uint8_t i = 0;
	EncodedPduInfo.Length = numberOfRemainingBytesToSend > 7 ? 7 : numberOfRemainingBytesToSend;
	EncodedPduInfo.Data[0]=(0x02 << 4) | ConsecSN;

	currentOffset = startOffset + ConsecSN * 7;

	for(i=0 ; i < EncodedPduInfo.Length ; i++)
	{
		EncodedPduInfo.Data[i+1] = PduInfoPtr->Data[i + currentOffset];
	}

	ConsecSN++;
	if(ConsecSN > 0xF){
		startOffset = currentOffset;
		ConsecSN = 0;
	}
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
	ConsecSN = 1;
	// Use CanIf_Transmit to send the flow control frame
	CanIf_Transmit(TxPduId, &EncodedPduInfo);
}

void CanTp_decodeSingleFrame(uint32_t RxPduId, PduInfoTRx* PduInfoPtr){
	// Extract the data length from the first byte of the CAN frame
	uint32_t dataLength = PduInfoPtr->Data[0] & 0x0F;
	numberOfRemainingBytesToReceive = dataLength;
	CompletePduInfo.Length = numberOfRemainingBytesToReceive;
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
	CompletePduInfo.Length = numberOfRemainingBytesToReceive;
	DecodedPduInfo.Length=6;
	uint8_t Counter=0;

	for(Counter=0;Counter<8;Counter++)
	{
		DecodedPduInfo.Data[Counter]=PduInfoPtr->Data[Counter+2];
	}
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
		startOffset = currentOffset;
		ConsecSN = 1;
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

#if CAN_MODE == CAN_TX
void UDS_MainFunction()
{
	while(1)
	{
		if(UDS_Tx_Confirm)
		{
			UDS_Tx_Confirm = 0;
			if ( UDS_Struct ->Data[Neg_Res] == 0x7f)
			{
				HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n Sorry Negative frame .\r\n",strlen("\r\n Sorry Negative frame .\r\n"), HAL_MAX_DELAY);
			}

			else {
				UDS_Struct->Data[SID] = UDS_Struct->Data[SID] - 0x40;
				if (UDS_Struct->Data[SID] == Read_Service)

				{
					if  ( UDS_Struct->Data[DID_1] == Oil_Temp_First_byte &&  UDS_Struct->Data[DID_2] == Oil_Temp_Second_byte )
					{

						HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n Oil Temp .\r\n", strlen("\r\n Oil Temp .\r\n"), HAL_MAX_DELAY);
						sendHexArrayAsASCII((uint8_t*)&UDS_Struct->Data[Data_DID],2);

					}

					else if  ( UDS_Struct->Data[DID_1] == Oil_Pressure_First_byte &&  UDS_Struct->Data[DID_2] == Oil_Pressure_Second_byte  )
					{
						HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n Oil Pressure .\r\n", strlen("\r\n Oil Pressure .\r\n"), HAL_MAX_DELAY);
						sendHexArrayAsASCII((uint8_t*)&UDS_Struct->Data[Data_DID],4);

					}
				}

				/*  Write IDS Message */
				else if (UDS_Struct->Data[SID] == Write_Service)

				{

					if ( UDS_Struct->Data[DID_1] == Oil_Temp_First_byte &&  UDS_Struct->Data[DID_2] == Oil_Temp_Second_byte )
					{
						HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n Oil Temperature Written successfully .\r\n", strlen("\r\n Oil Temperature Written successfully .\r\n"), HAL_MAX_DELAY);
					}

					else if  ( UDS_Struct->Data[DID_1] == Oil_Pressure_First_byte &&  UDS_Struct->Data[DID_2] == Oil_Pressure_Second_byte  )
					{
						HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n Oil Pressure Written successfully .\r\n", strlen("\r\n Oil Pressure Written successfully .\r\n"), HAL_MAX_DELAY);
					}
				}
				/*  Control Service  Session */

				else if (UDS_Struct->Data[SID] == Control_Service)

				{

					switch ( UDS_Struct->Data[Sub_F] )
					{

					case DefaultSession :
						HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n It's Default Session ! .\r\n", strlen("\r\n It's Default Session ! .\r\n"), HAL_MAX_DELAY);
						//UDS_Read_Data_Client(Frame_Info.DID);
						break ;

					case ExtendedSession :
						HAL_UART_Transmit(&huart2,(uint8_t*) "\r\n It's Extended Session ! .\r\n", strlen("\r\n It's Extended Session ! .\r\n"), HAL_MAX_DELAY);
						//UDS_Read_Data_Client(Frame_Info.DID);
						break ;
					}
				}
				/*  Security Service  */

				else if (UDS_Struct->Data[SID] == Security_Service)

				{

					switch ( UDS_Struct->Data[Sub_F] )
					{

					case Key :
						HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n Key is Compatible ! .\r\n", strlen ( "\r\n Key is Compatible ! .\r\n"), HAL_MAX_DELAY);
						break ;

					case Seed :
						HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n Seed is \r\n", strlen ("\r\n Seed is \r\n"), HAL_MAX_DELAY);
						sendHexArrayAsASCII((uint8_t*)&UDS_Struct->Data[Data_Sub_Fun],4);

						break ;
					}
				}
				/*  Tester_Representer_Service  */

				else if (UDS_Struct->Data[SID] == Tester_Representer_Service)

				{

					HAL_UART_Transmit(&huart2,(uint8_t*) "\r\n ECU Reseted ! .\r\n", strlen("\r\n ECU Reseted ! .\r\n"), HAL_MAX_DELAY);

				}

				else { /* no thing  */	}

			}

		}

		vTaskDelay(50);
	}
}

void UDS_Client_Callback(uint32_t TxPduId,PduInfoType *PduInfoPtr)
{
	UDS_Tx_Confirm = 1;
	UDS_Struct = PduInfoPtr;

}

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
void Sec_u32GetSeed (void)
{
	Sec_u32SeedValue = HAL_GetTick();
	//	printf("%d",Sec_u32SeedValue) ;
}

uint32_t Sec_u32GetAlgorithm(void)
{
	return 5 ;
}

uint32_t Sec_u32GetKey (void)
{
	uint32_t Local_u32KeyValue = 0 ;
	Local_u32KeyValue = Sec_u32SeedValue + Sec_u32GetAlgorithm() ;
//	Local_u32KeyValue = 0x01020304 + Sec_u32GetAlgorithm() ;
	return Local_u32KeyValue;
}


uint8_t Sec_uint32SecurityAccess (PduInfoType * Ptr)
{
	uint8_t Local_u8ErrorStates = E_OK ;

	if (Ptr->Data[2] == Seed)
	{
		/*Generate Seed */
				Sec_u32GetSeed();

		/*Send Frame with Positive Response */
		//		Frame_Info Response ;
		pos_Response.SID 		= Security_Service ;
		pos_Response.SUB_FUNC	= Seed ;
		pos_Response.DID_Length=0;
		pos_Response.Data_Length=4;
//		for(int i = 1; i < pos_Response.Data_Length + 1; i++){
//			pos_Response.Data[i-1] = i;
//		}
		for(int i =0 ; i< 4; i++)
		{
			pos_Response.Data[i] 		=(uint8_t) (Sec_u32SeedValue>>(24-(i*8))) ;
		}

		UDS_Send_Pos_Res(&pos_Response) ;
	}
	else if (Ptr->Data[2] == Key)
	{
		uint32_t user_key= Ptr->Data[3]<<24 | Ptr->Data[4]<<16 | Ptr->Data[5]<<8 |Ptr->Data[6];
		/*Check if Key sent is correct or Not */
		if (user_key == Sec_u32GetKey())
		{
			/*Change the state of security */
			Sec_State = Secure ;
			global_sec_flag = Secure;
			/*Send Positive Response */
			pos_Response.SID= Ptr->Data[1];
			pos_Response.SUB_FUNC = Key;
			pos_Response.DID_Length = 0;
			pos_Response.Data_Length = 0;
			UDS_Send_Pos_Res(&pos_Response);
		}
		else
		{
			Sec_State = Un_Secure ;
			global_sec_flag = Un_Secure;
			UDS_Send_Neg_Res(Ptr->Data[1] , NRC_sec_key_seed) ;
		}
	}
	else
	{
		Local_u8ErrorStates = E_NOK ;
		UDS_Send_Neg_Res(Ptr->Data[1] , NRC_sub_fun) ;
	}


	return Local_u8ErrorStates ;
}


/***************************************************************************************************/
void UDS_Read_Data_Server(uint8_t* data)
{
	pos_Response.SUB_FUNC = -1;
	uint8_t NRC = 2;
	//Send +ve responce
	pos_Response.SID = Read_Service ;
	pos_Response.DID_Length=2;

	//if DID --> Oil_Temp
	if((data[DID_1] == Oil_Temp_First_byte) && (data[DID_2] == Oil_Temp_Second_byte) )
	{
		pos_Response.DID[0]=Oil_Temp_First_byte;
		pos_Response.DID[1]=Oil_Temp_Second_byte;
		pos_Response.Data[0]=Oil_Temp_var>>8;
		pos_Response.Data[1]=Oil_Temp_var & 0xFF;
		pos_Response.Data_Length = 2;

		UDS_Send_Pos_Res(&pos_Response);
		//	UDS_Send_Pos_Res(Read_Data_Server);
	}//if DID --> Oil_Pressure
	else if((data[DID_1] == Oil_Pressure_First_byte) && (data[DID_2] == Oil_Pressure_Second_byte) )
	{
		pos_Response.DID[0]=Oil_Pressure_First_byte;
		pos_Response.DID[1]=Oil_Pressure_Second_byte;
		pos_Response.Data[0]=Oil_Pressure_var>>24;
		pos_Response.Data[1]=Oil_Pressure_var>>16;
		pos_Response.Data[2]=Oil_Pressure_var>>8;
		pos_Response.Data[3]=Oil_Pressure_var & 0xFF;

		pos_Response.Data_Length = 4;

		UDS_Send_Pos_Res(&pos_Response);

		//For Debugging
		//HAL_UART_Transmit(&huart2, "\r\nRead Frame Client DID:", 50, HAL_MAX_DELAY);
		//sendHexArrayAsASCII(Read_Data_Server.DID, Read_Data_Server.DID_Length );
		//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
	}
	else
	{
		//otherwize: send -ve responce
		UDS_Send_Neg_Res(Read_Service, NRC);
	}

}



/*********************************************************************************************/


void UDS_Control_Session_Server(uint8_t *Received)
{
	uint8_t NRC = 1;


	if(Received[2] == DefaultSession || Received[2] == ExtendedSession)
	{
		global_session = Received[2];
		Control.SID = Received[1];
		Control.SUB_FUNC = Received[2];
		Control.DID_Length = 0;
		Control.Data_Length = 0;
		UDS_Send_Pos_Res(&Control);
	}
	else
	{
		UDS_Send_Neg_Res(Received[1], NRC);
	}
}

void server_call_back(uint32_t TxPduId, PduInfoType* ptr)
{
	PduDataPTR = ptr;
	// create flag for check SID this is local bec . every frame i need to check the sid
	uint8_t local_sid_flag = 0;

	// this for test only
	//uint8_t ptr->Data[20] = {2 ,Control_Service , 5 };

	// for SID validation
	if (ptr->Data[1] == Control_Service || ptr->Data[1]== Read_Service || ptr->Data[1] == Write_Service || ptr->Data[1] == Security_Service || ptr->Data[1] == Tester_Representer_Service)
	{
		// tmam

		local_sid_flag = 1;


	}
	else
	{
		// let error code of NRC =0 ;
		uint8_t NRC = NRC_SID;
		// for test only
		//HAL_UART_Transmit(&huart2, (const uint8_t*)" -ive there is no SID has this name \r\n", 50, HAL_MAX_DELAY );

		// this mean the SID not supported
		UDS_Send_Neg_Res(ptr->Data[1],  NRC);
		// go out of isr
		return;
	}
	if (local_sid_flag)
	{
		if (ptr->Data[1] == Control_Service)
		{
			flag_sub_fun = 1;
		}
		else if (ptr->Data[1] == Read_Service)
		{
			//  for test
			//	printf("u are in Read_Service\n");
			// send read function (rad resp as the actual ptr->Data of temp or pressure)
			//	HAL_UART_Transmit(&huart2, (const uint8_t*)" UDS_Read_Data_Server() \r\n", 50, HAL_MAX_DELAY ); // delete ---> after write your func

			//UDS_Read_Data_Server();

			UDS_Read_Data_Server(ptr->Data);

		}
		else if (ptr->Data[1] == Security_Service )
		{

			//	printf("send_ser_sec() +ive resp \n");
			//			HAL_UART_Transmit(&huart2, (const uint8_t*)" send_ser_sec() +ive resp change the flag \r\n", 50, 100 ); // delete this after you put your func

			//		send_ser_sec() ; // send seed

			// (write here +ive resp for security) ------------------------> here

			Sec_uint32SecurityAccess(PduDataPTR);
			//			UDS_Send_Pos_Res(&pos_Response) ;
		}


		else if (ptr->Data[1] == Write_Service && global_sec_flag ==1 && global_session == ExtendedSession  )
		{
			//printf("u are in Write_Service\n");
			//			HAL_UART_Transmit(&huart2, (const uint8_t*)" u are in Write_Service \r\n", 50, 100 );
			//			// send write response
			//			//	printf("UDS_Write_Data_Server() \n");
			//			HAL_UART_Transmit(&huart2, (const uint8_t*)" UDS_Write_Data_Server() \r\n", 50, 100 ); // delete it after put your func

			// ptr->Data write with +ive resp


			UDS_Write_Data_Server(ptr->Data,  ptr->Data[0]);
		}
		else if (ptr->Data[1] == Write_Service && global_sec_flag == 0 && global_session == ExtendedSession )
		{
			//printf("u are not in Write_Service\n");
			//			HAL_UART_Transmit(&huart2, (const uint8_t*)" u are not in Write_Service \r\n", 50, 100 );
			//			// send -ive response
			//			//printf("UDS_Write_Data_Server() \n");
			//			HAL_UART_Transmit(&huart2, (const uint8_t*)"UDS_Write_Data_Server() \r\n", 50, 100 ); // delete this after put your func
			// (write here -ive resp for write security ) ------------------------> here

			UDS_Send_Neg_Res(ptr->Data[1], NRC_WRITE_secuirty);

		}
		else if (ptr->Data[1] == Write_Service  && global_session == DefaultSession)
		{
			// (write here -ive resp for write session (NRC ) ------------------------> here

			UDS_Send_Neg_Res(ptr->Data[1], NRC_WRITE_defualt);

		}
		else if (ptr->Data[1] == Tester_Representer_Service)
		{
			//printf("u are in Tester_Representer_Service\n");
			HAL_UART_Transmit(&huart2, (const uint8_t*)" u are in Tester_Representer_Service \r\n", 50, 100 );
			// call the fun of tester Representer
			//printf("void UDS_Tester_Present(void) \n");
			HAL_UART_Transmit(&huart2, (const uint8_t*)" void UDS_Tester_Present(void) \r\n", 50, 100 ); // delete this func after put your func

			// (write here +ive resp for  Tester_Representer_Service) ------------------------> here
		}
	}
	// check sub fun
	if (flag_sub_fun== sub_func_control)
	{
		// true sub fun
		if (ptr->Data[1]== Control_Service && ptr->Data[2] == DefaultSession)
		{
			// change the state to default
			//	printf(" UDS_Process_Session(void); \n ");
			HAL_UART_Transmit(&huart2, (const uint8_t*)" UDS_Process_Session(void) \r\n", 50, 100 );

			HAL_UART_Transmit(&huart2, (const uint8_t*)" changed to DefaultSession \r\n", 50, 100 );
			// (write here +ive resp for  change to def- session ) ------------------------> here


			UDS_Control_Session_Server(ptr->Data);
			//global_session = Default_Session;

		}
		else if (ptr->Data[1] == Control_Service && ptr->Data[2] == ExtendedSession)
		{
			//printf(" UDS_Process_Session(void); \n ");
			// change to extended
			//	printf(" changed to ExtendedSession ");
			//	HAL_UART_Transmit(&huart2, (const uint8_t*)" changed to ExtendedSession \r\n", 50, 100 );

			// (write here +ive resp for  change to ext session ) ------------------------> here
			//global_session = Extended_Session;
			UDS_Control_Session_Server(ptr->Data);
		}
		else
		{
			//printf(" not supported ");
			HAL_UART_Transmit(&huart2, (const uint8_t*)" not supported \r\n", 50, 100 );

			// (write here -ive resp for sub servise ) ------------------------> here
		}
	}
	else
	{

		// error in sub func

	}

}

/**********************************************************************************************/
//uint8_t check_DID(uint16_t DID, uint32_t data)
//{
//	if (Oil_Temp == DID)
//	{
//		oil_temp_var = (data>>16); // update the 2 bytes data for Oil temperature
//		return 1; // Indicating success
//	}
//	else if (Oil_Pressure == DID)
//	{
//		oil_pressure_var = data ; // update the 4 bytes data for Oil pressure
//		return 1; // Indicating success
//	}
//	else
//	{
//		return 0; // Indicating failure
//	}
//}


void UDS_Write_Data_Server(uint8_t* received_data, uint16_t received_length)
{
	/*???????????????????????*/
	uint8_t received_data_l = sizeof(received_data);
	if (received_length != received_data_l - 1)
	{
		//return -ive response NRC data length != length that assign to the frame
	}

	// Extract the DID from the received data
	//	uint16_t DID = (received_data[2] << 8) | received_data[3];

	// Extract the data from the received data
	//	uint32_t data;
	//	data = (received_data[4] << 24) | (received_data[5] << 16) | (received_data[6] << 8) | received_data[7];


	// update the data in the required DID
	//	uint8_t returnType = check_DID( DID, data);
	// according to the returnType if it is equal to 1 --> positive response - but if it is equal to 0 --> negative response

	/*********************************
		this is the logic of +ive resp ( we need edit )
	 *************************************/
	// Define the positive response array
	//	uint8_t arr[7]; // Adjust size as needed
	//	uint8_t  SID_response = 0x6E; // Positive response SID (0x2E + 0x40)

	// Fill the local array with SID_response, DID, and data
	//	arr[0] = SID_response;
	//	arr[1] = (DID >> 8) & 0xFF; // Most significant byte of DID
	//	arr[2] = DID & 0xFF;        // Least significant byte of DID
	//
	//	// Assuming Data is 4 bytes
	//	arr[3] = (data >> 24) & 0xFF; // Most significant byte of data
	//	arr[4] = (data >> 16) & 0xFF;
	//	arr[5] = (data >> 8) & 0xFF;
	//	arr[6] = data & 0xFF;		  // Least significant byte of data


	pos_Response.SID = Write_Service ;
	pos_Response.DID[0]=received_data[2];
	pos_Response.DID[1]=received_data[3];
	pos_Response.DID_Length=2;
	pos_Response.Data_Length = 0;
	pos_Response.SUB_FUNC = -1;

	if(received_data[DID_1] == Oil_Temp_First_byte && received_data[DID_2] == Oil_Temp_Second_byte){
		Oil_Temp_var = received_data[4] << 8 | received_data[5];
	}
	else if(received_data[DID_1] == Oil_Pressure_First_byte && received_data[DID_2] == Oil_Pressure_Second_byte){
		Oil_Pressure_var = received_data[4] << 24 | received_data[5] << 16 | received_data[6] << 8 | received_data[7];
	}
	UDS_Send_Pos_Res(&pos_Response);


	/*
	PduInfoType hamada_write;

	// Prepare the TP structure
	//hamada_write.Data = arr;

	for(int i = 0 ; i< 8; i++)
	{
		hamada_write.Data[i] = arr[i];
	}
	hamada_write.Length = sizeof(arr);

	// Transmit the data through CAN_TP using this function
	CanTP_Transmit(0, &hamada_write);*/
}



/*****************************************************************************/

void UDS_Send_Pos_Res(ServiceInfo* Response)
{

	uint8_t PCI = 2 + Response->DID_Length + Response->Data_Length;
	msg.Data[1] = Response->SID + 0x40;
	uint8_t currentIndex = 2;
	if(Response->SUB_FUNC != -1)
	{
		PCI++;
		msg.Data[currentIndex++]= Response->SUB_FUNC;
	}
	else
	{
		for(currentIndex = 2; currentIndex < Response->DID_Length + 2; currentIndex++)
		{
			msg.Data[currentIndex] = Response->DID[currentIndex - 2];
		}
	}

	uint8_t temp = currentIndex;
	while(currentIndex < Response->Data_Length + temp){
		msg.Data[currentIndex] = Response->Data[currentIndex - temp];
		currentIndex++;
	}
	msg.Data[0] = PCI;
	msg.Length = PCI;

	CanTp_Transmit(0, &msg);
}

void UDS_Send_Neg_Res(uint8_t SID, uint8_t NRC)
{
	msg.Data[0] = 4;
	msg.Data[1] = 0x7F;
	msg.Data[2] = SID;
	msg.Data[3] = NRC;
	msg.Length = 4;

	CanTp_Transmit(0, &msg);
}


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
#if CAN_MODE == CAN_RX
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
#endif

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

