
#ifndef INC_APP_UDS_DIAG_H_
#define INC_APP_UDS_DIAG_H_



#include "APP_UDS_Diag_CFG.h"
#include "stdint.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdio.h"

#define NULL_Value 0
//For Security
extern uint8_t* seed;
extern uint8_t* key;



//Can Tp Structs & functions
typedef struct
{
	uint8_t Data[4096];
	uint32_t Length;
}PduInfoType;

typedef struct
{
	uint8_t Data[8];
	uint32_t Length;
}PduInfoTRx;

//Std_ReturnType CanTp_Transmit(uint32_t TxPduId, PduInfoType* PduInfoPtr);
//Std_ReturnType CanTp_RxIndication (uint32_t RxPduId, PduInfoTRx* PduInfoPtr);


void UDS_Init(void);
void UDS_Read_Data_Client(DID did);
//void UDS_Read_Data_Server( PduInfoType* PduInfoTypePtr );

void UDS_Write_Data_Client(DID did, uint32_t data);
//void UDS_Write_Data_Server(PduInfoType* Received_data);

void UDS_Control_Session_Default(void);
void UDS_Control_Session_Extended(void);

void UDS_Send_Security_Client(Sub_Fun sub_fun);
//void UDS_Send_Security_Server(void);


//void UDS_Client_Callback(uint32_t RxPduId,PduInfoType *Ptr);

void sendHexArrayAsASCII(uint8_t* hexArray, uint16_t length);

void UART_ReceiveAndConvert(uint8_t RX_BUFFER_SIZE, PduInfoType* PduInfoType_Ptr);
uint8_t charToHex(uint8_t ascii);


#endif /* INC_APP_UDS_DIAG_H_ */
