/*
 * UDS_APP_Diag.c
 *
 *  Created on: Jun 4, 2024
 *      Author: Omnia
 */

#include "APP_UDS_Diag.h"

// there are many ser. has sub,  this var indecate for which sub servise
#define sub_func_control  	1
#define Num_of_Services 	5

Std_ReturnType CanTp_Transmit(uint32_t TxPduId, PduInfoType* PduInfoPtr);


PduInfoType Read_Data_Client;
PduInfoType Write_Data_Client;
PduInfoType Control_Session_Default;
PduInfoType Send_Security_Seed;
PduInfoType Control_Session_Extended;
PduInfoType PduInfoTypePtr;
volatile uint8_t global_sec_flag = 0;
volatile uint8_t global_session = DefaultSession;

volatile uint8_t Security_Service_Availability_Flag = Not_Available;
//uint8_t seed =50 ; // for example
/*Security Variables */
volatile uint32_t Sec_u32SeedValue = 0 ;
Security_Access_State Sec_State = Un_Secure ;
volatile uint32_t Oil_Pressure_var = 0x778899AA;
volatile uint32_t Oil_Temp_var = 0x5566;
ServiceInfo pos_Response;

ServiceInfo Control;
PduInfoType msg;
extern TIM_HandleTypeDef htim6;
volatile uint8_t flag_sub_fun;
uint8_t UDS_Tx_Confirm = 0;
PduInfoType* UDS_Struct;
PduInfoType* PduDataPTR;

uint8_t* seed = NULL;
uint8_t* key = NULL;

/***************************************************************Init**********************************************************************/


void UDS_Init(void)
{

}



void UDS_Tester_Presenter_Client(void)
{
	PduInfoTypePtr.Data[0]= 0x2; 	// TESTER_PRESENT_PCI
	PduInfoTypePtr.Data[1]= 0x3E; 	// TESTER_PRESENT_SID
	PduInfoTypePtr.Length = 2;		// TESTER_PRESENT_LENGTH
	//Sending Frame to Can TP
	CanTp_Transmit(0, &PduInfoTypePtr);
}



/***************************************************************Read**********************************************************************/

//Send Read Frame
void UDS_Read_Data_Client(DID did)
{
	Read_Data_Client.Data[SID] = 0x22; //SID of Read
	Read_Data_Client.Length = 4; //length of Read frame

	//check DID which Data
	if(did == Oil_Temp)
	{
		Read_Data_Client.Data[DID_1] = 0xF1;
		Read_Data_Client.Data[DID_2] = 0x3D;
	}
	else if (did == Oil_Pressure)
	{
		Read_Data_Client.Data[DID_1] = 0xF5;
		Read_Data_Client.Data[DID_2] = 0x3D;
	}
	//Send Frame to Can TP
	CanTp_Transmit(0, &Read_Data_Client);

	//For Debugging
	//HAL_UART_Transmit(&huart2, "\r\nRead Frame Client:", 50, HAL_MAX_DELAY);
	//sendHexArrayAsASCII(Read_Data_Client.Data, Read_Data_Client.Length);
	//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
}


//Recieve Read Frame
//before in call back: SID is checked
//void UDS_Read_Data_Server( PduInfoType* PduInfoTypePtr )
//{
//	ServiceInfo Read_Data_Server ;
//	Read_Data_Server.SID = 0x22;
//	Read_Data_Server.SUB_FUNC = -1;
//	//if DID --> Oil_Temp
//	if((PduInfoTypePtr->Data[1] == 0xF1) && (PduInfoTypePtr->Data[2] == 0x3D) )
//	{
//		Read_Data_Server.DID[0] = 0xF1;
//		Read_Data_Server.DID[1] = 0x3D;
//		Read_Data_Server.DID_Length = 2;
//		//For Debugging
//		//HAL_UART_Transmit(&huart2, "\r\nRead Frame Client DID:", 50, HAL_MAX_DELAY);
//		//sendHexArrayAsASCII(Read_Data_Server.DID, Read_Data_Server.DID_Length );
//		//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
//
//		Read_Data_Server.Data[SID] = 0x55; //value of Oil_Temp
//		Read_Data_Server.Data[Sub_F] = 0x66; //value of Oil_Temp
//		Read_Data_Server.Data_Length = 2;
//		//Send +ve responce
//		//UDS_Send_Pos_Res(Read_Data_Server);
//
//		//For Debugging
//		//HAL_UART_Transmit(&huart2, "\r\nRead Frame Server Data:", 50, HAL_MAX_DELAY);
//		//sendHexArrayAsASCII(Read_Data_Server.Data, Read_Data_Server.Data_Length);
//		//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
//
//	}//if DID --> Oil_Pressure
//	else if((PduInfoTypePtr->Data[1] == 0xF5) && (PduInfoTypePtr->Data[2] == 0x3D) )
//	{
//		Read_Data_Server.DID[0] = 0xF5;
//		Read_Data_Server.DID[1] = 0x3D;
//		Read_Data_Server.DID_Length = 2;
//		//For Debugging
//		//HAL_UART_Transmit(&huart2, "\r\nRead Frame Client DID:", 50, HAL_MAX_DELAY);
//		//sendHexArrayAsASCII(Read_Data_Server.DID, Read_Data_Server.DID_Length );
//		//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
//		Read_Data_Server.Data[SID] = 0x77; //value of Oil_Pressure
//		Read_Data_Server.Data[Sub_F] = 0x88; //value of Oil_Pressure
//		Read_Data_Server.Data[2] = 0x99; //value of Oil_Pressure
//		Read_Data_Server.Data[3] = 0xAA; //value of Oil_Pressure
//		Read_Data_Server.Data_Length = 4;
//		//Send +ve responce
//		//UDS_Send_Pos_Res(Read_Data_Server);
//
//		//For Debugging
//		//HAL_UART_Transmit(&huart2, "\r\nRead Frame Client DID:", 50, HAL_MAX_DELAY);
//		//sendHexArrayAsASCII(Read_Data_Server.DID, Read_Data_Server.DID_Length );
//		//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
//	}
//	else
//	{
//		//otherwize: send -ve responce
//		//UDS_Send_Neg_Res(Read_Data_Server.SID, NRC);
//	}
//
//}
//


/***************************************************************Write**********************************************************************/

void UDS_Write_Data_Client(DID did, uint32_t data)
{

	//PduInfoType Write_Data_Client;
	Write_Data_Client.Data[SID] = 0x2E; //SID of WDID

	//check DID which Data
	if(did == Oil_Temp)
	{
		Write_Data_Client.Data[DID_1] = 0xF1;
		Write_Data_Client.Data[DID_2] = 0x3D;

	    // Assuming data is 2 bytes
		Write_Data_Client.Data[Data_DID] = data >> 8; // Most significant byte of data
		Write_Data_Client.Data[Data_DID+1] = data & 0xFF;	   // Least significant byte of data

        Write_Data_Client.Length = 6; // SID + DID + Data



	}
	else if (did == Oil_Pressure)
	{
		Write_Data_Client.Data[DID_1] = 0xF5;
		Write_Data_Client.Data[DID_2] = 0x3D;

		// Assuming data is 4 bytes
		Write_Data_Client.Data[Data_DID] = (data >> 24) & 0xFF; // Most significant byte of data
		Write_Data_Client.Data[Data_DID+1] = (data >> 16) & 0xFF;
		Write_Data_Client.Data[Data_DID+2] = (data >> 8) & 0xFF;
		Write_Data_Client.Data[Data_DID+3] = data & 0xFF;		  // Least significant byte of data

		Write_Data_Client.Length = 8; // SID + DID + Data



	}

	//Sending Frame to Can TP
	CanTp_Transmit(0, &Write_Data_Client);

	//For Debugging
	//HAL_UART_Transmit(&huart2, "\r\nWrite Frame Client:", 50, HAL_MAX_DELAY);
//	sendHexArrayAsASCII(Write_Data_Client.Data,  Write_Data_Client.Length);
	//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
}





//void UDS_Write_Data_Server(PduInfoType* Received_data)
//{
//
//	uint16_t Oil_Temp;
//	uint32_t Oil_Pressure;
//	PduInfoType Write_Data_Server;
//
//
//	// did for Oil_Temp or we can check by the length of the received_data
//	if ((Received_data->Data[1] == 0xF1) && (Received_data->Data[2] == 0x3D))
//	{
//		// udate the value of oil temperature
//		Oil_Temp = ( (Received_data->Data[3] << 8) | (Received_data->Data[4]) );
//
//	}
//	// did for Oil_Pressure
//	else if (Received_data->Data[1] == 0xF5 && Received_data->Data[2] == 0x3D)
//	{
//		// udate the value of oil pressure
//		Oil_Pressure = ( (Received_data->Data[3] << 24) | (Received_data->Data[4] << 16) | (Received_data->Data[5] << 8) | (Received_data->Data[6]) );
//
//	}
//	else
//	{
//
//		// error in the did
//		// call the negative response function
//	}
//
//	// prepare positive response
//	Write_Data_Server.Data[SID] = 0x6E; //SID of WDID
//	Write_Data_Server.Data[Sub_F] = Received_data->Data[1];
//	Write_Data_Server.Data[2] = Received_data->Data[2];
//
//	// Calculating the data length
//	Write_Data_Server.Length = 3;
//
//
//	//Sending Frame to Can TP
//	//CanTp_Transmit(0, &Write_Data_Server);
//}
//

/***************************************************************Control Session**********************************************************************/
void UDS_Control_Session_Default(void)
{
	// init struct var  to use for send TP

	// fill the struct data
	Control_Session_Default.Data[SID] = Control_Service;
	Control_Session_Default.Data[Sub_F] = DefaultSession;
	Control_Session_Default.Length = 3;

	// send to can tp
	CanTp_Transmit(0, &Control_Session_Default);

	//For Debugging
	//HAL_UART_Transmit(&huart2, "\r\nControl_Session_Default:", 100, HAL_MAX_DELAY);
	//sendHexArrayAsASCII(Control_Session_Default.Data,  Control_Session_Default.Length);
	//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
}


void UDS_Control_Session_Extended(void)
{
	// init struct var  to use for send TP


	// fill the struct data
	Control_Session_Extended.Data[SID] = Control_Service;
	Control_Session_Extended.Data[Sub_F] = ExtendedSession;
	Control_Session_Extended.Length = 3;

	// send to can tp
	CanTp_Transmit(0, &Control_Session_Extended);

	//For Debugging
	//HAL_UART_Transmit(&huart2, "\r\Control_Session_Extended:", 100, HAL_MAX_DELAY);
	//sendHexArrayAsASCII(Control_Session_Extended.Data,  Control_Session_Extended.Length);
	//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
}



/***************************************************************Security**********************************************************************/


/***************************************************************Security**********************************************************************/

void UDS_Send_Security_Client(Sub_Fun sub_fun)
{
	Send_Security_Seed.Data[SID] = 0x27;//Security SID

	//Prepare the Key
	//Security_Key security_key;

	if(sub_fun == Seed)
	{
		Send_Security_Seed.Data[Sub_F] = 0x01;//Sub_Fun Seed
		Send_Security_Seed.Length = 3;
	}
	else if(sub_fun == Key)
	{
		Send_Security_Seed.Data[Sub_F] = 0x02;//Sub_Fun Key
		Send_Security_Seed.Length = 3+Seed_Key_Lenght;
		UART_ReceiveAndConvert(8,&Send_Security_Seed);
		//security_key.Seed = seed;
		//Prepare Key From Seed
		//uint8_t security_key_counter = Seed_Key_Lenght-1;
		//uint8_t Send_Security_Seed_counter = 2;
/*
		while(security_key_counter >= 0 )
		{
			//sendHexArrayAsASCII(security_key.Key,4);
			//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
			security_key.Key[security_key_counter] = security_key.Seed[security_key_counter] + Key_Code;//+0x05
			security_key_counter--;

			//Put New Key Calculated into Frame
			Send_Security_Seed.Data[Send_Security_Seed_counter] = security_key.Key[security_key_counter];
			Send_Security_Seed_counter++;
		}
*/
		//key = security_key.Key;

	}
	else
	{
		//Nothing
	}

	//Send Frame to Can TP
	CanTp_Transmit(0, &Send_Security_Seed);

	//For Debugging
	//HAL_UART_Transmit(&huart2, "\r\nUDS_Send_Security_Client", 50, HAL_MAX_DELAY);
	//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);
	//sendHexArrayAsASCII(Send_Security_Seed.Data, Send_Security_Seed.Length);
	//HAL_UART_Transmit(&huart2, "\r\n", 50, HAL_MAX_DELAY);


}




/***************************************************************CallBack Function**********************************************************************/
//
//void UDS_Client_Callback(uint32_t RxPduId,PduInfoType *Ptr)
//{
//	/*************************************************************************************/
//
//					//recieve response frame from tp
//					//check First Byte
//					// if(7F == arr[0])  // -ive  // print NRC
//					// else +ive
//					// read SID--> if read --> print data
//					// if write --> print success
//					// if control --> print current session
//					// if security --> security state
//
//	/*************************************************************************************/
//	//PduInfoType Glgl;
//	//CanTp_RxIndication ( 0, &Glgl);
//	//PduInfoType *Ptr1 = &Glgl;
//	//ptr->Data[0] = ptr->Data[0] - 0x40;
//
//	if ( Ptr ->Data[0] == 0x7f)
//	{
//		HAL_UART_Transmit(&huart2, (const uint8_t*) "\r\n Sorry Negative frame .\r\n",strlen("\r\n Sorry Negative frame .\r\n"), HAL_MAX_DELAY);
//	}
//
//	else {
//		/*  READ IDS Message */
//		if (Ptr->Data[0] == Read_Service)
//
//		{
//			if  ( Ptr->Data[1] == Oil_Temp_First_byte &&  Ptr->Data[2] == Oil_Temp_Second_byte )
//			{
//
//				HAL_UART_Transmit(&huart2, (const uint8_t*) "\r\n Oil Temp .\r\n", strlen("\r\n Oil Temp .\r\n"), HAL_MAX_DELAY);
//				sendHexArrayAsASCII((const uint8_t*)&Ptr->Data[3],2);
//
//			}
//
//			else if  ( Ptr->Data[1] == Oil_Pressure_First_byte &&  Ptr->Data[2] == Oil_Pressure_Second_byte  )
//			{
//				HAL_UART_Transmit(&huart2, (const uint8_t*) "\r\n Oil Pressure .\r\n", strlen("\r\n Oil Pressure .\r\n"), HAL_MAX_DELAY);
//				sendHexArrayAsASCII((const uint8_t*)&Ptr->Data[3],4);
//
//			}
//		}
//
//		/*  Write IDS Message */
//		else if (Ptr->Data[0] == Write_Service)
//
//		{
//
//			if ( Ptr->Data[1] == Oil_Temp_First_byte &&  Ptr->Data[2] == Oil_Temp_Second_byte )
//			{
//				HAL_UART_Transmit(&huart2, (const uint8_t*) "\r\n Oil Temperature Written successfully .\r\n", strlen("\r\n Oil Temperature Written successfully .\r\n"), HAL_MAX_DELAY);
//			}
//
//			else if  ( Ptr->Data[1] == Oil_Pressure_First_byte &&  Ptr->Data[2] == Oil_Pressure_Second_byte  )
//			{
//				HAL_UART_Transmit(&huart2, (const uint8_t*) "\r\n Oil Pressure Written successfully .\r\n", strlen("\r\n Oil Pressure Written successfully .\r\n"), HAL_MAX_DELAY);
//			}
//		}
//		/*  Control Service  Session */
//
//		else if (Ptr->Data[0] == Control_Service)
//
//		{
//
//			switch ( Ptr->Data[1] )
//			{
//
//			case Default_Session :
//				HAL_UART_Transmit(&huart2, (const uint8_t*) "\r\n It's Default Session ! .\r\n", strlen("\r\n It's Default Session ! .\r\n"), HAL_MAX_DELAY);
//				//UDS_Read_Data_Client(Frame_Info.DID);
//				break ;
//
//			case Extended_Session :
//				HAL_UART_Transmit(&huart2,(const uint8_t*) "\r\n It's Extended Session ! .\r\n", strlen("\r\n It's Extended Session ! .\r\n"), HAL_MAX_DELAY);
//				//UDS_Read_Data_Client(Frame_Info.DID);
//				break ;
//			}
//		}
//		/*  Security Service  */
//
//		else if (Ptr->Data[0] == Security_Service)
//
//		{
//
//			switch ( Ptr->Data[1] )
//			{
//
//			case Key :
//				HAL_UART_Transmit(&huart2, (const uint8_t*) "\r\n Key is Compatible ! .\r\n", strlen ( "\r\n Key is Compatible ! .\r\n"), HAL_MAX_DELAY);
//				break ;
//
//			case Seed :
//				HAL_UART_Transmit(&huart2, (const uint8_t*) "\r\n Seed is \r\n", strlen ("\r\n Seed is \r\n"), HAL_MAX_DELAY);
//				sendHexArrayAsASCII((const uint8_t*)&Ptr->Data[2],4);
//
//				break ;
//			}
//		}
//		/*  Tester_Representer_Service  */
//
//		else if (Ptr->Data[0] == Tester_Representer_Service)
//
//		{
//
//			HAL_UART_Transmit(&huart2,(const uint8_t*) "\r\n ECU Reseted ! .\r\n", strlen("\r\n ECU Reseted ! .\r\n"), HAL_MAX_DELAY);
//
//		}
//
//		else { /* no thing  */	}
//
//	}
//
//}
//

/***************************************************************Shared Functions**********************************************************************/

//Function to convert from Hex to ASCII and Displays Frame on UART
void sendHexArrayAsASCII(uint8_t* hexArray, uint16_t length)
{

    // Buffer to hold the ASCII representation (2 chars per byte + 1 for null terminator)
    char asciiBuffer[length * 2 + 1];
    char* pBuffer = asciiBuffer;

    // Convert each byte to its ASCII representation
    for (uint16_t i = 0; i < length; i++) {
        sprintf(pBuffer, "%02X", hexArray[i]);
        pBuffer += 2;
    }

    // Null-terminate the string
    *pBuffer = '\0';

    // Transmit the ASCII string over UART
    HAL_UART_Transmit(&huart2, (uint8_t*)asciiBuffer, strlen(asciiBuffer), HAL_MAX_DELAY);
}



void UART_ReceiveAndConvert(uint8_t RX_BUFFER_SIZE, PduInfoType* PduInfoType_Ptr)
{
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    uint8_t hexValue;
    uint8_t hexOutput[RX_BUFFER_SIZE / 2];
    uint16_t length = 0;

    // Receive ASCII characters from UART
    if (HAL_UART_Receive(&huart2, rxBuffer, RX_BUFFER_SIZE, HAL_MAX_DELAY) == HAL_OK) {
        // Calculate the length of the received string
        length = RX_BUFFER_SIZE/*strlen((char*)rxBuffer)*/;

        // Ensure the length is even (each hex byte is represented by 2 ASCII characters)
        if (length % 2 != 0) {
            // Send an error message
            char errorMsg[] = "Error: Odd number of characters received.\n";
            HAL_UART_Transmit(&huart2, (uint8_t*)errorMsg, strlen(errorMsg), HAL_MAX_DELAY);
            return;
        }

        // Process each pair of ASCII characters
        for (uint16_t i = 0; i < length; i += 2) {
            hexValue = (charToHex(rxBuffer[i]) << 4) | charToHex(rxBuffer[i + 1]);
            //update hexvalue into frame
            PduInfoType_Ptr->Data[(i / 2)+3] = hexValue;
            hexOutput[i / 2] = hexValue;
            // Optionally, send the hexadecimal values back via UART
            //HAL_UART_Transmit(&huart2, hexOutput, 1/*length / 2*/, HAL_MAX_DELAY);
        }
        sendHexArrayAsASCII(hexOutput,  length / 2);

    }
}

uint8_t charToHex(uint8_t ascii)
{
    if (ascii >= '0' && ascii <= '9') {
        return ascii - '0';
    } else if (ascii >= 'A' && ascii <= 'F') {
        return ascii - 'A' + 10;
    } else if (ascii >= 'a' && ascii <= 'f') {
        return ascii - 'a' + 10;
    } else {
        return 0; // Invalid character
    }
}

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





void UDS_Tester_Presenter_Server(void)//Khaled Waleed
{
	//globally in server:  there is a timer >= tout { reset timer + return to default (global ssesion flag in server) ask nour}
	//	reset_timer();
	pos_Response.SID = Tester_Representer_Service ;
	pos_Response.DID_Length=0;
	pos_Response.Data_Length = 0;
	pos_Response.SUB_FUNC = -1;
	UDS_Send_Pos_Res(&pos_Response);
}

void reset_timer(void)
{
	HAL_TIM_Base_Stop_IT(&htim6); // Stop Timer6
	TIM6->CNT = 0; // Reset Timer6 counter to 0
	//    HAL_TIM_Base_Start_IT(&htim6); // Start Timer6 again
}

void start_timer(void)
{
	//	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
}

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
		if(Received[2] == ExtendedSession){
			reset_timer();
			start_timer();
		}
	}
	else
	{
		UDS_Send_Neg_Res(Received[1], NRC);
	}
}

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
		reset_timer();
		if(global_session != DefaultSession){
			start_timer();
		}
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
			//			HAL_UART_Transmit(&huart2, (const uint8_t*)" u are in Tester_Representer_Service \r\n", 50, 100 );
			// call the fun of tester Representer
			UDS_Tester_Presenter_Server();
			//printf("void UDS_Tester_Present(void) \n");
			//			HAL_UART_Transmit(&huart2, (const uint8_t*)" void UDS_Tester_Present(void) \r\n", 50, 100 ); // delete this func after put your func

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
