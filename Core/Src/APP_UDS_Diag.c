/*
 * UDS_APP_Diag.c
 *
 *  Created on: Jun 4, 2024
 *      Author: Omnia
 */

#include "APP_UDS_Diag.h"
typedef enum
{
	E_OK=0,
	E_NOK
}Std_ReturnType;
Std_ReturnType CanTp_Transmit(uint32_t TxPduId, PduInfoType* PduInfoPtr);


PduInfoType Read_Data_Client;
PduInfoType Write_Data_Client;
PduInfoType Control_Session_Default;
PduInfoType Send_Security_Seed;
PduInfoType Control_Session_Extended;
/***************************************************************Init**********************************************************************/


void UDS_Init(void)
{

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
		Write_Data_Client.Data[Data_DID] = (data >> 8) & 0xFF; // Most significant byte of data
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
	//CanTp_Transmit(0, &Write_Data_Client);

	//For Debugging
	//HAL_UART_Transmit(&huart2, "\r\nWrite Frame Client:", 50, HAL_MAX_DELAY);
	sendHexArrayAsASCII(Write_Data_Client.Data,  Write_Data_Client.Length);
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
		UART_ReceiveAndConvert(((Send_Security_Seed.Length)-2)*2,&Send_Security_Seed);
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
            PduInfoType_Ptr->Data[(i / 2)+2] = hexValue;
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
