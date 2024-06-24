/*
 * CanNm.c
 *
 *  Created on: Jun 20, 2024
 *      Author: Abdelrahman
 */
#include "CanNm.h"

CanNm_Mode_t CanNm_Mode = BUS_SLEEP_MODE;                  // Represents the current mode of the CAN network management.
CanNm_WakeUpEvent_State_t WakeUp_Event = SLEEP;            // Indicates the current state of the wake-up event for the CAN network.
CanNm_NetworkMode_State_t NetworkModeState = NONE;         // Indicates the current state of the network mode.

CanNm_Timer_State_t NM_Timer = STOPPED;                    // State of the network management timer.
CanNm_Timer_State_t Repeat_Message_Timer = STOPPED;        // State of the repeat message timer.
CanNm_Timer_State_t Bus_Sleep_Timer = STOPPED;             // State of the bus sleep timer.

CanNm_Network_State_t Network_State = NETWORK_RELEASED;    // Current state of the CAN network.

uint8_t NM_MSG_RecFlag = 0;                                // Flag indicating whether a network management message has been received.
uint8_t NM_MSG_TranFlag = 0;                               // Flag indicating whether a network management message has been transmitted.
uint8_t Rep_MSG_Bit_State = 0;                             // State of the repeat message bit.
uint8_t Rep_MSG_State_Req = 0;                             // Request state for the repeat message.

void (*GlobalTxPTF)() = NULL;
void CanNm_Init(void)
{
	// Initialize the CAN network management to sleep mode and all related variables

	// Set CAN network mode to bus sleep mode
	CanNm_Mode = BUS_SLEEP_MODE;

	// Set wake-up event state to sleep
	WakeUp_Event = SLEEP;

	// Set network mode state to none
	NetworkModeState = NONE;

	// Initialize all timers to stopped state
	NM_Timer = STOPPED;
	Repeat_Message_Timer = STOPPED;
	Bus_Sleep_Timer = STOPPED;

	// Set network state to network released
	Network_State = NETWORK_RELEASED;

	// Initialize message flags and repeat message states to 0
	NM_MSG_RecFlag = 0;
	NM_MSG_TranFlag = 0;
	Rep_MSG_Bit_State = 0;
	Rep_MSG_State_Req = 0;

	// Enter Sleep Mode
}

Std_ReturnType CanNm_MainFunction(uint32_t RxPduId, PduInfoTRx* PduInfoPtr)
{
	Std_ReturnType Ret = E_OK;

	if(PduInfoPtr != NULL && RxPduId == 1)
	{
		switch(CanNm_Mode)
		{
		case BUS_SLEEP_MODE:
			if(WakeUp_Event != SLEEP)
			{
				// 1- Start NM TimeOut Timer
				NM_Timer = RUNNING;
				// Start NM Timer here

				// 2- Start REPEAT_MESSAGE Timer
				Repeat_Message_Timer = RUNNING; //timeout must be less than NM_Timer
				// Start REPEAT_MESSAGE Timer here

				// 3- Notify Nm_NetworkMode --> change mode to Networkmode
				CanNm_Mode = NETWORK_MODE;
				NetworkModeState = REPEAT_MESSAGE;
			}
			else
			{
				// Do nothing
			}
			break;
		case NETWORK_MODE:

			switch(NetworkModeState)
			{
			case REPEAT_MESSAGE:
				if(NM_Timer == TIMEOUT || NM_MSG_RecFlag == 1 || NM_MSG_TranFlag == 1 )
				{
					// Restart NM timer
				}
				// Send Repeated Msg to inform nodes that this ECU is awake
				//				CanIf_Transmit(RxPduId,PduInfoPtr); // we will not implement this
				/*
				 * To Do
				 * if there is a message corruption
				 * 1-Error Detection: ECU B (this ECU) detects that the critical message from ECU A is missing or corrupted.
				 * 2-Repeat Message Request (RMR): ECU B sends an RMR to CanNm, requesting ECU A to retransmit the critical message.
				 * 3-Transition to Repeat Message State: CanNm receives the RMR from ECU B and transitions into the Repeat Message State.
				 * 4-Retransmission: In the Repeat Message State, CanNm sends a request to ECU A, asking it to retransmit the critical message.
				 * 5-Message Retransmission: ECU A receives the request from CanNm and retransmits the critical message to ECU B.
				 * 6-Reception and Processing: ECU B receives the retransmitted critical message from ECU A. Now ECU B has the complete and accurate information it needs.
				 */

				if(Repeat_Message_Timer == TIMEOUT /*&& NM_Timer == RUNNING*/)
				{
					NM_Timer = RUNNING;
					//reset NM_Timer
					if(Network_State == NETWORK_RELEASED)
					{
						NetworkModeState = READY_SLEEP;

					}
					else if(Network_State == NETWORK_REQUESTED)
					{
						NetworkModeState = NORMAL_OPERATION;
					}
					else
					{
						// Do nothing
					}
				}
				break;

			case READY_SLEEP:
				if(NM_MSG_RecFlag == 1)
				{
					// Restart NM timer
					if(Rep_MSG_Bit_State){
						Rep_MSG_State_Req = 1;
					}
				}
				else
				{
					// Do nothing
				}
				if(NM_Timer == TIMEOUT)
				{
					NetworkModeState = NONE;
					CanNm_Mode = PREPARE_BUS_SLEEP_MODE;
					Bus_Sleep_Timer = RUNNING; // Start Bus Sleep Timer
				}
				else
				{
					// Do nothing
				}
				if(Rep_MSG_State_Req){
					NetworkModeState = REPEAT_MESSAGE;
					Repeat_Message_Timer = RUNNING;
					//reset rep msg timer
				}
				else{
					// Do nothing
				}
				/*
				 * To Do
				 * Add if (Repeat MSG Bit Or Repeat MSG State Requested)
				 * {
				 *  1- Start REPEAT_MESSAGE Timer
				 *	 Repeat_Message_Timer = RUNNING;
				 *	2- Notify Nm_NetworkMode --> change mode to Networkmode
				 *	CanNm_Mode = NETWORK_MODE;
				 *	NetworkModeState = REPEAT_MESSAGE;
				 * }
				 */
				break;

			case NORMAL_OPERATION:
				if(NM_MSG_RecFlag == 1 || NM_MSG_TranFlag == 1)
				{
					// Restart NM timer
					if(Rep_MSG_Bit_State){
						Rep_MSG_State_Req = 1;
					}
				}
				if(Network_State == NETWORK_RELEASED)
				{
					NetworkModeState = READY_SLEEP;
				}
				else if(Rep_MSG_State_Req){
					NetworkModeState = REPEAT_MESSAGE;
					Repeat_Message_Timer = RUNNING;
					NM_Timer = STOPPED;
					//reset rep msg timer
				}
				else if(NM_MSG_TranFlag == 1){
					//transmit
				}

				/*
				 * To Do
				 * Add if (Repeat MSG Bit Or Repeat MSG State Requested)
				 * {
				 *  1- Start REPEAT_MESSAGE Timer
				 *	 Repeat_Message_Timer = RUNNING;
				 *	2- Notify Nm_NetworkMode --> change mode to Networkmode
				 *	CanNm_Mode = NETWORK_MODE;
				 *	NetworkModeState = REPEAT_MESSAGE;
				 * }
				 */

				break;

			default:

				break;
			}
			break;

			case PREPARE_BUS_SLEEP_MODE:
				if(Bus_Sleep_Timer == TIMEOUT)
				{
					CanNm_Mode = BUS_SLEEP_MODE;
					Bus_Sleep_Timer = STOPPED;
				}
				if(NM_MSG_RecFlag == 1 || NM_MSG_TranFlag == 1)
				{
					CanNm_Mode = NETWORK_MODE;
					NetworkModeState = REPEAT_MESSAGE;
				}
				break;
			default:

				break;
		}
	}
	else
	{
		Ret = E_NOK;
	}
	return Ret;
}
//This should be passed to CanIf_setNmRxCallback()
Std_ReturnType CanNm_RxIndiaction(uint32_t RxPduId, PduInfoTRx* PduInfoPtr)
{
	Std_ReturnType Ret = E_OK;
	NM_MSG_RecFlag = 1;
	if(CanNm_Mode == BUS_SLEEP_MODE){
		WakeUp_Event = PASSIVE_WAKEUP;
	}
	return Ret;
}
//void CanNm_NetworkRequest(void)
//{
//	Network_State = NETWORK_REQUESTED;
//}
void CanNm_NetworkRelease(void)
{
	Network_State = NETWORK_RELEASED;
}
void CanNm_TimeOut(void)
{
	if(CanNm_Mode == NETWORK_MODE){
		if(NetworkModeState == REPEAT_MESSAGE){
			Repeat_Message_Timer = TIMEOUT;
		}
		else{
			NM_Timer = TIMEOUT;
		}
	}
	else if(CanNm_Mode == PREPARE_BUS_SLEEP_MODE){
		PREPARE_BUS_SLEEP_MODE = TIMEOUT;
	}
}
//This should be passed to CanIf_setNmTxCallback()
void CanNm_TxConfirmation(void)
{
	//send confirmation
	if(GlobalTxPTF != NULL){
		GlobalTxPTF();
	}
}

void CanNm_setTxCallback(void (*PTF)()){
	if(PTF != NULL){
		GlobalTxPTF = PTF;
	}
}

void CanNm_Transmit(){
	Network_State = NETWORK_REQUESTED;
	NM_MSG_TranFlag = 1;
	if(CanNm_Mode == BUS_SLEEP_MODE){
		WakeUp_Event = ACTIVE_WAKEUP;
	}
}
