#ifndef INC_APP_UDS_DIAG_CFG_H_
#define INC_APP_UDS_DIAG_CFG_H_

#include "stdint.h"



typedef enum {
	Main_Menu = 0,
	Control_Session_Menu,
	Read_Data_Menu,
	Write_Data_Menu,
	Security_Access_Menu,

} ClientMenu;


typedef enum {
	Control_Service = 0x10,
	Read_Service = 0x22,
	Security_Service = 0x27,
	Write_Service = 0x2E,
	Tester_Representer_Service = 0x3E,

} Services;


typedef enum {
	Oil_Temp = 0xF13D,
	Oil_Pressure = 0xF53D,
	Oil_Temp_First_byte=0xF1,
	Oil_Temp_Second_byte=0x3D,
	Oil_Pressure_First_byte=0xF5,
	Oil_Pressure_Second_byte=0x3D,
//	Oil_Pressure_Third_byte=0x00,
//	Oil_Pressure_forth_byte=0x00,


}DID;




typedef enum {
	Secure = 0x01,
	Un_Secure = 0x00,

}Security_Access_State;


typedef enum {
	DefaultSession = 0x01,
	ExtendedSession = 0x03,
	Seed = 0x01,
	Key = 0x02,

}Sub_Fun;



// Struct For:
//void UDS_Send_Pos_Res(void);
//void UDS_Send_Neg_Res(void);

typedef struct {
	Services SID;
	DID did;
	Sub_Fun sub_fun;
	int8_t* data;

}Frame_Info;


typedef struct
{
	uint8_t SID;
	int8_t SUB_FUNC;
	int8_t DID[2];
	uint8_t DID_Length;
	int8_t Data[4096];
	uint8_t Data_Length;
}ServiceInfo;

/*

typedef enum
{
	E_OK = 0x00,
	E_NOK
}Std_ReturnType;
*/

typedef enum
{
	Key_Code = 0x05,
	Seed_Key_Length = 4
}Security_Info;


typedef struct
{
	uint8_t* Seed;
	uint8_t* Key;
	Security_Access_State state;//enum
}Security_Key;

typedef enum {
	PCI = 0,
	SID = 1,
	DID_1 = 2,
	DID_2 = 3,
	Sub_F = 2,
	Data_DID = 4 ,
	Data_Sub_Fun = 3 ,

	Neg_Res_INDEX=1,
	SID_NR_INDEX=2,
	NRC_INDEX=3,

}Indices;

#endif
