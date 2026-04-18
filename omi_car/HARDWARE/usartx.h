#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"

#define DATA_STK_SIZE   512 
#define DATA_TASK_PRIO  4

#define FRAME_HEADER      0X7B //Frame_header //帧头
#define FRAME_TAIL        0X7D //Frame_tail   //帧尾
#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

#define AutoCharge_HEADER      0X7C //Frame_header //帧头
#define AutoCharge_TAIL        0X7F //Frame_tail   //帧尾
#define AutoCharge_DATA_SIZE    8

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****用于存放陀螺仪加速度计三轴数据的结构体*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2个字节
	short Y_data; //2 bytes //2个字节
	short Z_data; //2 bytes //2个字节
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******串口发送数据的结构体*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1个字节
		short X_speed;	            //2 bytes //2个字节
		short Y_speed;              //2 bytes //2个字节
		short Z_speed;              //2 bytes //2个字节
		short Power_Voltage;        //2 bytes //2个字节
		Mpu6050_Data Accelerometer; //6 bytes //6个字节
		Mpu6050_Data Gyroscope;     //6 bytes //6个字节	
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Sensor_Str;
}SEND_DATA;

typedef struct _SEND_AutoCharge_DATA_  
{
	unsigned char buffer[AutoCharge_DATA_SIZE];
	struct _AutoCharge_Str_
	{
		unsigned char Frame_Header; //1 bytes //1个字节
		short Charging_Current;	    //2 bytes //2个字节
		unsigned char RED;          //1 bytes //1个字节
		unsigned char Charging;     //1 bytes //1个字节
		unsigned char yuliu;		//1 bytes //1个字节
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}AutoCharge_Str;
}SEND_AutoCharge_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1个字节
		float X_speed;	            //4 bytes //4个字节
		float Y_speed;              //4 bytes //4个字节
		float Z_speed;              //4 bytes //4个字节
		unsigned char Frame_Tail;   //1 bytes //1个字节
	}Control_Str;
}RECEIVE_DATA;


void data_task(void *pvParameters);
void data_transition(void);
void USART1_SEND(void);
void USART3_SEND(void);
void CAN_SEND(void);
void uart1_init(u32 bound);
void uart4_init(u32 bound);
void uart3_init(u32 bound);
int USART1_IRQHandler(void);
int UART4_IRQHandler(void);
int USART3_IRQHandler(void);

float XYZ_Target_Speed_transition(u8 High,u8 Low);
void usart1_send(u8 data);
void usart4_send(u8 data);
void usart3_send(u8 data);
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);
u8 Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char Mode);
u8 AT_Command_Capture(u8 uart_recv);
void _System_Reset_(u8 uart_recv);

#if Mec
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C,float Encoder_D);
#elif Omni
void Motion_analysis_transformation(float Encoder_A,float Encoder_B,float Encoder_C);
#endif


#endif

