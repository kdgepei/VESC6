#ifndef __CAN_H
#define __CAN_H	 
#include "stm32f4xx.h"    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//CAN驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0 
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	0		//0,不使能;1,使能.								    
										 							 				    
char CAN1_Mode_Init(char tsjw,char tbs2,char tbs1,uint16_t brp,char mode);//CAN初始化
 
char CAN1_Send_Msg(char* msg,char len);						//发送数据

char CAN1_Receive_Msg(char *buf);							//接收数据

void can_my_tread(float dt);
void data_can_anal(char buf[8]);
#endif

















