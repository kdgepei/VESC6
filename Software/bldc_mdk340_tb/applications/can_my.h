#ifndef __CAN_H
#define __CAN_H	 
#include "stm32f4xx.h"    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	0		//0,��ʹ��;1,ʹ��.								    
										 							 				    
char CAN1_Mode_Init(char tsjw,char tbs2,char tbs1,uint16_t brp,char mode);//CAN��ʼ��
 
char CAN1_Send_Msg(char* msg,char len);						//��������

char CAN1_Receive_Msg(char *buf);							//��������

void can_my_tread(float dt);
void data_can_anal(char buf[8]);
#endif

















