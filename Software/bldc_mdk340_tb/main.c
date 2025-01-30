/*
    Copyright 2016 Benjamin Vedder  benjamin@vedder.se

    This file is part of the VESC firmware.

    The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "app.h"
#include "mc_interface.h"
#include "mcpwm.h"
#include "mcpwm_foc.h"
#include "ledpwm.h"
#include "comm_usb.h"
#include "ledpwm.h"
#include "terminal.h"
#include "hw.h"
#include "app.h"
#include "packet.h"
#include "commands.h"
#include "timeout.h"
#include "comm_can.h"
#include "ws2811.h"
#include "led_external.h"
#include "encoder.h"
#include "servo_simple.h"
#include "utils.h"
#include "nrf_driver.h"
#include "rfhelp.h"
#include "mcpwm_foc.h"
#include "spi_sw.h"
#include "dma.h"
#include "can_my.h"
/*
 * Timers used:
 * TIM1: mcpwm
 * TIM2: mcpwm
 * TIM12: mcpwm
 * TIM8: mcpwm
 * TIM3: servo_dec/Encoder (HW_R2)/servo_simple
 * TIM4: WS2811/WS2812 LEDs/Encoder (other HW)
 *
 * DMA/stream   Device      Function
 * 1, 2         I2C1        Nunchuk, temp on rev 4.5f
 * 1, 7         I2C1        Nunchuk, temp on rev 4.5f
 * 1, 1         UART3       HW_R2
 * 1, 3         UART3       HW_R2
 * 2, 2         UART6       Other HW
 * 2, 7         UART6       Other HW
 * 2, 4         ADC         mcpwm
 * 1, 0         TIM4        WS2811/WS2812 LEDs CH1 (Ch 1)
 * 1, 3         TIM4        WS2811/WS2812 LEDs CH2 (Ch 2)
 *
 */


#define SEND_BUF_SIZE1 60
#define MAX_BUF_NUM_LINK 80
#define BUAD_MINE 115200
void UsartSend(uint8_t ch)
{
while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
USART_SendData(USART6, ch); 
}


void Usart_Init(int br_num)//-------串口
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig((GPIO_TypeDef *) GPIOC_BASE, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig((GPIO_TypeDef *) GPIOC_BASE, GPIO_PinSource7, GPIO_AF_USART6);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init((GPIO_TypeDef *) GPIOC_BASE, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init((GPIO_TypeDef *) GPIOC_BASE, &GPIO_InitStructure); 

   //UART6 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure); //初始化串口1	
  USART_Cmd(USART6, ENABLE);  //使能串口1 
	USART_ClearFlag(USART6, USART_FLAG_TC);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART6, ENABLE); 
}


void Data_Receive_Anl(u8 *data_buf,u8 num)
{ 
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;	
	if(*(data_buf+2)==0x66)
  {
		bldc.connect=1;
    bldc.loss_cnt=0;
		bldc.run_heart++;
		bldc.m_state=*(data_buf+4);
		switch(bldc.m_state){
			case 1:
				 bldc.set_pos=(float)((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))/10.;
			break;
			case 2:
				 bldc.set_spd=(float)((int16_t)(*(data_buf+5)<<8)|*(data_buf+6));
			break;
			case 3:
				 bldc.set_i=(float)((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))/10.;
			break;
		}

	}else if(*(data_buf+2)==0x68)
  {
		bldc.param.current_kp=	 (float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/100.;
		bldc.param.current_ki=	 (float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7));
		bldc.param.pos_kp=	 		 (float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/1000.;
		bldc.param.pos_kd=	 		 (float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/1000.;
		bldc.param.spd_kp=	     (float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/1000.;
		bldc.param.spd_kd=	     (float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/1000.;
		
  }
}

u8 RxBuffer[MAX_BUF_NUM_LINK];
u8 RxState = 0;
u8 RxBufferNum = 0;
u8 RxBufferCnt = 0;
u8 RxLen = 0;
static u8 _data_len = 0,_data_cnt = 0;
char uart_rx_cnt;
void USART6_Packed(u8 com_data)
{ 

		uart_rx_cnt++;
		if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<MAX_BUF_NUM_LINK)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
}



char SendBuff1[SEND_BUF_SIZE1];
int nrf_uart_cnt1=0;
void clear_uart(void)
{
int	i;
nrf_uart_cnt1=0;
for(i=0;i<SEND_BUF_SIZE1;i++)
	SendBuff1[i]=0;
}

void data_per_uart(void)
{
	char i;	char sum = 0;
	uint16_t _cnt=0,cnt_reg;
	int16_t _temp;
  cnt_reg=nrf_uart_cnt1;
	SendBuff1[nrf_uart_cnt1++]=0xAA;
	SendBuff1[nrf_uart_cnt1++]=0xAF;
	SendBuff1[nrf_uart_cnt1++]=0x66;
	SendBuff1[nrf_uart_cnt1++]=0;
	_temp=bldc.motor.i_all*100;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	_temp=bldc.motor.i_bus*100;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	_temp=bldc.motor.vbus*100;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	_temp=bldc.motor.pos*10;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	_temp=bldc.motor.spd;
	SendBuff1[nrf_uart_cnt1++]=BYTE1(_temp);
	SendBuff1[nrf_uart_cnt1++]=BYTE0(_temp);
	
	SendBuff1[cnt_reg+3] =(nrf_uart_cnt1-cnt_reg)-4;
		for( i=cnt_reg;i<nrf_uart_cnt1;i++)
	sum += SendBuff1[i];
	SendBuff1[nrf_uart_cnt1++] = sum;
}

static THD_WORKING_AREA(periodic_thread_wa, 1024*2);
static THD_WORKING_AREA(timer_thread_wa, 128);
char dma_send_cnt=0;
char tx_cnt=0;
static float timer_per=0;
float per_t=0.00125;
float round_check_dead=1;
float timer_now;
static THD_FUNCTION(periodic_thread, arg) {
    (void)arg;

    chRegSetThreadName("Main periodic");

    for (;;) {
			  timer_per+=per_t;
			  timer_now+=per_t;
			  #if !CAN_ENABLE
					if(bldc.param.cal_now!=1)//不在校准中
						can_my_tread(timer_per);
        #endif
				
				bldc.motor.iq_set=m_iq_set;
				bldc.motor.i_all=m_motor_state.i_abs;
				bldc.motor.i_bus=m_motor_state.i_bus;//用于判断外力
				bldc.motor.vbus=m_motor_state.v_bus;
				bldc.motor.iq=m_motor_state.iq;
				bldc.motor.id=m_motor_state.id;
				bldc.motor.duty=m_motor_state.duty_now;
				bldc.motor.pos=encoder_read_deg_flt();
				bldc.motor.spd=mcpwm_foc_get_rpm();
				if(bldc.motor.posr>360-round_check_dead&&bldc.motor.pos<round_check_dead)
					bldc.motor.round++;
				else if(bldc.motor.pos>360-round_check_dead&&bldc.motor.posr<round_check_dead)
					bldc.motor.round--;
				bldc.motor.posr=bldc.motor.pos;
				bldc.motor.pos_all[0]=bldc.motor.pos+360*bldc.motor.round;
				//减速组计算
				static float angle_lastm = 0.0f;
				if(bldc.param.round_div>1.5){
				float diff_f = utils_angle_difference(bldc.motor.pos, angle_lastm);
				angle_lastm = bldc.motor.pos;
				bldc.motor.pos_all[1] += diff_f / bldc.param.round_div;
				utils_norm_angle((float*)&bldc.motor.pos_all[1]);
				}
				
				if((bldc.motor.spd)>bldc.max_spd*bldc.param.round_div||(bldc.motor.spd)<-bldc.max_spd*bldc.param.round_div)
          bldc.err=1;					
				if(!bldc.connect&&bldc.m_state)
					bldc.m_state=0;
				if(bldc.m_statef!=0)
					bldc.m_state=bldc.m_statef;
				if(bldc.m_state){
					timeout_reset();
					switch(bldc.m_state)
					{
						case 1://pos
					    mc_interface_set_pid_pos(bldc.set_pos );
						break;
						case 2://spd
					    mc_interface_set_pid_speed(LIMIT(bldc.set_spd,-bldc.max_spd,bldc.max_spd));
						break;
						case 3://current
					    mc_interface_set_current(bldc.set_i);
						break;
					}
				}
				
				if(bldc.err){
					mc_interface_release_motor();
				}
				
			  if(timer_per>0.01){
        if (mc_interface_get_state() == MC_STATE_RUNNING) {
            ledpwm_set_intensity(LED_GREEN, 1.0f);
        } else {
            ledpwm_set_intensity(LED_GREEN, 0.2f);
        }
			
				//通讯判断
				bldc.loss_cnt+=timer_per;
				if(bldc.loss_cnt>0.25)
					bldc.connect=0;
				
				#if CAN_ENABLE
				if(DMA_GetFlagStatus(DMA2_Stream6,DMA_FLAG_TCIF6)!=RESET  && 1)
					{ 
						DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);		
						dma_send_cnt++;
						clear_uart();
						data_per_uart();
						USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);      
						MYDMA_Enable(DMA2_Stream6,SEND_BUF_SIZE1+2);    
					}	
				#endif

        mc_fault_code fault = mc_interface_get_fault();
        if (fault != FAULT_CODE_NONE) {
            for (int i = 0; i < (int)fault; i++) {
                ledpwm_set_intensity(LED_RED, 1.0f);
                chThdSleepMilliseconds(250);
                ledpwm_set_intensity(LED_RED, 0.0f);
                chThdSleepMilliseconds(250);
            }
            chThdSleepMilliseconds(500);
        } else {
            ledpwm_set_intensity(LED_RED, 0.0f);
        }

        if (mc_interface_get_state() == MC_STATE_DETECTING) {
            commands_send_rotor_pos(mcpwm_get_detect_pos());
        }

        disp_pos_mode display_mode = commands_get_disp_pos_mode();

        switch (display_mode) {
            case DISP_POS_MODE_ENCODER:
                commands_send_rotor_pos(encoder_read_deg());
                break;

            case DISP_POS_MODE_PID_POS:
                commands_send_rotor_pos(mc_interface_get_pid_pos_now());
                break;

            case DISP_POS_MODE_PID_POS_ERROR:
                commands_send_rotor_pos(utils_angle_difference(mc_interface_get_pid_pos_set(), mc_interface_get_pid_pos_now()));
                break;

            default:
                break;
        }

        if (mc_interface_get_configuration()->motor_type == MOTOR_TYPE_FOC) {
            switch (display_mode) {
                case DISP_POS_MODE_OBSERVER:
                    commands_send_rotor_pos(mcpwm_foc_get_phase_observer());
                    break;

                case DISP_POS_MODE_ENCODER_OBSERVER_ERROR:
                    commands_send_rotor_pos(utils_angle_difference(mcpwm_foc_get_phase_observer(), mcpwm_foc_get_phase_encoder()));
                    break;

                default:
                    break;
            }
        }
        timer_per=0;
			}
        chThdSleepMilliseconds(per_t*1000);//100Hz
    }
}

static THD_FUNCTION(timer_thread, arg) {
    (void)arg;

    chRegSetThreadName("msec_timer");

    for (;;) {
        packet_timerfunc();
        chThdSleepMilliseconds(1);
    }
}

//参数初始化在mc_interface.c
//编码器引脚配置在conf_general.h 中 #define AS5047_USE_HW_SPI_PINS		1
//HALL口 SPI接法  H1：CLK H2:MISO H3：CS VCC:MOSI
mc_configuration mcconf_init;
int main(void) {
    halInit();//串口初始化
    chSysInit();

#ifdef HW_HAS_DRV8313
    INIT_BR();
#endif

    chThdSleepMilliseconds(100);

    hw_init_gpio();
    LED_RED_OFF();
    LED_GREEN_OFF();

    conf_general_init();
    ledpwm_init();


    conf_general_read_mc_configuration(&mcconf_init);
    mc_interface_init(&mcconf_init);

    commands_init();
    comm_usb_init();

		app_configuration appconf;
    conf_general_read_app_configuration(&appconf);
    app_set_configuration(&appconf);//串口初始化
		
#if CAN_ENABLE
    comm_can_init();
#else
    bldc.param.board_id=appconf.controller_id;
    CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,5,0);//CAN初始化环回模式,波特率500Kbps 
		//while(1){				can_my_tread(0.01);		}
#endif
    
   

    timeout_init();
    timeout_configure(appconf.timeout_msec, appconf.timeout_brake_current);
#if CAN_ENABLE
		MYDMA_Config(DMA2_Stream6,DMA_Channel_5,(u32)&USART6->DR,(u32)SendBuff1,SEND_BUF_SIZE1+2,2);
	  USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);     
	  MYDMA_Enable(DMA2_Stream6,SEND_BUF_SIZE1+2); 
#endif   
    // Threads
    chThdCreateStatic(periodic_thread_wa, sizeof(periodic_thread_wa), NORMALPRIO, periodic_thread, NULL);
    chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

    static uint32_t periodic_thread_cnt = 0;
    for (;;) {
        chThdSleepMilliseconds(10);

        {
            periodic_thread_cnt %= FAULT_CODE_DRV;
            if (periodic_thread_cnt != FAULT_CODE_NONE) {
                for (int i = 0; i < (int)(periodic_thread_cnt); i++) {
                    ledpwm_set_intensity(LED_BOOT1, 0.725f);
                    chThdSleepMilliseconds(1000);
                    ledpwm_set_intensity(LED_BOOT1, 0.0f);
                    chThdSleepMilliseconds(1000);
                }
                periodic_thread_cnt += chVTGetSystemTimeX();
                chThdSleepMilliseconds(FAULT_CODE_DRV);
            } else {
                periodic_thread_cnt++;
                ledpwm_set_intensity(LED_BOOT1, 0.0f);
            }
        }
    }
}
