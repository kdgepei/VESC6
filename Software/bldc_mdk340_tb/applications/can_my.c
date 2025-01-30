#include "can_my.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"

//CAN初始化
//tsjw:重新同步跳跃时间单元. @ref CAN_synchronisation_jump_width   范围: ; CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   @ref CAN_time_quantum_in_bit_segment_2 范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   @refCAN_time_quantum_in_bit_segment_1  范围: ;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//波特率=Fpclk1/((tsjw+tbs1+tbs2+3)*brp);
//mode: @ref CAN_operating_mode 范围：CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Normal_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((1+6+7)*6)=500Kbps
//返回值:0,初始化OK;
//    其他,初始化失败;


char CAN1_Mode_Init(char tsjw,char tbs2,char tbs1,uint16_t brp,char mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		
		CAN_FilterInitStructure.CAN_FilterIdHigh   = (((uint32_t)bldc.param.board_id<<21)&0xffff0000)>>16;  
    CAN_FilterInitStructure.CAN_FilterIdLow   = (((uint32_t)bldc.param.board_id<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;  
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh  = 0xFFFF;   //????  
    CAN_FilterInitStructure.CAN_FilterMaskIdLow   = 0xFFFF;   //????  
		
		
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE==1	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
	char buf[8];
  CanRxMsg RxMessage;
	int i=0;
  CAN_Receive(CAN1, 0, &RxMessage);
	for(i=0;i<8;i++)
	   buf[i]=RxMessage.Data[i];  
	
	if(RxMessage.DLC&&bldc.param.en_rx_can)//接收到有数据
	{			
		data_can_anal(buf);
	}
}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
char CAN1_Send_Msg(char* msg,char len)
{	
  char mbox;
  uint16_t i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=bldc.param.board_id;//0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
//  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
//		i++;	//等待发送结束
  if(i>=5)
		return 1;//超时
  return 0;		

}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
char CAN1_Receive_Msg(char *buf)
{		   		   
 	uint32_t i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}


char can_test=0;
char canbuf_tx[8];
char canbuf_rx[8];
char can_flag;
void data_can_per(char sel)
{

	char i;	char sum = 0;
	uint16_t _cnt=0,cnt_reg;
	int16_t _temp;
	canbuf_tx[_cnt++]=bldc.param.board_id;
	_temp=bldc.motor.i_all*100;
	canbuf_tx[_cnt++]=BYTE1(_temp);
	canbuf_tx[_cnt++]=BYTE0(_temp);
	_temp=bldc.motor.vbus*100;
	canbuf_tx[_cnt++]=BYTE1(_temp);
	canbuf_tx[_cnt++]=BYTE0(_temp);
	if(bldc.param.round_div>1.5)
		_temp=bldc.motor.pos_all[1]*10;
	else
		_temp=bldc.motor.pos*10;
	canbuf_tx[_cnt++]=BYTE1(_temp);
	canbuf_tx[_cnt++]=BYTE0(_temp);
}


void data_can_anal(char buf[8])
{

	char i;	char sum = 0;
	uint16_t _cnt=0,cnt_reg;
	int16_t _temp;
	bldc.connect=1;
	bldc.loss_cnt=0;
	bldc.run_heart++;
	bldc.m_state=buf[0];
	switch(bldc.m_state){
		case 1:
			 bldc.set_pos=(float)((int16_t)(buf[1]<<8)|buf[2])/10.;
		break;
		case 2:
			 bldc.set_spd=(float)((int16_t)(buf[1]<<8)|buf[2])/10.;
		break;
		case 3:
			 bldc.set_i=	(float)((int16_t)(buf[1]<<8)|buf[2])/10.;
		break;
	}
	bldc.m_state_param=buf[3];//param_set
	switch(bldc.m_state_param){
		case 1:
			bldc.param.pos_kp=	 		 (float)((int16_t)(buf[4]<<8)|buf[5])/1000.;
			bldc.param.pos_kd=	 		 (float)((int16_t)(buf[6]<<8)|buf[7])/1000.;
		break;
		case 2:
			bldc.param.spd_kp=	     (float)((int16_t)(buf[4]<<8)|buf[5])/1000.;
			bldc.param.spd_kd=	     (float)((int16_t)(buf[6]<<8)|buf[7])/1000.;
		break;
		case 3:
			bldc.param.current_kp=	 (float)((int16_t)(buf[4]<<8)|buf[5])/100.;
			bldc.param.current_ki=	 (float)((int16_t)(buf[6]<<8)|buf[7]);
		break;
	}
}

float dt_can=0.0025;
void can_my_tread(float dt)
{
char i=0;
static char cnt=0;
char canbuf[8];
char res;
char rx_cnt;
static float tx_cnt;
	  tx_cnt+=dt;
		if(tx_cnt>dt_can||can_test)//KEY0按下,发送一次数据
		{ can_test=tx_cnt=0;
			data_can_per(0);
			can_flag=CAN1_Send_Msg(canbuf_tx,8);//发送8个字节 
			if(can_flag==0)
				res=0;//提示发送失败
			else 
				res=1;//提示发送成功								   
		}
		#if CAN1_RX0_INT_ENABLE==0
		rx_cnt=CAN1_Receive_Msg(canbuf_rx);
		if(rx_cnt&&bldc.param.en_rx_can)//接收到有数据
		{			
 			data_can_anal(canbuf_rx);
		}
		#endif
		cnt++;
}
