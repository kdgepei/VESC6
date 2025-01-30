#include "can_my.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ. @ref CAN_synchronisation_jump_width   ��Χ: ; CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   @ref CAN_time_quantum_in_bit_segment_2 ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   @refCAN_time_quantum_in_bit_segment_1  ��Χ: ;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
//������=Fpclk1/((tsjw+tbs1+tbs2+3)*brp);
//mode: @ref CAN_operating_mode ��Χ��CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Normal_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((1+6+7)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��;


char CAN1_Mode_Init(char tsjw,char tbs2,char tbs1,uint16_t brp,char mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		
		CAN_FilterInitStructure.CAN_FilterIdHigh   = (((uint32_t)bldc.param.board_id<<21)&0xffff0000)>>16;  
    CAN_FilterInitStructure.CAN_FilterIdLow   = (((uint32_t)bldc.param.board_id<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;  
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh  = 0xFFFF;   //????  
    CAN_FilterInitStructure.CAN_FilterMaskIdLow   = 0xFFFF;   //????  
		
		
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
		
#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   
 
#if CAN1_RX0_INT_ENABLE==1	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN1_RX0_IRQHandler(void)
{
	char buf[8];
  CanRxMsg RxMessage;
	int i=0;
  CAN_Receive(CAN1, 0, &RxMessage);
	for(i=0;i<8;i++)
	   buf[i]=RxMessage.Data[i];  
	
	if(RxMessage.DLC&&bldc.param.en_rx_can)//���յ�������
	{			
		data_can_anal(buf);
	}
}
#endif

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
char CAN1_Send_Msg(char* msg,char len)
{	
  char mbox;
  uint16_t i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=bldc.param.board_id;//0x12;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
//  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
//		i++;	//�ȴ����ͽ���
  if(i>=5)
		return 1;//��ʱ
  return 0;		

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
char CAN1_Receive_Msg(char *buf)
{		   		   
 	uint32_t i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
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
		if(tx_cnt>dt_can||can_test)//KEY0����,����һ������
		{ can_test=tx_cnt=0;
			data_can_per(0);
			can_flag=CAN1_Send_Msg(canbuf_tx,8);//����8���ֽ� 
			if(can_flag==0)
				res=0;//��ʾ����ʧ��
			else 
				res=1;//��ʾ���ͳɹ�								   
		}
		#if CAN1_RX0_INT_ENABLE==0
		rx_cnt=CAN1_Receive_Msg(canbuf_rx);
		if(rx_cnt&&bldc.param.en_rx_can)//���յ�������
		{			
 			data_can_anal(canbuf_rx);
		}
		#endif
		cnt++;
}
