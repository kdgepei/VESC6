#ifndef __DMA_H
#define	__DMA_H	   
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr,u8 pri);//配置DMAx_CHx
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);	//使能一次DMA传输		   
#endif






























