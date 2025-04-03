/*
 * ISA.c
 *
 *  Created on: Apr 4, 2025
 *      Author: lucas
 */


#include "stm32f4xx.h"
#include <stdio.h>
#include "ISA.h"
#include "spi.h"


//get stuff for DMA transfer for MEMR and

void ISA_DMA_init(DMA *ISA, DMA_Stream_TypeDef *DMA_Stream, GPIO_TypeDef *port, uint8_t width, uint32_t source_address, uint32_t destination_buffer, uint16_t RX_BUFF_SIZE)
{

	ISA->stream = DMA_Stream;
}


