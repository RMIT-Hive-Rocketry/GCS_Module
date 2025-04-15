/*
 * DMA.c
 *
 *  Created on: Apr 8, 2025
 *      Author: lucas
 */


#include "DMA.h"
#include "stddef.h"


bool DMA_conf_flag = 0; //by default

static void _DMA_init(DMA_Stream_TypeDef *stream, DMA_Config *config);

DMA DMA_init(DMA_Stream_TypeDef *stream, DMA_Config *config)
{
	if(stream == NULL)
	{
		return (DMA){.stream = NULL}; //ensures DMA->stream is null
	}

	DMA dma;
	DMA_updateconfig(&stream,&config);
	dma.stream = stream; //defining address of where stream is stored at!
	dma.updateconfig = DMA_updateconfig;


	return dma;
}

void DMA_enable(DMA *dma)
{
	if(DMA_conf_flag) //if flag is set globally!
	dma->stream->CR |= DMA_ENABLE;
}
void DMA_disable(DMA *dma)
{
	dma->stream->CR &= ~DMA_ENABLE;
	//necessary for any configuration to occur
}


#ifndef DOXYGEN_PRIVATE //function cannot be called publically
static void _DMA_init(DMA_Stream_TypeDef *stream, DMA_Config *config)
{

		DMA_disable(&stream);
		__asm("NOP");
		//used for memory to peripheral or peripheral to memory
		stream->M0AR |= config->source_address;

	volatile uint32_t *mem_or_peripheral = (config->double_buffer && config->direction == 2) ? &stream->M1AR : &stream->PAR;
		*mem_or_peripheral &= ~config->destination_address;
		*mem_or_peripheral |= config->destination_address;

		//please note, the notation is flawed.
		//@param config->destination_address can either be a destination address in memory address space
		//or an input/out peripheral in address space. Called destination address for simplicities sake

		stream->CR &= ~(0x1 << 5);
		stream->CR |= (config->flow<<5);

		stream->CR &= ~(0b11 << 6);
		stream->CR |= (config->direction<<6);

		stream->CR &= ~(0x1 << 8);
		stream->CR |= (config->circular<<8);

		stream->CR &= ~(0x1 << 9);
		stream->CR |= (config->pinc<<9);

		stream->CR &= ~(0x1 << 10);
		stream->CR |= (config->minc<<10);

		stream->CR &= ~(0b11 << 11);
		stream->CR |= (config->psize<<11);

		stream->CR &= ~(0b11 << 13);
		stream->CR |= (config->msize<<13);

		stream->CR &= ~(0b11 << 16);
		stream->CR |= (config->priority<<16);

		stream->CR &= ~(0b11 << 18);
		stream->CR |= (config->double_buffer<<18);

		stream->CR &= ~(0b111 << 25);
		stream->CR |= (config->channel<<25);

		//enabling the protocol will come later!
		DMA_conf_flag = true;

}

#endif

DMA_updateconfig(DMA * dma, DMA_Config *config)
{
	if(config == NULL)
	{
		config = &DMA_CONFIG_DEFAULT;
		//no memory addresses are defined in default config to prevent memory leaks
		//config will NEED to be defined properly
	}

	dma->config = *config;
	_DMA_init(dma->stream, config);
}

