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
#include "gpio.h"


//get stuff for DMA transfer for MEMR and

void ISA_init(ISA *isa,
				GPIO *gpio_data ,
				GPIO *gpio_addr ,
				GPIO *gpio_control,
				GPIO_TypeDef *data_port,
				GPIO_TypeDef *addr_port,
				GPIO_TypeDef *control_port,
				ISA_Control_TypeDef control,
				uint8_t error)
{
	isa->data_port = data_port; //we are pointing the address to the struct
	isa->address_port = addr_port;
	isa->control_state = control; //must be 0.
	isa->error_flag_ISA = error;

//mass initialisation of GPIOD
 GPIO_init_group(&gpio_data,
		 	 	 &data_port,
		 	 	 GPIO_MODER_GENERAL_PURPOSE_OUTPUT,
				 GPIO_OTYPER_PUSH,
				 GPIO_OSPEEDR_VERY_HIGH,
				 GPIO_PUPDRy_NO);

 //mass initialisation of GPIOE ->Address pins for 0x0000 to 0xFFFF
 GPIO_init_group(&gpio_addr,
		 	 	 &addr_port,
		 	 	 GPIO_MODER_GENERAL_PURPOSE_OUTPUT,
				 GPIO_OTYPER_PUSH,
				 GPIO_OSPEEDR_VERY_HIGH,
				 GPIO_PUPDRy_NO);
//**************Essential Control Flow Signals**********************************
 //BALE initialisation
 GPIO_init(gpio_control,
		 control_port,
		 GPIO_MODER_GENERAL_PURPOSE_OUTPUT,
		 GPIO_OTYPER_PUSH,
		 GPIO_OSPEEDR_VERY_HIGH,
		 GPIO_PUPDRy_NO,
		 0); //PG0




 //*****************************************************************************
 /*Initialisaton order of below for loop for control states
  * PG2 - IOR
  * PG3 - IOW
  * PG4 - SMEMR
  * PG5 - SMEMW
  * PG6 - MEMW
  * PG7 - MEMR
  */
 for(uint8_t i = 2; i<8; i++)
 {
	 GPIO_init(&gpio_control,
		 &control_port,
		 GPIO_MODER_GENERAL_PURPOSE_OUTPUT,
		 GPIO_OTYPER_PUSH,
		 GPIO_OSPEEDR_VERY_HIGH,
		 GPIO_PUPDRy_NO,
		 i); //PG2
 }


	//The ISA bus will be initialised here!
}

void ISA_DMA_init(DMA *ISA, DMA_Stream_TypeDef *DMA_Stream_RX, GPIO_TypeDef *port, uint8_t width, uint32_t source_address, uint32_t destination_buffer, uint16_t RX_BUFF_SIZE)
{

	ISA->stream = DMA_Stream_RX;
	ISA->port = port; //data port
	ISA->width = width;
	ISA->destination_buffer = destination_buffer;
	ISA->source_address = source_address;
	ISA->RX_BUFF_SIZE = RX_BUFF_SIZE;

//Configure the Address as well!
	ISA->stream->CR = 0; //unconfigures everything the chosen DMA stream
//this will be a peripheral to memory DMA transaction
	//source address
		ISA->stream->PAR = (uint32_t)&(ISA->port->IDR); //points the address where port->IDR is stored
	//destination address -> this may need to be changed
		ISA->stream->M0AR = ISA->destination_buffer;

	//this configures the size of the DMA transaction -> 1 half word only in this case as 1 ISA transcation will occur at a time
		ISA->stream->NDTR = (uint16_t)RX_BUFF_SIZE;
		/*
		 * In order of DMA stream configuration
		 * Channel select -> Channel 1 (doesn't matter for GPIO as much)
		 * Peripheral Flow Controller -> Turn off as GPIO packets from ISA will be fixed
		 * DMA Stream Priority -> Turn this up fairly high -> state control will be interrupt driven stream
		 * Configure FIFO usage -> for Peripheral to MEM in this case, it will not be necessary as this INPUT stream will purely be state control
		 * Stream Direction -> Peripheral to Memory
		 * Circular mode -> enable
		 * memory data width -> 16 bit
		 * peripheral data width -> 16 bit
		 */
		ISA->stream->CR |= (DMA_SxCR_CHSEL_0)
				| (~DMA_SxCR_PFCTRL)
				| (0x02<<DMA_SxCR_PL_Pos)
				| (0x00<<DMA_SxCR_DIR_Pos)
				| (DMA_SxCR_CIRC)
				| (0x01<<DMA_SxCR_PSIZE_Pos)
				| (0x01<<DMA_SxCR_MSIZE_Pos);

		ISA->stream->CR |= DMA_SxCR_EN; //enables the DMA stream for data RX on GPIOD->IDR to Mem

}


