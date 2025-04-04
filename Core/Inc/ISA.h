/*
 * ISA.h
 *
 *  Created on: Apr 4, 2025
 *      Author: lucas
 */

#ifndef INC_ISA_H_
#define INC_ISA_H_
#include "stm32f4xx_hal.h"
#include "gpio.h"

#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256



#define ISA_DATA_GPIO GPIOD
#define ISA_ADDR_GPIO GPIOE
#define ISA_SHBE GPIOJ
#define ISA_CONTROL GPIOG

//note for GPIO, the STM32F4 does not have dedicated DMA streams for GPIO

typedef struct DMA{
DMA_Stream_TypeDef *stream; //DMA 1
DMA_HandleTypeDef *channel;
GPIO_TypeDef *port;
uint8_t width;
uint32_t source_address;
uint32_t destination_buffer;
uint16_t RX_BUFF_SIZE;
//add more stuff when needed
}DMA;

typedef enum
{
	IDLE,//default state!
	IOW,
	IOR,
	MEMW,
	MEMR
}ISA_Control_TypeDef;

typedef struct ISA{

	GPIO_TypeDef *data_port;
	GPIO_TypeDef *address_port;
	ISA_Control_TypeDef control_state; //current action being taken
	uint8_t error_flag_ISA;
	//add more if need be!

}ISA;

void ISA_init(struct ISA *, GPIO *,GPIO *,GPIO *,GPIO_TypeDef *, GPIO_TypeDef *,  GPIO_TypeDef *, ISA_Control_TypeDef, uint8_t );

void ISA_DMA_init(DMA *, DMA_Stream_TypeDef *, GPIO_TypeDef *, uint8_t, uint32_t, uint32_t, uint16_t);

void example_function(void);
#endif /* INC_ISA_H_ */
