/*
 * ISA.h
 *
 *  Created on: Apr 4, 2025
 *      Author: lucas
 */

#ifndef INC_ISA_H_
#define INC_ISA_H_
#include "stm32f4xx_hal.h"

#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256



#define ISA_DATA_GPIO GPIOD
#define ISA_ADDR_GPIO GPIOG

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

void ISA_init();

void ISA_DMA_init(DMA *, DMA_Stream_TypeDef *, GPIO_TypeDef *, uint8_t, uint32_t, uint32_t, uint16_t);

void example_function(void);
#endif /* INC_ISA_H_ */
