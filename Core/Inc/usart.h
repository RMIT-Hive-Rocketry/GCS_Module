/*
 * usart.h
 *
 *  Created on: Apr 11, 2025
 *      Author: lucas
 */

#ifndef INC_USART_H_
#define INC_USART_H_


#include "stdint.h"
#include "stdbool.h"

typedef struct
{
	uint8_t id; //important to characterise the data/same as lora
	uint8_t data;
}USART_Packet;




typedef enum
{
	DATA_8_BITS,
	DATA_9_BITS
}WORD_LENGTH;

typedef enum
{
	DATA_8_BITS,
	DATA_9_BITS
}RECEIVER_ENABLE;

typedef enum
{
	DATA_8_BITS,
	DATA_9_BITS
}TRANSMITTER_ENABLE;

typedef enum
{
	DATA_8_BITS,
	DATA_9_BITS
}STOP_BITS;


#endif /* INC_USART_H_ */
