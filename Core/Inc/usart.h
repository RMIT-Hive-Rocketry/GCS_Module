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

#define USART_MAX_BUFFER_SIZE_TX (0x20-1) + 3
#define USART_MAX_BUFFER_SIZE_RX 4
#define APB1_CLOCK 45000000
#define USART_CONFIG_DEFAULT 					\
	(USART_CONFIG){  							\
	.word = DATA_8_BITS,						\
	.rx = RECEIVER_ENABLED,						\
	.tx = TRANSMITTER_ENABLED,					\
	.stop = ONE_STOP_BIT,						\
	.sample = OVER8,							\
	.baud = 115200,							\
	.RX_Interrupt = true					\
}												\

typedef struct
{
	uint8_t id;
	uint8_t data_tx[USART_MAX_BUFFER_SIZE_TX];
	uint8_t data_rx[USART_MAX_BUFFER_SIZE_RX];
}USART_BUFFER;


typedef enum
{
	DATA_8_BITS,
	DATA_9_BITS
}WORD_LENGTH;

typedef enum
{
	RECEIVER_DISABLED,
	RECEIVER_ENABLED
}RECEIVER_ENABLE;

typedef enum
{
	TRANSMITTER_DISABLED,
	TRANSMITTER_ENABLED
}TRANSMITTER_ENABLE;


typedef enum{
	ONE_STOP_BIT,
	HALF_ONE_STOP_BIT,
	TWO_STOP_BIT,
	ONE_AND_HALF_STOP_BIT
}STOP_BITS;

typedef enum{
OVER16,
OVER8
}OVERSAMPLING_MODE;


typedef struct{
	WORD_LENGTH word;
	RECEIVER_ENABLE rx;
	TRANSMITTER_ENABLE tx;
	STOP_BITS stop;
	OVERSAMPLING_MODE sample;
	uint32_t baud;
	bool RX_Interrupt;
}USART_CONFIG;

typedef struct USART{
	USART_TypeDef *interface;
	GPIO_TypeDef *port;
	USART_CONFIG config;
	void (*send)(struct USART_t *, uint8_t);
	void (*sendString)(struct USART_t *, uint8_t , uint8_t );
	void(*receive)(struct USART_t *, uint8_t);
	void (*receiveString)(struct USART_t *,  uint8_t *);
	void(*updateConfig)(struct USART_t *, struct USART_CONFIG *);
}USART_t;

USART_t USART_t_init(USART_TypeDef *, USART_CONFIG *);
void USART_t_send(USART_t *, uint8_t);
void USART_t_sendString(USART_t *, uint8_t , uint8_t);
char USART_t_receive(USART_t *);
void USART_t_receiveString( USART_t *,  uint8_t[] );
void USART_t_updateConfig(USART_t *, USART_CONFIG *);

#endif /* INC_USART_H_ */
