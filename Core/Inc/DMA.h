/*
 * DMA.h
 *
 *  Created on: Apr 8, 2025
 *      Author: lucas
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#include "stm32f439xx.h"
#include "stdbool.h"

#define DMA_PERIPHERAL_SIZE 0x3FF

#define DMA_ENABLE			0x01
//DMA1: 0x4002 6000 - 0x4002 63FF
//DMA2: 0x4002 6400 - 0x4002 67FF

#define DMA_CONFIG_DEFAULT \
	(DMA_Config){	\
	PFCTRL_DMA,	\
	CIRC_ENABLED, \
	PINC_FIXED,\
	MINC_FIXED,\
	PSIZE_16, \
	MSIZE_16,\
	PL_HIGH,\
	DMA_CHANNEL0,\
	DIR_PER_TO_MEM,\
	DBM_OFF,\
	NULL,\
	NULL,\
	NULL,\
}	\



typedef enum{
	PFCTRL_DMA,
	PFCTRL_FLOW
}PERIPHERAL_FLOW;

typedef enum{
	CIRC_DISABLED,
	CIRC_ENABLED
}CIRCULAR_BUFFER;

typedef enum{
	PINC_FIXED,
	PINC_INCREMENTED
}PERIPHERAL_INCREMENT;

typedef enum{
	MINC_FIXED,
	MINC_INCREMENTED
}MEMORY_INCREMENT;

typedef enum{
	PSIZE_8,
	PSIZE_16,
	PSIZE_32
}PERIPHERAL_SIZE;

typedef enum{
	MSIZE_8,
	MSIZE_16,
	MSIZE_32
}MEMORY_SIZE;

typedef enum{
	PL_LOW,
	PL_MEDIUM,
	PL_HIGH,
	PL_VERY_HIGH
}PRIORITY_LEVEL;

typedef enum{
	DMA_CHANNEL0,
	DMA_CHANNEL1,
	DMA_CHANNEL2,
	DMA_CHANNEL3,
	DMA_CHANNEL4,
	DMA_CHANNEL5,
	DMA_CHANNEL6,
	DMA_CHANNEL7
}DMA_CHANNEL_SELECT;

typedef enum{
	DIR_PER_TO_MEM,
	DIR_MEM_TO_PER,
	DIR_MEM_TO_MEM
}DMA_DIRECTION;

typedef enum{
	DBM_OFF,
	DBM_ON
}DOUBLE_BUFFER_MODE;



typedef struct {
	PERIPHERAL_FLOW flow;
	CIRCULAR_BUFFER circular;
	PERIPHERAL_INCREMENT pinc;
	MEMORY_INCREMENT minc;
	PERIPHERAL_SIZE psize;
	MEMORY_SIZE	msize;
	PRIORITY_LEVEL priority;
	DMA_CHANNEL_SELECT channel;
	DMA_DIRECTION direction;
	DOUBLE_BUFFER_MODE double_buffer;
	uint32_t source_address;
	uint32_t destination_address;
}DMA_Config;

typedef struct DMA{

	DMA_Stream_TypeDef *stream;
	DMA_Config config;
	void (*updateconfig)(struct DMA*, DMA_Config *);
}DMA;

DMA DMA_init(DMA_Stream_TypeDef *, DMA_Config *);
void DMA_updateconfig(DMA *,DMA_Config *);
void DMA_enable(DMA *);
void DMA_disable(DMA *);


#endif /* INC_DMA_H_ */
