/*
 * ISA.h
 *
 *  Created on: Apr 8, 2025
 *      Author: lucas
 */

#ifndef INC_ISA_H_
#define INC_ISA_H_

#include "gpiopin.h"
#include "DMA.h" //has bool in here

#define ISA_DATA_GPIO GPIOD
#define ISA_ADDR_GPIO GPIOE
#define ISA_ADDR_GPIO_MSB GPIOH
#define ISA_SHBE GPIOJ
#define ISA_CONTROL GPIOG






//ORDER OF OPERATIONS FOR ISA BUS COMMUNICATIONS!
/* For RX  (MEMR, IOR) MASTER (PCM3365) SIDE
 *	 BALE gets detected to indicate transaction start!
 *	 	*Note DMA should automatically capture RX Data -> obtaining data is not necessary
 *	 SBHE is asserted (low) to indicate 16 bit device (slave side)
 * 	Interrupt Handler with BALE! Handle address! ->switch case for DMA packet selection!
 * 	Read IDR register for GPIOE for address A0:15
 *
 * 	BALE unasserts latching address!
 * 	Command is asserted (MEMR or IOR) low
 * 		Interrupt triggered by control pins!
 * 	DMA captures data driven by the CPU
 * 	Slave asserts IOCS16 or MEMS16 to assert 16 bit transaction
 *
 * 	DMA complete should indicate the end the data transaction
 * 		subsequentially as well the command being unasserted
 * 		This should take around 360ns for 8.33MHz speed!!!
 */

typedef struct {
	GPIO_Mode data_mode, address_mode, address_latch_mode, control_mode, SBHE_mode, interrupt_mode, dma_mode;
	GPIO_Speed data_speed, address_speed, address_latch_speed, control_speed, SBHE_speed, interrupt_speed, dma_speed;
	GPIO_PUPD data_pupd, address_pupd, address_latch_pupd, control_pupd, SBHE_pupd, interrupt_pupd, dma_pupd;
	GPIO_Type data_type, address_type, address_latch_type, control_type, SBHE_type, interrupt_type, dma_type;



}ISA_Config;

typedef struct ISA{
	GPIO_TypeDef *Data_port;
	GPIO_TypeDef *Address_port;
	GPIO_TypeDef *Address_latch_port;
	GPIO_TypeDef *Control_port;
	GPIO_TypeDef *SBHE_port;
	GPIO_TypeDef *Interrupt_port;
	GPIO_TypeDef *DMA_Port;
	bool Master;
	//for master mode operations!
	TIM_TypeDef *System_clock_master;
	TIM_TypeDef *Oscillator_clock_master;
	uint32_t configuration_error;
	ISA_Config config;
	//maybe include an error flag!
}ISA_t;

ISA_t ISA_BUS_init(GPIO_TypeDef * /*Data port*/,
				  GPIO_TypeDef * /*address port*/,
				  GPIO_TypeDef * /*latch address port*/,
				  GPIO_TypeDef * /*control port*/,
				  GPIO_TypeDef * /*SBHE port*/,
				  GPIO_TypeDef * /*interrupt port*/,
				  GPIO_TypeDef * /*DMA port*/,
				  bool /*DMA port*/, //will automatically be set to slave via hard-coding! this MUST not be set
				  TIM_TypeDef */*System clock timer*/,
				  TIM_TypeDef */*oscillator clock*/,
				  ISA_Config *);



void update_ISA_t_configuration(ISA_t *, ISA_Config *);
void Delay_ms(uint32_t );
void Delay_us(uint32_t );



#endif /* INC_ISA_H_ */
