/*
 * ISA.c
 *
 *  Created on: Apr 9, 2025
 *      Author: lucas
 */


#include "ISA.h"
#include "stddef.h"

bool config_error_flag = false;

GPIO_Config sbhe_config = {
		GPIO_INPUT_MODE,
		GPIO_TYPE_PUSHPULL,
		GPIO_VERYHIGH_SPEED,
		GPIO_PUPD_NONE, //recheck timing diagram for ISA
		GPIO_AF0 //no alternative function
};

GPIO_Config BALE_config = {
		GPIO_INPUT_MODE,
		GPIO_TYPE_PUSHPULL,
		GPIO_VERYHIGH_SPEED,
		GPIO_PUPD_NONE, //recheck timing diagram for ISA
		GPIO_AF0 //no alternative function
};

GPIO_Config SYS_CLK_config = {
		GPIO_OUTPUT_MODE,
		GPIO_TYPE_PUSHPULL,
		GPIO_VERYHIGH_SPEED,
		GPIO_PUPD_NONE,
		GPIO_AF1 //timer 1
};


static void _ISA_BUS_init(GPIO_TypeDef *data_port,
		  	  	  	  	  GPIO_TypeDef *address_port,
						  GPIO_TypeDef *latch_address_port,
						  GPIO_TypeDef *control_port,
						  GPIO_TypeDef *SBHE_port,
						  GPIO_TypeDef *interrupt_port,
						  GPIO_TypeDef *dma_port,
						  //checks need to be done to ensure each GPIO type is separate from each other!
						  bool master, //will automatically be set to slave via hard-coding! this MUST not be set
						  TIM_TypeDef *system_clock_timer,
						  TIM_TypeDef *oscillator_clock_timer,
						  ISA_Config *config); //-> this is a flawed initialisation and is not FULLY modular!




//even though this ISA bus is fixed on the module, it is still worth making initialisation modular!
ISA_t ISA_BUS_init(GPIO_TypeDef *data_port,
				  GPIO_TypeDef *address_port,
				  GPIO_TypeDef *latch_address_port,
				  GPIO_TypeDef *control_port,
				  GPIO_TypeDef *SBHE_port,
				  GPIO_TypeDef *interrupt_port,
				  GPIO_TypeDef *dma_port,
				  //checks need to be done to ensure each GPIO type is separate from each other!
				  bool master, //will automatically be set to slave via hard-coding! this MUST not be set
				  TIM_TypeDef *system_clock_timer,
				  TIM_TypeDef *oscillator_clock_timer,
				  ISA_Config *config)
{
	ISA_t isa;

//make sure this is in the privately called function!**********************************
		uint32_t port_ISA[] = {data_port, address_port, latch_address_port, control_port, SBHE_port, interrupt_port, dma_port};
		uint32_t timer_ISA[] = {system_clock_timer, oscillator_clock_timer};
		isa.configuration_error = 0;

		isa.Data_port = data_port;
		isa.Address_port = address_port;
		isa.Address_latch_port = latch_address_port;
		isa.Control_port = control_port;
		isa.SBHE_port = SBHE_port;
		isa.Interrupt_port = interrupt_port;
		isa.DMA_Port = dma_port;
		isa.Master = master;
		isa.System_clock_master = system_clock_timer;
		isa.Oscillator_clock_master = oscillator_clock_timer;

		if((data_port = (void *)NULL))
		{
			isa.configuration_error |= 0x01; //indicating NULL on data
		}

		else if((address_port = (void *)NULL))
		{
			isa.configuration_error |= (0x01 << 1); //indicating NULL on address
			config_error_flag = true;
		}

		else if((latch_address_port = (void *)NULL))
		{
			isa.configuration_error |= (0x01 << 2); //indicating NULL on latch address
			config_error_flag = true;
		}

		else if((control_port = (void *)NULL))
		{
			isa.configuration_error |= (0x01 << 3); //indicating NULL on address
			config_error_flag = true;
		}

		else if((SBHE_port = (void *)NULL))
		{
			isa.configuration_error |= (0x01 << 4); //indicating NULL on latch address
			config_error_flag = true;
		}

		else if((interrupt_port = (void *)NULL))
		{
			isa.configuration_error |= (0x01 << 5); //indicating NULL on address
			config_error_flag = true;
		}

		else if((dma_port = (void *)NULL))
		{
			isa.configuration_error |= (0x01 << 6); //indicating NULL on latch address
			config_error_flag = true;
		}

		else if((system_clock_timer = (void *)NULL))
		{
			isa.configuration_error |= (0x01 << 7); //indicating NULL on latch address
			config_error_flag = true;
		}

		else if((oscillator_clock_timer = (void *)NULL))
		{
			isa.configuration_error |= (0x01 << 8); //indicating NULL on latch address
			config_error_flag = true;
		}

		else if((system_clock_timer == oscillator_clock_timer))
		{
			isa.configuration_error |= 0x4FFFF; //bit 18
			config_error_flag = true;


		}
		else //if there are no NULL pointers
		{
			isa.configuration_error &= ~0x2FF; //bits 8-0

		}

		for(uint8_t i = 0; i<=8 ; i++)
		{
			for(uint8_t j = 0; j<=8 ; j++)
			{
				if(port_ISA[i] == port_ISA[j] && i != j) //if the port address are the same while the indexes are different
				{
					isa.configuration_error |=  ((0x01<<(9+i)) | (0x01<<(9+j)));
					//will bit flip which two port addresses are identical to each other
					//maybe an error LED can be asserted
					break;
				}

			}
		}
		if(config_error_flag) //if error is NOT 0;
		{
			return isa; //incomplete configuration here!
		}
		else{
			config_error_flag = false;
			isa.configuration_error = 0;
		}
//*********************************************************************************
		update_ISA_t_configuration(&isa, config);
		//from this point onwards -> all GPIO and TIMER ports are unique and therefore applicable!
		return isa;
}

//Addresses are checked prior to GPIO and Timer configuration registers to actually being updated!

#ifndef DOXYGEN_PRIVATE
static void _ISA_BUS_init(GPIO_TypeDef *data_port,
		  	  	  	  	  GPIO_TypeDef *address_port,
						  GPIO_TypeDef *latch_address_port,
						  GPIO_TypeDef *control_port,
						  GPIO_TypeDef *SBHE_port,
						  GPIO_TypeDef *interrupt_port,
						  GPIO_TypeDef *dma_port,
						  //checks need to be done to ensure each GPIO type is separate from each other!
						  bool master, //will automatically be set to slave via hard-coding! this MUST not be set
						  TIM_TypeDef *system_clock_timer,
						  TIM_TypeDef *oscillator_clock_timer,
						  ISA_Config *config) //-> this is a flawed initialisation and is not FULLY modular!
{ //group initialisations of WHOLE GPIO ports!
	for(uint8_t i = 0; i<16; i++)
	{
		//for DATA bus!**************************************************
		data_port->MODER &= ~(config->data_mode)<<(i*2);
		data_port->MODER |= (config->data_mode)<<(i*2);
		//choose very high for the majority of this config!
		data_port->OSPEEDR &= ~(config->data_speed)<<(i*2);
		data_port->OSPEEDR |= (config->data_speed)<<(i*2);

		data_port->OTYPER &= ~(config->data_type)<<i;
		data_port->OTYPER |= (config->data_type)<<i;

		data_port->PUPDR &= ~(config->data_pupd)<<(i*2);
		data_port->PUPDR |= (config->data_pupd)<<(i*2);

		//for ADDRESS bus **********************************************
		address_port->MODER &= ~(config->address_mode)<<(i*2);
		address_port->MODER |= (config->address_mode)<<(i*2);
		//choose very high for the majority of this config!
		address_port->OSPEEDR &= ~(config->address_speed)<<(i*2);
		address_port->OSPEEDR |= (config->address_speed)<<(i*2);

		address_port->OTYPER &= ~(config->address_type)<<i;
		address_port->OTYPER |= (config->address_type)<<i;

		address_port->PUPDR &= ~(config->address_pupd)<<(i*2);
		address_port->PUPDR |= (config->address_pupd)<<(i*2);
	}

	for(uint8_t i = 6 ; i<10 ; i++) //we only need 16:24 bit addresses up to bit 19 to add extra 0xF....
	{
		latch_address_port->MODER &= ~(config->address_latch_mode)<<(i*2);
		latch_address_port->MODER |= (config->address_latch_mode)<<(i*2);
		//choose very high for the majority of this config!
		latch_address_port->OSPEEDR &= ~(config->address_latch_speed)<<(i*2);
		latch_address_port->OSPEEDR |= (config->address_latch_speed)<<(i*2);

		latch_address_port->OTYPER &= ~(config->address_latch_type)<<i;
		latch_address_port->OTYPER |= (config->address_latch_type)<<i;

		latch_address_port->PUPDR &= ~(config->address_latch_pupd)<<(i*2);
		latch_address_port->PUPDR |= (config->address_latch_pupd)<<(i*2);
	}

//control flow GPIO -> 2:7
	for(uint8_t i = 2; i < 8; i++)
	{
		control_port->MODER &= ~(config->control_mode)<<(i*2);
		control_port->MODER |= (config->control_mode)<<(i*2);
		//choose very high for the majority of this config!
		control_port->OSPEEDR &= ~(config->control_speed)<<(i*2);
		control_port->OSPEEDR |= (config->control_speed)<<(i*2);

		control_port->OTYPER &= ~(config->control_type)<<i;
		control_port->OTYPER |= (config->control_type)<<i;

		control_port->PUPDR &= ~(config->control_pupd)<<(i*2);
		control_port->PUPDR |= (config->control_pupd)<<(i*2);

	}

	GPIOpin_init(SBHE_port, 10, &sbhe_config); //Px10
	GPIOpin_init(control_port, 0, &BALE_config); //Px0

//TIMER Configuration here ***********************************************************************
	//timer configuration down here -> do NOT enable the timer (only in MASTER MODE) at the start of a transaction!
	GPIOpin_init(control_port, 0, &BALE_config); //Px0
	system_clock_timer->ARR &= ~TIM_ARR_ARR_Msk;
	system_clock_timer->PSC &= ~TIM_PSC_PSC_Msk;

		//-> prescaler is (45-1)
		//-> ARR register is 4-1
	GPIOpin_init(GPIOB, 1, &SYS_CLK_config); //Px1 -> add modularity to GPIO init for sysclk!
	system_clock_timer->ARR |= 45-1;
	system_clock_timer->PSC |= 4-1;
	//this gives a frequency of 8Mhz -> this should ONLY be enabled when ISA Module is in master mode!

	//enable later!
	//do not configure Oscillator yet!
}



#endif
void update_ISA_t_configuration(ISA_t *isa, ISA_Config *config)
{
	if(config == NULL)
	{
		//add default configuration here!
	}
	isa->config = *config; //the contents of config is stored in config struct in isa struct variable
	_ISA_BUS_init(isa->Data_port,
			isa->Address_port,
			isa->Address_latch_port,
			isa->Control_port,
			isa->SBHE_port,
			isa->Interrupt_port,
			isa->DMA_Port,
			isa->Master,
			isa->System_clock_master,
			isa->Oscillator_clock_master,
			config);
}


void EXTI0_IRQHandler(void)
{
	//create function to allow for BALE to be detected automatically and assocaited logic according to flowchart!



}
void Delay_us(uint32_t usec)
{
    // To avoid delaying for less than usec, always round up.;
		uint32_t i = 0;
    for(i = 0; i < (usec * 21); i++);

}

void Delay_ms( uint32_t msec )
{
    // To avoid delaying for less than usec, always round up.;
		uint32_t i = 0;
    for(i = 0; i < (msec * 21000); i++);

}
