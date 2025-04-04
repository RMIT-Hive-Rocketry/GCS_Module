/*
 * gpio.c
 *
 *  Created on: Apr 4, 2025
 *      Author: lucas
 */

#include "gpio.h"
#include "stm32f4xx_hal.h"

void GPIO_init(GPIO *gpio, GPIO_TypeDef *port, uint8_t MODER, uint8_t OTYPER, uint8_t OSPEEDR, uint8_t PUPDR, uint8_t POS)
{
	gpio->port = port;
	gpio->MODER = MODER;
	gpio->OTYPER = OTYPER;
	gpio->OSPEEDR = OSPEEDR;
	gpio->PUPDR = PUPDR;
	gpio->POS = POS;

	gpio->port->MODER &= ~(MODER<<(2*POS)); //2* bc of 32 bit register
	gpio->port->MODER |= (MODER<<(2*POS));
	gpio->port->OTYPER &= ~(OTYPER<<POS);
	gpio->port->OTYPER |= (OTYPER<<POS);
	gpio->port->OSPEEDR &= ~(OSPEEDR<<(2*POS));
	gpio->port->OSPEEDR |= (OSPEEDR<<(2*POS));
	gpio->port->PUPDR &= ~(PUPDR<<(2*POS)); //clears the bit that we want to edit
	gpio->port->PUPDR |= (PUPDR<<(2*POS));

}

//for ISA bus mass initialisation!
void GPIO_init_group(GPIO *gpio,GPIO_TypeDef *isa_port, uint8_t MODER, uint8_t OTYPER, uint8_t OSPEEDR, uint8_t PUPDR)
{
	gpio->port = isa_port;
	gpio->MODER = MODER;
	gpio->OTYPER = OTYPER;
	gpio->OSPEEDR = OSPEEDR;
	gpio->PUPDR = PUPDR;

	//initialises the WHOLE GPIO port register! -> essential for GPIO parallel interface config
	for(uint8_t i = 0; i<16; i++){
		gpio->port->MODER &= ~(MODER<<(2*i)); //2* bc of 32 bit register
		gpio->port->MODER |= (MODER<<(2*i));
		gpio->port->OTYPER &= ~(OTYPER<<i);
		gpio->port->OTYPER |= (OTYPER<<i);
		gpio->port->OSPEEDR &= ~(OSPEEDR<<(2*i));
		gpio->port->OSPEEDR |= (OSPEEDR<<(2*i));
		gpio->port->PUPDR &= ~(PUPDR<<(2*i)); //clears the bit that we want to edit
		gpio->port->PUPDR |= (PUPDR<<(2*i));
	}
}
