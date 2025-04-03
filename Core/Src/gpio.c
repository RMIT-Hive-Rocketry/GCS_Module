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
