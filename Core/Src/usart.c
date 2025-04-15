/*
 * usart.c
 *
 *  Created on: Apr 11, 2025
 *      Author: lucas
 */


#include "stm32f439xx.h"
#include "usart.h"
#include "stddef.h"





static void _USART_t_init(USART_t *usart, USART_CONFIG *config);

 USART_t USART_t_init(USART_TypeDef *interface, USART_CONFIG *config)
 {
	 if(interface == NULL)
	 {
		 return(USART_t){.interface = NULL};
	 }
	 USART_t usart = {.interface = interface};


	//update config here!
	 USART_updateConfig(&usart, config);

	 usart.send = USART_t_send;
	 usart.receive = USART_t_receive;
	 usart.sendString = USART_t_sendString;
	 usart.updateConfig = USART_updateConfig;
 }

#ifndef DOXYGEN_PRIVATE

void _USART_t_init(USART_t *usart, USART_CONFIG *config)
 {
	usart->interface->CR1 &= ~((config->sample)<<15
								| (config->word)<<12
								| (config->RX_Interrupt) << 5
								| (config->tx) << 3
								| (config->rx) << 2);

	usart->interface->CR1 |= ((config->sample)<<15
								| (config->word)<<12
								| (config->RX_Interrupt) << 5
								| (config->tx) << 3
								| (config->rx) << 2);



	usart->interface->CR2 &= ~config->stop<<12;
	usart->interface->CR2 |= config->stop<<12;


	usart->interface->BRR &=  ~0xFFFF;
	bool over8 = (config->stop)? 1 : 0 ; //over8 variable is 1 if true and 0 when false
	usart->interface->BRR |= APB1_CLOCK/(8*(2-over8) *config->baud);

	usart->interface->CR1 |= USART_CR1_UE;

 }


#endif


 void USART_updateConfig(USART_t *usart, USART_CONFIG *config)
 {
	 	if(config == NULL)
	 	{
	 		 config = &USART_CONFIG_DEFAULT;
	 	}

	 	usart->config = *config;

	 	_USART_t_init(usart, config);
 }


 void USART_t_send(USART_t *usart, uint8_t ch)
 {
	 while(!(usart->interface->SR & USART_SR_TXE));
	 usart->interface->DR = (ch & 0xFF);
 }


 void USART_t_sendString(USART_t *usart, uint8_t data, uint8_t size)
 {
	 if(size ==0){size = 1;}
	 while(size>0)
	 {
		 USART_t_send(&usart, data);
		 size--;
	 }
 }

 char USART_t_receive(USART_t *usart)
 {
	 while(!(usart->interface->SR & USART_SR_RXNE));
	 return (usart->interface->DR & 0xFF);
 }


 void USART_t_receiveString( USART_t *usart,  uint8_t str[])
 {
	 for(uint8_t i = 0; i< USART_MAX_BUFFER_SIZE_RX; i++)
	 {
		 str[i] = USART_t_receive(&usart);
	 }
 }




