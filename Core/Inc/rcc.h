/*
 * rcc.h
 *
 *  Created on: Apr 4, 2025
 *      Author: lucas
 */

#ifndef SRC_RCC_H_
#define SRC_RCC_H_

void configureRCC_APB1(void);
void configureRCC_APB2(void);
void configureRCC_AHB1(void);

//stuff that needs to be enabled!
/*
 * Interrupts
 * GPIOA|B|C|D|E|F|G|H|I|J|K (all of them lol)
 * SPI1
 * I2C2
 * UART4/USART3
 * TIM1
 * TIM4
 * DMA stream 1
 * DMA stream 2
 */

#endif /* SRC_RCC_H_ */
