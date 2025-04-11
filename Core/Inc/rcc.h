/*
 * rcc.h
 *
 *  Created on: Apr 8, 2025
 *      Author: lucas
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#define RCC_START_PERIPHERAL(bus, peripheral)		\
		RCC->bus##ENR |= (RCC_##bus##ENR_##peripheral##EN);    \
		RCC->bus##RSTR |= (RCC_##bus##RSTR_##peripheral##RST);  \
		__ASM("NOP");                                           \
		__ASM("NOP");                                           \
		RCC->bus##RSTR &= ~(RCC_##bus##RSTR_##peripheral##RST);

//APB2 -> TIMER 4
#define RCC_RESET_PERIPHERAL(bus, peripheral)               \
		RCC->##bus##RSTR |= (RCC_##bus##RSTR_##peripheral##RST);  \
		__ASM("NOP");                                             \
		__ASM("NOP");                                             \
		RCC->##bus##RSTR &= ~(RCC_##bus##RSTR_##peripheral##RST);

//TIMER1 -> APB1

#endif /* INC_RCC_H_ */
