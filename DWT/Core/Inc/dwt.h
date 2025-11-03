/*
 * dwt.h
 */

#ifndef INC_DWT_H_
#define INC_DWT_H_

#include "main.h"

#define  DWT_CR				*(volatile uint32_t *)0xE0001000
#define  DWT_LAR			*(volatile uint32_t *)0xE0001FB0
#define  DWT_LAR_UNLOCK		(uint32_t)0xC5ACCE55
#define  DWT_CYCCNT			*(volatile uint32_t *)0xE0001004
#define  DEM_CR				*(volatile uint32_t *)0xE000EDFC
#define  DEM_CR_TRCENA		(1 << 24)
#define  DWT_CR_CYCCNTENA	(1 <<  0)

void DWTInit(void);
void DWTDelay_us(uint32_t us);
void DWTDelay_ms(uint32_t ms);

#endif /* INC_DWT_H_ */

