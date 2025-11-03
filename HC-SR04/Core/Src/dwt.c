/*
 * swt.c
 */


#include "dwt.h"

static uint32_t SYSCClk;

void DWTInit(void)
{
	SYSCClk = (SystemCoreClock / 1000000);
	DWT_LAR |= DWT_LAR_UNLOCK;
	DEM_CR |= (uint32_t)DEM_CR_TRCENA;
	DWT_CYCCNT = (uint32_t)0u; // Reset counter
	DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
}

inline void DWTDelay_us(uint32_t us)
{
	uint32_t start = DWT_CYCCNT;
	while(((DWT_CYCCNT - start) / SYSCClk) < us) {};
}

inline void DWTDelay_ms(uint32_t ms)
{
	uint32_t start = DWT_CYCCNT;
	while(((DWT_CYCCNT - start) / SYSCClk) < (ms * 1000)) {};
}
