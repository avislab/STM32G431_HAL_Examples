/*
 * console.c
 *
 *  Created on: Oct 10, 2025
 *      Author: andre
 */

#include "console.h"

extern UART_HandleTypeDef hlpuart1;

#ifdef SWV_CONSOLE
	int _write(int file, char *ptr, int len)
	{
		(void)file;
		int DataIdx;

		for (DataIdx = 0; DataIdx < len; DataIdx++)
		{
			ITM_SendChar(*ptr++);
		}
		return len;
	}
#endif

#ifdef LPUART_CONSOLE
	int _write(int fd, char *ptr, int len) {
		HAL_StatusTypeDef hstatus;

		if (fd == 1 || fd == 2) {
			hstatus = HAL_UART_Transmit(&hlpuart1, (uint8_t*) ptr, len, HAL_MAX_DELAY);
			if (hstatus == HAL_OK)
				return len;
			else
				return -1;
		}
		return -1;
	}
#endif
