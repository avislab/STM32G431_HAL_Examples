/*
 * console.h
 *
 *  Created on: Oct 10, 2025
 *      Author: andre
 */

#ifndef INC_CONSOLE_H_
#define INC_CONSOLE_H_

#include "main.h"
#include <stdio.h>

#define SWV_CONSOLE  // SWV_CONSOLE or UART1_CONSOLE

int _write(int fd, char *ptr, int len);

#endif /* INC_CONSOLE_H_ */
