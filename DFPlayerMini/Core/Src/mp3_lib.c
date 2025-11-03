/* MP3 LIB
 * V 0.1, Andrey Koryagin, Kremenchuk, Ukraine, 2016-2025
* Simple library to use the DFPlayer.
* It's can say a numbers from -9999 to 9999.
* Before using need to copy specially prepared mp3-files to SD-card.
* You can find a sample of this files in folder "SD_CARD_FILES"
* You can make same folders for different languages and choose language for playing.
* To choose a language (folder), please use MP3_set_folder function.
* You can free use and modify this lib as you like. Good luck.
*
 */
#include "main.h"
#include "mp3_lib.h"
#include "stdlib.h"

volatile uint8_t mp3_RXi;
volatile char mp3_RXc;
volatile char mp3_RX_Buf[mp3_RX_Buf_SIZE] = {'\0'};
volatile uint8_t mp3_folder = 1;
volatile uint8_t mp3_cmd_buf[10] = {0x7E, 0xFF, 0x06, 0x00, 0x01, 0x0, 0x0, 0x00, 0x00, 0xEF};
volatile uint8_t mp3_queue[MP3_QUEUE_LEN] = {MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE, MP3_NO_VALUE};
volatile int8_t mp3_queue_id = 0;
volatile uint8_t mp3_flag = 0;


extern UART_HandleTypeDef huart1;

/*
*/
void MP3_init(void)
{
    // Send first byte. First byte may be dropped
	uint8_t ch = ' ';
	HAL_UART_Transmit(&huart1, &ch, 1, 100);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&mp3_RXc, 1);

}

/* Clear receive data buffer.
 * DFPlayer sends messages when certain events.
 * This buffer is used to receive messages from the player.
*/
void MP3_clear_RXBuffer(void) {
	for (mp3_RXi=0; mp3_RXi<mp3_RX_Buf_SIZE; mp3_RXi++)
		mp3_RX_Buf[mp3_RXi] = '\0';
	mp3_RXi = 0;
}

/* Processing data received from a DFPlayer
 * We need to know the moment of the end of track playing.
 * Sets a flag when message received and the end of track playing is detected.
 * Other messages are ignored.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
/////	mp3_RXc = USART_ReceiveData(USART3);
	mp3_RX_Buf[mp3_RXi] = mp3_RXc;
	mp3_RXi++;

		if (mp3_RXc != 0xEF) { // End of DFPlayer message
			if (mp3_RXi > mp3_RX_Buf_SIZE-1) {
				MP3_clear_RXBuffer();
			}
		}
		else {
			//if (RX_BUF[3] == 0x3C) { // U-DISK finished playing tracks
			//if (RX_BUF[3] == 0x3E) { // FLASH finished playing tracks
			if (mp3_RX_Buf[3] == 0x3D) { // TF card finished playing tracks
				mp3_flag = 1;
			}
			MP3_clear_RXBuffer();
		}

	HAL_UART_Receive_IT(&huart1, (uint8_t *)&mp3_RXc, 1);

	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	//UART_Start_Receive_IT(&huart1, rx_buffer, BUFFER_SIZE);

	// Return to console received data
	//HAL_UART_Transmit(&huart1, rx_buffer, BUFFER_SIZE, HAL_MAX_DELAY);
}


/*
void USART3_IRQHandler(void)
{
    if ((USART3->SR & USART_FLAG_RXNE) != (u16)RESET)
	{
    	mp3_RXc = USART_ReceiveData(USART3);
    	mp3_RX_Buf[mp3_RXi] = mp3_RXc;
    	mp3_RXi++;

    		if (mp3_RXc != 0xEF) { // End of DFPlayer message
    			if (mp3_RXi > mp3_RX_Buf_SIZE-1) {
    				MP3_clear_RXBuffer();
    			}
    		}
    		else {
    			//if (RX_BUF[3] == 0x3C) { // U-DISK finished playing tracks
    			//if (RX_BUF[3] == 0x3E) { // FLASH finished playing tracks
    			if (mp3_RX_Buf[3] == 0x3D) { // TF card finished playing tracks
    				mp3_flag = 1;
    			}
    			MP3_clear_RXBuffer();
    		}
	}
}
*/

/* Calculate checksum
 */
uint16_t MP3_checksum (void) {
	uint16_t sum = 0;
	uint8_t i;
	for (i=1; i<7; i++) {
		sum += mp3_cmd_buf[i];
	}
	return -sum;
}

/* Send command to DFPlayer
 */
void MP3_send_cmd (uint8_t cmd, uint16_t high_arg, uint16_t low_arg) {
	//uint8_t i;
	uint16_t checksum;

	mp3_cmd_buf[3] = cmd;

	mp3_cmd_buf[5] = high_arg;
	mp3_cmd_buf[6] = low_arg;

	checksum = MP3_checksum();
	mp3_cmd_buf[7] = (uint8_t) ((checksum >> 8) & 0x00FF);
	mp3_cmd_buf[8] = (uint8_t) (checksum & 0x00FF);

	// Send command to UART3
	/*
	for (i=0; i<10; i++) {
        USART_SendData(USART3, mp3_cmd_buf[i]);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
        {
        }
    }*/

	HAL_UART_Transmit(&huart1, (const uint8_t *)&mp3_cmd_buf, 10, 100);

}

/* This function make a queue from simple mp3-files to say a long number.
 * Range -9999...9999
 * You can use prefix and suffix. It's a mp3-files which will play before and after number.
 */
void MP3_say(uint8_t prefix, int value, uint8_t suffix) {
	int i;
	int num;

	const int POW[4] = {1, 10, 100, 1000}; // Range -9999...9999

	// Erase MP3 Queue
	for (i=0; i<MP3_QUEUE_LEN; i++) {
		mp3_queue[i] = MP3_NO_VALUE;
	}

	// Fill MP3 Queue
	mp3_queue_id = -1;

	// Prefix
	if (prefix != MP3_NO_VALUE) {
		mp3_queue_id++;
		mp3_queue[mp3_queue_id] = prefix;
	}

	if (value < 0) {
		value = abs(value);
		mp3_queue_id++;
		mp3_queue[mp3_queue_id] = 250; // Minus
	}
	else {
		if (value == 0) {
			mp3_queue_id++;
			mp3_queue[mp3_queue_id] = 200;
		}
	}

	for (i = 3; i >= 0; i--) { // Range -9999...9999
		if (value > 19 ) {
			num = value / POW[i];
			value = value - num * POW[i];

			if (num > 0) {
				mp3_queue_id++;
				mp3_queue[mp3_queue_id] = (i+1)*10 + num;
				//mp3_queue_id++;
			}
		}
		else {
			if (value > 0) {
				mp3_queue_id++;
				mp3_queue[mp3_queue_id] = value;
				value = 0;
			}
		}
	}

	// Suffix
	if (suffix != MP3_NO_VALUE) {
		mp3_queue_id++;
		mp3_queue[mp3_queue_id] = suffix;
	}

	mp3_queue_id = 0;
	mp3_flag = 1; // Ready to play
}

/* QUEUE Processing.
 * This function handles the queue and starts playing the next file after the finish of file playing.
 */
void MP3_queue_processing(void) {
	// MP3 QUEUE Processing
	if ( (mp3_queue[mp3_queue_id] != MP3_NO_VALUE) & (mp3_queue_id < MP3_QUEUE_LEN) ) {
		if (mp3_flag == 1) {
			MP3_send_cmd(MP3_PLAY_FOLDER_FILE, mp3_folder, mp3_queue[mp3_queue_id]);
			mp3_queue_id++;
			MP3_clear_RXBuffer();
			mp3_flag = 0;
		}
	}
}

void MP3_set_folder (uint8_t folder) {
	mp3_folder = folder;
}
