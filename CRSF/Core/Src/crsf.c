/*
 * uart.c
 *
 *    Author: Andrey Koryagin https://blog.avislab.com/
 */
#include "crsf.h"
#include "main.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;

uint8_t RC_RxChar[1];
uint8_t RC_RxBuf[BUFFER_LENGTH];
uint8_t RxBufPos;

RC_BatterySensorsFrame RC_Frame;
RC_LinkStatistics RC_Link;
unsigned int RC_Channals[16];

int TICKS_TO_US (int x)
{
	return ((x - 992) * 5 / 8 + 1500);
}

unsigned char crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};


uint8_t crc8(uint8_t * ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=2; i<=len; i++)
        crc = crc8tab[crc ^ ptr[i]];
    return crc;
}

void processLink_Statistics()
{
	RC_LinkStatistics *ls = (RC_LinkStatistics *)&RC_RxBuf[3];

	RC_Link.up_rssi_ant1 = ls->up_rssi_ant1;
	RC_Link.up_rssi_ant2 = ls->up_rssi_ant2;
	RC_Link.up_link_quality = ls->up_link_quality;
	RC_Link.up_snr = ls->up_snr;
	RC_Link.active_antenna = ls->active_antenna;
	RC_Link.rf_profile = ls->rf_profile;
	RC_Link.up_rf_power = ls->up_rf_power;

	RC_Link.down_rssi = ls->down_rssi;
	RC_Link.down_link_quality = ls->down_link_quality;
	RC_Link.down_snr = ls->down_snr;
}

void processLink_RC_Channels()
{
	RC_Channels *ch = (RC_Channels *)&RC_RxBuf[3];

	RC_Channals[0] = TICKS_TO_US(ch->channel_01);
	RC_Channals[1] = TICKS_TO_US(ch->channel_02);
	RC_Channals[2] = TICKS_TO_US(ch->channel_03);
	RC_Channals[3] = TICKS_TO_US(ch->channel_04);
	RC_Channals[4] = TICKS_TO_US(ch->channel_05);
	RC_Channals[5] = TICKS_TO_US(ch->channel_06);
	RC_Channals[6] = TICKS_TO_US(ch->channel_07);
	RC_Channals[7] = TICKS_TO_US(ch->channel_08);
	RC_Channals[8] = TICKS_TO_US(ch->channel_09);
	RC_Channals[9] = TICKS_TO_US(ch->channel_10);
	RC_Channals[10] = TICKS_TO_US(ch->channel_11);
	RC_Channals[11] = TICKS_TO_US(ch->channel_12);
	RC_Channals[12] = TICKS_TO_US(ch->channel_13);
	RC_Channals[13] = TICKS_TO_US(ch->channel_14);
	RC_Channals[14] = TICKS_TO_US(ch->channel_15);
	RC_Channals[15] = TICKS_TO_US(ch->channel_16);
}

void checkValidPakage()
{
	uint8_t crc;
	uint8_t crc_frame;

	if (RxBufPos > 5) {
		if (RC_RxBuf[0] == SYNC_BYTE) {
			if (RC_RxBuf[FRAME_LENGTH_BYTE]+2 == RxBufPos) {

				crc = crc8((uint8_t * )&RC_RxBuf, RC_RxBuf[FRAME_LENGTH_BYTE]);
				crc_frame = RC_RxBuf[RxBufPos-1];

				if (crc == crc_frame) {
					// Process Pakage
					if (RC_RxBuf[FRAME_TYPE_BYTE] == FRAME_TYPE_Link_Statistics) {
						processLink_Statistics();
					}

					if (RC_RxBuf[FRAME_TYPE_BYTE] == FRAME_TYPE_RC_Channels) {
						processLink_RC_Channels();
					}

					// Reset Buffer
					RxBufPos = 0;
				} else {
					//ErrorPakageCounter++;
				}

			}
		}
	} else {
		//ErrorPakageCounter++;
	}
}

uint8_t UART_Counter = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART2)
	{
		if ((RxBufPos == 0) && (RC_RxChar[0] != SYNC_BYTE )) {
			return;
		}

		RC_RxBuf[RxBufPos] = RC_RxChar[0];
		RxBufPos++;
		if (RxBufPos > BUFFER_LENGTH-1) {
			RxBufPos = 0;
		}

		checkValidPakage();

		UART_Counter = 0;
	}
}

void CRSF_StartReceive() {
	RxBufPos = 0;

	HAL_UART_DMAStop(&huart2);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RC_RxChar, 1);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


void CRSF_StartSendFrame() {
	huart2.gState = HAL_UART_STATE_READY;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *) &RC_Frame, 12);
}


void CRSF_MakeFrame() {
	uint8_t crc;
	uint16_t voltage;

	//voltage = ADC_GetVoltage();
	voltage = 3300;

	voltage = (voltage << 8) | (voltage >> 8);

	RC_Frame.sync_byte = SYNC_BYTE;
	RC_Frame.length = 10;
	RC_Frame.type = FRAME_TYPE_Battery_Sensor;

	RC_Frame.voltage = voltage;
	RC_Frame.current = 0;
	RC_Frame.capacity = 0;
	RC_Frame.remaining = 0;

	crc = crc8((uint8_t * )&RC_Frame, 10);
	RC_Frame.crc = crc;
}

unsigned int CRSF_GetChannel(uint8_t ch) {
	return RC_Channals[ch];
}

void CRSF_IncCounter(void) {
	if (UART_Counter < 100) {
		UART_Counter++;
	}
}

uint8_t CRSF_GetCounter(void) {
	return UART_Counter;
}
