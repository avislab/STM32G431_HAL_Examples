/*
 * uart.h
 *
 *    Author: Andrey Koryagin https://blog.avislab.com/
 */

#ifndef INC_CRSF_H_
#define INC_CRSF_H_

#include "main.h"

#define BUFFER_LENGTH	64
#define SYNC_BYTE		0xC8

#define FRAME_LENGTH_BYTE	1
#define FRAME_TYPE_BYTE		2

#define FRAME_TYPE_Battery_Sensor	0x08
#define FRAME_TYPE_Link_Statistics 	0x14
#define FRAME_TYPE_RC_Channels		0x16

typedef struct
{
    uint8_t     up_rssi_ant1;       // Uplink RSSI Antenna 1 (dBm * -1)
    uint8_t     up_rssi_ant2;       // Uplink RSSI Antenna 2 (dBm * -1)
    uint8_t     up_link_quality;    // Uplink Package success rate / Link quality (%)
    int8_t      up_snr;             // Uplink SNR (dB)
    uint8_t     active_antenna;     // number of currently best antenna
    uint8_t     rf_profile;         // enum {4fps = 0 , 50fps, 150fps}
    uint8_t     up_rf_power;        // enum {0mW = 0, 10mW, 25mW, 100mW,
                                    // 500mW, 1000mW, 2000mW, 250mW, 50mW}
    uint8_t     down_rssi;          // Downlink RSSI (dBm * -1)
    uint8_t     down_link_quality;  // Downlink Package success rate / Link quality (%)
    int8_t      down_snr;
} RC_LinkStatistics;

typedef struct
{
	unsigned int channel_01: 11;
	unsigned int channel_02: 11;
	unsigned int channel_03: 11;
	unsigned int channel_04: 11;
	unsigned int channel_05: 11;
	unsigned int channel_06: 11;
	unsigned int channel_07: 11;
	unsigned int channel_08: 11;
	unsigned int channel_09: 11;
    unsigned int channel_10: 11;
    unsigned int channel_11: 11;
    unsigned int channel_12: 11;
    unsigned int channel_13: 11;
    unsigned int channel_14: 11;
    unsigned int channel_15: 11;
    unsigned int channel_16: 11;
} __PACKED RC_Channels;

/*
typedef struct
{
    int16_t     voltage;
    int16_t     current;
    int32_t     capacity:24;
    int8_t      remaining;
} __PACKED RC_Battery_Sensors;
*/
typedef struct
{
	uint8_t		sync_byte;
	uint8_t		length;
	uint8_t		type;

	unsigned int     voltage:16;
	unsigned int     current:16;
	unsigned int     capacity:24;
	unsigned int     remaining:8;

    uint8_t     crc;
} __PACKED RC_BatterySensorsFrame;

typedef struct
{
	UART_HandleTypeDef huart;
	DMA_HandleTypeDef hdma_usart_tx;
	DMA_HandleTypeDef hdma_usart_rx;

	uint8_t RxChar[1];
	uint8_t RxBuf[BUFFER_LENGTH];
	uint8_t RxBufPos;

	RC_BatterySensorsFrame Frame;
	RC_LinkStatistics LinkStatistics;
	unsigned int Channals[16];

} CRSF;


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void CRSF_StartReceive();
void CRSF_StartSendFrame();
void CRSF_MakeFrame();
unsigned int CRSF_GetChannel(uint8_t ch);
void CRSF_IncCounter(void);
uint8_t CRSF_GetCounter(void);

#endif /* INC_CRSF_H_ */
