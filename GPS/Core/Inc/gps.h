/*
 * gps.h
 *
 *  Created on: Nov 14, 2025
 *      Author: andre
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#define BUFFER_SIZE 64
#define MAX_TOKENS	20

//https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm
typedef struct {
	int time;
	char status;
    float longitude;
    char ns;
    float latitude;
    char ew;
    float speed_kn;
    float course_d;
    int date;
    float mag_d;
	float mag_k;
	char mode;
	int crc;

} GPS_RMC_typedef;

//https://docs.novatel.com/OEM7/Content/Logs/GPVTG.htm
typedef struct {
    float course_d;
    char ti;
    float mag_d;
    char mi;
    float speed_kn;
    char kn;
    float speed_km;
    char km;
    char status;
	int crc;

} GPS_VTG_typedef;

//https://docs.novatel.com/OEM7/Content/Logs/GPZDA.htm
typedef struct {
	float time;
	int day;
	int month;
	int year;
	int utc;
	int crc;

} GPS_ZDA_typedef;


//https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
typedef struct {
	float time;
    float longitude;
    char ns;
    float latitude;
    char ew;
    int status;
    int satels;
    float precision;
    float altitude;
    float geo_diff;
    float last_update;
	int crc;

} GPS_GGA_typedef;

//https://docs.novatel.com/OEM7/Content/Logs/GPGLL.htm
typedef struct {
    float longitude;
    char ns;
    float latitude;
    char ew;
	float time;
	char state;
	char status;
	int crc;

} GPS_GLL_typedef;

//https://docs.novatel.com/OEM7/Content/Logs/GPGSA.htm
typedef struct {
	char mode;
	int mode_d;
	int SV1;
	int SV2;
	int SV3;
	int SV4;
	int SV5;
	int SV6;
	int SV7;
	int SV8;
	int SV9;
	int SV10;
	int SV11;
	int SV12;
	float PDOP;
	float HDOP;
	float VDOP;
	int crc;

} GPS_GSA_typedef;

//https://docs.novatel.com/OEM7/Content/Logs/GPGSV.htm
typedef struct {
	int messages;
	int message;
	int satels;
	int prn;
	int elev;
	int azimuth;
	int snr;
} GPS_GSV_typedef;


typedef struct{
	UART_HandleTypeDef *huart;
	uint8_t rx_ch[1];
	uint8_t rx_buffer[BUFFER_SIZE];
	uint8_t rx_buffer_index;

	GPS_RMC_typedef RMC;
	GPS_VTG_typedef VTG;
	GPS_ZDA_typedef ZDA;
	GPS_GGA_typedef GGA;
	GPS_GLL_typedef GLL;
	GPS_GSA_typedef GSA;
	GPS_GSV_typedef GSV;
} GPS_typedef;


void GPS_RxCpltCallback(UART_HandleTypeDef *huart, GPS_typedef* GPS);
void GPS_Parse(GPS_typedef* GPS);
void GPS_Init(GPS_typedef* GPS);

#endif /* INC_GPS_H_ */
