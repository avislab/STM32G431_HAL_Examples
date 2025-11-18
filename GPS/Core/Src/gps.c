/*
 * gps.c
 *
 *  Created on: Nov 14, 2025
 *      Author: andre
 */

#include "main.h"
#include "gps.h"
#include "console.h"

#include <string.h>
#include <stdlib.h>

// Function to split a string without ignoring consecutive delimiters
void splitString(char *inputString, const char *delimiter, char *outputArray[], int *numTokens, int maxTokens) {
    char *token;
    char *saveptr; // For strtok_r
    int i = 0;

    // Make a copy of the input string as strtok_r modifies the string
    char tempString[strlen(inputString) + 1];
    strcpy(tempString, inputString);

    token = strtok_r(tempString, delimiter, &saveptr);

    while (token != NULL && i < maxTokens) {
        outputArray[i] = token; // Store pointer to the token
        i++;
        token = strtok_r(NULL, delimiter, &saveptr);
    }

    // Handle empty tokens resulting from consecutive delimiters
    // This part requires iterating through the original string and comparing with the tokens found
    // A more robust approach would be to manually parse the string

    // Manual parsing to preserve empty tokens:
    char *ptr = inputString;
    int currentTokenIndex = 0;
    while (*ptr != '\0' && currentTokenIndex < maxTokens) {
        char *start = ptr;
        // Find the next delimiter or end of string
        while (*ptr != *delimiter && *ptr != '\0') { // Assuming single-character delimiter
            ptr++;
        }

        // Allocate memory for the token and copy it (or just store pointers if inputString is mutable)
        // For simplicity, we'll just store pointers here, assuming inputString is handled appropriately
        outputArray[currentTokenIndex] = start;

        // Null-terminate the token if necessary (if modifying the original string)
        if (*ptr == *delimiter) {
            *ptr = '\0'; // Null-terminate the current token
            ptr++;       // Move past the delimiter
        }
        currentTokenIndex++;
    }
    *numTokens = currentTokenIndex;
}


void GPS_Parse(GPS_typedef* GPS) {
	char *tokens[MAX_TOKENS];
	int count;

	if(GPS->rx_buffer[0] == '$') {

		//https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm
		if (strncmp((char *)&GPS->rx_buffer, "$GPRMC", 6) == 0) {
			splitString((char *)&GPS->rx_buffer, ",", tokens, &count, MAX_TOKENS);
			GPS->RMC.time = atof(tokens[1]);
			GPS->RMC.status = tokens[2][0];
			GPS->RMC.longitude = atof(tokens[3]);
			GPS->RMC.ns = tokens[4][0];
			GPS->RMC.latitude = atof(tokens[5]);
			GPS->RMC.ew = tokens[6][0];
		    GPS->RMC.speed_kn = atof(tokens[7]);
		    GPS->RMC.course_d = atof(tokens[8]);
		    GPS->RMC.date = atoi(tokens[9]);
		    GPS->RMC.mag_d = atof(tokens[10]);
		    GPS->RMC.mag_k = atof(tokens[11]);
		    sscanf(tokens[count-1], "%c*%x", &GPS->RMC.mode, &GPS->RMC.crc);
		    return;
		}

		//https://docs.novatel.com/OEM7/Content/Logs/GPVTG.htm
		if (strncmp((char *)&GPS->rx_buffer, "$GPVTG", 6) == 0) {
			splitString((char *)&GPS->rx_buffer, ",", tokens, &count, MAX_TOKENS);
			GPS->VTG.course_d = atof(tokens[1]);
			GPS->VTG.ti = tokens[2][0];
			GPS->VTG.mag_d = atof(tokens[3]);
			GPS->VTG.mi = tokens[4][0];
			GPS->VTG.speed_kn = atof(tokens[5]);
			GPS->VTG.kn = tokens[6][0];
			GPS->VTG.speed_km = atof(tokens[7]);
			GPS->VTG.km = tokens[8][0];
		    sscanf(tokens[count-1], "%c*%x", &GPS->VTG.status, &GPS->VTG.crc);
			return;
		}

		//https://docs.novatel.com/OEM7/Content/Logs/GPZDA.htm
		if (strncmp((char *)&GPS->rx_buffer, "$GPZDA", 6) == 0) {
			splitString((char *)&GPS->rx_buffer, ",", tokens, &count, MAX_TOKENS);
			GPS->ZDA.time = atof(tokens[1]);
			GPS->ZDA.day = atoi(tokens[2]);
			GPS->ZDA.month = atoi(tokens[3]);
			GPS->ZDA.year = atoi(tokens[4]);
			GPS->ZDA.utc = atoi(tokens[5]);
			sscanf(tokens[count-1], "*%x", &GPS->ZDA.crc);
			return;
		}

		//https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm
		if (strncmp((char *)&GPS->rx_buffer, "$GPGGA", 6) == 0) {
			splitString((char *)&GPS->rx_buffer, ",", tokens, &count, MAX_TOKENS);
			GPS->GGA.time = atof(tokens[1]);
			GPS->GGA.longitude = atof(tokens[2]);
			GPS->GGA.ns = tokens[3][0];
			GPS->GGA.latitude = atof(tokens[4]);
			GPS->GGA.ew = tokens[5][0];
			GPS->GGA.status = atoi(tokens[6]);  //1 — GPS fix ( 0 = Данные не верны, 1 = Позиция зафиксирована, 2 = DGPS (повышенная точность))
			GPS->GGA.satels = atoi(tokens[7]);
			GPS->GGA.precision = atof(tokens[8]);
			GPS->GGA.altitude = atof(tokens[9]);
			GPS->GGA.geo_diff = atof(tokens[10]); // — Геоидальное различие — различие между земным эллипсоидом WGS-84 и уровнем моря(геоидом)
			GPS->GGA.last_update = atof(tokens[11]);
			sscanf(tokens[count-1], "*%x", &GPS->GGA.crc);
			return;
		}

		//https://docs.novatel.com/OEM7/Content/Logs/GPGLL.htm
		if (strncmp((char *)&GPS->rx_buffer, "$GPGLL", 6) == 0) {
			splitString((char *)&GPS->rx_buffer, ",", tokens, &count, MAX_TOKENS);
			GPS->GLL.longitude = atof(tokens[1]);
			GPS->GLL.ns = tokens[2][0];
			GPS->GLL.latitude = atof(tokens[3]);
			GPS->GLL.ew = tokens[4][0];
			GPS->GLL.time = atof(tokens[5]); // — время по гринвичу на момент определения местоположения (в формате ЧЧММСС.[доли секунд]).
			GPS->GLL.state = tokens[6][0];
			sscanf(tokens[count-1], "%c*%x", &GPS->GLL.status, &GPS->GLL.crc);
			return;
		}

		//https://docs.novatel.com/OEM7/Content/Logs/GPGSA.htm
		if (strncmp((char *)&GPS->rx_buffer, "$GPGSA", 6) == 0) {
			splitString((char *)&GPS->rx_buffer, ",", tokens, &count, MAX_TOKENS);
			GPS->GSA.mode = tokens[1][0];
			GPS->GSA.mode_d = atoi(tokens[2]);
			GPS->GSA.SV1 = atoi(tokens[3]);
			GPS->GSA.SV2 = atoi(tokens[4]);
			GPS->GSA.SV3 = atoi(tokens[5]);
			GPS->GSA.SV4 = atoi(tokens[6]);
			GPS->GSA.SV5 = atoi(tokens[7]);
			GPS->GSA.SV6 = atoi(tokens[8]);
			GPS->GSA.SV7 = atoi(tokens[9]);
			GPS->GSA.SV8 = atoi(tokens[10]);
			GPS->GSA.SV9 = atoi(tokens[11]);
			GPS->GSA.SV10 = atoi(tokens[12]);
			GPS->GSA.SV11 = atoi(tokens[13]);
			GPS->GSA.SV12 = atoi(tokens[14]);
			GPS->GSA.PDOP = atof(tokens[15]);
			GPS->GSA.HDOP = atof(tokens[16]);
			sscanf(tokens[count-1], "%f*%x", &GPS->GSA.VDOP, &GPS->GSA.crc);
			return;
		}

		//https://docs.novatel.com/OEM7/Content/Logs/GPGSV.htm
		if (strncmp((char *)&GPS->rx_buffer, "$GPGSV", 6) == 0) {
			splitString((char *)&GPS->rx_buffer, ",", tokens, &count, MAX_TOKENS);
			GPS->GSV.messages = atoi(tokens[1]);
			GPS->GSV.message = atoi(tokens[2]);
			GPS->GSV.satels = atoi(tokens[3]);
			GPS->GSV.prn = atoi(tokens[4]);
			GPS->GSV.elev = atoi(tokens[5]);
			GPS->GSV.azimuth = atoi(tokens[6]);
			GPS->GSV.snr = atoi(tokens[7]);
			return;
		}
	}
}


void GPS_RxCpltCallback(UART_HandleTypeDef *huart, GPS_typedef* GPS) {
	HAL_UART_Receive_IT(huart, GPS->rx_ch, 1);

	if ( (GPS->rx_ch[0] != '\n') && (GPS->rx_buffer_index < BUFFER_SIZE) ) {
		GPS->rx_buffer[GPS->rx_buffer_index] = GPS->rx_ch[0];
		GPS->rx_buffer_index++;
	} else {
		GPS_Parse(GPS);

		GPS->rx_buffer_index = 0;
		memset(GPS->rx_buffer, 0, BUFFER_SIZE);
	}
}


void GPS_Init(GPS_typedef* GPS) {
	UART_Start_Receive_IT(GPS->huart, GPS->rx_ch, 1);
}
