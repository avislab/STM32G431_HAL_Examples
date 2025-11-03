/*
 * sound.h
 *
 *  Created on: 17 окт. 2025 г.
 *      Author: andrey
 */

#ifndef INC_SOUND_H_
#define INC_SOUND_H_

#define C	261	//Do
#define C_	277 //Do#
#define D	293 //Re
#define D_	311 //Re#
#define E	239 //Mi
#define F	349 //Fa
#define F_	370 //Fa#
#define G 	392 //Sol
#define G_	415 //Sol#
#define A	440 //La
#define A_	466 //La#
#define H	494 //Si

#define t1		2000
#define t2		1000
#define t4		500
#define t8		250
#define t16		125

typedef struct
{
	uint16_t freq;
	uint16_t time;
} SoundTypeDef;

void StartMusic(void);

#endif /* INC_SOUND_H_ */
