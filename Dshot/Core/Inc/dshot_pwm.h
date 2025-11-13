#ifndef INC_INPUT_CAPTURE_H_
#define INC_INPUT_CAPTURE_H_

/*
#define CPU_FCK	170

#define DSHOT150_TIM_TRIG 					CPU_FCK*6670/2000
#define DSHOT300_TIM_TRIG 					CPU_FCK*3330/2000
#define DSHOT600_TIM_TRIG 					CPU_FCK*1670/2000
*/

#define CPU_FCK	170000000

#define DSHOT150_TIM_TRIG 					CPU_FCK/1000000*6670/2000
#define DSHOT300_TIM_TRIG 					CPU_FCK/1000000*3330/2000
#define DSHOT600_TIM_TRIG 					CPU_FCK/1000000*1670/2000


#define DShot_150	0
#define DShot_300	1
#define DShot_600	2
#define DShot_Auto	3

typedef struct
{
	uint16_t Value; // 0...47 command, 48...2047 throttle
	uint8_t Type;
	uint32_t T1[16];
	uint32_t T2[16];
	uint16_t Dif[16];
	uint16_t TDif;
	uint8_t Restart_Counter;
} DShotStruct;


void DMA1_Channel1_IRQ (DMA_HandleTypeDef *hdma);
void Dshot_Init(uint8_t type);
void Dshot_DeInit(void);
uint32_t DShot_Get(void);

typedef struct
{
	uint32_t Value;
	uint32_t OverCaptureCounter;
} PWMInputStruct;

void PWMInput_Init(void);
void PWMInput_DeInit(void);
uint32_t PWMInput_Get(void);
void TIM3_IRQ(void);

#endif
