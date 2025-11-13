#include "main.h"
#include "dshot_pwm.h"

extern TIM_HandleTypeDef htim3;

extern DMA_HandleTypeDef hdma_tim3_ch1;
extern DMA_HandleTypeDef hdma_tim3_ch2;

DShotStruct DShot;
PWMInputStruct PWMInput;

void DMA1_Channel1_IRQ (DMA_HandleTypeDef *hdma) {
	uint32_t flag_it = DMA1->ISR;
	uint32_t source_it = hdma->Instance->CCR;
	uint16_t value;
	uint16_t crc_d;
	uint16_t crc_p;
  	uint8_t i;
  	uint16_t dif_min = 0xFFFF;
  	uint8_t D;

	if ((0U != (flag_it & (DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1CU)))) && (0U != (source_it & DMA_IT_TC)))
	  {
	      if ((hdma->Instance->CCR & DMA_CCR_CIRC) == 0U)
	      {
	        // Disable the transfer complete and error interrupt
	        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TE | DMA_IT_TC);

	        // Change the DMA state
	        hdma->State = HAL_DMA_STATE_READY;
	      }
	      // Clear the transfer complete flag
	      __HAL_DMA_CLEAR_FLAG(hdma, (DMA_FLAG_TC1 << (hdma->ChannelIndex & 0x1CU)));

	      // Process Unlocked
	      __HAL_UNLOCK(hdma);



	  	TIM3->CNT = 0;

	  	// Check Packet
		for (i=0; i<16; i++) {
			DShot.Dif[i] = DShot.T1[i]-DShot.T2[i];
			if ((DShot.Type == DShot_Auto)) {
				if (DShot.Dif[i] < dif_min) {
					dif_min = DShot.Dif[i];
				}
			}
			if (DShot.Dif[i] > 64000) {
				DShot.Restart_Counter++;
				if (DShot.Restart_Counter < 100) {
					Dshot_DeInit();
					Dshot_Init(DShot.Type);
				} else {
					Dshot_DeInit();
				}
				return;
			}
		}

		// If DShot_Auto then autodetect 'TDif'
		if (DShot.Type == DShot_Auto) {
			DShot.TDif = dif_min + (dif_min / 2);
		}


	  	value = 0;
	  	crc_p = 0;
	  	for (i=0; i<16; i++) {
	  		D = (DShot.Dif[i] > DShot.TDif);

	  		// Calculate value
	  		if (i <= 11) {
	  			value = (value << 1) | D;
	  		} else {
	  			if (i > 11) { // Get CRC from packet
	  				crc_p = (crc_p << 1) | D;
	  			}
	  		}

	  	}

	  	// Calculate CRC
	  	crc_d = (value ^ (value >> 4) ^ (value >> 8)) & 0xF;
  		// Check CRC
  		if (crc_d == crc_p ) {
  			DShot.Value = (value >> 1) & 0x7FF;
  		}

	  }
}

void Dshot_Init(uint8_t type) {

	switch ( type )
	{
	    case DShot_150:
	    	DShot.TDif = DSHOT150_TIM_TRIG;
	    	DShot.Type = DShot_150;
	        break;
	    case DShot_300:
	    	DShot.TDif = DSHOT300_TIM_TRIG;
	    	DShot.Type = DShot_300;
	        break;
	    case DShot_600:
	    	DShot.TDif = DSHOT600_TIM_TRIG;
	    	DShot.Type = DShot_600;
	        break;
	    case DShot_Auto:
	    	DShot.TDif = 0;
	    	DShot.Type = DShot_Auto;
	        break;
	    default:
	    	DShot.TDif = 0;
	    	DShot.Type = DShot_Auto;
	    	break;

	}

	htim3.Init.Prescaler = 0;
	htim3.Init.Period = 53125;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_1, DShot.T1, 16);
	HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_2, DShot.T2, 16);
}

void Dshot_DeInit(void) {
	HAL_TIM_IC_Stop_DMA(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_DMA(&htim3, TIM_CHANNEL_2);
}

uint32_t DShot_Get(void) {
	return DShot.Value;
}


void PWMInput_Init(void) {
	PWMInput.Value = 0;

	htim3.Init.Prescaler = 64;
	htim3.Init.Period = 64000;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

void PWMInput_DeInit(void) {
	HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_1);
}

uint32_t PWMInput_Get(void) {
	return PWMInput.Value;
}

void TIM3_IRQ(void) {
	if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC1) != RESET)
	{
	  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) != RESET)
	  {
		  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1);
	  }
	  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
	}

	if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC2) != RESET)
	{
		if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC2) != RESET)
		{
			__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC2);

			PWMInput.Value = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

			TIM3->CNT = 0;
	        PWMInput.OverCaptureCounter = 0;
		}
		__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);
	 }

	if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
	{
		__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);

		if (PWMInput.OverCaptureCounter > 3) {
			PWMInput.Value = 0;
		} else {
			PWMInput.OverCaptureCounter++;
		}

	}
}
