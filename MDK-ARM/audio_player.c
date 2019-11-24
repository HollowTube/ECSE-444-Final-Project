#include "audio_player.h"
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */
extern int flag;
/*
void play_sine(uint16_t wave[], DAC_HandleTypeDef hdac1){
	uint32_t time = 0;
	
	while(time<32000){
		if (flag == 1) {
			flag = 0;
			time++;
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, wave[time]);
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, wave[time]);
		}
	}
}	
*/


