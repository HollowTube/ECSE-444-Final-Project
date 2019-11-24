#ifndef _AUDIO_PLAYER_H
#define _AUDIO_PLAYER_H
#include "arm_math.h"
#include "stm32l4xx_hal.h"
#include "main.h"

void play_sine(uint16_t wave[],DAC_HandleTypeDef hdac1);

#endif