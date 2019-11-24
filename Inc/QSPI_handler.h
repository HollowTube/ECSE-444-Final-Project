#ifndef _QSPI_handler_H
#define _QSPI_handler_H
#include "arm_math.h"

#define SAMPLE_RATE 16000
#define SINE_NSAMPLES 2*SAMPLE_RATE

typedef struct _SineDefinition{
	float32_t amplitude; 
	float32_t freq;
	uint32_t base_addr;
} SineDefinition;

int store_sine(SineDefinition* sine);
int sine_gen_init(void);
int store_mixed(SineDefinition* sine1,SineDefinition* sine2);
#endif