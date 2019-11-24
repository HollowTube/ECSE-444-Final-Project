#include "stm32l475e_iot01_qspi.h"
#include "arm_math.h"
#include <stdlib.h>
#include "stm32l4xx_hal.h"
#include "QSPI_handler.h"
#include "main.h"

int Mix_Add_1 = 0x00000;
int Mix_Add_2 = 0x10000;
int Mix_Add_3 = 0x20000;
int Mix_Add_4 = 0x30000;

int sine_gen_init(void) {
	printf("Sine Gen Initializing\n\r");	
	if (BSP_QSPI_Init() != QSPI_OK) {
		printf("Sine Gen Initialize Failed\n\r");	
		return -1;
	}
	return 0;
}



int store_sine(SineDefinition* sine) {
	// memory allocation
	float32_t freq = ((sine->freq)*2*PI)/SAMPLE_RATE;
	float32_t offset = 128.0;
	
	BSP_QSPI_Erase_Block(Mix_Add_1);
	//BSP_QSPI_Erase_Block(Mix_Add_2);
	
	uint32_t wr_addr = (sine->base_addr);
	for(uint32_t n=0; n < 16000; n++) {
		float32_t val = (sine->amplitude)*arm_sin_f32(freq*n);
		printf("%3.3f ", val);
		uint8_t data = (val+1.0) * 127.5f;
		//uint8_t BSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
		if(BSP_QSPI_Write((uint8_t*) &data, wr_addr, sizeof(float32_t)) != QSPI_OK) return -1;
		wr_addr += 4;
	}
	return 0;
}

int store_mixed(SineDefinition* sine1, SineDefinition* sine2){
	float a[2][2] = {{0.3,0.7},{0.4,0.6}};
	
	float32_t freq1 = ((sine1->freq)*2*PI)/SAMPLE_RATE;
	float32_t freq2 = ((sine2->freq)*2*PI)/SAMPLE_RATE;

	BSP_QSPI_Erase_Block(Mix_Add_1);
	BSP_QSPI_Erase_Block(Mix_Add_2);
	BSP_QSPI_Erase_Block(Mix_Add_3);
	BSP_QSPI_Erase_Block(Mix_Add_4);
	
	uint32_t wr_addr1 = (sine1->base_addr);
	uint32_t wr_addr2 = (sine2->base_addr);
		for(uint32_t n=0; n < 32000; n++) {
		float32_t val_a = (sine1->amplitude)*arm_sin_f32(freq1*n);
		float32_t val_b = (sine2->amplitude)*arm_sin_f32(freq2*n);
			
		float32_t mix_a = a[0][0]*val_a + a[0][1]*val_b;
		float32_t mix_b = a[1][0]*val_a + a[1][1]*val_b;
		
		//uint8_t data_a = (mix_a+1.0) * 2047.5f;
		//uint8_t data_b = (mix_b+1.0) * 2047.5f;
			
		//uint8_t BSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
		if(BSP_QSPI_Write((uint8_t*) &mix_a, wr_addr1, sizeof(float32_t)) != QSPI_OK) return -1;
		if(BSP_QSPI_Write((uint8_t*) &mix_b, wr_addr2, sizeof(float32_t)) != QSPI_OK) return -1;
		wr_addr1 += 4;
		wr_addr2 += 4;
	}
	
}
