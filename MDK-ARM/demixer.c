#include "demixer.h"
#include "stm32l475e_iot01_qspi.h"


int SINE_NSAMPLE = 32000;
void fast_ica(SineDefinition mix1, SineDefinition mix2){
	float cov[2][2] ={{0,0},{0,0}}; 
	float mean1;
	float mean2;

	float val1;
	float val2;
	float eigenD[2][2];

	float whitening[2][2];
	float deWhitening[2][2];

	float weight[2];
	
	int SAMPLES = SINE_NSAMPLES;

	mean(mix1,mix2, &mean1, &mean2);
	covariance(mix1,mix2,mean1,mean2,cov);
	eigen2by2(cov,eigenD,&val1,&val2);

	whiteningMx(eigenD,val1,val2,whitening);
	deWhiteningMx(eigenD,val1,val2,deWhitening);

	updateWeight(mix1,mix2,whitening,mean1,mean2,weight);

	constructBasisSet(mix1,mix2,weight,whitening);
}

void constructBasisSet(SineDefinition mix1, SineDefinition mix2, float weight[2], float whiteningMx[2][2]){

	// output address
	int UnMix_Add_1 = 0x40000;
	int UnMix_Add_2 = 0x50000;
	int UnMix_Add_3 = 0x60000;
	int UnMix_Add_4 = 0x70000;

	BSP_QSPI_Erase_Block(UnMix_Add_1);
	BSP_QSPI_Erase_Block(UnMix_Add_2);
	BSP_QSPI_Erase_Block(UnMix_Add_3);
	BSP_QSPI_Erase_Block(UnMix_Add_4);

	//construct basis set
	float32_t basis_set[2][2];
	basis_set[0][0] = weight[0]; 
	basis_set[0][1] = -weight[1]; 
	basis_set[1][0] = weight[1]; 
	basis_set[1][1] = weight[0]; 

	arm_matrix_instance_f32 basis_set_trans[2][2];

	arm_mat_trans_f32((arm_matrix_instance_f32*)basis_set, *basis_set_trans);

	//construct ICA filter
	float32_t ica_filter[2][2];
	arm_mat_mult_f32(*basis_set_trans, (arm_matrix_instance_f32*)whiteningMx, (arm_matrix_instance_f32*)ica_filter);

	//output original signals
	float32_t min1 = 1000.0f;
	float32_t min2 = 1000.0f;
	float32_t max1 = -1000.0f;
	float32_t max2 = -1000.0f;
	int j = 0; 
	for (j = 0; j < SINE_NSAMPLES; j++) {
		float32_t val1;
		float32_t val2;

		BSP_QSPI_Read((uint8_t*) &val1, (mix1.base_addr) + 4*j, 4);
		BSP_QSPI_Read((uint8_t*) &val2, (mix2.base_addr) + 4*j, 4);

		float32_t outval1, outval2;
		outval1 = ica_filter[1][0] * val1 + ica_filter[1][1] * val2;
		outval2 = ica_filter[0][0] * val1 + ica_filter[0][1] * val2;
		
		if (outval1 < min1){
			min1 = outval1;
		}
		if (outval1 > max1){
			max1 = outval1;
		}
		if (outval2 < min2){
			min2 = outval2;
		}
		if (outval2 > max2){
			max2 = outval2;
		}

		float32_t out1 = 4095.0f *(outval1 + min1)/ (max1 - min1);
		float32_t out2 = 4095.0f *(outval2 + min2)/ (max2 - min2);

		BSP_QSPI_Write((uint8_t*)&out1, (UnMix_Add_1) + 4*j, 4);
		BSP_QSPI_Write((uint8_t*)&out2, (UnMix_Add_3) + 4*j, 4);
	}
}

void eigen2by2(float cov[2][2], float eigenD[2][2], float* val1, float* val2){

	float det = cov[0][0]*cov[1][1] - cov[1][0]*cov[0][1];

	float b = cov[0][0]+cov[1][1];
	float32_t root = 0;
	arm_sqrt_f32(b*b-4*det, &root);

	*val1 = (b+root)/2;
	*val2 = (b-root)/2;

	eigenD[0][0] = 1.0; 
	eigenD[1][0] = - (cov[0][0] - *val1)/ cov[0][1] ;

	eigenD[0][1] = 1.0;
	eigenD[1][1] = -  (cov[0][0] - *val2)/ cov[0][1];

	float eigenDet = eigenD[0][0] * eigenD[1][1] - eigenD[1][0] * eigenD[0][1];
	float sqrtDet = sqrt(eigenDet); 

	eigenD[0][0] /=  sqrtDet; 
	eigenD[0][1] /=  sqrtDet; 
	eigenD[1][0] /=  sqrtDet; 
	eigenD[1][1] /=  sqrtDet; 

}

void covariance(SineDefinition mix1, SineDefinition mix2, float mean1, float mean2, float cov[2][2]){
	float32_t val1;
	float32_t val2;
	for(int i = 0; i<SINE_NSAMPLES; i++){
		BSP_QSPI_Read((uint8_t*) &val1, (mix1.base_addr) + 4*i, 4);
		BSP_QSPI_Read((uint8_t*) &val2, (mix2.base_addr) + 4*i, 4);
		val1 = val1-mean1;
		val2 = val2-mean2;
		cov[0][0] += val1*val1;
		cov[0][1] += val1*val2;
		cov[1][0] += val1*val2;
		cov[1][1] += val2*val2;
	}
		cov[0][0] /=  (SINE_NSAMPLES - 1);
		cov[0][1] /=  (SINE_NSAMPLES - 1);
		cov[1][0] /=  (SINE_NSAMPLES - 1);
		cov[1][1] /=  (SINE_NSAMPLES - 1);
}


void mean(SineDefinition mix1, SineDefinition mix2, float* mean1, float* mean2){
	float32_t val1 = 0;
	float32_t val2 = 0;

	float32_t sum1 = 0;
	float32_t sum2 = 0;

	for(int i = 0; i<32000; i++){
		BSP_QSPI_Read((uint8_t*) &val1, (mix1.base_addr) + 4*i, 4);
		BSP_QSPI_Read((uint8_t*) &val2, (mix2.base_addr) + 4*i, 4);
		sum1 += val1;
		sum2 += val2;
	}
	*mean1 = sum1/32000.0;
	*mean2 = sum2/32000.0;
}



void whiteningMx(float eigenD[2][2], float val1, float val2, float whiteningMx [2][2]){

	float diagonalMatrix[2][2];
	diagonalMatrix[0][0] = 1/ sqrt(val1);
	diagonalMatrix[0][1] = 0; 
	diagonalMatrix[1][0] = 0;
	diagonalMatrix[1][1] = 1/ sqrt(val2); 

	arm_matrix_instance_f32 egienVector[2][2];
	arm_mat_trans_f32((arm_matrix_instance_f32*) eigenD, *egienVector);

	arm_mat_mult_f32((arm_matrix_instance_f32*) diagonalMatrix, *egienVector, (arm_matrix_instance_f32*)whiteningMx);
}

void deWhiteningMx (float eigenD[2][2], float val1, float val2, float deWhiteningMx [2][2]){
	float diagonalMatrix[2][2];
	diagonalMatrix[0][0] = sqrt(val1);
	diagonalMatrix[0][1] = 0; 
	diagonalMatrix[1][0] = 0;
	diagonalMatrix[1][1] = sqrt(val2); 

	arm_mat_mult_f32((arm_matrix_instance_f32*) eigenD,  (arm_matrix_instance_f32*) diagonalMatrix, (arm_matrix_instance_f32*)deWhiteningMx);

}

void updateWeight(SineDefinition mix1, SineDefinition mix2, float whiteningMx[2][2], float mean1, float mean2, float weight[2]){

	int count = 100; 
	int i = 0; 
	float epsilon = 0.0001f; 

	//randomize weight 
	float normalized = sqrt(0.5*0.5 + 0.5 * 0.5);
	weight[0] = 0.5 / normalized; 	
	weight[1] = 0.5 / normalized;

	float oldWeight[2];
	oldWeight[0] = 0.0f; 
	oldWeight[1] = 0.0f; 

	float tempWeight[2]; 

	//convert to matrix 
	arm_matrix_instance_f32 weight_new = {2, 1, weight}; 
	arm_matrix_instance_f32 weight_old = {2, 1, oldWeight};
	arm_matrix_instance_f32 weight_temp = {2, 1, tempWeight};
 
	//iterate until convergence 
	while (i < count) {

		arm_mat_sub_f32(&weight_new, &weight_old, &weight_temp);

		float weight_norm = sqrt(weight_temp.pData[0] * weight_temp.pData[0] + weight_temp.pData[1] *weight_temp.pData[1]); 
		if (weight_norm < epsilon) {
			break;
		}

		arm_mat_add_f32(&weight_new, &weight_old, &weight_temp);
		weight_norm = sqrt(weight_temp.pData[0] * weight_temp.pData[0] + weight_temp.pData[1] *weight_temp.pData[1]);
		if (weight_norm < epsilon) {
			break;
		}

		//update weight 
		oldWeight[0] = weight[0]; 
		oldWeight[1] = weight[1]; 

		weight[0] = 0.f; 
		weight[1] = 0.f; 

		int j = 0; 
		for (j = 0; j < SINE_NSAMPLES; j++) {
			float32_t val1;
			float32_t val2;

			BSP_QSPI_Read((uint8_t*) &val1, (mix1.base_addr) + 4*i, 4);
			BSP_QSPI_Read((uint8_t*) &val2, (mix2.base_addr) + 4*i, 4);

			float32_t whiteVal1 = whiteningMx[0][0] * (val1 - mean1) + whiteningMx[0][1] * (val2 - mean2); 
			float32_t whiteVal2 = whiteningMx[1][0] * (val1 - mean1) + whiteningMx[1][1] * (val2 - mean2); 

			float32_t white1_temp = whiteVal1 * oldWeight[0] + whiteVal2 * oldWeight[1]; 
			float32_t white2_temp = white1_temp; 

			weight[0] += whiteVal1 * pow(white1_temp, 3); 
			weight[1] += whiteVal1 * pow(white2_temp, 3); 

		}

		weight[0] /= SINE_NSAMPLES;
		weight[1] /= SINE_NSAMPLES;

		weight[0] -= 3 * oldWeight[0];
		weight[1] -= 3 * oldWeight[1]; 

		float norm_weight = sqrt(weight[0] * weight[0] + weight[1] * weight[1]); 

		weight[0] /= norm_weight; 
		weight[1] /= norm_weight; 

		i++; 
	}
}
