#ifndef _DEMIXER_H
#define _DEMIXER_H
#include "arm_math.h"
#include "main.h"
#include "QSPI_handler.h"

void fast_ica(SineDefinition mix1, SineDefinition mix2);

void constructBasisSet(SineDefinition mix1, SineDefinition mix2, float weight[2], float whiteningMx[2][2]);

void eigen2by2(float cov[2][2], float eigenD[2][2], float* val1, float* val2);

void covariance(SineDefinition mix1, SineDefinition mix2, float mean1, float mean2, float cov[2][2]);

void mean(SineDefinition mix1, SineDefinition mix2, float* mean1, float *mean2);

void whiteningMx(float eigenD[2][2], float val1, float val2, float whiteningMx [2][2]);

void deWhiteningMx (float eigenD[2][2], float val1, float val2, float deWhiteningMx [2][2]);

void updateWeight(SineDefinition mix1, SineDefinition mix2, float whiteningMx[2][2], float mean1, float mean2, float weight[2]);


#endif