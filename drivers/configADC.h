/*
 * configADC.h
 *
 *  Created on: 22/4/2016
 *      Author: jcgar
 */

#ifndef CONFIGADC_H_
#define CONFIGADC_H_

#include<stdint.h>

typedef struct
{
	uint32_t chan1;
	uint32_t chan2;
} MuestrasADC;

typedef struct
{
	uint32_t chan1;
	uint32_t chan2;
	uint32_t chan3;
	uint32_t chan4;
	uint32_t chan5;
	uint32_t chan6;
	uint32_t chan7;
	uint32_t chan8;
} MuestrasLeidasADC;


void configADC_ISR(void);
void configADC_DisparaADC(void);
void configADC_LeeADC(MuestrasADC *datos);
void configADC_IniciaADC(void);


#endif /* CONFIGADC_H_ */
