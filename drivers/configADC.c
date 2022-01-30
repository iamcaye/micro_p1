#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


extern QueueHandle_t cola_adc;



//Provoca el disparo de una conversion (hemos configurado el ADC con "disparo software" (Processor trigger)
void configADC_DisparaADC(void)
{
	ADCProcessorTrigger(ADC0_BASE,1);
}


void configADC_IniciaADC(void)
{
			    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
			    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

				//HABILITAMOS EL GPIOE
				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
				SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);


				// Enable pin PE4 for ADC AIN0|AIN1|AIN2|AIN3
				GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);

				ADCHardwareOversampleConfigure(ADC0_BASE, 8);

				//CONFIGURAR SECUENCIADOR 1
				ADCSequenceDisable(ADC0_BASE,0);

				//Configuramos la velocidad de conversion al maximo (1MS/s)
				ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

				SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
				TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
				uint32_t ui32Period = SysCtlClockGet()/5; // Interrupts evrey quarter of a second
				TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);
			    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

				ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_TIMER,0);	//Disparo timer
				ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_CH9);	//La ultima muestra provoca la interrupcion
				ADCSequenceStepConfigure(ADC0_BASE,0,1,ADC_CTL_CH8|ADC_CTL_IE |ADC_CTL_END );	//La ultima muestra provoca la interrupcion
				ADCSequenceEnable(ADC0_BASE,0); //ACTIVO LA SECUENCIA

				//Habilita las interrupciones
				ADCIntClear(ADC0_BASE,0);
				ADCIntEnable(ADC0_BASE,0);
				IntPrioritySet(INT_ADC0SS0,configMAX_SYSCALL_INTERRUPT_PRIORITY);
				IntEnable(INT_ADC0SS0);
				TimerEnable(TIMER0_BASE, TIMER_A);

				//Creamos una cola de mensajes para la comunicacion entre la ISR y la tara que llame a configADC_LeeADC(...)
				cola_adc=xQueueCreate(8,sizeof(MuestrasADC));
				if (cola_adc==NULL)
				{
					while(1);
				}
}


void configADC_LeeADC(MuestrasADC *datos)
{
	xQueueReceive(cola_adc,datos,portMAX_DELAY);
}

void configADC_ISR(void)
{
	portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

	MuestrasADC leido;
	ADCIntClear(ADC0_BASE,0);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
	ADCSequenceDataGet(ADC0_BASE,0,(uint32_t *)&leido);//COGEMOS LOS DATOS GUARDADOS

	//Guardamos en la cola
	xQueueSendFromISR(cola_adc,(void *)&leido,&higherPriorityTaskWoken);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
