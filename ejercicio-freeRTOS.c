//*****************************************************************************
//
// Ejercicio FreeRTOS
//
// Ignacio Herrero, Jose Manuel Cano, Eva Gonzalez.
// 
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "drivers/rgb.h"
#include "drivers/servos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "commands.h"

#define CTL_TASKPRIO tskIDLE_PRIORITY + 1
#define CTL_STACKSIZE 128

//Globales
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
QueueHandle_t servos_ctl;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    while(1)
    {
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

void botones_ISR(void){
    //uint32_t v_out_6 = 0, v_out_7 = 0;
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    int r = GPIOPinRead(GPIO_PORTF_BASE, ALL_BUTTONS);
    xQueueSendFromISR(servos_ctl, (void *)&r, &xHigherPriorityTaskWoken);
    GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(TaskHandle_t pxTask,  char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//Esta es la funcion que ejecuta cuando el RTOS se queda sin memoria dinamica
void vApplicationMallocFailedHook (void)
{
    while(1);
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t ui8Count = 0;

	if (++ui8Count == 10)
	{
    	g_ui32CPUUsage = CPUUsageTick();
    	ui8Count = 0;
	}
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void )
{
    	SysCtlSleep();
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

static portTASK_FUNCTION(CTLTask,pvParameters)
{
    uint32_t r = 0, v_out_6 = 0, v_out_7 = 0;
    while(1){
        xQueueReceive(servos_ctl, (void *)&r, portMAX_DELAY);

        v_out_6 = PWMPulseWidthGet(PWM1_BASE, LEFT_SERVO);
        v_out_7 = PWMPulseWidthGet(PWM1_BASE, RIGHT_SERVO);
        if((r & GPIO_PIN_0) && (v_out_6 < COUNT_2MS) && (v_out_7 > COUNT_1MS)){
            //Aceleramos
            v_out_6 += CYCLE_INCREMENTS;
            v_out_7 -= CYCLE_INCREMENTS;
            PWMPulseWidthSet(PWM1_BASE, LEFT_SERVO, v_out_6);
            PWMPulseWidthSet(PWM1_BASE, RIGHT_SERVO, v_out_7);
        } else if ((r & GPIO_PIN_4) && (v_out_7 < COUNT_2MS) && (v_out_6 > COUNT_1MS)){
            // Frenamos
            v_out_6 -= CYCLE_INCREMENTS;
            v_out_7 += CYCLE_INCREMENTS;
            PWMPulseWidthSet(PWM1_BASE, LEFT_SERVO, v_out_6);
            PWMPulseWidthSet(PWM1_BASE, RIGHT_SERVO, v_out_7);
        }
    }
}

//Esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos.
//Aqui solo la declaramos para poderla crear en la funcion main.
//extern void vUARTTask( void *pvParameters );

//Aqui podria definir y/o declarar otras tareas definidas en otro fichero....



//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{


    //
    // Set the clocking to run at 40 MHz from the PLL.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_50 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)


    // Get the system clock speed.
    g_ui32SystemClock = SysCtlClockGet();


    //Habilita el clock gating de los perifericos durante el bajo consumo --> Hay que decirle los perifericos que queramos que sigan andando usando la funcion SysCtlSleepEnable(...) en cada uno de ellos
    SysCtlPeripheralClockGating(true);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER5 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 5);

    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);	//Esto es necesario para que el timer0 siga funcionando en bajo consumo
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);  //Esto es necesario para que el timer1 siga funcionando en bajo consumo


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);
    ButtonsInit();
    configServos();

    servos_ctl = xQueueCreate(10, sizeof(uint32_t));
    if(servos_ctl == NULL){
        while(1);
    }

    servos_mode = xQueueCreate(1, sizeof(uint8_t));
    if(servos_ctl == NULL){
        while(1);
    }

    if((xTaskCreate(CTLTask, (portCHAR *)"CTL Task", CTL_STACKSIZE, NULL, CTL_TASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }

    /*if (initCommandLine(512,tskIDLE_PRIORITY + 1) != pdTRUE)
    {
        while(1);
    }*/


    //
    // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
    //
    vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

    //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
    while(1)
    {
    	//Si llego aqui es que algo raro ha pasado
    }
}

