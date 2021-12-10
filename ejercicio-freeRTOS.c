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
#include "drivers/configADC.h"
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
uint32_t g_servo_mode;
QueueHandle_t servos_ctl, servos_mode, cola_adc, q_encoders;
QueueSetHandle_t servos_set;

uint32_t g_d41[] = {4,6,8,10,12,14,16,18,20,22,24,26,28,30};
uint32_t g_v41[] = {2810,2143,1707,1328,1257,972,901,759,687,608,567,493,464,438};
uint32_t g_n41 = 14;
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
    GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS);
    xQueueSendFromISR(servos_ctl, (void *)&r, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void whisker_ISR (void){
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    uint8_t res = SERVO_CRASH;
    int r = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_5);
    xQueueSendFromISR(servos_mode, (void *)&res, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

 void encoder_ISR (void) {
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    uint8_t r = GPIOPinRead(GPIO_PORTA_BASE, (GPIO_PIN_3|GPIO_PIN_6));
    GPIOIntClear(GPIO_PORTA_BASE, (GPIO_PIN_3|GPIO_PIN_6));
    xQueueSendFromISR(q_encoders, (void *)&r, &xHigherPriorityTaskWoken);
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
    uint32_t r = 0, v_r = 0, v_l = 0;
    uint8_t res = 0;
    QueueSetMemberHandle_t q;
    while(1){
        q = xQueueSelectFromSet(servos_set, portMAX_DELAY);
        if(q == servos_ctl){
            xQueueReceive(servos_ctl, (void *)&r, 0);

            v_l = PWMPulseWidthGet(PWM1_BASE, LEFT_SERVO);
            v_r = PWMPulseWidthGet(PWM1_BASE, RIGHT_SERVO);
            if((r & GPIO_PIN_0) && (v_l < COUNT_2MS) && (v_r > COUNT_1MS)){
                //Aceleramos
                v_l += CYCLE_INCREMENTS;
                v_r -= CYCLE_INCREMENTS;
            } else if ((r & GPIO_PIN_4) && (v_r < COUNT_2MS) && (v_l > COUNT_1MS)){
                // Frenamos
                v_l -= CYCLE_INCREMENTS;
                v_r += CYCLE_INCREMENTS;
            }

            if(g_servo_mode == SERVO_STRAIGHT){
                PWMPulseWidthSet(PWM1_BASE, LEFT_SERVO, v_l);
                PWMPulseWidthSet(PWM1_BASE, RIGHT_SERVO, v_r);
            } else if(g_servo_mode == SERVO_TURN_LEFT){
                PWMPulseWidthSet(PWM1_BASE, RIGHT_SERVO, v_r);
            } else if(g_servo_mode == SERVO_TURN_RIGHT){
                PWMPulseWidthSet(PWM1_BASE, LEFT_SERVO, v_l);
            } else if (g_servo_mode == SERVO_ROTATE){
                PWMPulseWidthSet(PWM1_BASE, LEFT_SERVO, v_l);
                PWMPulseWidthSet(PWM1_BASE, RIGHT_SERVO, v_l);
            }
        } else if (q == servos_mode){
            xQueueReceive(servos_mode, (void *)&res, 0);
            g_servo_mode = res;
            switch(g_servo_mode){
                case SERVO_STRAIGHT:
                    setServosSpeed(0.0f);
                    break;

                case SERVO_TURN_LEFT:
                    setSingleServoSpeed(LEFT_SERVO, 0.1f);
                    setSingleServoSpeed(RIGHT_SERVO, 0.5f);
                    break;

                case SERVO_TURN_RIGHT:
                    setSingleServoSpeed(RIGHT_SERVO, 0.1f);
                    setSingleServoSpeed(LEFT_SERVO, 0.5f);
                    break;

                case SERVO_ROTATE:
                    setSingleServoSpeed(RIGHT_SERVO, 0.1f);
                    setSingleServoSpeed(LEFT_SERVO, -0.1f);
                    break;

                case SERVO_CRASH:
                    setSingleServoSpeed(LEFT_SERVO, -0.05f);
                    setSingleServoSpeed(RIGHT_SERVO, -0.2f);
                    vTaskDelay(1000000);
                    setServosSpeed(0.3f);
                    g_servo_mode = SERVO_STRAIGHT;
                    break;
                default:
                    break;

            }
        }else if (q == cola_adc) {
            xQueueReceive(cola_adc, (void *)&r, 0);
            uint32_t i = 0;
            while(i < g_n41){
                if(r > g_v41[i]){
                    break;
                }
                i++;
            }

            if(i < g_n41){
                //UARTprintf("Estamos a %d de un obstaculo\r\n", g_d41[i]);
                if(g_d41[i] < 10){
                    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 1);
                    uint32_t res = SERVO_CRASH;
                    //xQueueSend(servos_mode, (void *)&res, 0);

                }
            }
        }
    }
}

static portTASK_FUNCTION(EncoderTask,pvParameters)
{
    uint8_t r = 0;
    uint32_t n_l = 0, n_r = 0;
    uint8_t last = 0;
    while(1){
        xQueueReceive(q_encoders, (void *)&r, portMAX_DELAY);
        if((r & GPIO_PIN_3) && !(GPIO_PIN_3 & last)){
            n_l++;
            UARTprintf("n_l: %d\n", n_l);
        } else if(!(r & GPIO_PIN_3) && (GPIO_PIN_3 & last)) {
            n_l++;
            UARTprintf("n_l: %d\n", n_l);
        }

        if((r & GPIO_PIN_6) && !(GPIO_PIN_6 & last)){
            n_r++;
            UARTprintf("n_r: %d\n", n_r);
        } else if(!(r & GPIO_PIN_6) && (GPIO_PIN_6 & last)) {
            n_r++;
            UARTprintf("n_r: %d\n", n_r);
        }

        if(n_r == 36){
            UARTprintf("Rueda derecha ha dado una vuelta\n");
            n_r = 0;
        }

        if(n_l == 36){
            UARTprintf("Rueda izquierda ha dado una vuelta\n");
            n_l = 0;
        }

        last = r;
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
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);
    ButtonsInit();
    configServos();
    configADC_IniciaADC();

    // Configuracion del whisker (PIN B5)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_DIR_MODE_IN);
    MAP_GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
    MAP_IntPrioritySet(INT_GPIOB, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_5);
    IntEnable(INT_GPIOB);

    // Configuracion de los encoders (PIN A3,5,6)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, (GPIO_PIN_3|GPIO_PIN_6));
    //MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, (GPIO_PIN_3|GPIO_PIN_6), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

    GPIOIntTypeSet(GPIO_PORTA_BASE, (GPIO_PIN_3|GPIO_PIN_6), GPIO_BOTH_EDGES);
    MAP_IntPrioritySet(INT_GPIOA, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    GPIOIntEnable(GPIO_PORTA_BASE, (GPIO_PIN_3|GPIO_PIN_6));
    IntEnable(INT_GPIOA);

    q_encoders = xQueueCreate(10, sizeof(uint8_t));
    if(q_encoders == NULL){
        while(1);
    }

    servos_ctl = xQueueCreate(10, sizeof(uint32_t));
    if(servos_ctl == NULL){
        while(1);
    }

    servos_mode = xQueueCreate(1, sizeof(uint8_t));
    if(servos_mode == NULL){
        while(1);
    }

    servos_set = xQueueCreateSet(19);
    if(servos_set == NULL){
        while(1);
    }

    if(xQueueAddToSet(cola_adc, servos_set) != pdPASS){
        while(1);
    }

    if(xQueueAddToSet(servos_ctl, servos_set) != pdPASS){
        while(1);
    }

    if(xQueueAddToSet(servos_mode, servos_set) != pdPASS){
        while(1);
    }

    if((xTaskCreate(CTLTask, (portCHAR *)"CTL Task", CTL_STACKSIZE, NULL, CTL_TASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(EncoderTask, (portCHAR *)"Encoder Task", CTL_STACKSIZE, NULL, CTL_TASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }

    if (initCommandLine(512,tskIDLE_PRIORITY + 1) != pdTRUE)
    {
        while(1);
    }


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

