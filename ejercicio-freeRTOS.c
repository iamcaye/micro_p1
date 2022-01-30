//*****************************************************************************
//
// Ejercicio FreeRTOS
//
// Laura Sanchez, Cayetano Biehler
// 
//
//*****************************************************************************


#include "FreeRTOS.h"
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
#include "drivers/configADC.h"
#include "task.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "commands.h"
#include "drivers/servos.h"

typedef enum {
    STATUS_FINDING_BOX,
    STATUS_BOX_FOUND,
    STATUS_FINDING_CENTER,
    STATUS_CENTER_FOUND,
    STATUS_GO_BACK,
} MACHINE_STATUS;

typedef enum {
    NO_EVENT,
    EV_BOX,
    EV_NO_BOX,
    EV_DONT_HAVE_BOX,
    EV_HAS_BOX,
    EV_CENTER,
    EV_ON_CENTER,
    EV_NOT_ON_CENTER,
    EV_SUELO,
    EV_NO_CENTER,
} MACHINE_EVENTS;

#define CTL_TASKPRIO tskIDLE_PRIORITY + 1
#define CTL_STACKSIZE 128

//Globales
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
uint32_t g_servo_mode;
QueueHandle_t servos_ctl, servos_mode, cola_adc, q_encoders, q_mover, q_girar;
QueueSetHandle_t servos_set, move_set, ordenes_set;
extern QueueHandle_t q_steps;
SemaphoreHandle_t s_der, s_izq, s_control, s_suelo;
MACHINE_STATUS g_status = STATUS_FINDING_BOX;
MACHINE_EVENTS g_event;

uint32_t g_d41[] = {4,6,8,10,12,14,16,18,20,22,24,26,28,30};
uint32_t g_v41[] = {2810,2143,1707,1328,1257,972,901,759,687,608,567,493,464,438};
uint32_t g_n41 = 14;

uint32_t g_d21[] = {10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,50,60,70,80};
uint32_t g_v21[] = {2444,2041,1771,1558,1400,1294,1203,1140,1083,1029,997,962,892,823,791,671,601,568,538,512,478,419,388};
uint32_t g_n21 = 20;

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

 void encoder_ISR (void) {
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;
    uint8_t r = GPIOPinRead(GPIO_PORTA_BASE, (GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_6));
    GPIOIntClear(GPIO_PORTA_BASE, (GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_6));
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

static portTASK_FUNCTION(EncoderTask,pvParameters)
{
    uint8_t r = 0;
    uint8_t last = 0;
    while(1){
        xQueueReceive(q_encoders, (void *)&r, portMAX_DELAY);
        if(((r & GPIO_PIN_3) && !(GPIO_PIN_3 & last)) || (!(r & GPIO_PIN_3) && (GPIO_PIN_3 & last))){
            xSemaphoreGive(s_izq);
        }

        if(((r & GPIO_PIN_6) && !(GPIO_PIN_6 & last)) || (!(r & GPIO_PIN_6) && (GPIO_PIN_6 & last))){
            xSemaphoreGive(s_der);
        }

        if((!(r & GPIO_PIN_4) && (GPIO_PIN_4 & last))){
            xSemaphoreGive(s_suelo);
        }

        last = r;
    }
}

static portTASK_FUNCTION(MovimientoTask,pvParameters)
{
    QueueSetMemberHandle_t q;
    Step_t paso;

    while(1){
        xQueueReceive(q_steps, (void *)&paso, portMAX_DELAY);
        if(paso.izq == 0){
            setSingleServoSpeed(LEFT_SERVO, 0.0f);
        } else if (paso.izq > 0){
            setSingleServoSpeed(LEFT_SERVO, 0.2f);
        } else {
            setSingleServoSpeed(LEFT_SERVO, -0.2f);
        }

        if(paso.der == 0){
            setSingleServoSpeed(RIGHT_SERVO, 0.0f);
        } else if (paso.der > 0){
            setSingleServoSpeed(RIGHT_SERVO, 0.2f);
        } else {
            setSingleServoSpeed(RIGHT_SERVO, -0.2f);
        }

        if(paso.der < 0){
            paso.der = -paso.der;
        }

        if(paso.izq < 0){
            paso.izq = -paso.izq;
        }

        while(paso.der > 0 || paso.izq > 0){
            q = xQueueSelectFromSet(move_set, portMAX_DELAY);
            if(q == s_der){
                xSemaphoreTake(s_der, 0);
                paso.der -= 1;
            }else if(q == s_izq){
                xSemaphoreTake(s_izq, 0);
                paso.izq -= 1;
            }else if(q == s_control) {
                xSemaphoreTake(s_control, 0);
                break;
            }
        }
        setServosSpeed(0.0f);
    }
}

static portTASK_FUNCTION(OrdenesTask,pvParameters)
{
    QueueSetMemberHandle_t q;
    MuestrasADC r_adc;
    uint32_t i = 0, j = 0;
    while(1){
        q = xQueueSelectFromSet(ordenes_set, portMAX_DELAY);
        if(q == cola_adc){
           xQueueReceive(cola_adc, (void *)&r_adc, 0);
           g_event = NO_EVENT;
           i = 0;
           // Obtener distancia captada por el sensor gp41
           while((i < g_n41) && (r_adc.chan1 < g_v41[i])){
               i++;
           }

           UARTprintf("Tensiones: %d \t %d\n", r_adc.chan1, r_adc.chan2);

           if(i < g_n41){
               //UARTprintf("Sensor gp41: %d\n", g_d41[i]);
               if((g_status == STATUS_FINDING_BOX) && (g_d41[i] > 4)){
                   g_event = EV_BOX;
               } else if((g_status == STATUS_BOX_FOUND)){
                  if(g_d41[i] <= 4){
                      g_event = EV_HAS_BOX;
                  } else {
                      g_event = EV_DONT_HAVE_BOX;
                  }
               }
           } else {
               g_event = EV_NO_BOX;
           }

           j = 0;
           if(g_event == NO_EVENT){
               // Obtener distancia captada por el sensor gp21
               while((j < g_n21) && (r_adc.chan2 < g_v21[j])){
                   j++;
               }

               if(j < g_n21){
                   UARTprintf("Sensor gp21: %d\n", g_d21[j]);
                   if((g_status == STATUS_FINDING_CENTER) && (g_d21[j] > 24)){
                       g_event = EV_CENTER;
                   } else if((g_status == STATUS_CENTER_FOUND) && (g_d21[j] > 24)){
                       g_event = EV_NOT_ON_CENTER;
                   }
               } else if (g_status == STATUS_FINDING_CENTER){
                   g_event = EV_NO_CENTER;
               }
           }
        } else if(q == s_suelo) {
            xSemaphoreTake(s_suelo, 0);
            if(g_status != STATUS_CENTER_FOUND){
                g_event = EV_SUELO;
            } else {
                g_event = EV_ON_CENTER;
            }
        }

        switch(g_status){
            case STATUS_FINDING_BOX:
                if(g_event == EV_NO_BOX){
                    g_status = STATUS_FINDING_BOX;
                    mover_robot(10);
                    girar_robot(360);
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
                } else if (g_event == EV_BOX){
                    UARTprintf("STATUS_FINDING_BOX -> STATUS_BOX_FOUND\n");
                    g_status = STATUS_BOX_FOUND;
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                    mover_robot(g_d41[i]-4);
                } else if(g_event == EV_SUELO){
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                    mover_robot(-10);
                    girar_robot(180);
                    vTaskDelay(1.5*configTICK_RATE_HZ);
                }
                break;
            case STATUS_BOX_FOUND:
                if(g_event == EV_HAS_BOX){
                    UARTprintf("STATUS_BOX_FOUND -> STATUS_FINDING_CENTER\n");
                    g_status = STATUS_FINDING_CENTER;
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                    girar_robot(360);
                } else if (g_event == EV_SUELO){
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                    mover_robot(-10);
                    girar_robot(-90);
                    vTaskDelay(1.5*configTICK_RATE_HZ);
                    xQueueReset(cola_adc);
                    g_status = STATUS_FINDING_BOX;
                    UARTprintf("STATUS_BOX_FOUND -> STATUS_FINDING_BOX\n");
                } else if (g_event == EV_DONT_HAVE_BOX){
                    mover_robot(g_d41[i]);
                } else if(g_event == EV_NO_BOX){
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                    g_status = STATUS_FINDING_BOX;
                }
                break;
            case STATUS_FINDING_CENTER:
                if(g_event == EV_CENTER){
                    UARTprintf("STATUS_FINDING_CENTER -> STATUS_CENTER_FOUND\n");
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
                    g_status = STATUS_CENTER_FOUND;
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                    mover_robot(g_d21[j]); // TODO: poner bien la distancia que tiene que movese para no comerse el centro
                } else if(g_event == EV_NO_CENTER){
                    girar_robot(360);
                } else if (g_event == EV_NO_BOX){
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                    g_status = STATUS_FINDING_BOX;
                }
                break;
            case STATUS_CENTER_FOUND:
                if(g_event == EV_ON_CENTER){
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                    mover_robot(-10);
                    girar_robot(180);
                    vTaskDelay(1.5*configTICK_RATE_HZ);
                    xQueueReset(cola_adc);
                    g_status = STATUS_FINDING_BOX;
                    UARTprintf("STATUS_CENTER_FOUND -> STATUS_FINDING_BOX\n");
                } else if(g_event == EV_NOT_ON_CENTER){
                    mover_robot(g_d21[j]);
                } else if (g_event == EV_NO_CENTER){
                    g_status = STATUS_FINDING_CENTER;
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                } else if (g_event == EV_NO_BOX){
                    xQueueReset(q_steps);
                    xSemaphoreGive(s_control);
                    g_status = STATUS_FINDING_BOX;
                }
                break;
            default:
                break;
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
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, (GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_6));

    GPIOIntTypeSet(GPIO_PORTA_BASE, (GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_6), GPIO_BOTH_EDGES);
    MAP_IntPrioritySet(INT_GPIOA, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    GPIOIntEnable(GPIO_PORTA_BASE, (GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_6));
    IntEnable(INT_GPIOA);

    s_der = xSemaphoreCreateBinary();
    if(s_der == NULL){
        while(1);
    }

    s_izq = xSemaphoreCreateBinary();
    if(s_izq == NULL){
        while(1);
    }

    s_suelo = xSemaphoreCreateBinary();
    if(s_suelo == NULL){
        while(1);
    }

    s_control = xSemaphoreCreateBinary();
    if(s_control == NULL){
        while(1);
    }

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

    servos_set = xQueueCreateSet(11);
    if(servos_set == NULL){
        while(1);
    }


    if(xQueueAddToSet(servos_ctl, servos_set) != pdPASS){
        while(1);
    }

    if(xQueueAddToSet(servos_mode, servos_set) != pdPASS){
        while(1);
    }

    move_set = xQueueCreateSet(3);
    if(servos_set == NULL){
        while(1);
    }

    if(xQueueAddToSet(s_der, move_set) != pdPASS){
        while(1);
    }

    if(xQueueAddToSet(s_izq, move_set) != pdPASS){
        while(1);
    }

    if(xQueueAddToSet(s_control, move_set) != pdPASS){
        while(1);
    }

    ordenes_set = xQueueCreateSet(9);
    if(ordenes_set == NULL){
        while(1);
    }

    if(xQueueAddToSet(cola_adc, ordenes_set) != pdPASS){
        while(1);
    }

    if(xQueueAddToSet(s_suelo, ordenes_set) != pdPASS){
        while(1);
    }

    if((xTaskCreate(EncoderTask, (portCHAR *)"Encoder Task", CTL_STACKSIZE, NULL, CTL_TASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(MovimientoTask, (portCHAR *)"Movimiento Task", CTL_STACKSIZE, NULL, CTL_TASKPRIO, NULL) != pdTRUE))
    {
        while(1);
    }

    if((xTaskCreate(OrdenesTask, (portCHAR *)"Ordenes Task", CTL_STACKSIZE, NULL, CTL_TASKPRIO, NULL) != pdTRUE))
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

