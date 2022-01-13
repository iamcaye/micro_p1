/*
 * servos.c
 *
 *  Created on: 25 oct. 2021
 *      Author: cayet
 */


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include "servos.h"
#include "buttons.h"
#include "queue.h"

QueueHandle_t q_steps;


void configServos() {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); //Habilita modulo PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // Habilita puerto salida para señal PWM (ver en documentacion que pin se corresponde a cada módulo PWM)

    GPIOPinTypePWM(GPIO_PORTF_BASE, (GPIO_PIN_2|GPIO_PIN_3));    // PF2,3 como salida PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);

    GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);
    MAP_IntPrioritySet(INT_GPIOF, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    GPIOIntEnable(GPIO_PORTF_BASE, ALL_BUTTONS);
    IntEnable(INT_GPIOF);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // Módulo PWM contara hacia abajo
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_L);  // Establece el periodo (en este caso, un porcentaje del valor máximo
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_R);
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); // Habilita la salida de la señal
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3); //Habilita/pone en marcha el generador PWM

    q_steps = xQueueCreate(10, sizeof(Step_t));
    if(q_steps == NULL){
        while(1);
    }

}

void setSingleServoSpeed (uint32_t servo, float vel){
    if(servo == LEFT_SERVO){
        PWMPulseWidthSet(PWM1_BASE, servo, (uint32_t)(STOPCOUNT_L+vel*ACC));  // Establece el periodo (en este caso, un porcentaje del valor máximo
    }else{
        PWMPulseWidthSet(PWM1_BASE, servo, (uint32_t)(STOPCOUNT_R-vel*ACC));  // Establece el periodo (en este caso, un porcentaje del valor máximo
    }
}

void setServosSpeed(float vel){
    setSingleServoSpeed(LEFT_SERVO, vel);
    setSingleServoSpeed(RIGHT_SERVO, vel);
}

void mover_robot(int32_t c){
    Step_t mov;
    mov.der = mov.izq = c*2;
    xQueueSend(q_steps, (void *)&mov, 0);
}

void mover_robot_v(int32_t c, float v){
    Step_t mov;
    mov.der = mov.izq = c*2;
    mov.v_izq = mov.v_der = v;
    xQueueSend(q_steps, (void *)&mov, 0);
}

void girar_robot(int32_t g){
    Step_t mov;
    if(g > 0){
        mov.izq = g/10;
        mov.der = 0;
    } else {
        mov.der = g/10;
        mov.izq = 0;
    }
    xQueueSend(q_steps, (void *)&mov, 0);
}

void girar_robot_v(int32_t g, float v_izq, float v_der){
    Step_t mov;
    if(g > 0){ // Girar a la derecha
        mov.izq = g/10;
        mov.der = 0;
        if(v_izq > v_der){
            mov.v_izq =v_izq;
            mov.v_der =v_der;
        } else {
            mov.v_izq = 0.1f;
            mov.v_der = 0.0f;
        }
    } else { // Girar a la izquiera
        mov.der = g/10;
        mov.izq = 0;
        if(v_izq < v_der){
            mov.v_izq =v_izq;
            mov.v_der =v_der;
        } else {
            mov.v_izq = 0.1f;
            mov.v_der = 0.0f;
        }
    }
    xQueueSend(q_steps, (void *)&mov, 0);
}
