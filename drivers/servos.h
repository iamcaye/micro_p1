/*
 * servos.h
 *
 *  Created on: 25 oct. 2021
 *      Author: cayet
 */

#ifndef DRIVERS_SERVOS_H_
#define DRIVERS_SERVOS_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************

#ifdef __cplusplus
extern "C"
{
#endif


#define LEFT_SERVO PWM_OUT_7
#define RIGHT_SERVO PWM_OUT_6
#define BOTH_SERVOS (PWM_OUT_6|PWM_OUT_7)

#define ACC 1000

#define PERIOD_PWM 40000  // TODO: Ciclos de reloj para conseguir una señal periódica de 50Hz (según reloj de periférico usado)
#define COUNT_1MS 2000  // TODO: Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT_L 2700  // TODO: Ciclos para amplitud de pulso de parada (1.52ms)
#define STOPCOUNT_R  2820 // TODO: Ciclos para amplitud de pulso de parada (1.52ms)
#define COUNT_2MS 4000   // TODO: Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)
#define NUM_STEPS 50    // Pasos para cambiar entre el pulso de 2ms al de 1ms
#define CYCLE_INCREMENTS (abs(COUNT_1MS-COUNT_2MS))/NUM_STEPS  // Variacion de amplitud tras pulsacion

// Modos de desplazamiento
typedef enum {
    SERVO_STRAIGHT,
    SERVO_TURN_LEFT,
    SERVO_TURN_RIGHT,
    SERVO_ROTATE,
    SERVO_CRASH,
    // ...
};

typedef struct {
    int32_t der;
    int32_t izq;
    float v_izq;
    float v_der;
} Step_t;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C"
{
#endif


void configServos();
void setSingleServoSpeed(uint32_t servo, float vel);
void setServosSpeed(float vel);

void mover_robot(int32_t c);
void girar_robot(int32_t g);

void mover_robot_v(int32_t c, float v);
void girar_robot_v(int32_t g, float v_der, float v_izq);

#ifdef __cplusplus
}
#endif
#endif /* DRIVERS_SERVOS_H_ */
