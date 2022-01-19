# TODO Microbot

Escenario: tablero de 1x1 con un cuadrado negro en el centro.
Objetivo: buscar cajas en el tablero y llevarlas al cuadrado negro del centro que podemos encontrarlo porque hay una torre.

TODO:
- [x] Re-estructurar las tareas y separar las colas en tareas.
- [x] Caracterizar el sentor ak21 que servira para encontrar la torre central.
- [ ] Realizar los calculos para que el robot coja las cajas mirando ya hacia la torre central.
- [ ] Implementar el encoder inferior para detectar los bordes del tablero (EncoderTask).
- [ ] Hacer que despues de coger una caja el robot vuelva a su posicion anterior


## Tareas implementadas:
* **CTLTask:** 
  * Recibir informacion de los botones para aumentar y disminuir la velocidad del robot (servos_ctl).
  * Recibir el modo de desplazamiento del robot (recto, girar, rotar sobre si mismo, etc) (servos_mode).

* **EncoderTask:** se encarga de recibir la informacion de los encoders de las ruedas (por ahora) y manda esa informacion a traves de semaforos a la tarea MovimientoTask.

* **MovimientoTask:** se encarga de llevar a cabo las peticiones de movimiento hechas con las funciones girar_robot y mover_robot, recibiendo informacion de los encoders.

* **OrdenesTask:** por ahora no hace nada, pero sera la encargada de correr la maquina de estados para mandar al robot en funcion de la informacion que reciba a traves de los sensores infrarrojos (cola_adc).

## Colas de mensajes
* q_encoders: informacion de los encoders.
* servos_ctl: informacion de los botones.
* servos_mode: modo de desplazamiento.
* cola_adc: informacion del modulo ADC (sensores inflarrojos).


## Semaforos
* s_der: cambio de estado del encoder derecho.
* s_izq: cambio de estado del encoder izquierdo.
* (Por hacer): s_suelo: detectar las lineas negras del suelo.
