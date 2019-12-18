#include <Arduino.h>

//=================== VARIABLES CONFIGURABLES POR EL USUARIO ===========

struct ConfigContainer {
	// -------------- DAC

	// Dirección del bus I2C [DAC] --> (0x60) suele ser por defecto.
	// Si dejamos el valor a 0 realizará la detección automática al
	// arranque. No cambiar si no sabemos con certeza la dirección de
	// memoria denuestro DAC.
	uint8_t dir_dac = 0;

	// -------------- TONOS

	// Habilita los tonos de inicialización del sistema.
	// Recomendado poner a True si se tiene zumbador en el pin 11.
	boolean buzzer_activo = true;

	// -------------- ACELERADOR

	// Recalcula el valor real en reposo del acelerador al inicio.
	boolean recalcular_rango_min_acelerador = true;
	// Recalcula el valor máximo del acelerador.
	boolean recalcular_rango_max_acelerador = true;

	// -------------- PEDALEO

	// Número de interrupciones para activar pedaleo.
	// Valores a elegir --> 2, 3 ó 4.
	byte interrupciones_activacion_pedaleo = 3;

	// -------------- ASISTENCIA 6 KM/H

	// (True) si se desea activar la posibilidad de acelerar desde
	// parado a 6 km/h arrancando con el freno pulsado.
	boolean ayuda_salida_activa = true;

	// Tiempo en ms que tarda en iniciarse la ayuda al arranque.
	unsigned int retardo_ayuda_arranque = 600;

	// (True) Habilita la ayuda la asistencia 6 km/h con inicio
	// progresivo desde alta potencia.
	boolean activar_progresivo_ayuda_arranque = true;

	// Valor inicial de salida en la asistencia 6 km/h.
	// Como mínimo tendrá que tener el valor de la constante
	// a0_valor_6kmh --> 450.
	int v_salida_progresivo_ayuda_arranque = 700;

	// -------------- CRUCERO

	// Cantidad de pasadas para fijar el crucero por tiempo.
	// Con el valor 2 se va actualizando constantemente y mantiene la
	// última medida al soltar el acelerador (0-255).
	// 2 * 140 = 280 ms. Crucero continuo.
	// 20 * 140 = 2800 ms. Crucero por tiempo.
	byte pulsos_fijar_crucero = 20;

	// Para que el acelerador funcione como en el coche. Si se fija el
	// crucero, la potencia del motor solo cambia si se supera con el
	// acelerador la velocidad de crucero fijada o si se mueve el
	// acelerador por debajo de la potencia de crucero durante los
	// ciclos definidos en esta variable.

	// Pulsos que tarda en fijar velocidad por debajo de crucero.
	// No Cambiar Valor. Solo para fijar por tiempo. 5 --> 700 ms.
	// Con el valor de [pulsos_fijar_crucero] en 5 o debajo de 5,
	// poner esta variable [pulsos_fijar_debajo_crucero] a 0.
	byte pulsos_fijar_debajo_crucero = 5;

	// --------- +++

	// False --> Mantiene valor que tenía el crucero antes de entrar a
	// la asistencia de 6km/h.
	// True -->  En esta situación anula el valor de crucero al
	// incrementar y soltar acelerador.
	boolean liberar_crucero_con_acelerador = true;

	// Cantidad de pasadas con el freno pulsado para liberar crucero.
	// De 0 a 255.
	// 36 * 140 = 5040 ms.
	byte pulsos_liberar_crucero = 36;

	// Tiempo necesario para ejecutar el procedimiento de cancelar 
	// crucero por acelerador. Hay que acelerar y soltar el acelerador sin 
	// pedalear dentro de los segundos definidos en esta variable.
	unsigned int tiempo_anula_crucero_acelerador = 150;
  
	// -------------- PROGRESIVOS
  
	// Retardo en segundos para ponerse a velocidad máxima o crucero.
	int retardo_aceleracion = 5;

	// Retardo para inciar progresivo tras parar pedales.
	// Freno anula el tiempo.
	unsigned long retardo_inicio_progresivo = 10;

	// Suavidad de los progresivos, varía entre 1-10.
	// Al crecer se hacen más bruscos.
	int suavidad_progresivos = 5;

	// Suavidad de los autoprogresivos, varía entre 1-10.
	// Al crecer se hacen más bruscos.
	int suavidad_autoprogresivos = 5;

	// Tiempo de ejecución del progresivo en la asistencia a 6 km/h.
	// 1500 ms.
	int tiempo_ejecucion_progresivo_ayuda_arranque = 1500;

	// -------------- DEBUG
	// Habilita la salida de datos por consola.
	boolean habilitar_consola = false;
};

// EOF
