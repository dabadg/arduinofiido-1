#include <Arduino.h>

//=================== VARIABLES CONFIGURABLES POR EL USUARIO ===========

struct ConfigContainer {

	// -------------- DAC

	// Dirección del bus I2C [DAC] (0x60) si está soldado,
	// si no (0x62).
	int dir_dac = 0x60;

	// -------------- TONOS

	// Habilita los tonos de inicialización del sistema.
	// Recomendado poner a True si se tiene zumbador en el pin 11.
	boolean buzzer_activo = true;

	// -------------- ACELERADOR

	boolean recalcular_rango_min_acelerador = true;

	// -------------- ASISTENCIA 6 KM/H

	// (True) si se desea activar la posibilidad de acelerar desde
	// parado a 6 km/h arrancando con el freno pulsado.
	boolean freno_pulsado = false;

	// Tiempo en ms que tarda en iniciarse la ayuda al arranque.
	unsigned int retardo_ayuda_arranque = 600;

	// (True) Habilita la ayuda la asistencia 6 km/h con inicio
	// progresivo desde alta potencia.
	boolean activar_progresivo_ayuda_arranque = false;

	// Valor inicial de salida en la asistencia 6 km/h.
	// Como mínimo tendrá que tener el valor de la constante
	// a0_valor_6kmh --> 450.
	float v_salida_progresivo_ayuda_arranque = 700;

	// -------------- CRUCERO

	// True --> Modo crucero.
	// False --> Manda señal del acelerador.
	boolean modo_crucero = false;

	// True --> Habilita método de crucero contínuo. La variable
	// pulsos_fijar_crucero no tiene efecto en este modo.
	// False --> Habilita método de crucero por tiempo.
	boolean modo_crucero_continuo = true;

	// Cantidad de pasadas para fijar el crucero por tiempo.
	// 20 * 140 = 2800 ms.
	unsigned int pulsos_fijar_crucero = 20;

	// False --> Mantiene valor que tenía el crucero antes de entrar a
	// la asistencia de 6km/h.
	// True -->  En esta situación anula el valor de crucero al
	// incrementar y soltar acelerador.
	boolean liberar_crucero_con_acelerador = false;

	// Cantidad de pasadas con el freno pulsado para liberar crucero.
	// 33 * 140 = 4620 ms.
	unsigned int pulsos_liberar_crucero = 33;

	// Retardo en segundos para ponerse a velocidad máxima o crucero.
	int retardo_aceleracion = 5;

	// Retardo para inciar progresivo tras parar pedales.
	// Freno anula el tiempo.
	unsigned int retardo_inicio_progresivo = 10;

	// -------------- PROGRESIVOS

	// Suavidad de los progresivos, varía entre 1-10.
	// Al crecer se hacen más bruscos.
	float suavidad_progresivos = 5.0;

	// Suavidad de los autoprogresivos, varía entre 1-10.
	// Al crecer se hacen más bruscos.
	float suavidad_autoprogresivos = 5.0;

	// Tiempo de ejecución del progresivo en la asistencia a 6 km/h.
	// 1500 ms.
	int tiempo_ejecucion_progresivo_ayuda_arranque = 1500;

	// -------------- DEBUG
	// Habilita la salida de datos por consola.
	boolean habilitar_consola = false;
};

// EOF
