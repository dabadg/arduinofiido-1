#include <Arduino.h>

//=================== VARIABLES CONFIGURABLES POR EL USUARIO =======

struct ConfigContainer {

	// -------------- DAC

	// Dirección del bus I2C [DAC] (0x60) si está soldado,
	// si no (0x62).
	int dir_dac = 0x60;

	// -------------- Tonos

	// Habilita los tonos de inicialización del sistema.
	// Recomendado poner a True si se tiene zumbador en el pin 11.
	boolean buzzer_activo = true;

	// -------------- Acelerador

	boolean recalcular_rangos_acelerador = true;

	// -------------- Asistencia 6kmh

	// (True) si se desea activar la posibilidad de acelerar desde
	// parado a 6 km/h arrancando con el freno pulsado.
	boolean freno_pulsado = true;

	// Tiempo en ms que tarda en iniciarse la ayuda al arranque.
	unsigned int retardo_ayuda_arranque = 600;

	// (True) Habilita la ayuda la asistencia 6 km/h con inicio
	// progresivo desde alta potencia.
	boolean activar_progresivo_ayuda_arranque = false;

	// Valor inicial de salida en la asistencia 6 km/h.
	// Como mínimo tendrá que tener el valor de la constante
	// a0_valor_6kmh --> 450.
	float v_salida_progresivo_ayuda_arranque = 700;

	// -------------- Crucero

	// True --> Modo crucero.
	// False --> Manda señal del acelerador.
	boolean modo_crucero = true;

	// Cantidad de pasadas para fijar el crucero por tiempo.
	// Con el valor 2 se va actualizando la configuración
	// constantemente y mantiene la última medida al soltar el acelerador.
	// 2 * 140 = 280 ms.
	unsigned int pulsos_fijar_crucero = 2;

	// False --> Mantiene valor que tenía el crucero antes de entrar a
	// la asistencia de 6km/h.
	// True -->  En esta situación anula el valor de crucero al
	// incrementar y soltar acelerador.
	boolean liberar_crucero_con_acelerador = true;

	// Cantidad de pasadas con el freno pulsado para liberar crucero.
	// 23 * 140 = 3220 ms.
	unsigned int pulsos_liberar_crucero = 23;

	// Retardo en segundos para ponerse a velocidad máxima o crucero.
	int retardo_aceleracion = 5;

	// Retardo para inciar progresivo tras parar pedales.
	// Freno anula el tiempo.
	unsigned int retardo_inicio_progresivo = 10;

	// -------------- Progresivos

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