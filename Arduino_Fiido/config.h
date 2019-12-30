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

	// -------------- PEDALEO

	// Aumentar para hacer que el sensor PAS sea más lento pero más
	// tolerante frente a cambios de velocidad. De 0 a ...
	int tolerancia_pas = 3;

	// -------------- ASISTENCIA 6 KM/H

	// (True) si se desea activar la posibilidad de acelerar desde
	// parado a 6 km/h arrancando con el freno pulsado.
	boolean ayuda_salida_activa = true;

	// -------------- CRUCERO

	// Cantidad de pasadas para fijar el crucero por tiempo.
	// Con el valor 2 se va actualizando constantemente y mantiene la
	// última medida al soltar el acelerador.
	// Valores de 2 a 40.
	// 2 * 90 = 180 ms. Crucero continuo.
	// 22 * 90 = 1980 ms. Crucero por tiempo.
	byte pulsos_fijar_crucero = 2;

	// --------- +++

	// Cortar crucero con el freno.
	boolean liberar_crucero_con_freno = false;

	// Cantidad de pasadas con el freno pulsado para liberar crucero.
	// Valores de 2 a 40.
	// 36 * 140 = 5040 ms.
	byte pulsos_liberar_crucero_con_freno = 36;
	
	// Cortar crucero con el acelerador sin pedalear.
	boolean liberar_crucero_con_acelerador_sin_pedaleo = true;

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

	// -------------- DEBUG
	// Habilita la salida de datos por consola.
	boolean habilitar_consola = false;
};

// EOF
