#include <Arduino.h>

//=================== VARIABLES CONFIGURABLES POR EL USUARIO ===========

struct ConfigContainer {
	// -------------- DAC

	// Dirección del bus I2C [DAC] --> (0x60) suele ser por defecto.
	// Si dejamos el valor a 0 realizará la detección automática al
	// arranque. No cambiar si no sabemos con certeza la dirección
	// de memoria denuestro DAC.
	uint8_t dir_dac = 0;

	// -------------- TONOS

	// Habilita los tonos de inicialización del sistema.
	// Recomendado poner a True si se tiene zumbador en el pin 11.
	boolean buzzer_activo = true;

	// -------------- PEDALEO

	// True --> Activa pedaleo al paso del pedal por el 2 imán.
	// False --> Activa pedaleo al paso del pedal por el 1 imán.
	boolean interrupciones_pedaleo_segundo_iman = true;

	// -------------- CRUCERO

	// True --> Modo crucero.
	// False --> Manda señal del acelerador.
	boolean modo_crucero = true;

	// -------------- PROGRESIVOS
  
	// Retardo en segundos para ponerse a velocidad máxima o crucero.
	int retardo_aceleracion = 5;

	// Retardo para inciar progresivo tras parar pedales.
	// Freno anula el tiempo.
	unsigned long retardo_inicio_progresivo = 10;

	// Suavidad de los progresivos, varía entre 1-10.
	// Al crecer se hacen más bruscos.
	int suavidad_progresivos = 2;

	// Suavidad de los autoprogresivos, varía entre 1-10.
	// Al crecer se hacen más bruscos.
	int suavidad_autoprogresivos = 2;

	// -------------- DEBUG

	// Habilita la salida de datos por consola.
	boolean habilitar_consola = false;
};

// EOF
