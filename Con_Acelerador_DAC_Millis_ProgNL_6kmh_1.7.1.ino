/* 
                    Version Con Acelerador y DAC
------------------------------------------------------------------------
PRINCIPALES NOVEDADES:
 * Deteccion de pulsos con Millis()
 * Progresivos y Auto Progresivos no lineales
 * Posibilidadad de asistir a 6km/h desde parado
 * Posibilidadad de cortar crucero al frenar
------------------------------------------------------------------------
VERSION CRUCERO: 
 * Se trata de guardar el ultimo valor del acelerador
 * para no tener que estar sujetando el acelerador.
 * La idea es detectar cuando se suelta de golpe el acelerador
 * y guardar el voltaje anterior como de crucero.
 * Al parar y volver a pedalear, se va incrementando voltaje
 * gradualmente hasta llegar al valor de crucero.
 * Si se vuelve a mover el acelerador se toma este como nuevo crucero.
 * Usamos un pin analogico de entrada conectado al
 * acelerador, por otra parte mandaremos a la placa DAC mediante
 * comunicacion i2c el valor de salida hacia la controladora.
 * El acelerador da un voltaje variable entre 0.85 y 3.9
 * Se puede configurar que el freno anule el crucero.
------------------------------------------------------------------------
LEGALIZACION ACELERADOR:
 * Basicamente lo que hace es detectar pulsos mediante una
 * interrupcion en el pin (pin_pedal).
 * Si los pulsos obtenidos son menores que la cadencia espera
 * que se cumpla el retardo establecido y no funciona el acelerador.
------------------------------------------------------------------------
LINKS:
 * Ayuda, sugerencias, preguntas, etc. en el grupo Fiido Telegram:
 * 
 *                      http://t.me/FiidoD1Spain
 * 
 * Grupo Telegram de desarrollo privado, si vas a montar el circuito y
 * necesitas ayuda o colaborar pide acceso en el general de arriba.
 *  
 * Canal con montaje, enlaces, programas, etc.
 * 
 *                      http://t.me/fiidolegal
 */

//=================== VARIABLES CONFIGURABLES POR EL USUARIO ===========

// Numero de pulsos para que se considere que se esta pedaleando
// Configurar segun sensor y gustos
int cadencia = 1;

// Retardo en segundos para parar el motor una vez se deja de pedalear
// Usar multiplos de 0.25
// No se recomienda subir mas de 1.00 segundo
float retardo_paro_motor = 0.50; // 0.25 = 1/4 de segundo

// Retardo en segundos para ponerse a velocidad maxima o crucero
int retardo_aceleracion = 5;

// (True) modo crucero (False) manda señal del acelerador
const boolean modo_crucero = true;

// (True) si se desea anular la velocidad de crucero al frenar
const boolean freno_anula_crucero = false;

// Retardo para inciar progresivo tras parar pedales
// Freno anula el tiempo
int retardo_inicio_progresivo = 10;

// Suavidad de los progresivos, varia entre 1-10
// Al crecer se hacen más bruscos (Se sale con mas tiron)
float suavidad_progresivos = 5;

// Suavidad de los autoprogresivos, varia entre 1-10
// al crecer se hacen más brucos
float suavidad_autoprogresivos = 5;

// Direccion del bus I2C [DAC] (0x60) si esta soldado, si no (0x62)
const int dir_dac = 0x60;

// (True) si se desea desacelerar motor al dejar de pedalear
const boolean desacelera_al_parar_pedal = true;

// Voltios maximos que da el acelerador, subir si se nota falta de 
// potencia, bajar si para el motor a tope o se producen ruidos raros
const float v_max_acelerador = 3.9;

// Voltaje mínimo de acelerador en reposo
const float voltaje_minimo = 0.85;

// Valor minimo del acelerador para evitar fallos por picos                       
const float minimo_acelerador = 1.15;

// (True) si se desea activar la posibilidad de acelerar desde parado a
// 6 km/h arrancando con el freno pulsado
const boolean frenopulsado = false;

// Permite usar el acelerador desde parado a 6 km/h
// Si frenopulsado esta en True, esta se debe dejar en False
boolean ayuda_salida = false;

//======= FIN VARIABLES CONFIGURABLES POR EL USUARIO ===================

#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

//======= PINES ========================================================
const int pin_pedal = 2; // Pin sensor pas, en Nano/Uno usar solo 2 o 3
const int pin_freno = 3; // Pin de activacion del freno
const int pin_acelerador = A0; // Pin acelerador
// Resto de pines 9,10,11

//======= VARIABLES PARA CALCULOS ======================================

// Tiempo en milisegundos para contar pulsos
const int tiempo_cadencia = 250;

// Variables para Millis()
unsigned long tcadencia1;
unsigned long tcadencia2;
unsigned long tcrucero;
boolean delta = false;

// Backup voltaje
float bkp_voltaje = voltaje_minimo;

// Contadores de paro, aceleracion y auto_progresivo
unsigned contador_retardo_paro_motor = 0;
int contador_retardo_aceleracion = 0;
unsigned contador_retardo_inicio_progresivo = 0;
int bkp_contador_retardo_aceleracion = 0;
boolean auto_progresivo = false;

// Variables progresivos
float fac_m = 0;
float fac_n = 0;
float fac_p = 0.6222 - 0.0222 * suavidad_progresivos;

// Variables para autoprogresivos
float fac_s = 0;
float fac_t = 0;
float fac_b = 1;
float fac_a = 0;
float fac_c = suavidad_autoprogresivos / 10.0;

float v_acelerador; // Valor recogido del acelerador
float v_crucero_ac; // Valor de crucero del acelerador
float v_crucero = 0.85; // Velocidad de crucero
// Los voltios que se mandan a la controladora
float nivel_aceleracion = voltaje_minimo;

// Contador de pulsos del pedal
int pulsos = 0;

//======= Variables interrupcion =======================================
// Variable donde se suman los pulsos del sensor PAS
volatile int p_pulsos = 0;

//======= FUNCIONES ====================================================

void pedal() {
	p_pulsos++;
}

void establece_crucero() {
	// Pasa a escala de 0-5 voltios
	v_acelerador = (v_acelerador * 5 / 1023);

	if (v_acelerador > minimo_acelerador) {
		v_crucero_ac = v_acelerador;
		v_crucero = v_crucero_ac;
	}
}

void lee_acelerador() {
	for (int f=1; f <= 30 ; f++) {
		// Lee valor
		v_acelerador = v_acelerador + analogRead(pin_acelerador); 
	}

	v_acelerador = v_acelerador / 30;
}

void manda_acelerador() {
	if (modo_crucero == true) {
		// Progresivo no lineal		
		fac_n = voltaje_minimo;
		fac_m = (v_crucero - voltaje_minimo)
		/ pow(retardo_aceleracion,fac_p);
		nivel_aceleracion = fac_n + fac_m *
		pow(contador_retardo_aceleracion,fac_p);

		if (nivel_aceleracion < voltaje_minimo) {
			nivel_aceleracion = voltaje_minimo;
		}

		if (nivel_aceleracion > v_crucero) {
			nivel_aceleracion = v_crucero;
		}
	} else {
		nivel_aceleracion = v_acelerador;
	}

	if (nivel_aceleracion != bkp_voltaje) {
		bkp_voltaje = nivel_aceleracion;
		// Ajusta el voltaje a valor entre 0-4096 (Resolucion del DAC)
		uint32_t valor = (4096/5) * nivel_aceleracion;
		// Fija voltaje en DAC
		dac.setVoltage(valor,false);
	}  
}

void para_motor() {
	contador_retardo_aceleracion = 0;
}

void freno() {
	contador_retardo_inicio_progresivo = retardo_inicio_progresivo;
	bkp_contador_retardo_aceleracion = 0;
	para_motor();

	if (freno_anula_crucero == true) {
		v_crucero = voltaje_minimo;
	}
}

void ayuda_arranque() {
	// Guardamos valor de velocidad de crucero anterior a la asistencia
	// Puede ser que no hubiera crucero (0.85)
	float vcruceroprev = v_crucero;

	// Mientras aceleramos y no pedaleamos
	while (analogRead(pin_acelerador) > 220 && p_pulsos == 0) { // De 190 a 897
		// Fijamos crucero a 6 km/h
		v_crucero = 2.10;
		contador_retardo_aceleracion++;
		manda_acelerador();
		// Corrige duracion del bucle de 30 sg
		delay(50);
	}

	// Recuperamos valor de velocidad de crucero anterior a la asistencia
	// Puede ser que no hubiera crucero (0.85)
	v_crucero = vcruceroprev;
}

void setup() {
	// Configura pines
	pinMode(pin_freno,OUTPUT);
	digitalWrite(pin_freno,HIGH);
	pinMode(pin_pedal,INPUT_PULLUP);
	pinMode(pin_acelerador,INPUT);

	// Prepara las interrupciones
	// Interrupcion pedal
	attachInterrupt(digitalPinToInterrupt(pin_pedal),pedal,CHANGE);
	// Interrupcion freno
	attachInterrupt(digitalPinToInterrupt(pin_freno),freno,FALLING);

	// Si arrancamos con el freno pulsado
	if (frenopulsado == true) {	
		if (digitalRead(pin_freno) == LOW) {
			// Activamos la ayuda desde parado a 6kmh
			ayuda_salida = true;
		}
	}

	// Ajusta configuracion 
	retardo_paro_motor = retardo_paro_motor * (1000 / tiempo_cadencia);
	retardo_aceleracion = retardo_aceleracion
	* (1000 / tiempo_cadencia);
	retardo_inicio_progresivo = retardo_inicio_progresivo
	* (1000 / tiempo_cadencia);
	// Anulamos el retardo por seguridad para que empiece progresivo
	// al encender la bici
	contador_retardo_inicio_progresivo = retardo_inicio_progresivo;

	// Calculo de factores para autoprogresivo
	if (retardo_inicio_progresivo > 0) {
		fac_s = retardo_paro_motor * 2.0;
		fac_t = (retardo_aceleracion * 1.0)
		/((retardo_aceleracion-fac_s)* 1.0);
		fac_b = (1.0/(retardo_aceleracion-fac_s)-fac_t)
		/(pow((retardo_inicio_progresivo-1.0),fac_c) - pow(1.0,fac_c));
		fac_a = fac_t-pow(1.0,fac_c) * fac_b;
		if (!desacelera_al_parar_pedal) {
			fac_b = (1.0/retardo_aceleracion-1.0)
			/(pow((retardo_inicio_progresivo-1.0),fac_c)
			- pow(1.0,fac_c));
			fac_a = 1.0-pow(1.0,fac_c) * fac_b;
		}
	}

	// Configura DAC
	dac.begin(dir_dac);
	// Fija voltaje inicial en Dac (0.85v)
	dac.setVoltage(810,false);
	// Arrancar tiempo inicio para comprobar cadencia
	tcadencia1 = millis();
	// Arrancar tiempo inicio para establecer crucero
	tcrucero = millis();
}

void loop() {
	tcadencia2 = millis();

	if (tcadencia2 > tcadencia1+(unsigned long)tiempo_cadencia) {
		pulsos = p_pulsos;
		tcadencia1 = millis();
		p_pulsos = 0;
		delta = true;
	}

	// Filtro de picos en acelerador
	v_acelerador = 0;

	lee_acelerador();

	// Si lo leemos y establecemos continuamente, no lo fija
	if (tcadencia2 > tcrucero+100) { // Si ha pasado 100 ms 
		tcrucero = millis(); // Actualiza tiempo actual
		establece_crucero();
	}

	if (pulsos < cadencia) {
		if (delta) {
			contador_retardo_paro_motor++;
		}

		if (contador_retardo_paro_motor > retardo_paro_motor) {
			if (delta) {
				contador_retardo_inicio_progresivo++;
			}
			auto_progresivo = true;

    		if (contador_retardo_aceleracion > 4) {
				bkp_contador_retardo_aceleracion =
				contador_retardo_aceleracion;
			}
        	para_motor();
		}
	}

	if (pulsos >= cadencia) {
		if (contador_retardo_inicio_progresivo <
		retardo_inicio_progresivo && auto_progresivo) {

			if (bkp_contador_retardo_aceleracion >
			retardo_aceleracion - fac_s) {
				bkp_contador_retardo_aceleracion =
				retardo_aceleracion - fac_s;
			}

			contador_retardo_aceleracion =
			bkp_contador_retardo_aceleracion *
			(fac_a+fac_b*pow(contador_retardo_inicio_progresivo,fac_c))
			* v_crucero/v_max_acelerador;
		}

		auto_progresivo = false;

		contador_retardo_inicio_progresivo = 0;
		contador_retardo_paro_motor = 0;

		if (delta && contador_retardo_aceleracion <
		retardo_aceleracion) {
			contador_retardo_aceleracion++;
		}
	}

	// Desacelera motor si se dejan de mover los pedales
	// Si se pedalea mas flojo no desacelera      
	if (delta && pulsos == 0 && desacelera_al_parar_pedal == true) {
		if (contador_retardo_aceleracion > 0) {
			contador_retardo_aceleracion =
			contador_retardo_aceleracion - 2;
			if (contador_retardo_aceleracion < 0) {
				contador_retardo_aceleracion = 0;
			}
		}
	}

	if (pulsos == 0 && contador_retardo_aceleracion == 0
	&& contador_retardo_paro_motor >= retardo_paro_motor
	&& ayuda_salida) {
		ayuda_arranque();
	}

	manda_acelerador();

	delta = false;
}

// Con_Acelerador_DAC_Millis_ProgNL_6kmh 1.7.1
