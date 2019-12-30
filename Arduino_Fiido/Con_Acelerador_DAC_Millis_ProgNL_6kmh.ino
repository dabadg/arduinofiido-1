#include <Adafruit_MCP4725.h>
#include <Arduino.h>
#include "I2CScanner.h"
#include "Tones.h"
#include "config.h"

const char* version = "Custom d0s1s v1.0";

/*
                     Versión Con Acelerador y DAC
                 Con_Acelerador_DAC_Millis_ProgNL_6kmh
------------------------------------------------------------------------
PRINCIPALES NOVEDADES:
 * Detección de pedaleo con millis().
 * Progresivos y Auto Progresivos no lineales.
 * Posibilidadad de asistir a 6km/h desde parado.
 * Posibilidadad de cortar crucero al frenar.
 * Añadido buzzer para emitir avisos en la inicialización.
 * Posibilidad de elegir el tipo de curcero.
 * Ajuste a la normativa de las Pedelec.
------------------------------------------------------------------------
CÓDIGO FUENTE:
 * https://github.com/d0s1s/arduinofiido/custom-d0s1s
------------------------------------------------------------------------
SELECCIÓN DE MODOS EN EL ARRANQUE:
 * Al encender la bici se pueden seleccionar distintos modos jugando con
 * el freno.
 * MODO_ACELERADOR --> Acelerador de fábrica legalizado. No interactuar
 * el freno.
 * MODO_CRUCERO (1 segundo) --> Crucero. Manteniendo el freno sin soltar
 * en el arranque durante -desde 1.5 a 2.5- segundos y soltarlo.
 * MODO_CRUCERO6KMH (3 segundos) --> Crucero + Asistencia 6 km/h.
 * Manteniendo el freno sin soltar en el arranque durante -desde 3.5 a
 * 9.5- segundos y soltarlo.
 * MODO_TEST_SENSORES (10 segundos) --> Modo Debug Serial.
 * Manteniendo el freno sin soltar en el arranque durante más de 10.5
 * segundos y soltarlo.
------------------------------------------------------------------------
FIJADO Y DESFIJADO DEL NIVEL DE ASISTENCIA:
 * Se trata de guardar el último valor del acelerador
 * para no tener que estar sujetando el acelerador.
 *
 * La idea es fijar el acelerador a la velocidad deseada, esperar el
 * tiempo deseado y soltar de golpe para guardar el voltaje como nivel
 * de asistencia.
 *
 * Al parar y volver a pedalear, se va incrementando voltaje
 * gradualmente hasta llegar al nivel fijado.
 * Si se vuelve a mover el acelerador se toma este como nuevo valor.
 * Usamos un pin analógico de entrada conectado al
 * acelerador, por otra parte, mandaremos a la placa DAC mediante
 * comunicacion i2c el valor de salida hacia la controladora.
 *
 * Para desfijar el nivel de asistencia, simplemente accionar
 * rápidamente el acelerador y soltar.
------------------------------------------------------------------------
LEGALIZACIÓN ACELERADOR:
 * Básicamente lo que hace es detectar pulsos mediante una
 * interrupción en el pin (pin_pedal). Si no se pedalea, no asiste el
 * acelerador.
------------------------------------------------------------------------
PROGRESIVOS:
 * Una vez fijemos una velocidad de crucero con el acelerador, si
 * paramos de pedalear durante más de 10 segundos o accionamos el freno,
 * al volver a reanudar el pedaleo, la asistencia se iniciará desde 0
 * progresivamente hasta volver a alcanzar la velocidad anteriormente
 * fijada.
------------------------------------------------------------------------
AUTO PROGRESIVOS:
 * Si se deja de pedalear, el motor se para como de costumbre, pero si
 * continuamos pedaleando antes de transcurridos 10 segundos no inciará
 * el progresivo desde 0 si no que el motor continuará a una velocidad
 * ligeramente inferior a la que íbamos.
 *
 * Si se frena antes de los 10 segundos se anula la función y comenzará
 * el progresivo desde cero.
------------------------------------------------------------------------
ASISTENCIA A 6 KM/H DESDE PARADO:
 * Si no se pedalea y mientras el acelerador esté accionado, se asiste a
 * 6 km/h, ajustándose a la normativa.
 * Si se comienza a pedalear sin dejar de accionar el acelerador --> se
 * sale a la velocidad con la que vayamos regulando con el acelerador.
------------------------------------------------------------------------
LINKS:
 * Ayuda, sugerencias, preguntas, etc. en el grupo Fiido Telegram:
 * 
 *                      http://t.me/FiidoD1Spain
 * 
 * Grupo Telegram de desarrollo privado. Si vas a montar el circuito y
 * necesitas ayuda o colaborar pide acceso en el general de arriba.
 *  
 * Canal con montaje, enlaces, programas, etc. http://t.me/fiidolegal
------------------------------------------------------------------------
DEVELOPERS:
 * dabadg y d0s1s a partir de la versión de ciberus y fulano con las
 * aportaciones de chusquete.
------------------------------------------------------------------------
AGRADECIMIENTOS:
 * Grupo de Telegram de desarrollo privado y toda su gente
 * pruebas, ideas, feedback, etc.
 *
 * Gracias a zereal por sus ideas de concepto y a faeletronic,
 * basbonald, Manoplas y demás compañeros por el testing.
 */

I2CScanner i2cScanner;
Adafruit_MCP4725 dac;
ConfigContainer cnf;

//======= PINES ========================================================

// Pin del acelerador.
const int pin_acelerador = A0;
// Pin sensor PAS, en Nano/Uno usar 2 ó 3.
const byte pin_pedal = 2;
// Pin de activación del freno.
const byte pin_freno = 3;
// Pin del zumbador.
const byte pin_piezo = 11;

//======= CONSTANTES MODOS =============================================

// Sólo Acelerador.
const byte MODO_ACELERADOR = 0;
// Acelerador con crucero y progresivos.
const byte MODO_CRUCERO = 1;
// Acelerador con crucero, asistencia 6km/h y progresivos.
const byte MODO_CRUCERO6KMH = 2;
// Serial Plotter.
const byte MODO_PLOTTER = 20;

//======= VARIABLES PARA CÁLCULOS ======================================

/* Valores del acelerador en escala de 4.3 voltios.
 * 0 --> 1023 = 0 --> 4.3v.
 * 209 --> 0.88v.
 * 238 --> 1.00v.
 * 440 --> 1.85v.
 * 809 --> 3.40v.
 * 842 --> 3.54v.
 */

// Valores de reposo y límite tomados de la lectura real del acelerador
// en el inicio, alimentando a 4.3 voltios.

int a0_valor_reposo = 209;
const int a0_valor_corte = 238;
const int a0_valor_minimo = 330;
const int a0_valor_6kmh = 440;
const int a0_valor_alto = 809;
int a0_valor_limite = 842;

// Constantes para la detección del pedaleo.
const double pas_factor_min = 0.05;
const double pas_factor_max = 1.50;

// Contadores de paro, aceleración y auto_progresivo.
int contador_retardo_aceleracion = 0;
unsigned long contador_retardo_inicio_progresivo = 0;
int bkp_contador_retardo_aceleracion = 0;
boolean auto_progresivo = false;

// Constante progresivos.
const float fac_p = 1.056 - 0.056 * cnf.suavidad_progresivos;

// Variables para auto_progresivos.
float fac_b = 0.0;
float fac_a = 0.0;
const float fac_c = cnf.suavidad_autoprogresivos / 10.0;

// Almacena el último valor asignado al DAC.
int nivel_aceleracion_prev = a0_valor_reposo;
// Valor recogido del acelerador.
int v_acelerador = a0_valor_reposo;
// Valor de crucero del acelerador.
int v_crucero = a0_valor_reposo;
// Variable que almacena el estado de notificación de fijar crucero.
boolean crucero_fijado = false;

// Controles de tiempo.
const unsigned long tiempo_act = 500;
unsigned long control_contadores;
unsigned long establece_crucero_ultima_ejecucion_millis;
unsigned long anula_crucero_con_freno_ultima_ejecucion_millis;
boolean actualizacion_contadores = false;

// Almacena la velocidad de crucero del loop anterior.
int vl_acelerador_prev = 0;
// Contador para la lectura del acelerador no accionado.
byte contador_acelerador_noaccionado = 0;
// Contador para la lectura del acelerador accionado.
byte contador_acelerador_accionado = 0;
// Contador para el loop del crucero.
byte contador_loop_crucero = 0;
// Cantidad de loops para cortar crucero con freno.
byte contador_freno_anulacion_crucero = 0;

// Pulsos de fijación crucero a partir de los que se emite el tono por
// el buzzer. 22 * 90 = 1980 ms.
const int limite_tono_pulsos_fijar_crucero = 22;

// Flag de activación de asistencia 6km/h y crucero, al arrancar con el
// freno pulsado.
byte flag_modo_asistencia = MODO_ACELERADOR;

// Estado del freno. True --> Desactivado. False --> Activado.
boolean freno = true;

//======= Variables de interrupción ====================================

// Último tiempo de cambio de estado del sensor PAS.
volatile unsigned long ultimo_evento_pas = millis();
// Variable para la detección del pedaleo.
volatile boolean pedaleo = false;
// Señales altas del PAS (Determina dirección de pedaleo).
volatile int tiempo_pas_activado = 0;
// Señales bajas del PAS (Determina dirección de pedaleo).
volatile int tiempo_pas_desactivado = 0;
// Cuántos valores erróneos consecutivos da el PAS.
volatile int fallo_pas = 0;

//======= FUNCIONES ====================================================

// --------- Utilidades

// Pasamos de escala acelerador --> DAC.
int aceleradorEnDac(int vl_acelerador) {
	return map(vl_acelerador, 0, 1023, 0 , 4095); 
}

// Calcula si el valor se encuantra entre el rango de valores con
// tolerancia calculados con el valor2.
// valor2-tolerancia > valor < valor2+tolerancia
boolean comparaConTolerancia(int valor, int valorReferencia, byte toleranciaValor2) {
	return (valor > (valorReferencia - toleranciaValor2)) && (valor < (valorReferencia + toleranciaValor2));
}

// --------- Pedal

void pedal() {
	if (ultimo_evento_pas > (unsigned long)(millis() - 10))
		return;

	// Lectura del Sensor PAS.
    boolean estado_pas = digitalRead(pin_pedal);

	// Tomamos medidas.
    if (estado_pas) {
		// Medimos señal baja.
        tiempo_pas_desactivado = millis() - ultimo_evento_pas;
	} else {
		// Medimos señal alta.
		tiempo_pas_activado = millis() - ultimo_evento_pas;
	}

    ultimo_evento_pas = millis();
	fallo_pas = fallo_pas + 1;

	// Dividimos las señales altas entre las bajas.
	double pas_factor = (double) tiempo_pas_activado / (double) tiempo_pas_desactivado;

	// Comprobamos el resultados con un factor de tolerancia máximo y mínimo para la activación de pedaleo.
	if ((pas_factor > pas_factor_min) && (pas_factor < pas_factor_max)) {
		// Activamos pedaleo.
		pedaleo = true;
		// Reiniciamos variable.
		fallo_pas = 0;
	}

	// Debug del PAS.
	/*if (cnf.habilitar_consola) {
		Serial.print(tiempo_pas_activado);
		Serial.print(" ");
		Serial.print(tiempo_pas_desactivado);
		Serial.print(" ");
		Serial.println("");
	}*/
}

// --------- Acelerador

int leeAcelerador(byte nmuestras, boolean nivelar) {
	int cl_acelerador = 0;

	// Leemos nivel de acelerador tomando n medidas.
	for (byte f = 1; f <= nmuestras; f++) {
		cl_acelerador = cl_acelerador + analogRead(pin_acelerador);
	}

	cl_acelerador = (int) cl_acelerador / nmuestras;

	// Para corregir el valor por el real obtenido de la lectura.
	if (cl_acelerador > a0_valor_alto)
		a0_valor_limite = constrain(cl_acelerador, a0_valor_alto, a0_valor_limite);

	if (nivelar)
		cl_acelerador = constrain(cl_acelerador, a0_valor_reposo, a0_valor_limite);

	return cl_acelerador;
}

boolean validaMinAcelerador(byte nmuestras) {
	boolean status = false;

	// Inicializamos el valor mínimo del acelerador, calculando la media de las medidas si tiene acelerador.
	// En caso de no tener acelerador, mantenemos valor por defecto.
	// Esto es útil para controlar el corecto funcionamiento del acelerador, si este está presente.
	int l_acelerador_reposo = 0;

	// Tomamos 30 medidas para calcular la media.
	for (byte f = 1; f <= nmuestras; f++) {
		l_acelerador_reposo = l_acelerador_reposo + analogRead(pin_acelerador);
	}

	l_acelerador_reposo = (int) l_acelerador_reposo / nmuestras;

	// No queremos que la tolerancia supere el [a0_valor_corte].
	// Rango aceptable de 0.79 a 0.96 v.
	if (comparaConTolerancia(l_acelerador_reposo, a0_valor_reposo, 20)) {
		a0_valor_reposo = l_acelerador_reposo;
		status = true;
	}

	delay(100);

	return status;
}

// Compatible con Plotter y Monitor.
void testSensores(unsigned long tiempoMs) {
	delay(1000);
	repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, 1000, 0);

	Serial.begin(19200);
	while (!Serial) {};
	Serial.print("Con_Acelerador_DAC_Millis_ProgNL_6kmh ");
	Serial.println(version);

	delay(1000);

	// Durante [N] sg monitorizamos los valores de los sensores cada 200 ms.
	unsigned long inicio_ejecucion_millis = millis();
	byte pp = 0;

	while (tiempoMs == 0 || (tiempoMs > 0 && (unsigned long)(millis() - inicio_ejecucion_millis) < tiempoMs)) {
		delay(100);
		// Acelerador.
		Serial.print(leeAcelerador(30, false));
		Serial.print("\t");
		// Pedaleo.
		Serial.print(digitalRead(pin_pedal) ? 500 : 250);
		Serial.print("\t");
		// Frenadas.
		Serial.print(digitalRead(pin_freno) ? 250 : 500);
		Serial.println("");
	}

	// Nunca va a llegar a este punto si no se produce algún error, ya que el anterior while es bloqueante.
	repeatTones(pin_piezo, cnf.buzzer_activo, 3, 3000, 1000, 100);
	Serial.end();
}

void seleccionaModo() {
	// Ejecutamos la selección de modos y esperamos a que se suelte el freno.
	// Según el tiempo que se tenga el freno pulsado, se cambiará de modo.
	// MODO_ACELERADOR - Sólo Acelerador.
	// MODO_CRUCERO (1 segundo) - Crucero.
	// MODO_CRUCERO6KMH (3 segundos) - Crucero + Asistencia 6 km/h.
	// MODO_TEST_SENSORES (10 segundos) - Modo Debug Serial.
	// Modo por defecto.
	flag_modo_asistencia = MODO_ACELERADOR;

	unsigned long timer_seleccion_modos = millis();
	byte ccont = 0;

	// Si se arranca con el freno sin activar, se emite tono de inicialización.
	if (digitalRead(pin_freno) != LOW)
		repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, 100, 1000);

	// Si se arranca con el freno activado, los tonos de inicialización serán los de selección de modos.
	while (digitalRead(pin_freno) == LOW) {
		if ((unsigned long)(millis() - timer_seleccion_modos) > 1000) {
			ccont++;

			switch (ccont) {
			  case 1:
				repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3100, 100, 0);
				flag_modo_asistencia = MODO_CRUCERO;
				break;

			  case 3:
				// Sólo se selecciona este modo si en el cnf está la variable ayuda_salida_activa a true
				// Se mantiene esta opción por si alquien no quiere llevarla activa bajo nungún concepto.
				if (cnf.ayuda_salida_activa) {
					repeatTones(pin_piezo, cnf.buzzer_activo, 3, 3100, 50, 50);
					flag_modo_asistencia = MODO_CRUCERO6KMH;
				}
				break;

			  case 10:
				repeatTones(pin_piezo, cnf.buzzer_activo, 1, 4000, 500, 0);
				testSensores(0);
				break;
			}

			delay(100);
			timer_seleccion_modos = millis();
		}
	}
}

// Progresivo no lineal.
int calculaAceleradorProgresivoNoLineal() {
	float fac_n = a0_valor_reposo + 0.2 * v_crucero;
	float fac_m = (v_crucero - a0_valor_reposo) / pow (cnf.retardo_aceleracion, fac_p);
	int nivel_aceleraciontmp = (int) freno * (fac_n + fac_m * pow (contador_retardo_aceleracion, fac_p));

	return constrain(nivel_aceleraciontmp, a0_valor_reposo, v_crucero);
}

// --------- Crucero

void anulaCrucero() {
	if (crucero_fijado) {
		v_crucero = a0_valor_reposo;
		crucero_fijado = false;
		repeatTones(pin_piezo, cnf.buzzer_activo, 1, 2000, 290, 100);
	}
}

void estableceNivel(int vl_acelerador) {
	// Esperamos 50 ms para ejecutar.
	if ((unsigned long) (millis() - establece_crucero_ultima_ejecucion_millis) > 50) {
		contador_loop_crucero++;

		// Si el acelerador está accionado.
		if (vl_acelerador > a0_valor_corte) {
			contador_acelerador_accionado++;
			contador_acelerador_noaccionado = 0;
		// Si el acelerador no está accionado.
		} else {
			contador_acelerador_noaccionado++;

			if (contador_acelerador_accionado > 7 || contador_acelerador_noaccionado > 10) {
				contador_acelerador_accionado = 0;
			}
		}

		// Cortamos crucero con toque corto (accionar y soltar) del acelerador.
		if (contador_acelerador_accionado > 2 && contador_acelerador_accionado < 8 && contador_acelerador_noaccionado > 0) {
			// Reinicio de contador.
			contador_acelerador_accionado = 0;
			// Anulamos crucero.
			anulaCrucero();
		}

		establece_crucero_ultima_ejecucion_millis = millis();
	}

	// Si las vueltas del loop son mayores a los pulsos para fijar crucero.
	if (contador_loop_crucero > cnf.pulsos_fijar_crucero) {
		// Reinicio de contador.
		contador_loop_crucero = 0;

		// Si tenemos accionado el acelerador y la comparación de valores está dentro del rango de tolerancia.
		if (vl_acelerador > a0_valor_minimo && comparaConTolerancia(vl_acelerador, vl_acelerador_prev, 20)) {
			// Si la velocidad a fijar es diferente a la ya fijada.
			if (!comparaConTolerancia(vl_acelerador, v_crucero, 20)) {
				// Fijamos crucero.
				crucero_fijado = true;

				// Sólo permitimos que suene el buzzer avisando de que se ha fijado el crucero con valores altos.
				if (cnf.pulsos_fijar_crucero >= limite_tono_pulsos_fijar_crucero) {
					repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, 190, 1);
				}

				// Nunca se actualizará la velocidad de crucero por debajo del [a0_valor_minimo].
				v_crucero = vl_acelerador < a0_valor_minimo ? a0_valor_reposo : vl_acelerador;
				// Reiniciamos variable.
				vl_acelerador_prev = 0;
			}
		} else {
			// Asignamos el valor actual a [vl_acelerador_prev].
			vl_acelerador_prev = vl_acelerador;
		}
	}
}

void anulaCruceroConFreno() {
	// Esperamos 100 ms para ejecutar.
	if ((unsigned long)(millis() - anula_crucero_con_freno_ultima_ejecucion_millis) > 100) {
		if (freno == 0) {
			contador_freno_anulacion_crucero++;

			if (crucero_fijado) {
				// Añadido % 8 para solo ejecutar la acción para los múltiplos de 8 y evitar excesivos tonos.
				if (contador_freno_anulacion_crucero % 8 == 0) {
					repeatTones(pin_piezo, cnf.buzzer_activo, 1, (3000 + (contador_freno_anulacion_crucero * 20)), 90, 200);
				}

				if (contador_freno_anulacion_crucero >= cnf.pulsos_liberar_crucero_con_freno) {
					anulaCrucero();
				}
			}
		} else {
			if (contador_freno_anulacion_crucero > 0)
				contador_freno_anulacion_crucero--;
		}

		anula_crucero_con_freno_ultima_ejecucion_millis = millis();
	}
}

// --------- Generales

void setup() {
	// Si cnf.dir_dac está a 0 se autodetecta la dirección del DAC.
	if (cnf.dir_dac == 0) {
		i2cScanner.Init();
	} else {
		i2cScanner.Init(cnf.dir_dac);
	}

	if (i2cScanner.isDacDetected()) {
		// Configura DAC.
		dac.begin(i2cScanner.getDacAddress());
		// Fija voltaje inicial en DAC.
		dac.setVoltage(aceleradorEnDac(a0_valor_reposo), false);

		// Configura pines.
		pinMode(pin_piezo, OUTPUT);
		pinMode(pin_pedal, INPUT_PULLUP);
		pinMode(pin_freno, INPUT_PULLUP);
		pinMode(pin_acelerador, INPUT);
		pinMode(pin_freno, INPUT);

		// Interrupción pedal.
		attachInterrupt(digitalPinToInterrupt(pin_pedal), pedal, CHANGE);

		// Seleccionamos el modo de funcionamiento.
		seleccionaModo();

		// Validamos el acelerador en reposo.
		validaMinAcelerador(30);

		// Inicia serial.
		if (cnf.habilitar_consola) {
			Serial.begin(19200);
			while (!Serial) {};
			Serial.print("Con_Acelerador_DAC_Millis_ProgNL_6kmh ");
			Serial.println(version);
		}

		// Ajusta configuración.
		cnf.retardo_aceleracion = cnf.retardo_aceleracion * (1000 / tiempo_act);
		cnf.retardo_inicio_progresivo = cnf.retardo_inicio_progresivo * (1000 / tiempo_act);

		// Anulamos el retardo por seguridad para que empiece progresivo al encender la bici.
		contador_retardo_inicio_progresivo = cnf.retardo_inicio_progresivo;

		// Cálculo de factores para auto_progresivo.
		if (cnf.retardo_inicio_progresivo > 0) {
			fac_b = (1.0 / cnf.retardo_aceleracion - 1.0) / (pow ((cnf.retardo_inicio_progresivo - 1.0), fac_c) - pow (1.0, fac_c));
			fac_a = 1.0 - pow (1.0, fac_c) * fac_b;
		}

		// Estabiliza pulsos_fijar_crucero.
		cnf.pulsos_fijar_crucero = constrain(cnf.pulsos_fijar_crucero, 2, 40);
		// Estabiliza pulsos para liberar crucero con el freno.
		cnf.pulsos_liberar_crucero_con_freno = constrain(cnf.pulsos_liberar_crucero_con_freno, 2, 40);
		// Estabiliza suavidad de los progresivos.
		cnf.suavidad_progresivos = constrain(cnf.suavidad_progresivos, 1, 10);
		// Estabiliza suavidad de los auto_progresivos.
		cnf.suavidad_autoprogresivos = constrain(cnf.suavidad_autoprogresivos, 1, 10);
		// Tono de finalización configuración del sistema.
		repeatTones(pin_piezo, cnf.buzzer_activo, 3, 3000, 90, 90);
	} else {
		// Tonos de error en detección de DAC.
		DAC_ERR_TONE(pin_piezo);
	}
}

void loop() {
	// Si el DAC es detectado.
	if (i2cScanner.isDacDetected()) {
		// Reposo al nivel de aceleración.
		int nivel_aceleracion = a0_valor_reposo;
		
		// Esperamos 500 ms para verificar la desactivación del pedaleo.
		if (((unsigned long)(millis() - ultimo_evento_pas) > 500) || (fallo_pas > cnf.tolerancia_pas)) {
			// Si el sensor PAS no cambia en más de 0.5 segundos, no estamos pedaleando.
			pedaleo = false;
		}

		// Control para el incremento de contadores.
		if (flag_modo_asistencia >= MODO_CRUCERO) {
		// Esperamos [tiempo_act] para ejecutar.
			if (((unsigned long)(millis() - control_contadores) > tiempo_act)) {
				actualizacion_contadores = true;
				control_contadores = millis();
			}
		}

		// Lecturas de los sensores.
		v_acelerador = leeAcelerador(30, true);
		freno = digitalRead(pin_freno);

		// Si el freno está pulsado.
		if (freno == 0) {
			// Reiniciamos contadores.
			contador_retardo_inicio_progresivo = cnf.retardo_inicio_progresivo;
			contador_retardo_aceleracion = 0;
			bkp_contador_retardo_aceleracion = 0;
		// Si el freno no está pulsado.
		} else {
			// Si no se pedalea.
			if (!pedaleo) {
				// Control para la asistencia de 6 km/h desde parado y corte de crucero sin pedalear.
				if (flag_modo_asistencia >= MODO_CRUCERO && v_acelerador > a0_valor_minimo) {
					if (flag_modo_asistencia == MODO_CRUCERO6KMH)
						nivel_aceleracion = a0_valor_6kmh;

					if (cnf.liberar_crucero_con_acelerador_sin_pedaleo == true)
						anulaCrucero();
				}

				if (actualizacion_contadores)
					contador_retardo_inicio_progresivo++;

				// Lanzamos auto_progresivo.
				auto_progresivo = true;

				if (contador_retardo_aceleracion > 4) {
					bkp_contador_retardo_aceleracion = contador_retardo_aceleracion;
				}

				// Reiniciamos contador.
				contador_retardo_aceleracion = 0;
			// Si se pedalea.
			} else {
				if (auto_progresivo && contador_retardo_inicio_progresivo < cnf.retardo_inicio_progresivo) {
					if (bkp_contador_retardo_aceleracion > cnf.retardo_aceleracion) {
						bkp_contador_retardo_aceleracion = cnf.retardo_aceleracion;
					}

					contador_retardo_aceleracion = (int) freno * bkp_contador_retardo_aceleracion * (fac_a + fac_b * pow (contador_retardo_inicio_progresivo, fac_c)) * v_crucero / a0_valor_limite;
				}

				// Quitamos auto_progresivo.
				auto_progresivo = false;
				// Reiniciamos contador.
				contador_retardo_inicio_progresivo = 0;

				if (actualizacion_contadores && contador_retardo_aceleracion < cnf.retardo_aceleracion) {
					contador_retardo_aceleracion++;
				}

				if (flag_modo_asistencia >= MODO_CRUCERO) {
					// Control del crucero.
					estableceNivel(v_acelerador);

					// Si el crucero está fijado.
					if (crucero_fijado) {
						// Si no se está acelerando.
						if (v_acelerador < a0_valor_minimo) {
							nivel_aceleracion = calculaAceleradorProgresivoNoLineal();
						// Si se acelera.
						} else {
							nivel_aceleracion = v_acelerador;
						}
					// Si el crucero no está fijado.
					} else {
						nivel_aceleracion = v_acelerador;
					}
				// Modo crucero desactivado.
				} else {
					nivel_aceleracion = v_acelerador;
				}
			}
		}

		// Fijamos el acelerador si el valor anterior es distinto al actual.
		if (nivel_aceleracion_prev != nivel_aceleracion) {
			dac.setVoltage(aceleradorEnDac(nivel_aceleracion), false);
			nivel_aceleracion_prev = nivel_aceleracion;
		}

		if (flag_modo_asistencia >= MODO_CRUCERO) {
			if (cnf.liberar_crucero_con_freno)
				anulaCruceroConFreno();
			
			// Reinicio de variable.
			actualizacion_contadores = false;
		}
	}
}

// EOF
