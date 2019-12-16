#include <Adafruit_MCP4725.h>
#include <Arduino.h>
#include "I2CScanner.h"
#include "Level.h"
#include "Tones.h"

const char* version = "2.7.0 Develop N";

/*
                     Versión Con Acelerador y DAC
                 Con_Acelerador_DAC_Millis_ProgNL_6kmh
------------------------------------------------------------------------
PRINCIPALES NOVEDADES:
 * Detección de pulsos con millis().
 * Progresivos y Auto Progresivos no lineales.
 * Posibilidadad de asistir a 6km/h desde parado.
 * Posibilidadad de cortar crucero al frenar.
 * Añadido buzzer para emitir avisos en la inicialización.
 * Posibilidad de elegir el tipo de curcero.
------------------------------------------------------------------------
WIKI:
 * https://github.com/d0s1s/arduinofiido/wiki/Wiki-Arduino-Fiido
------------------------------------------------------------------------
FIJADO Y DESFIJADO DEL NIVEL DE ASISTENCIA (CONTINUO):
 * Se trata de guardar el último valor del acelerador
 * para no tener que estar sujetando el acelerador.
 *
 * La idea es fijar el acelerador a la velocidad deseada y soltar de
 * golpe o acompañar rápidamente hasta el valor 0, para guardar el
 * voltaje como nivel de asistencia.
 * Al parar y volver a pedalear, se va incrementando voltaje
 * gradualmente hasta llegar al nivel fijado.
 * Si se vuelve a mover el acelerador se toma este como nuevo valor.
 * Usamos un pin analógico de entrada conectado al
 * acelerador, por otra parte, mandaremos a la placa DAC mediante
 * comunicacion i2c el valor de salida hacia la controladora.
 *
 * Para desfijar el nivel de asistencia, simplemente usar el acelerador
 * sin pedalear.
------------------------------------------------------------------------
VERSIÓN CRUCERO TIPO "MONOPATÍN" (POR TIEMPO) --> FIJADO Y DESFIJADO DEL
NIVEL DE ASISTENCIA:
 * Al igual que la versión continua, se trata de fijar la potencia del
 * motor para no tener que estar sujetando el acelerador.
 * La idea principal es que mientras se pedalea, una vez se llegue a la
 * velocidad deseada, se mantenga la posición del acelerador durante
 * varios segundos para fijar la potencia de crucero y desde ese
 * instante se pueda soltar el acelerador.
 *
 * Con la configuración por defecto mientras el crucero permanezca
 * fijado, el acelerador actuará de un modo similar al sistema de
 * crucero que utilizan los coches.
 * Si se actúa sobre el acelerador estando su valor por debajo de la
 * potencia fijada, se mantendrá la potencia de crucero hasta que el
 * valor del acelerador la supere, momento en el que el valor del
 * acelerador prevalecerá y actualizará la potencia del motor.
 *
 * En todo momento mientras se pedalea, se podrá cambiar la potencia
 * de crucero manteniendo el acelerador en la posición deseada durante
 * 1.5 segundos ya sea por encima o por debajo de la potencia de crucero
 * seleccionada anteriormente.
 *
 * Existe una variante que es la versión tiempo_min que permite
 * interactuar con el acelerador fijando el crucero manteniendo 3s el
 * acelerador en la misma posición, pero permitiendo cambiar el crucero
 * a un valor más bajo cambiando el el valor del acelerador y
 * manteniendo la posición más baja durante 280ms.
 *
 * Para anular la potencia de crucero, existen dos procedimientos
 * principales --> Uno manteniendo pulsada la maneta de freno unos
 * segundos y otro, sin pedalear,subiendo el acelerador a un nivel
 * alto y soltando de golpe.
 * En cualquier caso, si se utiliza el sistema con la asistencia 6km/h
 * activada, al salir de la misma, siempre será anulada la potencia de
 * crucero.
 *
 * Al dejar de pedalear con el crucero fijado, la potencia del motor
 * caerá hasta la potencia de reposo y no se aplicará ninguna
 * asistencia. Una vez que se reinicie la marcha y se empiece a
 * pedalear, la potencia se irá incrementando progresivamente hasta
 * llegar a la potencia de crucero previamente fijada.
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
 * progresivo desde cero.
------------------------------------------------------------------------
ASISTENCIA A 6 KM/H DESDE PARADO:
 * Si no se pedalea y mientras el acelerador esté accionado, se asiste a
 * 6 km/h, ajustándose a la normativa.
 * Si se comienza a pedalear sin dejar de accionar el acelerador --> se
 * sale a la velocidad con la que vayamos regulando con el acelerador.
 * En caso de tener una potencia de crucero fijada, si se utiliza la
 * asistencia 6kmh, al salir de la misma el crucero será anulado.
 *
 * Según la configuración por defecto, la asistencia 6kmh empezará
 * aplicando una potencia alta y bajará hasta la potencia que se ajusta
 * a la normativa.
 * Este sistema de potencia invertida ha sido implementado para que las
 * salidas desde parado alcancen la potencia 6km/h rápidamente.
------------------------------------------------------------------------
VALIDACIÓN DEL ACELERADOR
 * Se ha implementado una validación de seguridad para que en caso de
 * detectarse una medida erronea del acelerador al inicializar el
 * sistema, este quede anulado y permita monitorizar todos los sensores
 * de la bicicleta durante 60 segundos.
 * Para la monitorización, será necesario activar un serial plotter.
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

//=================== VARIABLES CONFIGURABLES POR EL USUARIO ===========

/*
 * Externalización de fichero de configuración. Podrás seleccionar las
 * configuraciones que más te gusten.
 */

#include "config.h"

//======= FIN VARIABLES CONFIGURABLES POR EL USUARIO ===================

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

//======= CONSTANTES MODOS ======================================

const byte MODO_ACELERADOR = 0; // Solo Acelerador.
const byte MODO_CRUCERO = 1; // Acelerador con crucero y progresivos.
const byte MODO_CRUCERO6KMH = 2; // Acelerador con crucero, asistencia 6kmh y progresivos.
const byte MODO_PLOTTER = 20; // Serial Plotter.

//======= VARIABLES PARA CÁLCULOS ======================================

// Valores mínimos y máximos del acelerador leídos por el pin A0.
// Al inicializar, lee el valor real (a0_valor_reposo).
// 0 --> 1023 = 0 --> 5V.

int a0_valor_reposo = 194;			// 0.95
const int a0_valor_minimo = 330;	// 1.62
const int a0_valor_6kmh = 440;		// 2.16
int a0_valor_maximo = 808;			// 3.95
const int a0_valor_LIMITE = 832;	// 4.06

// Variables para la detección del pedaleo.
byte pulsos = 0;
byte a_pulsos = 0;

// Contadores de paro, aceleración y auto_progresivo.
int contador_retardo_aceleracion = 0;
unsigned long contador_retardo_inicio_progresivo = 0;
int bkp_contador_retardo_aceleracion = 0;
boolean auto_progresivo = false;

// Constante progresivos.
float fac_n = 0.0;
const float fac_p = 1.056 - 0.056 * cnf.suavidad_progresivos;

// Variables para auto_progresivos.
float fac_b = 0.0;
float fac_a = 0.0;
const float fac_c = cnf.suavidad_autoprogresivos / 10.0;

// Los voltios que se mandan a la controladora.
int nivel_aceleracion = a0_valor_reposo;
// Almacena el último valor asignado al DAC.
int nivel_aceleracion_prev = a0_valor_reposo;

// Ciclos de decremento cada 50 ms. De 0 a 255.
const byte ciclo_decremento_progresivo_ayuda_arranque = 50;
// Progresivo inverso para la asistencia de 6 km/h.
int decremento_progresivo_ayuda_arranque;

// Valor recogido del acelerador.
int v_acelerador = a0_valor_reposo;
// Valor de crucero del acelerador.
int v_crucero = a0_valor_reposo;
// Variable que almacena el estado de notificación de fijar crucero.
boolean crucero_fijado = false;

// Controles de tiempo.
const unsigned long tiempo_act = 500;
unsigned long loop_ultima_ejecucion_millis;
unsigned long crucero_fijado_millis;
unsigned long establece_crucero_ultima_ejecucion_millis;
unsigned long anula_crucero_con_freno_ultima_ejecucion_millis;
boolean actualizacion_contadores = false;

// Almacena la velocidad de crucero del loop anterior.
int vl_acelerador_prev = 0;
// Cantidad de loops que lleva la velocidad en el mismo valor.
byte contador_crucero_mismo_valor = 0;
// Cantidad de loops para cortar crucero con freno.
byte contador_freno_anulacion_crucero = 0;

// Pulsos de fijación crucero a partir de los que se emite el tono por
// el buzzer.
const int limite_tono_pulsos_fijar_crucero = 14;

unsigned long tiempo_sensores_habilitado = 60000;

// Flag de activación de asistencia 6kmh y crucero, al arrancar con el freno pulsado.
volatile byte flag_modo_asistencia = MODO_ACELERADOR;

// ¿Freno activado?.
boolean freno = true;

//======= Variables de interrupción ====================================

// Variable donde se suman los pulsos del sensor PAS.
volatile byte p_pulsos = 0;
// Variable para la detección del pedaleo.
volatile boolean pedaleo = false;

//======= FUNCIONES ====================================================

// --------- Utilidades

// Pasamos de escala acelerador -> DAC.
int aceleradorEnDac(int vl_acelerador) {
	return vl_acelerador * (4096 / 1024);
}

// --------- Pedal

void pedal() {
	// Pulsos por loop.
	p_pulsos++;

	// Activamos pedaleo por interrupciones.
	if (++a_pulsos >= cnf.interrupciones_activacion_pedaleo) {
		pedaleo = true;
		a_pulsos = 0;
	}
}

// --------- Motor

void paraMotor() {
	dac.setVoltage(aceleradorEnDac(a0_valor_reposo), false);
	nivel_aceleracion_prev = a0_valor_reposo;
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
 	if (cnf.recalcular_rango_max_acelerador && cl_acelerador > a0_valor_maximo && cl_acelerador <= a0_valor_LIMITE)
 		a0_valor_maximo = cl_acelerador;

	if (nivelar) {
		cl_acelerador = constrain(cl_acelerador, a0_valor_reposo, a0_valor_maximo);
	}

	return cl_acelerador;
}

int leeAcelerador(byte nmuestras) {
	return leeAcelerador(nmuestras, true);
}

void testSensoresPlotter(unsigned long tiempoMs) {
	delay(1000);
	repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, 1000, 0);

	if (!cnf.habilitar_consola){
		Serial.begin(19200);
		while (!Serial) {};
	}

	delay(1000);

	// Durante [N] sg monitorizamos los valores de los sensores cada 200 ms.
	unsigned long inicio_ejecucion_millis = millis();
	byte pp = 0;

	while (tiempoMs==0 || (tiempoMs > 0 && (unsigned long)(millis() - inicio_ejecucion_millis) < tiempoMs)) {
		delay(200);
		Serial.print(leeAcelerador(3, false));
		Serial.print("\t");
		p_pulsos = (p_pulsos > pp)?p_pulsos:0;
		pp = p_pulsos;
		Serial.print(p_pulsos * 2);
		Serial.print("\t");
		Serial.print(digitalRead(pin_freno) ? 10 : 500);
		Serial.println("");
	}

	Serial.end();
}

void seleccionaModo() {
	// Ejecutamos la selección de modos y esperamos a que se suelte el freno para dejar
	// paso a tomar la medida del acelerador evitando una lectura erronea por la caida de tensión.

	//Según el tiempo que se tenga el freno pulsado, se cambiará de modo.

	// MODO 0 - Solo Acelerador
	// MODO 1 (1 segundo) - Crucero
	// MODO 2 (3 segundos) - Crucero + asistencia 6kmh
	// MODO 20 (20 segundos) - Modo debug Serial Plotter

	// Modo por defecto.
	flag_modo_asistencia = MODO_ACELERADOR;

	unsigned long timer_seleccion_modos=millis();
	byte ccont = 0;
	repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, 100, 1000);

	while (digitalRead(pin_freno) == LOW) {
		if ((unsigned long)(millis() - timer_seleccion_modos) > 1000) {
			ccont++;

			switch (ccont) {
			  case 1:
				repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3100, 100, 0);
				flag_modo_asistencia = MODO_CRUCERO;
				break ;

			  case 3:
				// Solo se selecciona este modo si en el cnf está la variable ayuda_salida_activa a true
				// Se mantiene esta opción por si alquien no quiere llevarla activa bajo nungún concepto.
				if(cnf.ayuda_salida_activa){
					repeatTones(pin_piezo, cnf.buzzer_activo, 2, 3100, 100, 100);
					flag_modo_asistencia = MODO_CRUCERO6KMH ;
					decremento_progresivo_ayuda_arranque = (int) (cnf.v_salida_progresivo_ayuda_arranque - a0_valor_minimo) / ((cnf.tiempo_ejecucion_progresivo_ayuda_arranque / ciclo_decremento_progresivo_ayuda_arranque));
				}
				break;

			  case 20:
				repeatTones(pin_piezo, cnf.buzzer_activo, 1, 4000, 500, 0);
				testSensoresPlotter(0);
				break;

			}
			timer_seleccion_modos=millis();
		}
	}
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

	if (comparaConTolerancia(l_acelerador_reposo, a0_valor_reposo, 45)) {
		// Si queremos actualizar la variable de valor reposo con los valores reales tomados por el acelerador.
		if (cnf.recalcular_rango_min_acelerador) {
			a0_valor_reposo = l_acelerador_reposo;
		}

		status = true;
	// Si la medida del acelerador no es correcta, desactivamos el acelerador.
	} else {
		a0_valor_reposo = 0;
	}

	delay(100);

	return status;
}

// Progresivo no lineal.
int calculaAceleradorProgresivoNoLineal() {
	int nivel_aceleraciontmp;
	float fac_m = 0.0;
	float resultado = 0.0;

	fac_n = a0_valor_reposo + 0.2 * v_crucero;
	fac_m = (v_crucero - a0_valor_reposo) / pow (cnf.retardo_aceleracion, fac_p);
	resultado = freno * (fac_n + fac_m * pow (contador_retardo_aceleracion, fac_p));
	nivel_aceleraciontmp = (int) constrain(nivel_aceleraciontmp, a0_valor_reposo, v_crucero);

	return nivel_aceleraciontmp;
}


// --------- Crucero
void estableceCruceroPorTiempo(int vl_acelerador) {
	// Esperamos 100 ms para ejecutar.
	if ((unsigned long) (millis() - establece_crucero_ultima_ejecucion_millis) > 100) {

		// Calculamos la media de la velocidad de crucero actual y la de la vuelta anterior.
		float media_con_vcrucero_prev = (vl_acelerador_prev + vl_acelerador) / 2;

		// Si la velocidad es la misma incrementa el contador de control de fijación de crucero.
		if (pedaleo && vl_acelerador > a0_valor_minimo && comparaConTolerancia(vl_acelerador, (int) media_con_vcrucero_prev, 10)) {
			contador_crucero_mismo_valor++;

			// Si el contador de crucero ha llegado a su tope, se fija el crucero o si el acelerador está por debajo del crucero y el contador de debajo crucero ha llegado a su tope.
			if (contador_crucero_mismo_valor == cnf.pulsos_fijar_crucero || (cnf.pulsos_fijar_debajo_crucero > 0 && contador_crucero_mismo_valor == cnf.pulsos_fijar_debajo_crucero && vl_acelerador < v_crucero)) {

				// Solo se fija el crucero si se ha notado una variación de más de +-20 pasos entre la medida actual y la de crucero ya fijada.
				if (!comparaConTolerancia(vl_acelerador, v_crucero, 20)) {
					boolean crucero_arriba = vl_acelerador > v_crucero;
					crucero_fijado = true;
					// Nunca se actualizará la velocidad de crucero por debajo del valor_minimo
					v_crucero = vl_acelerador < a0_valor_minimo ? a0_valor_reposo : vl_acelerador;
					contador_crucero_mismo_valor = 0;
					crucero_fijado_millis = millis();

					// Sólo permitimos que suene el buzzer avisando de que se ha fijado el crucero con valores altos.
					// Valores altos se considera a partir de 2 segundos (14 pasos).
					if (cnf.pulsos_fijar_crucero >= limite_tono_pulsos_fijar_crucero) {
						repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, crucero_arriba?190:80, 1);
					}
				}
			}

		} else {
			contador_crucero_mismo_valor = 0;
		}

		vl_acelerador_prev = vl_acelerador;
		establece_crucero_ultima_ejecucion_millis = millis();
	}
}

void anulaCrucero() {
	if (crucero_fijado) {
		v_crucero = a0_valor_reposo;
		crucero_fijado = false;
		repeatTones(pin_piezo, cnf.buzzer_activo, 1, 2000, 290, 100);
	}
}

void anulaCruceroConFreno() {
	// Esperamos 100 ms para ejecutar.
	if ((unsigned long)(millis() - anula_crucero_con_freno_ultima_ejecucion_millis) > 100) {
		if (digitalRead(pin_freno) == LOW) {
			contador_freno_anulacion_crucero++;

			if (crucero_fijado) {
				// Añadido % 8 para solo ejecutar la acción para los múltiplos de 8 y evitar excesivos tonos.
				if (contador_freno_anulacion_crucero % 8 == 0) {
					repeatTones(pin_piezo, cnf.buzzer_activo, 1, (3000 + (contador_freno_anulacion_crucero * 20)), 90, 200);
				}

				if (contador_freno_anulacion_crucero >= cnf.pulsos_liberar_crucero) {
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

void anulaCruceroAcelerador() {
	// El procedimiento se ejecuta mientras no se pedalea, haciendo un cambio rápido
	// desde reposo hasta velocidad minima y vuelta a reposo.
	if (!pedaleo && crucero_fijado && cnf.liberar_crucero_con_acelerador){
		// Inicia en valor reposo
		if (comparaConTolerancia(leeAcelerador(10), a0_valor_reposo, 30)) {
			boolean unlock = false;
			// Espera a detectar interacción con el acelerador.
			unsigned long timer_liberar_crucero = millis();
			while((unsigned long)(millis() - timer_liberar_crucero) < 50) {
				delay(1);

				if(leeAcelerador(10) > a0_valor_reposo + 100) {
					repeatTones(pin_piezo, cnf.buzzer_activo, 1, 2300, 90, 120);
					unlock = true;
					break;
				}
			}

			if (unlock) {
				// Espera a que se suelte el acelerador para anular el crucero.
				timer_liberar_crucero = millis();

				while ((unsigned long)(millis() - timer_liberar_crucero) < cnf.tiempo_anula_crucero_acelerador) {
					delay(1);

					// Cancelamos el crucero si existía, en caso de no pedalear y haber soltado el acelerador.
					if (!pedaleo && comparaConTolerancia(leeAcelerador(30), a0_valor_reposo, 30)) {
						anulaCrucero();
						break;
					}
				}
			}
		}
	}
}

// --------- Asistencia 6 Km/h

void ayudaArranque() {
	unsigned long timer_progresivo_ayuda_arranque = millis();
	boolean ayuda_arranque_fijada = false;
	float v_salida_progresivo = cnf.v_salida_progresivo_ayuda_arranque;

	// Si está configurado el retardo de ayuda al arranque, retardamos la entrada
	// de la ayuda durante los ms leidos de la variable cnf.retardo_ayuda_arranque.
	// Con esto conseguimos evitar que si se toca acelerador se ejecute automáticamente
	// la asistencia y la bicicleta se ponga en marcha.
	if (cnf.retardo_ayuda_arranque > 0 && leeAcelerador(3) > a0_valor_minimo) {
		while (!pedaleo && (unsigned long)(millis() - timer_progresivo_ayuda_arranque) < cnf.retardo_ayuda_arranque) {
			delay(1);
		}
	}

	// Mientras no pedaleamos y aceleramos.
	while (!pedaleo && leeAcelerador(30) > a0_valor_minimo) {
		// Iniciamos la salida progresiva inversa.
		if (cnf.activar_progresivo_ayuda_arranque && v_salida_progresivo > a0_valor_6kmh) {
			// Ejecutamos la bajada de potencia hasta a0_valor_6kmh cada 50 ms.
			if ((unsigned long)(millis() - timer_progresivo_ayuda_arranque) >= ciclo_decremento_progresivo_ayuda_arranque) {
				v_salida_progresivo -= decremento_progresivo_ayuda_arranque;

				if (v_salida_progresivo < a0_valor_6kmh)
					v_salida_progresivo = a0_valor_6kmh;

				dac.setVoltage(aceleradorEnDac(v_salida_progresivo), false);
				nivel_aceleracion_prev = v_salida_progresivo;
				timer_progresivo_ayuda_arranque = millis();
			}
		} else {
			if (!ayuda_arranque_fijada) {
				// Mandamos 6 km/h directamente al DAC.
				dac.setVoltage(aceleradorEnDac(a0_valor_6kmh), false);
				nivel_aceleracion_prev = a0_valor_6kmh;
				ayuda_arranque_fijada = true;
			}
		}
	}

	if (!pedaleo && leeAcelerador(30) <= a0_valor_reposo) {
		paraMotor();
	}
}

// --------- Establecimiento de voltaje

void mandaAcelerador(int vf_acelerador) {
	// Asistencia desde parado a 6 km/h mientras se use el acelerador sin pedalear.
	if (flag_modo_asistencia == MODO_CRUCERO6KMH && !pedaleo && vf_acelerador > a0_valor_minimo && freno == 1) {
		ayudaArranque();
	} else {
		// Pedalemaos y no tenemos accionado el freno.
		if (pedaleo && freno == 1) {
			// Modo crucero activado.
			if (flag_modo_asistencia >= MODO_CRUCERO) {
				// Si el crucero está fijado.
				if (crucero_fijado) {
					// Si no se está acelerando o si mientras está activa la opción de acelerador bloqueado por debajo de crucero,  teniendo los pulsos de fijación crucero están por encima de 10 (fijación por tiempo) y se acciona el acelerador por debajo de la velocidad de crucero.
					if (vf_acelerador < a0_valor_minimo || (cnf.pulsos_fijar_debajo_crucero > 0 && cnf.pulsos_fijar_crucero >= 10 && vf_acelerador < v_crucero)) {
						nivel_aceleracion = calculaAceleradorProgresivoNoLineal();
					// Si se acelera.
					} else {
						nivel_aceleracion = vf_acelerador;
					}
				// Si el crucero no está fijado.
				} else {
					nivel_aceleracion = vf_acelerador;
				}
			// Modo crucero desactivado.
			} else {
				nivel_aceleracion = vf_acelerador;
			}
		// No pedaleamos o tenemos accionado el freno.
		} else {
			nivel_aceleracion = a0_valor_reposo;
		}

		// Fijamos el acelerador si el valor anterior es distinto al actual.
		if (nivel_aceleracion_prev != nivel_aceleracion) {
			dac.setVoltage(aceleradorEnDac(nivel_aceleracion), false);
			nivel_aceleracion_prev = nivel_aceleracion;
		}
	}
}

// --------- Generales

void frena() {
	contador_retardo_inicio_progresivo = cnf.retardo_inicio_progresivo;
	contador_retardo_aceleracion = 0;
	bkp_contador_retardo_aceleracion = 0;
}

void ejecutar_bloqueo_loop (int contador_bloqueo_sistema) {
	// Bloqueamos el loop.
	while (true) {
		delay(500);

		if (contador_bloqueo_sistema > 0) {
			repeatTones(pin_piezo, cnf.buzzer_activo, 1, (contador_bloqueo_sistema--) % 2 ?3000:2000, 1000, 0);
		}
	}
}

void setup() {
	// Inicia serial.
	if (cnf.habilitar_consola) {
		Serial.begin(19200);
		while (!Serial) {};
		Serial.print("Con_Acelerador_DAC_Millis_ProgNL_6kmh ");
		Serial.println(version);
	}

	// Si cnf.dir_dac está a 0 se autodetecta la dirección del dac.
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
		//pinMode(pin_freno, OUTPUT);
		pinMode(pin_pedal, INPUT_PULLUP);
		pinMode(pin_acelerador, INPUT);
		
		// Activamos el pullup del freno.
		digitalWrite(pin_freno, HIGH);

		// Interrupción pedal.
		attachInterrupt(digitalPinToInterrupt(pin_pedal), pedal, CHANGE);
		// Interrupción freno.
		//attachInterrupt(digitalPinToInterrupt(pin_freno), freno, FALLING);

		// Seleccionamos el modo de funcionamiento.
		seleccionaModo();

		// Validamos el acelerador en reposo.
		if (validaMinAcelerador(30)) {
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

			// Estabiliza pulsos_fijar_crucero para que sean siempre superiores a 2.
			if (cnf.pulsos_fijar_crucero < 2)
				cnf.pulsos_fijar_crucero = 2;

			// Estabiliza el progresivo inverso si se supera el valor de referencia.
			if (cnf.v_salida_progresivo_ayuda_arranque > 710)
				cnf.v_salida_progresivo_ayuda_arranque = 710;

			// Estabiliza suavidad de los progresivos.
			nivelarRango(cnf.suavidad_progresivos, 1, 10);

			// Estabiliza suavidad de los auto_progresivos.
			nivelarRango(cnf.suavidad_autoprogresivos, 1, 10);

			// Tono de finalización configuración del sistema.
			repeatTones(pin_piezo, cnf.buzzer_activo, 3, 3000, 90, 90);
		}
	} else {
		// Tonos de error en detección de DAC.
		DAC_ERR_TONE(pin_piezo);
	}
}

void loop() {
	if (a0_valor_reposo > 0) {
		// Si el DAC es detectado.
		if (i2cScanner.isDacDetected()) {
			// Esperamos [tiempo_act - 500ms] para ejecutar.
			if ((unsigned long)(millis() - loop_ultima_ejecucion_millis) > tiempo_act) {
				pulsos = p_pulsos;
				p_pulsos = 0;

				// Desactivamos pedaleo por cadencia.
				if (pulsos < cnf.interrupciones_activacion_pedaleo) {
					pedaleo = false;
				}
				
				if (flag_modo_asistencia >= MODO_CRUCERO)
					actualizacion_contadores = true;

				loop_ultima_ejecucion_millis = millis();
			}

			freno = digitalRead(pin_freno);
			
			// Si el freno está pulsado.
			if (freno == 0)
				// Valor de reposo al DAC.
				paraMotor();

			v_acelerador = leeAcelerador(30);

			if (flag_modo_asistencia >= MODO_CRUCERO)
				estableceCruceroPorTiempo(v_acelerador);

			// Si no se pedalea.
			if (!pedaleo) {
				if (actualizacion_contadores)
					contador_retardo_inicio_progresivo++;

				// Lanzamos auto_progresivo.
				auto_progresivo = true;

				if (contador_retardo_aceleracion > 4) {
					bkp_contador_retardo_aceleracion = contador_retardo_aceleracion;
				}

				contador_retardo_aceleracion = 0;
			// Si se pedalea y no se frena.
			} else if (pedaleo && freno == 1) {
				if (auto_progresivo && contador_retardo_inicio_progresivo < cnf.retardo_inicio_progresivo) {
					if (bkp_contador_retardo_aceleracion > cnf.retardo_aceleracion) {
						bkp_contador_retardo_aceleracion = cnf.retardo_aceleracion;
					}

					contador_retardo_aceleracion = (int) freno * bkp_contador_retardo_aceleracion * (fac_a + fac_b * pow (contador_retardo_inicio_progresivo, fac_c)) * v_crucero / a0_valor_LIMITE;
				}

				auto_progresivo = false;
				contador_retardo_inicio_progresivo = 0;

				if (actualizacion_contadores && contador_retardo_aceleracion < cnf.retardo_aceleracion) {
					contador_retardo_aceleracion++;
				}
			}

			// Si el freno está pulsado.
			if (freno == 0)
				// Reinicia contadores.
				frena();

			if (flag_modo_asistencia >= MODO_CRUCERO) {
				anulaCruceroConFreno();
				anulaCruceroAcelerador();
			}

			mandaAcelerador(v_acelerador);

			// Reinicio de variable.
			if (flag_modo_asistencia >= MODO_CRUCERO)
				actualizacion_contadores = false;
		}
	// Si a0_valor_reposo está forzado a 0 significa que ha habido un error en la inicialización del acelerador o que no se ha detectado.
	} else {
		SOS_TONE(pin_piezo);
		ejecutar_bloqueo_loop(0);
	}
}

// EOF
