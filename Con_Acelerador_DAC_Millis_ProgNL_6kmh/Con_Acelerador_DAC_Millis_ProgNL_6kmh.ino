#include <Adafruit_MCP4725.h>
#include <Arduino.h>
#include "I2CScanner.h"
#include "Tones.h"

const char* version = "2.7.5 Develop N";

/*
                Versión Con Acelerador y DAC Freno Int
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
 * golpe para guardar el voltaje como nivel de asistencia.
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
FIJADO Y DESFIJADO DEL NIVEL DE ASISTENCIA "MONOPATÍN" (POR TIEMPO):
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
 * Si se actúa sobre el acelerador estando su valor por encima de la
 * potencia fijada, la respuesta es instantánea hasta fijar el nuevo
 * valor de crucero.
 *
 * Existe una variante que es la versión tiempo_min que permite
 * interactuar con el acelerador fijando el crucero el tiempo definido
 * con el acelerador en la misma posición, pero permitiendo cambiar
 * el crucero a un valor más bajo cambiando el el valor del acelerador y
 * manteniendo la posición más baja durante el valor de referencia.
 *
 * Para anular la potencia de crucero, existen dos procedimientos
 * principales --> Uno manteniendo pulsada la maneta de freno unos
 * segundos y otro, sin pedalear, subiendo el acelerador a un nivel
 * alto y soltando de golpe. También con el acelerador se puede desfijar
 * la asistencia pedalenado al igual que el crucero continuo.
 * 
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
 * Es posible configurar la asistencia para que empiece aplicando una
 * potencia alta y bajará hasta la potencia que se ajusta a la
 * normativa.
 * Este sistema de potencia invertida ha sido implementado para que las
 * salidas desde parado alcancen la potencia de 6km/h rápidamente.
------------------------------------------------------------------------
VALIDACIÓN DEL ACELERADOR
 * Se ha implementado una validación de seguridad para que en caso de
 * detectarse una medida errónea del acelerador al inicializar el
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

//======= CONSTANTES MODOS =============================================

// Sólo Acelerador.
const byte MODO_ACELERADOR = 0;
// Acelerador con crucero y progresivos.
const byte MODO_CRUCERO = 1;
// Acelerador con crucero, asistencia 6km/h y progresivos.
const byte MODO_CRUCERO6KMH = 2;
// Serial Plotter.
const byte MODO_PLOTTER = 20;

//======= FRENO ========================================================
const int NOACCIONADO = HIGH;
const int ACCIONADO = LOW;
volatile int estadoFreno;

//======= VARIABLES PARA CÁLCULOS ======================================

// Valores mínimos y máximos del acelerador leídos por el pin A0.
// Al inicializar, lee el valor real (a0_valor_reposo).
// 0 --> 1023 = 0 --> 5V.

int a0_valor_reposo = 196;		// 0.96
const int a0_valor_corte = 236;		// 1.15
const int a0_valor_minimo = 330;	// 1.62
const int a0_valor_6kmh = 440;		// 2.16
int a0_valor_maximo = 808;		// 3.95
const int a0_valor_LIMITE = 832;	// 4.06

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
// el buzzer. 22 * 90 = 1980 ms.
const int limite_tono_pulsos_fijar_crucero = 14;

// Flag de activación de asistencia 6km/h y crucero, al arrancar con el
// freno pulsado.
volatile byte flag_modo_asistencia = MODO_ACELERADOR;

//======= Variables de interrupción ====================================

// Variable donde se suman los pulsos del sensor PAS.
volatile byte p_pulsos = 0;
// Variables para la detección del pedaleo.
volatile byte a_pulsos = 0;
volatile boolean pedaleo = false;

//======= FUNCIONES ====================================================

// --------- Utilidades

// Pasamos de escala acelerador -> DAC.
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
	// Pulsos por [tiempo_act - 500 ms] .
	p_pulsos++;

	// Activamos pedaleo por interrupciones.
	if (++a_pulsos >= cnf.interrupciones_activacion_pedaleo) {
		pedaleo = true;
		a_pulsos = 0;
	}
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
	if (cnf.recalcular_rango_max_acelerador && cl_acelerador > a0_valor_maximo)
		a0_valor_maximo = constrain(cl_acelerador, a0_valor_maximo, a0_valor_LIMITE);

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

	if (!cnf.habilitar_consola) {
		Serial.begin(19200);
		while (!Serial) {};
		Serial.print("Con_Acelerador_DAC_Millis_ProgNL_6kmh ");
		Serial.println(version);
	}

	delay(1000);

	// Durante [N] sg monitorizamos los valores de los sensores cada 200 ms.
	unsigned long inicio_ejecucion_millis = millis();

	while (tiempoMs==0 || (tiempoMs > 0 && (unsigned long)(millis() - inicio_ejecucion_millis) < tiempoMs)) {
		delay(200);
		Serial.print(leeAcelerador(3, false)); // Acelerador.
		Serial.print("\t");
		Serial.print(p_pulsos * 10); // Incremento de pulsos pedaleo.
		Serial.print("\t");
		Serial.print(p_pulsos > 0 ? 50 : 5); // Incremento de pulsos pedaleo.
		Serial.print("\t");
		Serial.print(digitalRead(pin_freno) ? 250 : 500); // Frenando si/no.
		Serial.println("");
		p_pulsos = 0;
	}

	// Nunca va a llegar a este punto si no se produce algún error, ya que el anterior while es bloqueante.
	repeatTones(pin_piezo, cnf.buzzer_activo, 3, 3000, 1000, 100);
	Serial.end();
}

void seleccionaModo() {
	// Ejecutamos la selección de modos y esperamos a que se suelte el freno para dejar
	// paso a tomar la medida del acelerador evitando una lectura errónea por la caída de tensión.

	// Según el tiempo que se tenga el freno pulsado, se cambiará de modo.

	// MODO MODO_ACELERADOR - Sólo Acelerador.
	// MODO MODO_CRUCERO (1 segundo) - Crucero.
	// MODO MODO_CRUCERO6KMH (3 segundos) - Crucero + asistencia 6 km/h.
	// MODO MODO_CRUCERO6KMH+ (6 segundos) - Crucero + asistencia 6 km/h + Arranque desde alta potencia.
	// MODO MODO_TEST_SENSORES (20 segundos) - Modo debug Serial Plotter.

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
				break ;

			  case 3:
				// Sólo se selecciona este modo si en el cnf está la variable ayuda_arranque_activa a true
				// Se mantiene esta opción por si alquien no quiere llevarla activa bajo nungún concepto.
				if (cnf.ayuda_arranque_activa) {
					repeatTones(pin_piezo, cnf.buzzer_activo, 3, 3100, 50, 50);
					flag_modo_asistencia = MODO_CRUCERO6KMH ;
					decremento_progresivo_ayuda_arranque = (int) (cnf.v_salida_progresivo_ayuda_arranque - a0_valor_minimo) / ((cnf.tiempo_ejecucion_progresivo_ayuda_arranque / ciclo_decremento_progresivo_ayuda_arranque));
				}
				break;

			  case 6:
				if (cnf.ayuda_arranque_activa) {
					repeatTones(pin_piezo, cnf.buzzer_activo, 6, 3100, 50, 50);
					cnf.activar_progresivo_ayuda_arranque = true;
				}
				break;

			  case 20:
				repeatTones(pin_piezo, cnf.buzzer_activo, 1, 4000, 500, 0);
				testSensoresPlotter(0);
				break;

			}

			delay(100);
			timer_seleccion_modos = millis();
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

	// No queremos que la tolerancia supere el [a0_valor_corte].
	if (comparaConTolerancia(l_acelerador_reposo, a0_valor_reposo, 30)) {
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
	float fac_n = a0_valor_reposo + 0.2 * v_crucero;
	float fac_m = (v_crucero - a0_valor_reposo) / pow (cnf.retardo_aceleracion, fac_p);
	int nivel_aceleraciontmp = (int) estadoFreno * (fac_n + fac_m * pow (contador_retardo_aceleracion, fac_p));

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

void estableceNivelCrucero(int vl_acelerador) {
	// Esperamos 50 ms para ejecutar.
	if ((unsigned long) (millis() - establece_crucero_ultima_ejecucion_millis) > 50) {

		// Calculamos la media de la velocidad de crucero actual y la de la vuelta anterior.
		float media_con_vcrucero_prev = (vl_acelerador_prev + vl_acelerador) / 2;

		// Si la velocidad es la misma incrementa el contador de control de fijación de crucero.
		if (pedaleo && vl_acelerador > a0_valor_minimo && comparaConTolerancia(vl_acelerador, (int) media_con_vcrucero_prev, 10)) {
			contador_crucero_mismo_valor++;

			// Si el contador de crucero ha llegado a su tope, se fija el crucero o si el acelerador está por debajo del crucero y el contador de debajo crucero ha llegado a su tope.
			if (contador_crucero_mismo_valor == cnf.pulsos_fijar_crucero || (cnf.pulsos_fijar_debajo_crucero > 0 && contador_crucero_mismo_valor == cnf.pulsos_fijar_debajo_crucero && vl_acelerador < v_crucero)) {

				// Sólo se fija el crucero si se ha notado una variación de más de +-20 pasos entre la medida actual y la de crucero ya fijada.
				if (!comparaConTolerancia(vl_acelerador, v_crucero, 20)) {
					boolean crucero_arriba = vl_acelerador > v_crucero;
					crucero_fijado = true;
					// Nunca se actualizará la velocidad de crucero por debajo del [a0_valor_minimo].
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

void anulaCruceroConFreno() {
	// Esperamos 100 ms para ejecutar.
	if ((unsigned long)(millis() - anula_crucero_con_freno_ultima_ejecucion_millis) > 100) {
		if (estadoFreno == ACCIONADO) {
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

void anulaCruceroAcelerador() {
	// El procedimiento se ejecuta mientras no se pedalea, haciendo un cambio rápido
	// desde reposo hasta velocidad mínima y vuelta a reposo.
	if (crucero_fijado && cnf.tiempo_liberar_crucero_con_acelerador > 0){
		// Inicia en valor reposo.
		if(comparaConTolerancia(leeAcelerador(10), a0_valor_reposo, 30)) {
			boolean unlock = false;
			// Espera a detectar interacción con el acelerador.
			unsigned long timer_liberar_crucero = millis();

			while ((unsigned long)(millis() - timer_liberar_crucero) < 50) {
				delay(1);
				if (leeAcelerador(10) > a0_valor_reposo + 100) {
					if(!pedaleo)
						repeatTones(pin_piezo, cnf.buzzer_activo, 1, 2300, 90, 120);
					unlock = true;
					break;
				}
			}

			if (unlock) {
				// Espera a que se suelte el acelerador para anular el crucero.
				timer_liberar_crucero = millis();
				while ((unsigned long)(millis() - timer_liberar_crucero) < cnf.tiempo_liberar_crucero_con_acelerador) {
					delay(1);
					// Cancelamos el crucero si existía, en caso de no pedalear y haber soltado el acelerador.
					if (comparaConTolerancia(leeAcelerador(30), a0_valor_reposo, 30)) {
						anulaCrucero();
						break;
					}
				}
			}
		}
	}
}

// --------- Asistencia 6 Km/h

unsigned long ayuda_arranque_ultima_ejecucion = millis();

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
		if (cnf.activar_progresivo_ayuda_arranque && v_salida_progresivo > a0_valor_6kmh && ((unsigned long)(millis() - ayuda_arranque_ultima_ejecucion) > cnf.retardo_ejecucion_progresivo_ayuda_arranque)) {
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

	// Si no pedaleamos y soltamos el acelerador.
	if (!pedaleo && leeAcelerador(30) <= a0_valor_reposo) {
		dac.setVoltage(aceleradorEnDac(a0_valor_reposo), false);
		nivel_aceleracion_prev = a0_valor_reposo;
	}
	
	ayuda_arranque_ultima_ejecucion = millis();
}

// --------- Establecimiento de voltaje

void mandaAcelerador(int vf_acelerador) {
	// Asistencia desde parado a 6 km/h mientras se use el acelerador sin pedalear.
	if (flag_modo_asistencia == MODO_CRUCERO6KMH && !pedaleo && vf_acelerador > a0_valor_minimo && contador_retardo_aceleracion == 0) {
		ayudaArranque();
	} else {
		// Freno No Accionado
		if (estadoFreno == NOACCIONADO) {
			if (pedaleo) {
				// Modo crucero activado.
				if (flag_modo_asistencia >= MODO_CRUCERO) {
					// Si el crucero está fijado.
					if (crucero_fijado) {
						// Si no se está acelerando o si mientras está activa la opción de acelerador bloqueado por debajo de crucero, teniendo los pulsos de fijación crucero están por encima de 8 y se acciona el acelerador por debajo de la velocidad de crucero.
						if (vf_acelerador < a0_valor_minimo || (cnf.pulsos_fijar_debajo_crucero > 0 && cnf.pulsos_fijar_crucero > 8 && vf_acelerador < v_crucero)) {
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
		// Freno Accionado.	
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
		pinMode(pin_pedal, INPUT_PULLUP);
		pinMode(pin_acelerador, INPUT);
		pinMode(pin_freno, INPUT_PULLUP);

		// Interrupción pedal.
		attachInterrupt(digitalPinToInterrupt(pin_pedal), pedal, CHANGE);

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

			// Estabiliza número de interrupciones para activar / desactivar pedaleo.
			cnf.interrupciones_activacion_pedaleo = constrain(cnf.interrupciones_activacion_pedaleo, 2, 4);
			
			// Estabiliza pulsos_fijar_crucero.
			cnf.pulsos_fijar_crucero = constrain(cnf.pulsos_fijar_crucero, 2, 40);
			
			// Estabiliza el progresivo inverso.
			cnf.v_salida_progresivo_ayuda_arranque = constrain(cnf.v_salida_progresivo_ayuda_arranque, a0_valor_6kmh + 1, 710);

			// Estabiliza suavidad de los progresivos.
			cnf.suavidad_progresivos = constrain(cnf.suavidad_progresivos, 1, 10);

			// Estabiliza suavidad de los auto_progresivos.
			cnf.suavidad_autoprogresivos = constrain(cnf.suavidad_autoprogresivos, 1, 10);

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

			// Esperamos [tiempo_act - 500 ms] para ejecutar.
			if ((unsigned long)(millis() - loop_ultima_ejecucion_millis) > tiempo_act) {

				// Desactivamos pedaleo por cadencia.
				if (p_pulsos < cnf.interrupciones_activacion_pedaleo) {
					pedaleo = false;
				}
				p_pulsos = 0;
				
				if (flag_modo_asistencia >= MODO_CRUCERO)
					actualizacion_contadores = true;

				loop_ultima_ejecucion_millis = millis();
			}

			// Toma medidas de los sensores en cada loop
			estadoFreno = digitalRead(pin_freno);
			v_acelerador = leeAcelerador(30);

			// Si el freno está pulsado en este loop.
			if (estadoFreno == ACCIONADO){

				pedaleo = false;
				contador_retardo_inicio_progresivo = cnf.retardo_inicio_progresivo;
				contador_retardo_aceleracion = 0;
				bkp_contador_retardo_aceleracion = 0;

				if(nivel_aceleracion_prev != a0_valor_reposo){
					dac.setVoltage(aceleradorEnDac(a0_valor_reposo), false);
					nivel_aceleracion_prev = a0_valor_reposo;
				}

			} else {

				// Si se pedalea.
				if (pedaleo) {

					if (flag_modo_asistencia >= MODO_CRUCERO)
						estableceNivelCrucero(v_acelerador);

					if (auto_progresivo && contador_retardo_inicio_progresivo < cnf.retardo_inicio_progresivo) {
						if (bkp_contador_retardo_aceleracion > cnf.retardo_aceleracion) {
							bkp_contador_retardo_aceleracion = cnf.retardo_aceleracion;
						}

						contador_retardo_aceleracion = (int) estadoFreno * bkp_contador_retardo_aceleracion * (fac_a + fac_b * pow (contador_retardo_inicio_progresivo, fac_c)) * v_crucero / a0_valor_maximo;
					}

					auto_progresivo = false;
					contador_retardo_inicio_progresivo = 0;

					if (actualizacion_contadores && contador_retardo_aceleracion < cnf.retardo_aceleracion) {
						contador_retardo_aceleracion++;
					}

				} else {

					if (actualizacion_contadores)
						contador_retardo_inicio_progresivo++;

					// Lanzamos auto_progresivo.
					auto_progresivo = true;

					if (contador_retardo_aceleracion > 4) {
						bkp_contador_retardo_aceleracion = contador_retardo_aceleracion;
					}

					contador_retardo_aceleracion = 0;

				}

				mandaAcelerador(v_acelerador);

			}

			if (flag_modo_asistencia >= MODO_CRUCERO) {
				if (cnf.pulsos_liberar_crucero_con_freno > 0)
					anulaCruceroConFreno();
				anulaCruceroAcelerador();

				// Reinicio de variable.
				actualizacion_contadores = false;
			}

		}
	// Si a0_valor_reposo está forzado a 0 significa que ha habido un error en la inicialización del acelerador o que no se ha detectado.
	} else {
		SOS_TONE(pin_piezo);
		ejecutar_bloqueo_loop(0);
	}
}

// EOF
