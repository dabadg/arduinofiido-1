#include <Adafruit_MCP4725.h>
#include <Arduino.h>
#include "Level.h"
#include "Tones.h"
#include "I2CScanner.h"

const char* version = "2.5.3";

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
VERSIÓN CRUCERO SOLTANDO DE GOLPE (CONTINUO) --> FIJADO Y DESFIJADO DEL
NIVEL DE ASISTENCIA:
 * Se trata de guardar el último valor del acelerador
 * para no tener que estar sujetando el acelerador.
 *
 * La idea es fijar el acelerador a la velocidad deseada y soltar de
 * de golpe para guardar el voltaje como de crucero.
 * Al parar y volver a pedalear, se va incrementando voltaje
 * gradualmente hasta llegar al valor de crucero.
 * Si se vuelve a mover el acelerador se toma este como nuevo crucero.
 * Usamos un pin analógico de entrada conectado al
 * acelerador, por otra parte, mandaremos a la placa DAC mediante
 * comunicacion i2c el valor de salida hacia la controladora.
 *
 * El acelerador da un voltaje variable entre 0.85 y 3.9
 * Se puede configurar que el freno anule el crucero.
 * Sólo se fija el valor si pedaleamos.
 *
 * Con toque corto del freno no se anula crucero, con toque largo, sí.
 * También se puede anular el valor del crucero con el acelerador,
 * abriendo un poco de gas y desacelerando hasta el final sin soltar
 * de golpe.
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
 * Existe una variante que es la versión tiempo_min que permite interactuar
 * con el acelerador fijando el crucero manteniendo 3s el acelerador en la
 * misma posición, pero permitiendo cambiar el crucero a un valor más bajo
 * cambiando el el valor del acelerador y manteniendo la posición más baja
 * durante 280ms
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
 * Externalización de fichero de configuración. Podrás seleccionar el
 * tipo de versión que quieres utilizar o utilizar el Custom con los
 * cambios que más te gusten.
 */

// Versión con fijación contínua de crucero (soltando de golpe).
//#include "config_continuo.h"
// Versión para jugar con los parámetros. ;)
//#include "config_custom.h"
// Versión sin fijación de crucero --> Comportamiento como de fábrica
// con el acelerador legalizado.
//#include "config_sincrucero.h"
// Versión con fijación de crucero a los 2,8 segundos.
//#include "config_tiempo.h"
// Versión con fijación de crucero a los 2,8 segundos y bajada de crucero a 280ms.
#include "config_tiempo_min.h"

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

//======= VARIABLES PARA CÁLCULOS ======================================

// Valores mínimos y máximos del acelerador leídos por el pin A0.
// Al inicializar, lee el valor real (a0_valor_reposo).

int a0_valor_reposo = 174;		// 0.85
const int a0_valor_minimo = 235;	// 1.15
const int a0_valor_suave = 307;		// 1.50
const int a0_valor_6kmh = 448;		// 2.19
int a0_valor_alto = 798;		// 3.90
const int a0_valor_max = 809;		// 3.95

// Variables para la detección del pedaleo.
byte pulsos = 0;
byte a_pulsos = 0;

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
int nivel_aceleracion_prev = 0;

// Permite usar el acelerador desde parado a 6 km/h.
boolean ayuda_salida = false;
// Ciclos de decremento cada 50 ms. De 0 a 255.
const byte ciclo_decremento_progresivo_ayuda_arranque = 50;
int decremento_progresivo_ayuda_arranque;

// Valor recogido del acelerador.
int v_acelerador;
// Valor de crucero del acelerador.
int v_crucero = a0_valor_reposo;
// Variable que almacena el estado de notificación de fijar crucero.
boolean crucero_fijado = false;

// Controles de tiempo.
unsigned long tiempo_act;
unsigned long loop_ultima_ejecucion_millis;
unsigned long crucero_fijado_millis;
unsigned long establece_crucero_ultima_ejecucion_millis;
unsigned long anula_crucero_con_freno_ultima_ejecucion_millis;

// Almacena la velocidad de crucero del loop anterior.
int vl_acelerador_prev = 0;
// Cantidad de loops que lleva la velocidad en el mismo valor.
byte contador_crucero_mismo_valor = 0;
// Cantidad de loops para cortar crucero con freno.
byte contador_freno_anulacion_crucero;

//======= Variables de interrupción ====================================

// Variable donde se suman los pulsos del sensor PAS.
volatile byte p_pulsos = 0;
// Variable para la detección del pedaleo.
volatile boolean pedaleo = false;
// Número de interrupciones para activar pedaleo.
volatile byte interrupciones_activacion_pedaleo = 2;

// Pulsos de fijación crucero a partir de los que se emite el tono por el buzzer.
const int limite_tono_pulsos_fijar_crucero = 14;

boolean test_sensores_habilitado;
unsigned long tiempo_sensores_habilitado = 60000;

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
	if (++a_pulsos >= interrupciones_activacion_pedaleo) {
		pedaleo = true;
		a_pulsos = 0;
	}
}

// --------- Crucero

void estableceCrucero(int vl_acelerador) {
	// Ejecutamos método cada 280 ms.
	if ((unsigned long)(millis() - establece_crucero_ultima_ejecucion_millis) > 280) {
		// El crucero se actualiza mientras se esté pedaleando con la lectura del acelerador siempre
		// que esta sea superior al valor de referencia.
		if (pedaleo && vl_acelerador > a0_valor_minimo) {
			v_crucero = vl_acelerador;
			crucero_fijado = true;
		}

		establece_crucero_ultima_ejecucion_millis = millis();
	}
}

void estableceCruceroPorTiempo(int vl_acelerador) {
	// Ejecutamos método cada 100 ms.
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
					v_crucero = vl_acelerador;
					contador_crucero_mismo_valor = 0;
					crucero_fijado_millis = millis();
					// Solo permitimos que suene el buzzer avisando de que se ha fijado
					// el crucero con valores altos. Valores altos se considera a partir de
					// 2 segundos (14 pasos).
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
	// Ejecutamos método cada 100 ms.
	if ((unsigned long)(millis() - anula_crucero_con_freno_ultima_ejecucion_millis) > 100) {
		if (digitalRead(pin_freno) == LOW) {
			contador_freno_anulacion_crucero++;

			if (crucero_fijado){
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
 	if (cnf.recalcular_rango_max_acelerador && cl_acelerador > a0_valor_alto && cl_acelerador <= a0_valor_max)
 		a0_valor_alto = cl_acelerador;

	if (nivelar) {
		nivelarRango(cl_acelerador, a0_valor_reposo, a0_valor_alto);
	}

	return cl_acelerador;
}

int leeAcelerador(byte nmuestras) {
	return leeAcelerador(nmuestras, true);
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

	if (comparaConTolerancia(l_acelerador_reposo, a0_valor_reposo, 50)) {
		// Si queremos arrancar con la actualización de los valores reales tomados por el ecelerador.
		if (cnf.recalcular_rango_min_acelerador) {
			a0_valor_reposo = l_acelerador_reposo;
		}
		status = true;
	// Si la medida el acelerador no es correcta, emitimos un aviso sonoro SOS para avisar del posible error
	// del acelerador y desactivamos el acelerador.
	} else {
		a0_valor_reposo = 0;
		test_sensores_habilitado=true;
		SOS_TONE(pin_piezo);
	}

	delay(100);

	return status;
}

void ayudaArranque() {
	unsigned long timer_progresivo_ayuda_arranque = millis();
	boolean ayuda_arranque_fijada = false;
	float v_salida_progresivo = cnf.v_salida_progresivo_ayuda_arranque;

	// Espera hasta 250ms la liberación de crucero por acelerador si se encuentra activa.
	if (cnf.liberar_crucero_con_acelerador) {
		// Delay a la espera de que se suelte el acelerador para anular crucero.
		while ((unsigned long)(millis() - timer_progresivo_ayuda_arranque) < 250) {
			delay(1);
			// Cancelamos el crucero si existía, en caso de no pedalear y haber soltado el acelerador.
			if (!pedaleo && comparaConTolerancia(leeAcelerador(3), a0_valor_reposo, 50)) {
				anulaCrucero();
				break;
			}
		}
	}

	// Si está configurado el retardo de ayuda al arranque, retardamos la entrada
	// de la ayuda durante los ms leidos de la variable cnf.retardo_ayuda_arranque.
	// Con esto conseguimos evitar que si se toca acelerador se ejecute automáticamente
	// la asistencia y la bicicleta se ponga en marcha.
	if (cnf.retardo_ayuda_arranque > 0 && leeAcelerador(3) > a0_valor_6kmh) {
		while (!pedaleo && (unsigned long)(millis() - timer_progresivo_ayuda_arranque) < cnf.retardo_ayuda_arranque) {
			delay(1);
		}
	}

	// Activación de pedaleo al pasar por el segundo imán.
	interrupciones_activacion_pedaleo = 4;

	// Mientras no pedaleamos y aceleramos.
	while (!pedaleo && leeAcelerador(3) > a0_valor_6kmh) {
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

	// Activación de pedaleo al pasar por el primer imán.
	if (cnf.interrupciones_pedaleo_primer_iman) {
		interrupciones_activacion_pedaleo = 2;
	} else if (cnf.interrupciones_pedaleo_segundo_iman) {
		interrupciones_activacion_pedaleo = 3;
	}

	if (!pedaleo && leeAcelerador(3) <= a0_valor_reposo) {
		paraMotor();
	}
}

int calculaAceleradorProgresivoNoLineal() {
	int nivel_aceleraciontmp;
	int fac_m = 0;

	// Progresivo no lineal.
	fac_m = (a0_valor_max - a0_valor_minimo) / pow(cnf.retardo_aceleracion, fac_p);
	nivel_aceleraciontmp = (int) a0_valor_minimo + fac_m * pow(contador_retardo_aceleracion, fac_p);

	nivelarRango(nivel_aceleraciontmp, a0_valor_reposo, v_crucero > a0_valor_reposo ? v_crucero:a0_valor_max);

	return nivel_aceleraciontmp;
}

void mandaAcelerador(int vf_acelerador) {
	// Asistencia desde parado a 6 km/h mientras se use el acelerador sin pedalear.
	if (ayuda_salida && !pedaleo && leeAcelerador(3) > a0_valor_suave + 5) {
		ayudaArranque();
	} else {
		if (pedaleo) {
			// Modo crucero activado.
			if (cnf.modo_crucero) {
				// Si el crucero está fijado.
				if (crucero_fijado) {
					// Si no se está acelerando o si mientras está activa la opción de acelerador bloqueado por debajo de crucero,  teniendo los pulsos de fijación crucero están por encima de 10 (fijación por tiempo) y se acciona el acelerador por debajo de la velocidad de crucero.
					if (comparaConTolerancia(vf_acelerador, a0_valor_reposo, 50) || (cnf.pulsos_fijar_debajo_crucero > 0 && cnf.pulsos_fijar_crucero >=10 && vf_acelerador < v_crucero )) {
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

void ejecutar_bloqueo_loop(){
	// Bloqueamos el loop.
	int contador_bloqueo_sistema = 6;
	while (true) {
		delay(500);
		if (contador_bloqueo_sistema > 0) {
			repeatTones(pin_piezo, cnf.buzzer_activo, 1, (contador_bloqueo_sistema--) % 2 ?3000:2000, 1000, 0);
		}
	}
}

// --------- Generales

void freno() {
	pedaleo = false;
	contador_retardo_inicio_progresivo = cnf.retardo_inicio_progresivo;
	contador_retardo_aceleracion = 0;
	bkp_contador_retardo_aceleracion = 0;
}

void testSensoresPlotter(unsigned long tiempoMs){
	delay(1000);
	repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, 1000, 0);

	if (!cnf.habilitar_consola){
		Serial.begin(19200);
		while (!Serial) {};
	}

	delay(1000);

	// Durante 60s monitorizamos los valores de los sensores cada 200 ms.
	unsigned long inicio_ejecucion_millis = millis();
	byte pp = 0;

	while ((unsigned long)(millis() - inicio_ejecucion_millis) < tiempoMs) {
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
	test_sensores_habilitado=false;
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
	if (cnf.dir_dac==0) {
		i2cScanner.Init();
	} else {
		i2cScanner.Init(cnf.dir_dac);
	}

	if (i2cScanner.isDacDetected()) {

		// Configura DAC.
		dac.begin(i2cScanner.getDacAddress());
		// Fija voltaje inicial en DAC.
		dac.setVoltage(810, false);

		// Configura pines.
		pinMode(pin_piezo, OUTPUT);
		pinMode(pin_freno, OUTPUT);
		digitalWrite(pin_freno, HIGH);
		pinMode(pin_pedal, INPUT_PULLUP);
		pinMode(pin_acelerador, INPUT);

		// Interrupción pedal.
		attachInterrupt(digitalPinToInterrupt(pin_pedal), pedal, CHANGE);
		// Interrupción freno.
		attachInterrupt(digitalPinToInterrupt(pin_freno), freno, FALLING);

		if (validaMinAcelerador(30)) {
			// Tono aviso de inicio de configuración del sistema.
			repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, 90, 350);

			// Si arrancamos con el freno pulsado.
			if (cnf.freno_pulsado) {
				if (digitalRead(pin_freno) == LOW) {
					// Activamos la ayuda desde parado a 6kmh.
					ayuda_salida = true;
					// Calculamos el decremento de velocidad desde la salida inicial hasta la potencia 6kmh.
					decremento_progresivo_ayuda_arranque = (int) (cnf.v_salida_progresivo_ayuda_arranque - a0_valor_suave) / ((cnf.tiempo_ejecucion_progresivo_ayuda_arranque / ciclo_decremento_progresivo_ayuda_arranque));
					delay(200);
					// Tono aviso de modo con asistencia desde parado.
					repeatTones(pin_piezo, cnf.buzzer_activo, 2, 2900, 90, 200);
					delay(200);
				}
			}

			// Tiempo para las comprobaciones de cadencia según el número de
			// interrupciones para activar / desactivar el pedaleo.
			if (cnf.interrupciones_pedaleo_primer_iman) {
				// Medio segundo.
				tiempo_act = 500;
			} else if (cnf.interrupciones_pedaleo_segundo_iman) {
				// Un segundo.
				tiempo_act = 1000;
				interrupciones_activacion_pedaleo = 3;
			}

			// Ajusta configuración.
			cnf.retardo_aceleracion = cnf.retardo_aceleracion * (1000 / tiempo_act);
			cnf.retardo_inicio_progresivo = cnf.retardo_inicio_progresivo * (1000 / tiempo_act);
			// Anulamos el retardo por seguridad para que empiece progresivo al encender la bici.
			contador_retardo_inicio_progresivo = cnf.retardo_inicio_progresivo;

			// Cálculo de factores para auto_progresivo.
			if (cnf.retardo_inicio_progresivo > 0) {
				fac_b = (1.0 / cnf.retardo_aceleracion - 1.0) / (pow((cnf.retardo_inicio_progresivo - 1.0), fac_c) - pow(1.0, fac_c));
				fac_a = 1.0 - pow(1.0, fac_c) * fac_b;
			}

			// Estabiliza interrupciones para activar pedaleo.
			if (cnf.interrupciones_pedaleo_segundo_iman == true)
				cnf.interrupciones_pedaleo_primer_iman = false;

			// Estabiliza interrupciones para activar pedaleo.
			if (cnf.interrupciones_pedaleo_primer_iman == false && cnf.interrupciones_pedaleo_segundo_iman == false)
				cnf.interrupciones_pedaleo_primer_iman = true;

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
		// tonos de error en detección de dac
    DAC_ERR_TONE(pin_piezo);
	}
}

void loop() {
	if (a0_valor_reposo > 0) {
		// Solo se ejecutará en caso de que el dac haya sido detectado.
		if (i2cScanner.isDacDetected()) {
    
			v_acelerador = leeAcelerador(30);

			if (cnf.modo_crucero) {
				if (cnf.pulsos_fijar_crucero <= 2) {
					estableceCrucero(v_acelerador);
				} else {
					estableceCruceroPorTiempo(v_acelerador);
				}
			}

			// Ejecutamos en función del [tiempo_act].
			if ((unsigned long)(millis() - loop_ultima_ejecucion_millis) > tiempo_act) {
				pulsos = p_pulsos;
				p_pulsos = 0;

				// Desactivamos pedaleo por cadencia.
				if (cnf.interrupciones_pedaleo_primer_iman) {
					if (pulsos < 2)
						pedaleo = false;
				} else if (cnf.interrupciones_pedaleo_segundo_iman) {
					if (pulsos < 3)
						pedaleo = false;
				}

				// Si no se pedalea.
				if (!pedaleo) {
					contador_retardo_inicio_progresivo++;
					auto_progresivo = true;

					if (contador_retardo_aceleracion > 4) {
						bkp_contador_retardo_aceleracion = contador_retardo_aceleracion;
					}

					paraMotor();
					contador_retardo_aceleracion = 0;
				// Si se pedalea.
				} else {
					if (auto_progresivo && contador_retardo_inicio_progresivo < cnf.retardo_inicio_progresivo) {
						if (bkp_contador_retardo_aceleracion > cnf.retardo_aceleracion) {
							bkp_contador_retardo_aceleracion = cnf.retardo_aceleracion;
						}

						contador_retardo_aceleracion = (int) bkp_contador_retardo_aceleracion * (fac_a + fac_b * pow(contador_retardo_inicio_progresivo, fac_c)) * v_crucero / a0_valor_alto;
						auto_progresivo = false;
					} else {
						auto_progresivo = false;
					}

					contador_retardo_inicio_progresivo = 0;

					if (contador_retardo_aceleracion < cnf.retardo_aceleracion) {
						contador_retardo_aceleracion++;
					}
				}

				loop_ultima_ejecucion_millis = millis();
			}

			if (cnf.modo_crucero)
				anulaCruceroConFreno();

			mandaAcelerador(v_acelerador);
		}
	// Si a0_valor_reposo está forzado a 0 significa que ha habido un error en la inicialización del acelerador o que no se ha detectado.
	} else {
		// Ejecutamos el procedimiento de monitorización de sensores.
		if (test_sensores_habilitado)
			testSensoresPlotter(tiempo_sensores_habilitado);

		ejecutar_bloqueo_loop();
	}
}
//
