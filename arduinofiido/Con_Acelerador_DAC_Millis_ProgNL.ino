#include <Adafruit_MCP4725.h>
#include <Arduino.h>
#include "I2CScanner.h"
#include "Level.h"
#include "Tones.h"

const char* version = "2.6 Light";

/*
                     Versión Con Acelerador y DAC
                 Con_Acelerador_DAC_Millis_ProgNL_6kmh
------------------------------------------------------------------------
WIKI:
 * https://github.com/d0s1s/arduinofiido/wiki/Wiki-Arduino-Fiido
------------------------------------------------------------------------
FIJADO Y DESFIJADO DEL NIVEL DE ASISTENCIA:
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
MODO DEBUG:
 * Si está como verdadera la varibale para el modo debug o no se detecta
 * la dirección del DAC en el inicio, se entra en un modo que no se
 * puede usar la bici en modo eléctrico pero permite monitorizar todos
 * los sensores de la bicicleta durante 60 segundos.
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
 * Gracias a zereal por sus ideas de concepto. A faeletronic, basbonald,
 * Manoplas y demás compañeros por el testing.
 */

//=================== VARIABLES CONFIGURABLES POR EL USUARIO ===========

/*
 * Externalización de fichero de configuración. Podrás seleccionar los
 * cambios que más te gusten.
 */

#include "Config.h"

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
int a0_valor_reposo = 197; // 0.83 voltios [Escala 4.3].
const int a0_valor_minimo = 330;
const int a0_valor_6kmh = 440;
const int a0_valor_maximo = 808;

// Variables para la detección del pedaleo.
byte pulsos = 0;
byte a_pulsos = 0;

// Contadores de paro, aceleración y auto_progresivo.
int contador_retardo_aceleracion = 0;
unsigned long contador_retardo_inicio_progresivo = 0;
int bkp_contador_retardo_aceleracion = 0;
boolean auto_progresivo = false;

// Variables para progresivos.
float fac_m = 0.0;
const float fac_p = 1.056 - 0.056 * cnf.suavidad_progresivos;

// Variables para auto_progresivos.
float fac_b = 0.0;
float fac_a = 0.0;
const float fac_c = cnf.suavidad_autoprogresivos / 10.0;

// Los voltios que se mandan a la controladora.
int nivel_aceleracion = a0_valor_reposo;
// Almacena el último valor asignado al DAC.
int nivel_aceleracion_prev = a0_valor_reposo;
// Valor recogido del acelerador.
int v_acelerador;
// Almacena la velocidad de crucero del loop anterior.
int vl_acelerador_prev;
// Valor de crucero.
int v_crucero = a0_valor_reposo;
// Cantidad de loops que lleva la velocidad en el mismo valor.
unsigned int contador_crucero_mismo_valor = 0;

// Controles de tiempo.
const int tiempo_act = 500;
unsigned long loop_ultima_ejecucion_millis = 0;
unsigned long establece_crucero_ultima_ejecucion_millis = 0;
boolean actualizacion_contadores = false;

// Número de interrupciones para activar pedaleo.
byte interrupciones_activacion_pedaleo = 2;

// Para el modo debug.
boolean test_sensores_habilitado = true;
// 60 segundos.
unsigned long tiempo_sensores_habilitado = 60000;

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
	if (++a_pulsos >= interrupciones_activacion_pedaleo) {
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

	//  Nivela valor máximo del acelerador.
	if (nivelar) { 
		if (cl_acelerador > a0_valor_maximo)
			cl_acelerador = a0_valor_maximo;
	}

	return cl_acelerador;
}

void validaMinAcelerador(byte nmuestras) {
	// Inicializamos el valor mínimo del acelerador, calculando la media de las medidas si tiene acelerador.
	// En caso de no tener acelerador o dar medida errónea, mantenemos valor por defecto.
	// Esto es útil para controlar el correcto funcionamiento del acelerador, si este está presente.
	int l_acelerador_reposo = 0;

	// Tomamos 30 medidas para calcular la media.
	for (byte f = 1; f <= nmuestras; f++) {
		l_acelerador_reposo = l_acelerador_reposo + analogRead(pin_acelerador);
	}

	l_acelerador_reposo = (int) l_acelerador_reposo / nmuestras;

	if (comparaConTolerancia(l_acelerador_reposo, a0_valor_reposo, 30)) {
		// Si queremos arrancar con la actualización de los valores reales tomados por el acelerador.
		a0_valor_reposo = l_acelerador_reposo;
	// Si la medida el acelerador no es correcta, emitimos un aviso sonoro SOS para avisar del posible error
	// del acelerador y dejamos valores por defecto.
	} else {
		SOS_TONE(pin_piezo);
	}

	delay(100);
}

// --------- Niveles de asistencia para crucero.

void estableceNivel(int vl_acelerador) {
	// Esperamos 100 ms para ejecutar.
	if ((unsigned long) (millis() - establece_crucero_ultima_ejecucion_millis) > 100) {
		// Calculamos la media de la velocidad de crucero actual y la de la vuelta anterior.
		float media_con_vcrucero_prev = (vl_acelerador_prev + vl_acelerador) / 2;

		// Si la velocidad es la misma incrementa el contador de control de fijación de crucero.
		if (pedaleo && vl_acelerador > a0_valor_minimo && comparaConTolerancia(vl_acelerador, (int) media_con_vcrucero_prev, 10)) {
			contador_crucero_mismo_valor++;

			// Si el contador de crucero ha llegado a su tope, se fija el crucero.
			if (contador_crucero_mismo_valor == 3) {
				// Solo se fija el crucero si se ha notado una variación de más de +-20 pasos entre la medida actual y la de crucero ya fijada.
				if (!comparaConTolerancia(vl_acelerador, v_crucero, 20)) {
					v_crucero = vl_acelerador;
					contador_crucero_mismo_valor = 0;
				}
			}
		} else {
			contador_crucero_mismo_valor = 0;
		}

		vl_acelerador_prev = vl_acelerador;
		establece_crucero_ultima_ejecucion_millis = millis();
	}
}

void anulaCruceroAcelerador(int vf_acelerador) {
	if (!pedaleo && vf_acelerador > a0_valor_minimo) {
		v_crucero = a0_valor_reposo;
		repeatTones(pin_piezo, cnf.buzzer_activo, 1, 2000, 290, 100);
	}
}

// --------- Establecimiento de voltaje

void mandaAcelerador(int vf_acelerador) {
	if (cnf.modo_crucero) {
		if (v_crucero < a0_valor_minimo)
			v_crucero = a0_valor_reposo;
	}

	if (pedaleo) {
		// Modo crucero activado.
		if (cnf.modo_crucero) {
			// Si no se está acelerando.
			if (vf_acelerador < a0_valor_minimo) {
				fac_m = (a0_valor_maximo - a0_valor_reposo) / pow (cnf.retardo_aceleracion, fac_p);
				nivel_aceleracion = (int) a0_valor_minimo + fac_m * pow (contador_retardo_aceleracion, fac_p);

				if (nivel_aceleracion == a0_valor_minimo)
					nivel_aceleracion = a0_valor_reposo;

				nivelarRango(nivel_aceleracion, a0_valor_reposo, v_crucero);
			// Si se acelera.
			} else {
				nivel_aceleracion = vf_acelerador;
			}
		// Modo crucero desactivado.
		} else {
			nivel_aceleracion = vf_acelerador;
		}
	} else {
		if (cnf.modo_crucero && vf_acelerador > a0_valor_minimo && cnf.asistencia6) {
			nivel_aceleracion = a0_valor_6kmh;
		} else {
			nivel_aceleracion = a0_valor_reposo;
		}
	}

	// Fijamos el acelerador si el valor anterior es distinto al actual.
	if (nivel_aceleracion_prev != nivel_aceleracion) {
		dac.setVoltage(aceleradorEnDac(nivel_aceleracion), false);
		nivel_aceleracion_prev = nivel_aceleracion;
	}
}

// --------- Motor

void paraMotor() {
	dac.setVoltage(aceleradorEnDac(a0_valor_reposo), false);
	nivel_aceleracion_prev = a0_valor_reposo;
}

// --------- Generales

void freno() {
	pedaleo = false;
	contador_retardo_inicio_progresivo = cnf.retardo_inicio_progresivo;
	contador_retardo_aceleracion = 0;
	bkp_contador_retardo_aceleracion = 0;
}

void ejecutar_bloqueo_loop() {
	// Bloqueamos el loop.
	int contador_bloqueo_sistema = 6;

	while (true) {
		delay(500);

		if (contador_bloqueo_sistema > 0) {
			repeatTones(pin_piezo, cnf.buzzer_activo, 1, (contador_bloqueo_sistema--) % 2 ?3000:2000, 1000, 0);
		}
	}
}

void testSensoresPlotter(unsigned long tiempoMs) {
	// Inicia serial.
	Serial.begin(19200);
	while (!Serial) {};
	Serial.print("Con_Acelerador_Light_DAC_Millis_ProgNL ");
	Serial.println(version);

	delay(1000);
	repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, 1000, 0);
	delay(1000);

	// Durante 60 segundos monitorizamos los valores de los sensores cada 200 ms.
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
	test_sensores_habilitado = false;
}

void setup() {
	// Si [cnf.dir_dac] está a 0, se autodetecta la dirección del DAC.
	if (cnf.dir_dac == 0) {
		i2cScanner.Init();
	// Lee valor del config.
	} else {
		i2cScanner.Init(cnf.dir_dac);
	}

	// Configura DAC.
	dac.begin(i2cScanner.getDacAddress());
	// Fija voltaje inicial en DAC en reposo.
	dac.setVoltage(aceleradorEnDac(a0_valor_reposo), false);

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

	// Validación del valor real del acelerador en reposo.
	validaMinAcelerador(30);

	// Tono aviso de inicio de configuración del sistema.
	repeatTones(pin_piezo, cnf.buzzer_activo, 1, 3000, 90, 350);
	
	delay(200);
	
	// Si arrancamos con el freno pulsado.
	if (digitalRead(pin_freno) == LOW) {
		// Desactivamos el modo crucero.
		cnf.modo_crucero = false;
		delay(200);
		// Tono aviso de modo de fábrica legalizado.
		repeatTones(pin_piezo, cnf.buzzer_activo, 2, 2900, 90, 200);
		delay(200);
	}

	// Tiempo para las comprobaciones de cadencia según el número de
	// interrupciones para activar / desactivar el pedaleo.
	if (cnf.interrupciones_pedaleo_segundo_iman) {
		interrupciones_activacion_pedaleo = 3;
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

	// Estabiliza suavidad de los progresivos.
	nivelarRango(cnf.suavidad_progresivos, 1, 10);

	// Estabiliza suavidad de los auto_progresivos.
	nivelarRango(cnf.suavidad_autoprogresivos, 1, 10);

	// Tono de finalización configuración del sistema.
	repeatTones(pin_piezo, cnf.buzzer_activo, 3, 3000, 90, 90);
}

void loop() {
	// Si no está el modo Debug y la dirección del DAC es detectada, adelante.
	if (!cnf.habilitar_consola && i2cScanner.isDacDetected()) {
		// Esperamos 500 ms para ejecutar.
		if ((unsigned long)(millis() - loop_ultima_ejecucion_millis) > tiempo_act) {
			pulsos = p_pulsos;
			p_pulsos = 0;

			// Desactivamos pedaleo por cadencia.
			if (cnf.interrupciones_pedaleo_segundo_iman) {
				if (pulsos < 3)
					pedaleo = false;
			} else {
				if (pulsos < 2)
					pedaleo = false;
			}

			actualizacion_contadores = true;
			loop_ultima_ejecucion_millis = millis();
		}

		if (cnf.modo_crucero) {
			v_acelerador = leeAcelerador(30, true);
		} else {
			v_acelerador = leeAcelerador(30, false);
		}

		if (cnf.modo_crucero)
			estableceNivel(v_acelerador);

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
			//paraMotor();
		// Si se pedalea.
		} else {
			if (auto_progresivo && contador_retardo_inicio_progresivo < cnf.retardo_inicio_progresivo) {
				if (bkp_contador_retardo_aceleracion > cnf.retardo_aceleracion) {
					bkp_contador_retardo_aceleracion = cnf.retardo_aceleracion;
				}

				contador_retardo_aceleracion = (int) bkp_contador_retardo_aceleracion * (fac_a + fac_b * pow (contador_retardo_inicio_progresivo, fac_c)) * v_crucero / a0_valor_maximo;
			}

			auto_progresivo = false;
			contador_retardo_inicio_progresivo = 0;

			if (actualizacion_contadores && contador_retardo_aceleracion < cnf.retardo_aceleracion) {
				contador_retardo_aceleracion++;
			}
		}

		if (cnf.modo_crucero)
			anulaCruceroAcelerador(v_acelerador);

		mandaAcelerador(v_acelerador);

		// Reinicio de variable.
		actualizacion_contadores = false;
	} else {
		// Ejecutamos el procedimiento de monitorización de sensores.
		if (test_sensores_habilitado)
			testSensoresPlotter(tiempo_sensores_habilitado);

		ejecutar_bloqueo_loop();
	}
}

// EOF
