#include <Arduino.h>
#include <Adafruit_MCP4725.h>
//#include <EEPROM.h>

const char* version = "Develop";

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
------------------------------------------------------------------------
VERSIÓN CRUCERO: 
 * Se trata de guardar el último valor del acelerador
 * para no tener que estar sujetando el acelerador.
 * La idea es fijar el acelerador a la velocidad deseada y esperar el
 * el tiempo definido para guardar el voltaje como de crucero.
 * Al parar y volver a pedalear, se va incrementando voltaje
 * gradualmente hasta llegar al valor de crucero.
 * Si se vuelve a mover el acelerador se toma este como nuevo crucero.
 * Usamos un pin analógico de entrada conectado al
 * acelerador, por otra parte, mandaremos a la placa DAC mediante
 * comunicacion i2c el valor de salida hacia la controladora.
 * El acelerador da un voltaje variable entre 0.85 y 3.9
 * Se puede configurar que el freno anule el crucero.
 * Sólo se fija el valor si pedaleamos.
 * Con toque corto del freno no se anula crucero, con toque largo, sí.
 * También se puede anular el valor del crucero con el acelerador,
 * abriendo un poco de gas y desacelerando hasta el final sin soltar
 * de golpe.
------------------------------------------------------------------------
LEGALIZACIÓN ACELERADOR:
 * Básicamente lo que hace es detectar pulsos mediante una
 * interrupción en el pin (pin_pedal). Si no se pedalea, no asiste el
 * acelerador.
------------------------------------------------------------------------
AUTO PROGRESIVOS:
 * Si se deja de pedalear, el motor se para como de costumbre, pero si
 * continuamos pedaleando antes de transcurridos 10 segundos no inciará
 * el progresivo desde 0 si no que el motor continuará a una velocidad
 * ligeramente inferior a la que íbamos.
 * Si se frena antes de los 10 segundos se anula la función y comenzará
 * progresivo desde cero.
------------------------------------------------------------------------
ASISTENCIA A 6 KM/H DESDE PARADO:
 * Si no se pedalea y mientras el acelerador esté accionado, se asiste a
 * 6 km/h, ajustándose a la normativa.
 * Si se suelta el acelerador --> deja de asistir y cortamos valor de
 * de crucero, si lo hubiera.
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
 *
------------------------------------------------------------------------
DEVELOPERS:
 * dabadg y d0s1s a partir de la versión de ciberus y fulano con las
 * aportaciones de chusquete.
------------------------------------------------------------------------
AGRADECIMIENTOS:
 * Grupo de Telegram de desarrollo privado y toda su gente --> pruebas,
 * ideas, feedback, etc.
 * Gracias a zereal por sus ideas de concepto.
 */

struct ConfigContainer {
	//=================== VARIABLES CONFIGURABLES POR EL USUARIO =======

	// (True) si se desea activar la posibilidad de acelerar desde
	// parado a 6 km/h arrancando con el freno pulsado.
	boolean freno_pulsado = true;

	// Retardo en segundos para ponerse a velocidad máxima o crucero.
	int retardo_aceleracion = 5;

	// True --> Modo crucero.
	// False --> Manda señal del acelerador.
	boolean modo_crucero = true;

	// Cantidad de pasadas para fijar el crucero por tiempo.
	// 14 * 140 = 1960 ms.
	unsigned int pulsos_fijar_crucero = 14;

	// Cantidad de pasadas con el freno pulsado para liberar crucero.
	// 23 * 140 = 3220 ms.
	unsigned int pulsos_liberar_crucero = 23;

	// Retardo para inciar progresivo tras parar pedales.
	// Freno anula el tiempo.
	unsigned int retardo_inicio_progresivo = 10;

	// Suavidad de los progresivos, varía entre 1-10.
	// Al crecer se hacen más bruscos.
	float suavidad_progresivos = 5.0;

	// Suavidad de los autoprogresivos, varía entre 1-10.
	// Al crecer se hacen más bruscos.
	float suavidad_autoprogresivos = 5.0;

	// Dirección del bus I2C [DAC] (0x60) si está soldado,
	// si no (0x62).
	int dir_dac = 0x60;

	// Habilita los tonos de inicialización del sistema.
	// Recomendado poner a True si se tiene zumbador en el pin 11.
	boolean buzzer_activo = true;

	// False --> Mantiene valor que tenía el crucero antes de entrar a
	// la asistencia de 6km/h.
	// True -->  En esta situación anula el valor de crucero al
	// incrementar y soltar acelerador.
	boolean liberar_crucero_con_acelerador = true;

	// Tiempo en ms que tarda en iniciarse la ayuda al arranque.
	unsigned int retardo_ayuda_arranque = 600;

	// (True) Habilita la ayuda la asistencia 6 km/h con inicio
	// progresivo desde alta potencia.
	boolean activar_progresivo_ayuda_arranque = false;

	// Valor inicial de salida en la asistencia 6 km/h.
	// Como mínimo tendrá que tener el valor de la constante
	// a0_valor_6kmh --> 450.
	float v_salida_progresivo_ayuda_arranque = 700;

	// Tiempo de ejecución del progresivo en la asistencia a 6 km/h.
	// 1500 ms.
	int tiempo_ejecucion_progresivo_ayuda_arranque = 1500;

	// Habilita la salida de datos por consola
	boolean habilitar_consola = false;
};

//======= FIN VARIABLES CONFIGURABLES POR EL USUARIO ===================

Adafruit_MCP4725 dac;

ConfigContainer cnf;

//======= PINES ========================================================
const int pin_acelerador = A0; // Pin acelerador.
const int pin_pedal = 2; // Pin sensor pas, en Nano/Uno usar 2 ó 3.
const int pin_freno = 3; // Pin de activación del freno.
const int pin_piezo = 11; // Pin del zumbador.
// Resto de pines 9 y 10.

//======= VARIABLES PARA CÁLCULOS ======================================

// Valores mínimos y máximos del acelerador leídos por el pin A0.
// Al inicializar, lee el valor real (a0_valor_reposo).
float a0_valor_reposo = 190.0;		// 0.85
//const float a0_valor_corte = 216.0;	// 1.05
const float a0_valor_minimo = 235.0;	// 1.15
const float a0_valor_suave = 307.0;	// 1.50
const float a0_valor_6kmh = 450.0;	// 2.19
//const float a0_valor_medio = 550.0;	// 2.68
float a0_valor_alto = 798.0;		// 3.90
const float a0_valor_max = 847.0;	// 4.13

// Variables de tiempo.
const unsigned long tiempo_act = 333;
unsigned long loop_ultima_ejecucion_millis;

// Variables para la detección del pedaleo.
byte pulsos = 0;
byte a_pulsos = 0;
boolean pedaleo = false;

// Contadores de paro, aceleración y auto_progresivo.
int contador_retardo_aceleracion = 0;
unsigned long contador_retardo_inicio_progresivo = 0;
int bkp_contador_retardo_aceleracion = 0;
boolean auto_progresivo = false;

// Variables progresivos.
float fac_m = 0;
float fac_n = 0;
float fac_p = 1.056 - 0.056 * cnf.suavidad_progresivos;

// Variables para auto_progresivos.
float fac_b = 0;
float fac_a = 0;
float fac_c = cnf.suavidad_autoprogresivos / 10.0;

// Los voltios que se mandan a la controladora.
float nivel_aceleracion = a0_valor_reposo;
// Almacena el último valor asignado al DAC.
float nivel_aceleracion_prev = 0;

// Permite usar el acelerador desde parado a 6 km/h.
boolean ayuda_salida = false;
// Ciclos de decremento cada 50 ms.
const int ciclo_decremento_progresivo_ayuda_arranque = 50;
int decremento_progresivo_ayuda_arranque;

// Valor de crucero del acelerador.
float v_crucero = a0_valor_reposo;
// Variable que almacena el estado de notificación de fijar crucero.
boolean crucero_fijado = false;

// Controles de tiempo.
unsigned long crucero_fijado_millis;
unsigned long establece_crucero_ultima_ejecucion_millis;
unsigned long anula_crucero_con_freno_ultima_ejecucion_millis;

// Almacena la velocidad de crucero del loop anterior.
float vl_acelerador_prev;
// Cantidad de loops que lleva la velocidad en el mismo valor.
unsigned int contador_crucero_mismo_valor = 0;
// Cantidad de loops para cortar crucero con freno.
unsigned int contador_freno_anulacion_crucero;

//======= Variables interrupción =======================================
// Variable donde se suman los pulsos del sensor PAS.
volatile byte p_pulsos = 0;

//======= FUNCIONES DE TONOS ===========================================

//================== TONES ==================
// Frecuencias 4 octavas de
int c[5]={131,262,523,1046,2093};       // Do
int cs[5]={139,277,554,1108,2217};      // Do#
int d[5]={147,294,587,1175,2349};       // Re
int ds[5]={156,311,622,1244,2489};      // Re#
int e[5]={165,330,659,1319,2637};       // Mi
int f[5]={175,349,698,1397,2794};       // Fa
int fs[5]={185,370,740,1480,2960};      // Fa#
int g[5]={196,392,784,1568,3136};       // Sol
int gs[5]={208,415,831,1661,3322};      // Sol#
int a[5]={220,440,880,1760,3520};       // La
int as[5]={233,466,932,1866,3729};      // La#
int b[5]={247,494,988,1976,3951};       // Si

void nota(int frec, int ttime) {
	tone(pin_piezo,frec);
	delay(ttime);
	noTone(pin_piezo);
}

void SOS_TONE() {
	nota(b[3],150);delay(40);
	nota(b[3],150);delay(40);
	nota(b[3],150);delay(70);
	nota(b[3],100);delay(40);
	nota(b[3],100);delay(40);
	nota(b[3],100);delay(70);
	nota(b[3],150);delay(40);
	nota(b[3],150);delay(40);
	nota(b[3],150);delay(100);
}

void repeatTones(boolean trigger, int steps, int frequency, int duration, int delayTime) {
	if (trigger) {
		int cont = steps;
		while (cont-- > 0) {
			tone(pin_piezo,frequency,duration);
			if (delayTime > 0)
				delay(delayTime);
			//noTone(pin_piezo);
		}
	}
}

//======= FUNCIONES ====================================================

// --------- Utilidades

// Calcula si el valor se encuantra entre el rango de valores con
// tolerancia calculados con el valor2.
// valor2-tolerancia > valor < valor2+tolerancia
boolean comparaConTolerancia(float valor, float valorReferencia, float toleranciaValor2) {
	return (valor > (valorReferencia - toleranciaValor2)) && (valor < (valorReferencia + toleranciaValor2));
}

// Pasamos de escala acelerador -> DAC.
float aceleradorEnDac(float vl_acelerador) {
	return vl_acelerador * 4096 / 1023;
}

// --------- Pedal

void pedal() {
	// Pulsos por loop.
	p_pulsos++;

	// A la cuarta interrupción, se activa pedaleo.
	if (++a_pulsos >= 3) {
		pedaleo = true;
		a_pulsos = 0;
	}
}

// --------- Crucero

void estableceCruceroPorTiempo(float vl_acelerador) {
	// Ejecutamos método cada 100 ms.
	if ((unsigned long)(millis() - establece_crucero_ultima_ejecucion_millis) > 100) {
		// Calculamos la media de la velocidad de crucero actual y la de la vuelta anterior.
		float media_con_vcrucero_prev = (vl_acelerador_prev + vl_acelerador) / 2;

		vl_acelerador_prev = vl_acelerador;

		// Si la velocidad es la misma incrementa el contador de control de fijación de crucero.
		if (pedaleo && vl_acelerador > a0_valor_minimo && comparaConTolerancia(vl_acelerador, media_con_vcrucero_prev, 10.0)) {
			contador_crucero_mismo_valor++;

			// Si el contador de crucero ha llegado a su tope, se fija el crucero.
			if (contador_crucero_mismo_valor == cnf.pulsos_fijar_crucero) {
				crucero_fijado = true;
				v_crucero = vl_acelerador;
				crucero_fijado_millis = millis();
				// Solo permitimos que suene el buzzer avisando de que se ha fijado el crucero con valores altos.
				// Valores altos se considera a partir de 2 segundos.
				if (cnf.pulsos_fijar_crucero >= 14)
					repeatTones(cnf.buzzer_activo, 1, 3000, 190, 1);
			}
		} else {
			contador_crucero_mismo_valor = 0;
		}

		establece_crucero_ultima_ejecucion_millis = millis();
	}
}

void anulaCrucero() {
	if (crucero_fijado) {
		v_crucero = a0_valor_reposo;
		crucero_fijado = false;
		repeatTones(cnf.buzzer_activo, 1, 2000, 190, 100);
	}
}

void anulaCruceroConFreno() {
	// Ejecutamos método cada 100 ms.
	if ((unsigned long)(millis() - anula_crucero_con_freno_ultima_ejecucion_millis) > 100) {
		if (digitalRead(pin_freno) == LOW) {
			contador_freno_anulacion_crucero++;

			if (crucero_fijado){
				// Añadido % 4 para solo ejecutar la acción para los múltiplos de 4 y evitar excesivos tonos.
				if (contador_freno_anulacion_crucero % 4 == 0) {
					repeatTones(cnf.buzzer_activo, 1, (3000 + (contador_freno_anulacion_crucero * 20)), 90, 200);
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

// --------- Acelerador

float leeAcelerador(int nmuestras) {
	float cl_acelerador = 0;

	// Leemos nivel de acelerador tomando 30 medidas.
	for (int f = 1; f <= nmuestras; f++) {
		cl_acelerador = cl_acelerador + analogRead(pin_acelerador);
	}

	cl_acelerador = cl_acelerador / nmuestras;

	// Nivelamos los valores de la media para que no se salgan del rango de máximo/mínimo.
	if (cl_acelerador < a0_valor_reposo) {
		return a0_valor_reposo;
	} else if (cl_acelerador > a0_valor_alto) {
		return a0_valor_alto;
	}

	// Actualizamos el valor a0_valor_alto, al máximo medido por el acelerador.
	// Para corregir el valor por el real obtenido de la lectura.
	if (cl_acelerador > a0_valor_alto && cl_acelerador <= a0_valor_max)
		a0_valor_alto = cl_acelerador;

	return cl_acelerador;
}


void validaMinAcelerador() {
	// Inicializamos el valor mínimo del acelerador, calculando la media de las medidas si tiene acelerador.
	// En caso de no tener acelerador, mantenemos valor por defecto.
	// Esto es útil para controlar el corecto funcionamiento del acelerador, si este está presente.
	float l_acelerador_reposo = 0;

	// Tomamos 30 medidas para calcular la media.
	for (int f = 1; f <= 30; f++) {
		l_acelerador_reposo = l_acelerador_reposo + leeAcelerador(3);
	}

	l_acelerador_reposo = l_acelerador_reposo / 30;

	// Si la medida no es correcta, emitimos un aviso sonoro SOS para poder localizar el error y desactivamos el acelerador.
	if (comparaConTolerancia(l_acelerador_reposo, a0_valor_reposo, 30)) {
		a0_valor_reposo = l_acelerador_reposo;
	} else {
		SOS_TONE();
	}

	delay(100);
}


void ayudaArranque() {
	unsigned long timer_progresivo_ayuda_arranque = millis();
	boolean ayuda_arranque_fijada = false;
	float v_salida_progresivo = cnf.v_salida_progresivo_ayuda_arranque;

	// Espera hasta 250ms la liberación de crucero por acelerador si se encuentra activa.
	if (cnf.liberar_crucero_con_acelerador) {
		//Delay a la espera de que se suelte el acelerador para anular crucero.
		while ((unsigned long)(millis() - timer_progresivo_ayuda_arranque) < 250) {
			delay(1);
			// Cancelamos el crucero si existía, en caso de no pedalear y haber soltado el acelerador.
			if (p_pulsos <= 2 && leeAcelerador(3) <= a0_valor_reposo) {
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
		while (p_pulsos <= 2 && (unsigned long)(millis() - timer_progresivo_ayuda_arranque) < cnf.retardo_ayuda_arranque) {
			delay(1);
		}
	}

	// Mientras no pedaleamos y aceleramos.
	while (p_pulsos <= 2 && leeAcelerador(3) > a0_valor_6kmh) { // TODO Cambiar p_pulsos<=2 por comparación de valor cadencia.
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
				// Ajustamos contador para cálculo del progresivo.
				contador_retardo_aceleracion = 5;
				ayuda_arranque_fijada = true;
			}
		}
	}

	if (p_pulsos <= 2 && leeAcelerador(3) <= a0_valor_reposo) {
		dac.setVoltage(aceleradorEnDac(a0_valor_reposo), false);
		nivel_aceleracion_prev = a0_valor_reposo;
	}
}

float calculaAceleradorProgresivoNoLineal(float v_cruceroin) {
	float nivel_aceleraciontmp;

	// Progresivo no lineal.
	fac_n = a0_valor_reposo + 60;
	fac_m = (v_cruceroin - a0_valor_reposo) / pow(cnf.retardo_aceleracion, fac_p);
	nivel_aceleraciontmp = fac_n + fac_m * pow(contador_retardo_aceleracion, fac_p);

	if (nivel_aceleraciontmp < a0_valor_reposo) {
		nivel_aceleraciontmp = a0_valor_reposo;
	} else if (nivel_aceleraciontmp > v_cruceroin) {
		nivel_aceleraciontmp = v_cruceroin;
	}

	return nivel_aceleraciontmp;
}

float calculaAceleradorProgresivoNoLineal2(float v_cruceroin) {
	float nivel_aceleraciontmp;

	// Progresivo no lineal.
	fac_n = 410;
	fac_m = (a0_valor_max - 410) / pow(cnf.retardo_aceleracion, fac_p);
	nivel_aceleraciontmp = fac_n + fac_m * pow(contador_retardo_aceleracion, fac_p);

	if (nivel_aceleraciontmp < a0_valor_reposo) {
		nivel_aceleraciontmp = a0_valor_reposo;
	} else if (nivel_aceleraciontmp > v_cruceroin) {
		nivel_aceleraciontmp = v_cruceroin;
	}

	return nivel_aceleraciontmp;
}

void mandaAcelerador(float vf_acelerador) {
	// Asistencia desde parado a 6 km/h mientras se use el acelerador sin pedalear.
	if (ayuda_salida && !pedaleo && leeAcelerador(3) > a0_valor_suave) {
		ayudaArranque();
	} else {
		if (pedaleo) {
			// El crucero entra solo si el modo crucero está activo, si el crucero está fijado y el acelerador es menor que el valor de reposo.
			if (cnf.modo_crucero && crucero_fijado) {
				// Si no se está acelerando.
				if (comparaConTolerancia(vf_acelerador, a0_valor_reposo,20)) {
					nivel_aceleracion = calculaAceleradorProgresivoNoLineal(v_crucero);
				} else {
					nivel_aceleracion = vf_acelerador;
				}
			} else if (cnf.modo_crucero && !crucero_fijado) {
				// Si no se está acelerando.
				if (comparaConTolerancia(vf_acelerador, a0_valor_reposo,20)) {
					nivel_aceleracion = calculaAceleradorProgresivoNoLineal2(v_crucero);
				} else {
					nivel_aceleracion = vf_acelerador;
				}
			} else {
				nivel_aceleracion = vf_acelerador;
			}
		} else {
			nivel_aceleracion = a0_valor_reposo;
		}

		// Solo fijamos el acelerador si el valor anterior es distinto al actual.
		if (nivel_aceleracion_prev != nivel_aceleracion) {
			  dac.setVoltage(aceleradorEnDac(nivel_aceleracion), false);
			  nivel_aceleracion_prev = nivel_aceleracion;
		}
	}
}

// --------- Generales

void paraMotor() {
	contador_retardo_aceleracion = 0;
}

void freno() {
	contador_retardo_inicio_progresivo = cnf.retardo_inicio_progresivo;
	bkp_contador_retardo_aceleracion = 0;
	paraMotor();
}

void setup() {
	// Inicia serial.
	if (cnf.habilitar_consola) {
		Serial.begin(19200);
		Serial.println(version);
	}

	// Configura DAC.
	dac.begin(cnf.dir_dac);
	// Fija voltaje inicial en Dac (0.85v).
	dac.setVoltage(aceleradorEnDac(a0_valor_reposo), false);

	// Lee configuración desde la eeprom.
	//const byte EEPROM_INIT_ADDRESS = 11; // Posición de memoria que almacena los datos de modo.
	//EEPROM.get(EEPROM_INIT_ADDRESS, cnf); // Captura los valores desde la eeprom.

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

	validaMinAcelerador();

	// Tono aviso de inicio a la espera de frenadas (al encender bici).
	repeatTones(cnf.buzzer_activo, 1, 3000, 90, 190);

	// Si arrancamos con el freno pulsado.
	if (cnf.freno_pulsado) {
		if (digitalRead(pin_freno) == LOW) {
			// Activamos la ayuda desde parado a 6kmh.
			ayuda_salida = true;
			// Calculamos el decremento de velocidad desde la salida inicial hasta la potencia 6kmh
			decremento_progresivo_ayuda_arranque = (int) (cnf.v_salida_progresivo_ayuda_arranque - a0_valor_suave) / ((cnf.tiempo_ejecucion_progresivo_ayuda_arranque / ciclo_decremento_progresivo_ayuda_arranque));
			delay(200);
			// Tono aviso de modo con asistencia desde parado.
			repeatTones(cnf.buzzer_activo, 2, 2900, 90, 200);
			delay(200);
		}
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

	// Tono de finalización de setup.
	repeatTones(cnf.buzzer_activo, 3, 3000, 90, 90);
}

void loop() {
	float v_acelerador = leeAcelerador(30);

	if (cnf.modo_crucero)
		estableceCruceroPorTiempo(v_acelerador);

	// Ejecutamos cada 333 ms.
	if ((unsigned long)(millis() - loop_ultima_ejecucion_millis) > tiempo_act) {
		pulsos = p_pulsos;

		// Si no se pedalea.
		if (!pedaleo) {
			contador_retardo_inicio_progresivo++;
			auto_progresivo = true;

			if (contador_retardo_aceleracion > 4) {
				bkp_contador_retardo_aceleracion = contador_retardo_aceleracion;
			}

			paraMotor();
		// Si se pedalea.
		} else {
			if (auto_progresivo && contador_retardo_inicio_progresivo < cnf.retardo_inicio_progresivo) {
				if (bkp_contador_retardo_aceleracion > cnf.retardo_aceleracion) {
					bkp_contador_retardo_aceleracion = cnf.retardo_aceleracion;
				}

				contador_retardo_aceleracion = bkp_contador_retardo_aceleracion * (fac_a + fac_b * pow(contador_retardo_inicio_progresivo, fac_c)) * v_crucero / a0_valor_alto;
				auto_progresivo = false;
			} else {
				auto_progresivo = false;
			}

			contador_retardo_inicio_progresivo = 0;

			if (contador_retardo_aceleracion < cnf.retardo_aceleracion) {
				contador_retardo_aceleracion++;
			}
		}

		p_pulsos = 0;

		// Desactivamos pedaleo por cadencia.
		if (pulsos < 2) {
			pedaleo = false;
		}

		loop_ultima_ejecucion_millis = millis();
	}

	anulaCruceroConFreno();
	mandaAcelerador(v_acelerador);
}

// EOF
