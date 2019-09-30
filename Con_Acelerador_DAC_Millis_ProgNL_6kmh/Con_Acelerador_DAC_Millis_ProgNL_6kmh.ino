/* 
                     Versión Con Acelerador y DAC
              Con_Acelerador_DAC_Millis_ProgNL_6kmh 2.1 RC3
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
 * La idea es detectar cuando se suelta de golpe el acelerador
 * y guardar el voltaje anterior como de crucero.
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
 * interrupción en el pin (pin_pedal).
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
 * Si no se pedalea y mientras el acelerador esté accionado, se asiste
 * a 6 km/h, ajustándose a la normativa.
 * Si se suelta el acelerador --> deja de asistir.
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
 * dabadg, d0s1s, chusquete, ciberus y fulano.
------------------------------------------------------------------------
AGRADECIMIENTOS:
 * Grupo de Telegram de desarrollo privado y toda su gente --> pruebas,
 * ideas, etc.
 */

struct ConfigContainer {
	//=================== VARIABLES CONFIGURABLES POR EL USUARIO =======

	// (True) si se desea activar la posibilidad de acelerar desde
	// parado a 6 km/h arrancando con el freno pulsado.
	boolean freno_pulsado = true;
	
	// (True) si se desea que cuando se use la asistencia de 6km/h
	// desde parado se anule el valor de crucero.
	boolean anula_crucero_en_asistencia = true;

	// Retardo en segundos para ponerse a velocidad máxima o crucero.
	int retardo_aceleracion = 5;

	// True --> Modo crucero.
	// False --> Manda señal del acelerador.
	boolean modo_crucero = true;

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

	// Constante que habilita los tonos de inicialización del sistema.
	// Recomendado poner a True si se tiene zumbador en el pin 11.
	boolean buzzer_activo = true;
	//======= FIN VARIABLES CONFIGURABLES POR EL USUARIO ===============
};

//======= FIN VARIABLES CONFIGURABLES POR EL USUARIO ===================

#include <Adafruit_MCP4725.h>
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
float a0_valor_reposo = 190.0; // Al inicializar, lee el valor real.
//const float a0_valor_corte = 216.0;  // 1.05
const float a0_valor_minimo = 235.0; // 1.15
const float a0_valor_suave = 410.0;  // 2.00
const float a0_valor_6kmh = 450.0;   // 2.19
//const float a0_valor_medio = 550.0;  // 2.68
const float a0_valor_alto = 798.0;   // 3.90
const float a0_valor_max = 847.0;    // 4.13

// Variables de tiempo.
const int tiempo_act = 333;
unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;

// Variables para la detección del pedaleo.
byte pulsos = 0;
byte a_pulsos = 0;
boolean pedaleo = false;
// A la segunda interrupción, se activa pedaleo.
unsigned int interrupciones_pedaleo = 1;

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

// Valor recogido del acelerador.
float v_acelerador;
// Valor de crucero del acelerador.
float v_crucero = a0_valor_reposo;
// Los voltios que se mandan a la controladora.
float nivel_aceleracion = a0_valor_reposo;

// Permite usar el acelerador desde parado a 6 km/h.
boolean ayuda_salida = false;

// Variable que almacena el estado de notificación de fijar crucero.
boolean crucero_actualizado = false;
boolean crucero_fijado = false;
//unsigned const int segundos_anular_crucero_freno = 4;
unsigned int brakeCounter;

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

void pedal() {
	p_pulsos++;
	a_pulsos++;

	if (a_pulsos >= interrupciones_pedaleo) {
		pedaleo = true;
		a_pulsos = 0;
	}
}

// Pasamos de escala acelerador -> DAC.
float aceleradorEnDac(float vl_acelerador) {
	return vl_acelerador * 4096 / 1023;
}

void estableceCrucero(float vl_acelerador) {
		// El crucero se actualiza mientras se esté pedaleando con la
		// lectura del acelerador, siempre que esta sea superior al valor de referencia.
		if (vl_acelerador > a0_valor_minimo && pedaleo) {
			v_crucero = vl_acelerador;
			crucero_actualizado = true;
		// Si el crucero se ha actualizado por encima de 2.00 v y si
		// detecta que el acelerador está por debajo del valor mínimo, fija el crucero.
		} else if (crucero_actualizado && v_crucero > a0_valor_suave && vl_acelerador <= a0_valor_reposo) {
			crucero_actualizado = false;
			crucero_fijado = true;
			repeatTones(cnf.buzzer_activo, 1, 3000, 190, 1);
		}
}

// Calcula si el valor se encuantra entre el rango de valores con tolerancia calculados con el valor2.
// valor2-tolerancia > valor < valor2+tolerancia
boolean comparaConTolerancia(float valor, float valor2, float toleranciaValor2) {
	return (valor > (valor2 - toleranciaValor2)) && (valor < (valor2 + toleranciaValor2));
}

float leeAcelerador() {
	float cl_acelerador = 0;

	// Leemos nivel de acelerador tomando 30 medidas.
	for (int f=1; f <= 30; f++) {
		cl_acelerador = cl_acelerador + analogRead(pin_acelerador);
	}

	cl_acelerador = cl_acelerador / 30;

	// Nivelamos los valores de la media para que no se salgan del rango de máximo/mínimo.
	if (cl_acelerador < a0_valor_reposo) {
		return a0_valor_reposo;
	} else if (cl_acelerador > a0_valor_max) {
		return a0_valor_max;
	}

	return cl_acelerador;
}

void mandaAcelerador() {
	if (cnf.modo_crucero == true) {
		// Progresivo no lineal.
		fac_n = a0_valor_reposo + 60;
		fac_m = (v_crucero - a0_valor_reposo) / pow(cnf.retardo_aceleracion, fac_p);
		nivel_aceleracion = fac_n + fac_m * pow(contador_retardo_aceleracion, fac_p);

		if (nivel_aceleracion < a0_valor_reposo) {
			nivel_aceleracion = a0_valor_reposo;
		}

		if (nivel_aceleracion > v_crucero) {
			nivel_aceleracion = v_crucero;
		}
	} else {
		nivel_aceleracion = v_acelerador;
	}

	dac.setVoltage(aceleradorEnDac(nivel_aceleracion), false);
}

void paraMotor() {
	contador_retardo_aceleracion = 0;
}

void freno() {
	contador_retardo_inicio_progresivo = cnf.retardo_inicio_progresivo;
	bkp_contador_retardo_aceleracion = 0;
	interrupciones_pedaleo = 1;
	paraMotor();
}

void anulaCrucero() {
	v_crucero = a0_valor_reposo;
	crucero_actualizado = false;
	crucero_fijado = false;
	repeatTones(cnf.buzzer_activo, 1, 2000, 190, 100);
}

void anulaCruceroConFreno() {
	unsigned int vueltas = 4;
	//((int) (segundos_anular_crucero_freno * 1000) / tiempo_act);
	// Calcular las vueltas de loop necesarias para anular el crucero.

	if (digitalRead(pin_freno) == LOW) {
		brakeCounter++;
		if (crucero_fijado) {
			repeatTones(cnf.buzzer_activo, 1, brakeCounter * 1000, 90, 200);
			if (brakeCounter >= vueltas)
				anulaCrucero();
		}
	} else {
		if (brakeCounter > 0)
			brakeCounter--;
	}
}

void ayudaArranque() {
	// A la tercera interrupción, se activa pedaleo.
	interrupciones_pedaleo = 2;

	if (cnf.anula_crucero_en_asistencia) {
		anulaCrucero();
	}

	// Mientras aceleramos y no pedaleamos.
	while (analogRead(pin_acelerador) > a0_valor_minimo && !pedaleo) {
		contador_retardo_aceleracion++;

		// Preparamos el auto_progresivo.
		contador_retardo_inicio_progresivo = 0;
		auto_progresivo = true;

		dac.setVoltage(aceleradorEnDac(a0_valor_6kmh), false);
	}

	// A la segunda interrupción, se activa pedaleo.
	interrupciones_pedaleo = 1;
}

/*void ayudaArranque() {
	// Fijamos valor de crucero a 6 km/h
	v_crucero = a0_valor_6kmh;

	// Mientras aceleramos y no pedaleamos.
	while (analogRead(pin_acelerador) > a0_valor_minimo && !pedaleo) {
		contador_retardo_aceleracion++;

		// Preparamos el auto_progresivo.
		contador_retardo_inicio_progresivo = 0;
		auto_progresivo = true;

		// Si no está el modo crucero.
		if (!modo_crucero) {
			// Mandamos el voltaje directamente al DAC de 6 km/h.
			dac.setVoltage(aceleradorEnDac(a0_valor_6kmh), false);
		// Si lo está.
		} else {
			// Llamamos a a la función con el crucero de 6 km/h ya fijado.
			mandaAcelerador();
		}
	}
}*/

void validaMinAcelerador() {
	// Inicializamos el valor mínimo del acelerador, calculando la media de las medidas si tiene acelerador.
	// En caso de no tener acelerador, mantenemos valor por defecto.
	// Esto es útil para controlar el corecto funcionamiento del acelerador, si este está presente.
	float l_acelerador_reposo = 0;

	// Tomamos 30 medidas para calcular la media.
	for (int f=1; f <= 30; f++) {
		l_acelerador_reposo = l_acelerador_reposo + analogRead(pin_acelerador);
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

void setup() {
	// Configura DAC.
	dac.begin(cnf.dir_dac);
	// Fija voltaje inicial en Dac (0.85v).
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

	validaMinAcelerador();

	// Tono aviso de inicio a la espera de frenadas (al encender bici).
	repeatTones(cnf.buzzer_activo, 1, 3000, 90, 190);

	// Si arrancamos con el freno pulsado.
	if (cnf.freno_pulsado == true) {
		if (digitalRead(pin_freno) == LOW) {
			// Activamos la ayuda desde parado a 6kmh.
			ayuda_salida = true;
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
	// Arrancar tiempo de inicio.
	tiempo1 = millis();
}

void loop() {
	tiempo2 = millis();

	v_acelerador = leeAcelerador();

	if (tiempo2 > tiempo1 + (unsigned long) tiempo_act) {
		tiempo1 = millis();
		pulsos = p_pulsos;
		estableceCrucero(v_acelerador);

		// Si no se pedalea.
		if (!pedaleo) {
			contador_retardo_inicio_progresivo++;
			auto_progresivo = true;

			if (contador_retardo_aceleracion > 4) {
				bkp_contador_retardo_aceleracion = contador_retardo_aceleracion;
				interrupciones_pedaleo = 2;
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
				interrupciones_pedaleo = 1;
			} else {
				auto_progresivo = false;
			}

			contador_retardo_inicio_progresivo = 0;

			if (contador_retardo_aceleracion < cnf.retardo_aceleracion) {
				contador_retardo_aceleracion++;
			}
		}

		// Asistencia desde parado a 6 km/h mientras se use el acelerador.
		if (ayuda_salida && pulsos == 0 && analogRead(pin_acelerador) > a0_valor_minimo && contador_retardo_aceleracion == 0) {
			ayudaArranque();
		}

		p_pulsos = 0;

		if (pulsos < 2) {
			pedaleo = false;
		}
	}

	anulaCruceroConFreno();
	mandaAcelerador();
}

// EOF
