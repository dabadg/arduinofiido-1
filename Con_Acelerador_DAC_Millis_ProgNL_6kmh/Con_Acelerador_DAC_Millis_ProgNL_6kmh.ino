/* 
                    Versión Con Acelerador y DAC
         Con_Acelerador_DAC_Millis_ProgNL_6kmh 2.0
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
------------------------------------------------------------------------
LEGALIZACIÓN ACELERADOR:
 * Básicamente lo que hace es detectar pulsos mediante una
 * interrupción en el pin (pin_pedal).
 * Si los pulsos obtenidos son menores que la cadencia espera
 * que se cumpla el retardo establecido y no funciona el acelerador.
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
 * Si no se pedalea y mientras el acelerador esté accionado, se fija un
 * crucero para que el motor asista hasta 6 km/h, ajustándose a la
 * normativa. Si se suelta el acelerador --> deja de asistir.
 * Si se comienza a pedalear sin dejar de accionar el acelerador --> se
 * sale a la velocidad con la que vayamos regulando con el acelerador.
 * El crucero de 6 km/h se puede aprovechar como un "modo peatonal" en
 * el caso de haber soltado el acelerador y empezar a pedalear.
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

//=================== VARIABLES CONFIGURABLES POR EL USUARIO ===========

// Número de pulsos para que se considere que se está pedaleando.
// Configurar según sensor y gustos.
const int cadencia_pedaleo = 2;

// (True) si se desea activar la posibilidad de acelerar desde parado a
// 6 km/h arrancando con el freno pulsado.
const boolean frenopulsado = true;

// Retardo en segundos para parar el motor una vez se deja de pedalear.
// Usar múltiplos de 0.25 --> 0.25 = 1/4 de segundo.
// No se recomienda subir más de 1.00 segundo.
float retardo_paro_motor = 0.75;

// Retardo en segundos para ponerse a velocidad máxima o crucero.
int retardo_aceleracion = 5;

// True --> Modo crucero.
// False --> Manda señal del acelerador.
const boolean modo_crucero = true;

// (True) si se desea anular la velocidad de crucero al frenar.
// Se recomienda activarlo para mayor seguridad.
const boolean freno_anula_crucero = true;

// Nivel al que se desea iniciar el progresivo. Aumentar si se desea
// salir con mas tirón. >> NO PASAR DE 410, NI BAJAR DE 190 <<.	
const float a0_valor_inicial_arranque_progresivo = 306; // 1.50.

// Retardo para inciar progresivo tras parar pedales.
// Freno anula el tiempo.
int retardo_inicio_progresivo = 10;

// Suavidad de los progresivos, varía entre 1-10.
// Al crecer se hacen más bruscos.
float suavidad_progresivos = 3;

// Suavidad de los autoprogresivos, varía entre 1-10.
// Al crecer se hacen más bruscos.
float suavidad_autoprogresivos = 5;

// Dirección del bus I2C [DAC] (0x60) si está soldado, si no (0x62).
const int dir_dac = 0x60;

// En True la cadencia durante la asistencia desde parado a 6km/h se
// pone a 3 (puede dificultar las salidas en cuestas desde parado).
// Una vez salgamos de dicha asistencia, la cadencia vuelve a la 
// definida en la variable cadencia_pedaleo.
const boolean cadencia_en_asistencia = false;

// Constante que habilita los tonos de inicialización del sistema.
// Recomendado poner a True si se tiene zumbador en el pin 11.
const boolean tono_inicial = false;

//======= FIN VARIABLES CONFIGURABLES POR EL USUARIO ===================

#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

//======= PINES ========================================================
const int pin_acelerador = A0; // Pin acelerador.
const int pin_pedal = 2; // Pin sensor pas, en Nano/Uno usar 2 ó 3.
const int pin_freno = 3; // Pin de activación del freno.
const int pin_piezo = 11; // Pin del zumbador.
// Resto de pines 9 y 10.

//======= VARIABLES PARA CÁLCULOS ======================================

// Número de pulsos para que se considere que se está pedaleando.
int cadencia = cadencia_pedaleo;

// Tiempo en milisegundos para contar pulsos.
const int tiempo_cadencia = 250;
//const float tiempo_cadencia = 333.3; // Para cadencia 2 (6 imanes --> vuelta por segundo).
//const float tiempo_cadencia = 166.6; // Para cadencia 1 (6 imanes --> vuelta por segundo).

// Valores mínimos y máximos del acelerador leídos por el pin A0.
float a0_valor_reposo = 190.0; // Al inicializar, lee el valor real.
const float a0_valor_corte = 216.0;  // 1.05
const float a0_valor_minimo = 235.0; // 1.15
const float a0_valor_suave = 410.0;  // 2.00
const float a0_valor_6kmh = 450.0;   // 2.19
const float a0_valor_medio = 550.0;  // 2.68
const float a0_valor_alto = 798.0;   // 3.90
const float a0_valor_max = 847.0;    // 4.13

// Variables para millis().
unsigned long tcadencia;
unsigned long tiempo;

// Contadores de paro, aceleración y auto_progresivo.
unsigned contador_retardo_paro_motor = 0;
long contador_retardo_aceleracion = 0;
unsigned contador_retardo_inicio_progresivo = 0;
long bkp_contador_retardo_aceleracion = 0;
boolean auto_progresivo = false;

// Variables progresivos.
float fac_m = 0;
float fac_n = 0;
float fac_p = 0.6222 - 0.0222 * suavidad_progresivos;
float nivel_inicial_progresivo = a0_valor_inicial_arranque_progresivo;

// Variables para auto_progresivos.
float fac_s = 0;
float fac_b = 0;
float fac_a = 0;
float fac_c = suavidad_autoprogresivos / 10.0;

// Valor recogido del acelerador.
float v_acelerador;
// Valor de crucero del acelerador.
float v_crucero = a0_valor_reposo;
// Los voltios que se mandan a la controladora.
float nivel_aceleracion = a0_valor_inicial_arranque_progresivo;

// Contador de pulsos del pedal.
int pulsos = 0;

// Permite usar el acelerador desde parado a 6 km/h.
boolean ayuda_salida = false;

// Variable que almacena el estado de notificación de fijar crucero.
boolean crucero_actualizado = false;

//======= Variables interrupción =======================================
// Variable donde se suman los pulsos del sensor PAS.
volatile int p_pulsos = 0;

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
		while(cont-- > 0) {
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
}

// Pasamos de escala acelerador -> DAC.
float aceleradorEnDac(float vl_acelerador) {
	return vl_acelerador * 4096 / 1023;
}

void estableceCrucero(float vl_acelerador) {
	// El crucero se actualiza mientras se esté pedaleando con la lectura del acelerador siempre que esta sea superior al valor de referencia.
	if (vl_acelerador > a0_valor_minimo && p_pulsos > 0) {
		v_crucero = vl_acelerador;
		crucero_actualizado = true;
	// Si el acelerador está al mínimo en la siguiente vuelta, se emite un tono de aviso.
	} else if (vl_acelerador <= a0_valor_minimo && crucero_actualizado) {
		crucero_actualizado = false;
		repeatTones(tono_inicial, 1, 3000, 190, 1);
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
	// Anula crucero por debajo del nivel inicial del progresivo.
	if (v_crucero < nivel_inicial_progresivo) {		
		v_crucero = a0_valor_corte;	
	}

	// Evita salidas demasiado bruscas.	
	if (nivel_inicial_progresivo > a0_valor_suave) {	
		nivel_inicial_progresivo = a0_valor_suave;	
	}

	if (modo_crucero == true) {
		// Progresivo no lineal.
		fac_n = nivel_inicial_progresivo;
		fac_m = ((v_crucero + 60) - nivel_inicial_progresivo) / pow(retardo_aceleracion, fac_p);
		nivel_aceleracion = fac_n + fac_m * pow(contador_retardo_aceleracion, fac_p);

		// Work around para volver del modo asistivo nativo de la bici.
		if (nivel_aceleracion == nivel_inicial_progresivo || nivel_aceleracion < a0_valor_reposo) {
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
	contador_retardo_inicio_progresivo = retardo_inicio_progresivo;
	bkp_contador_retardo_aceleracion = 0;

	paraMotor();

	if (freno_anula_crucero == true) {
		v_crucero = a0_valor_corte;
	}
}

void ayudaArranque() {
	// Fijamos valor de crucero a 6 km/h
	v_crucero = a0_valor_6kmh;

	// Ponemos cadencia a 3 si así está definido en variable de usuario.
	if (cadencia_en_asistencia) {
		cadencia = 3;
	}

	// Mientras aceleramos y no pedaleamos.
	while (analogRead(pin_acelerador) > a0_valor_minimo && p_pulsos < cadencia) {
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
	
	// Recuperamos cadencia habitual.
	if (cadencia_en_asistencia) {
		cadencia = cadencia_pedaleo;
	}
}

void validaMinAcelerador() {
	// Inicializamos el valor mínimo del acelerador, calculando la media de las medidas si tiene acelerador. En caso de no tener acelerador, mantenemos valor por defecto.
	// Esto es útil para controlar el corecto funcionamiento del acelerador, si este está presente.
	float l_acelerador_reposo;

	for (int f=1; f <= 30; f++) {
		l_acelerador_reposo = l_acelerador_reposo + analogRead(pin_acelerador); // Tomamos 30 medidas para calcular la media.
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
	dac.begin(dir_dac);
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
	repeatTones(tono_inicial, 1, 3000, 90, 190);

	// Si arrancamos con el freno pulsado.
	if (frenopulsado == true) {	
		if (digitalRead(pin_freno) == LOW) {
			// Activamos la ayuda desde parado a 6kmh.	
			ayuda_salida = true;
			delay(200);
			// Tono aviso de modo con asistencia desde parado.
			repeatTones(tono_inicial, 2, 2900, 90, 200);
			delay(200);
		}
	}

	// Ajusta configuración.
	retardo_paro_motor = retardo_paro_motor * (1000 / tiempo_cadencia);
	retardo_aceleracion = retardo_aceleracion * (1000 / tiempo_cadencia);
	retardo_inicio_progresivo = retardo_inicio_progresivo * (1000 / tiempo_cadencia);
	// Anulamos el retardo por seguridad para que empiece progresivo al encender la bici.
	contador_retardo_inicio_progresivo = retardo_inicio_progresivo;

	// Cálculo de factores para auto_progresivo.
	if (retardo_inicio_progresivo > 0) {
		fac_s = retardo_paro_motor * 2.0;
		fac_b = (1.0 / retardo_aceleracion - 1.0) / (pow((retardo_inicio_progresivo - 1.0), fac_c) - pow(1.0, fac_c));
		fac_a = 1.0 - pow(1.0, fac_c) * fac_b;
	}

	// Tono de finalización de setup.
	repeatTones(tono_inicial, 3, 3000, 90, 90);
	// Arrancar tiempo de inicio para comprobar cadencia.
	tcadencia = millis();
}

void loop() {
	tiempo = millis();

	v_acelerador = leeAcelerador();

	if (tiempo > tcadencia + (unsigned long) tiempo_cadencia) {
		pulsos = p_pulsos;
		tcadencia = millis();
		estableceCrucero(v_acelerador);

		// Si se pedalea despacio o se paran los pedales.
		if (pulsos < cadencia) {
			contador_retardo_paro_motor++;

			if (contador_retardo_paro_motor >= retardo_paro_motor) {
				contador_retardo_inicio_progresivo++;
				auto_progresivo = true;

				if (contador_retardo_aceleracion > 4) {
					bkp_contador_retardo_aceleracion = contador_retardo_aceleracion;
				}

				paraMotor();
			}
		// Si se pedalea normal (por encima de la cadencia).
		} else if (pulsos >= cadencia) {
			if (auto_progresivo && contador_retardo_inicio_progresivo < retardo_inicio_progresivo) {
				if (bkp_contador_retardo_aceleracion > retardo_aceleracion - fac_s) {
					bkp_contador_retardo_aceleracion = retardo_aceleracion - fac_s;
				}

				contador_retardo_aceleracion = bkp_contador_retardo_aceleracion * (fac_a + fac_b * pow(contador_retardo_inicio_progresivo, fac_c)) * v_crucero / a0_valor_alto;
				auto_progresivo = false;
			} else {
				auto_progresivo = false;
			}

			contador_retardo_inicio_progresivo = 0;
			contador_retardo_paro_motor = 0;

			if (contador_retardo_aceleracion < retardo_aceleracion) {
				contador_retardo_aceleracion++;
			}
		}

		// Asistencia desde parado a 6 km/h mientras se use el acelerador.
		if (ayuda_salida && pulsos == 0 && analogRead(pin_acelerador) > a0_valor_minimo && contador_retardo_aceleracion == 0 && contador_retardo_paro_motor >= retardo_paro_motor) {
			ayudaArranque();
		}

		p_pulsos = 0;
	}

	mandaAcelerador();
}

