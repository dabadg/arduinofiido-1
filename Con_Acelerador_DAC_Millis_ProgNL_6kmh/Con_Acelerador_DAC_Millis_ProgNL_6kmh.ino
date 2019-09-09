/* 
                    Versión Con Acelerador y DAC
         Con_Acelerador_DAC_Millis_ProgNL_6kmh 1.8 Develop
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
 * acelerador, por otra parte mandaremos a la placa DAC mediante
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
AUTOPROGRESIVOS:
 * Si se deja de pedalear, el motor se para como de costumbre, pero si
 * continuamos pedaleando antes de transcurridos 10 segundos no inciará
 * el progresivo desde 0 si no que el motor continuará a una velocidad
 * ligeramente inferior a la que íbamos.
 * Si se frena antes de los 10 segundos se anula la función y comenzará
 * progresivo desde cero.
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
const int cadencia1 = 1;

// (True) si se desea activar la posibilidad de acelerar desde parado a
// 6 km/h arrancando con el freno pulsado.
const boolean frenopulsado = true;

// Retardo en segundos para parar el motor una vez se deja de pedalear.
// Usar múltiplos de 0.25 --> 0.25 = 1/4 de segundo.
// No se recomienda subir más de 1.00 segundo.
float retardo_paro_motor = 0.50;

// Retardo en segundos para ponerse a velocidad máxima o crucero.
int retardo_aceleracion = 5;

// (True) modo crucero (False) manda señal del acelerador.
const boolean modo_crucero = true;

// (True) si se desea anular la velocidad de crucero al frenar.
const boolean freno_anula_crucero = false;

// Nivel al que se desea iniciar el progresivo.
// Aumentar si se desea salir con mas tirón.
// >> NO PASAR DE 2.00, NI BAJAR DE 0.85 <<.
float nivel_inicial_progresivo = 1.5;

// Retardo para inciar progresivo tras parar pedales.
// Freno anula el tiempo.
int retardo_inicio_progresivo = 10;

// Suavidad de los progresivos, varía entre 1-10.
// Al crecer se hacen más bruscos.
float suavidad_progresivos = 5;

// Suavidad de los autoprogresivos, varía entre 1-10.
// Al crecer se hacen más brucos
float suavidad_autoprogresivos = 5;

// Ideado para evitar posibles falsos positivos de pedal,
// puede dificultar encontrar la cadencia en cuestas empinadas.
const boolean cadencia_dinamica_ap = true;

// Dirección del bus I2C [DAC] (0x60) si está soldado, si no (0x62).
const int dir_dac = 0x60;

// (True) si se desea desacelerar motor al dejar de pedalear.
const boolean desacelera_al_parar_pedal = true;

// Constante que habilita los tonos de inicialización del sistema.
const boolean tono_inicial = true;

//======= FIN VARIABLES CONFIGURABLES POR EL USUARIO ===================

#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

//======= PINES ========================================================
const int pin_acelerador = A0; // Pin acelerador
const int pin_pedal = 2; // Pin sensor pas, en Nano/Uno usar 2 ó 3
const int pin_freno = 3; // Pin de activación del freno
const int pin_piezo = 11; // Pin del zumbador
// Resto de pines 9,10

//======= VARIABLES PARA CALCULOS ======================================

// Número de pulsos para que se considere que se está pedaleando.
int cadencia = cadencia1;

// Tiempo en milisegundos para contar pulsos.
const int tiempo_cadencia = 250;

// Voltios máximos que da el acelerador, subir si se nota falta de 
// potencia, bajar si para el motor a tope o se producen ruidos raros.
// No pasar de 4.20.
const float v_max_acelerador = 3.90;

// Voltaje mínimo de acelerador en reposo.
const float voltaje_minimo = 0.85;

// Valor mínimo del acelerador para evitar fallos por picos.
const float minimo_acelerador = 1.15;

// 6 km/h en el acelerador.
const float sixkmh_acelerador = 2.19;

// Valores mínimos y máximos del acelerador leídos por el pin A0.
float a0_min_value = 190.0; // Valor por defecto, al inicializar, lee el valor real del acelerador.
const float a0_6km_value = 450.0;
const float a0_med_value = 550.0;
const float a0_max_value = 847.0;

// Variables para millis().
unsigned long tcadencia;
unsigned long tcrucero;
unsigned long tiempo;

// Backup voltaje.
float bkp_voltaje = voltaje_minimo;

// Contadores de paro, aceleración y auto_progresivo.
unsigned contador_retardo_paro_motor = 0;
int contador_retardo_aceleracion = 0;
unsigned contador_retardo_inicio_progresivo = 0;
int bkp_contador_retardo_aceleracion = 0;
boolean auto_progresivo = false;

// Variables progresivos.
float fac_m = 0;
float fac_n = 0;
float fac_p = 0.6222 - 0.0222 * suavidad_progresivos;

// Variables para autoprogresivos.
float fac_s = 0;
float fac_t = 0;
float fac_b = 1;
float fac_a = 0;
float fac_c = suavidad_autoprogresivos / 10.0;

float v_acelerador; // Valor recogido del acelerador.
float v_crucero_ac; // Valor de crucero del acelerador.
float v_crucero = 0.85; // Velocidad de crucero.
// Los voltios que se mandan a la controladora.
float nivel_aceleracion = nivel_inicial_progresivo;

// Contador de pulsos del pedal.
int pulsos = 0;

// Permite usar el acelerador desde parado a 6 km/h.
boolean ayuda_salida = false;

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


float aceleradorEnVoltios(float throttle) {
	return throttle * 5 / 1023;
}

float aceleradorEnDac(float throttle) {
	return (4096 / 5) * throttle;
}

void estableceCrucero() {
	// Pasa a escala de 0-5 voltios.
	v_acelerador = aceleradorEnVoltios(v_acelerador);

	if (v_acelerador > minimo_acelerador) {
		v_crucero_ac = v_acelerador;
		v_crucero = v_crucero_ac;
	}
}

float leeAcelerador() {
	float cl_acelerador = 0;

	// Leemos nivel de acelerador.
	for (int f=1; f <= 30; f++) {
		cl_acelerador = cl_acelerador + analogRead(pin_acelerador);
	}

	cl_acelerador = cl_acelerador / 30;
	return nivelaAcelerador(cl_acelerador);
}


float nivelaAcelerador(float n_acelerador) {
  // Nivelamos los valores para que no salgan del rango de máximo/mínimo.
  if (n_acelerador <= a0_min_value) {
    return a0_min_value;
  } else if (n_acelerador >= a0_max_value) {
    return a0_max_value;
  }
  return n_acelerador;
}

void mandaAcelerador() {
	// Anula crucero por debajo del nivel inicial del progresivo.
	if (v_crucero < nivel_inicial_progresivo) {	
		v_crucero = voltaje_minimo;	
	}	

	// Evita salidas demasiado bruscas.
	if (nivel_inicial_progresivo > 2) {	
		nivel_inicial_progresivo = 2.00;
	}

	if (modo_crucero == true) {
		// Progresivo no lineal.
		fac_n = nivel_inicial_progresivo;
		fac_m = (v_crucero - nivel_inicial_progresivo) / pow(retardo_aceleracion,fac_p);
		nivel_aceleracion = fac_n + fac_m * pow(contador_retardo_aceleracion,fac_p);

		if (nivel_aceleracion == nivel_inicial_progresivo || nivel_aceleracion < voltaje_minimo) {
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
		// Ajusta el voltaje a valor entre 0-4096 (Resolución del DAC).
		uint32_t valor = aceleradorEnDac(nivel_aceleracion);
		// Fija voltaje en DAC.
		dac.setVoltage(valor,false);
	}  
}

void paraMotor() {
	contador_retardo_aceleracion = 0;
}

void freno() {
	contador_retardo_inicio_progresivo = retardo_inicio_progresivo;
	bkp_contador_retardo_aceleracion = 0;

	if (cadencia_dinamica_ap == true) {
		cadencia = cadencia1;
	}

	paraMotor();

	if (freno_anula_crucero == true) {
		v_crucero = voltaje_minimo;
	}
}

void ayudaArranque() {
	// Mientras aceleramos y no pedaleamos.
	while (analogRead(pin_acelerador) > a0_min_value + 10 && p_pulsos == 0) {
		contador_retardo_aceleracion++;
		// No queremos iniciar un progresivo si empezamos a pedalear con el acelerador accionado.
		contador_retardo_inicio_progresivo++;
		// Mandamos al DAC 6 km/h.
		dac.setVoltage(aceleradorEnDac(sixkmh_acelerador),false);
	}

	// Dejamos de asistir en el DAC.
	dac.setVoltage(aceleradorEnDac(voltaje_minimo),false);
	// Cortamos crucero.
	v_crucero = voltaje_minimo;
}

void validaMinAcelerador() {
	// Inicializamos el valor mínimo del acelerador, calculando la media de las medidas si tiene acelerador, en caso de no tener acelerador, mantenemos valor por defecto.
	// Esto es útil para controlar el corecto funcionamiento del acelerador, si este está presente.
	float min_acelerador;

	for (int f=1; f <= 30; f++) {
		min_acelerador = min_acelerador + analogRead(pin_acelerador); // Tomamos 30 medidas para calcular la media.
	}

	min_acelerador = min_acelerador / 30;

	// Si la medida no es correcta, emitimos un aviso sonoro SOS para poder localizar el error y desactivamos el acelerador.
	if ((min_acelerador < a0_min_value - 50) || (min_acelerador > a0_min_value + 50)) {
		SOS_TONE();
	} else {
		a0_min_value = min_acelerador;
	}

	delay(100);
}

void setup() {
	dac.begin(dir_dac); // Configura DAC.
	dac.setVoltage(810,false); // Fija voltaje inicial en Dac (0.85v).

	// Configura pines y prepara las interrupciones.
	pinMode(pin_piezo,OUTPUT);
	pinMode(pin_freno,OUTPUT);
	digitalWrite(pin_freno,HIGH);
	pinMode(pin_pedal,INPUT_PULLUP);
	pinMode(pin_acelerador,INPUT);
	attachInterrupt(digitalPinToInterrupt(pin_pedal),pedal,CHANGE); // Interrupción pedal.
	attachInterrupt(digitalPinToInterrupt(pin_freno),freno,FALLING); // Interrupción freno.

	validaMinAcelerador();

	repeatTones(tono_inicial, 1, 3000, 90, 190); // Tono aviso de inicio a la espera de frenadas (al encender bici).

	// Si arrancamos con el freno pulsado.
	if (frenopulsado == true) {	
		if (digitalRead(pin_freno) == LOW) {			
			ayuda_salida = true; // Activamos la ayuda desde parado a 6kmh.
			delay(200);
			repeatTones(tono_inicial,2,2900,90,200); // Tono aviso de modo con asistencia desde parado.
			delay(200);
		}
	}

	// Ajusta configuración.
	retardo_paro_motor = retardo_paro_motor * (1000 / tiempo_cadencia);
	retardo_aceleracion = retardo_aceleracion * (1000 / tiempo_cadencia);
	retardo_inicio_progresivo = retardo_inicio_progresivo * (1000 / tiempo_cadencia);
	// Anulamos el retardo por seguridad para que empiece progresivo al encender la bici
	contador_retardo_inicio_progresivo = retardo_inicio_progresivo;

	// Cálculo de factores para auto_progresivo.
	if (retardo_inicio_progresivo > 0) {
		fac_s = retardo_paro_motor * 2.0;
		fac_t = (retardo_aceleracion * 1.0) / ((retardo_aceleracion-fac_s) * 1.0);
		fac_b = (1.0 / (retardo_aceleracion - fac_s) - fac_t) / (pow((retardo_inicio_progresivo - 1.0),fac_c) - pow(1.0,fac_c));
		fac_a = fac_t - pow(1.0,fac_c) * fac_b;
		if (!desacelera_al_parar_pedal) {
			fac_b = (1.0 / retardo_aceleracion - 1.0) / (pow((retardo_inicio_progresivo - 1.0),fac_c) - pow(1.0,fac_c));
			fac_a = 1.0 - pow(1.0,fac_c) * fac_b;
		}
	}
	
	repeatTones(tono_inicial,3,3000,90,90); // Tono de finalización de setup.
	/*delay(100);
	repeatTones(tono_inicial,1,2500,90,150); // Tono verificación inicialización de modo x.*/
	
	tcadencia = millis(); // Arrancar tiempo inicio para comprobar cadencia.
	tcrucero = millis(); // Arrancar tiempo inicio para establecer crucero.
}

void loop() {
	tiempo = millis();

	v_acelerador = leeAcelerador();

	// Establecemos un retardo para detectar la caída de voltaje en el crucero.
	if (tiempo > tcrucero + 125) { // Si ha pasado 125 ms.
		tcrucero = millis(); // Actualiza tiempo actual.
		estableceCrucero();
	}

	if (tiempo > tcadencia + (unsigned long) tiempo_cadencia) {
		pulsos = p_pulsos;
		tcadencia = millis();

		if (pulsos < cadencia) { // Si se pedalea despacio o se paran los pedales.
			contador_retardo_paro_motor++;

			if (contador_retardo_paro_motor >= retardo_paro_motor) {
				contador_retardo_inicio_progresivo++;
				auto_progresivo = true;

				if (contador_retardo_aceleracion > 4) {
					bkp_contador_retardo_aceleracion = contador_retardo_aceleracion;
					
					if (cadencia_dinamica_ap == true) {
						cadencia = 3;
					}
				}
				paraMotor();
			}
		} else if (pulsos >= cadencia) { // Si se pedalea normal (por encima de la cadencia).
			if (contador_retardo_inicio_progresivo < retardo_inicio_progresivo && auto_progresivo) {
				if (bkp_contador_retardo_aceleracion > retardo_aceleracion - fac_s) {
					bkp_contador_retardo_aceleracion = retardo_aceleracion - fac_s;
				}

				contador_retardo_aceleracion = bkp_contador_retardo_aceleracion * (fac_a+fac_b * pow(contador_retardo_inicio_progresivo,fac_c)) * v_crucero/v_max_acelerador;
				auto_progresivo = false;

				if (cadencia_dinamica_ap == true) {
					cadencia = cadencia1;
				}
			} else {
				auto_progresivo = false;
			}

			contador_retardo_inicio_progresivo = 0;
			contador_retardo_paro_motor = 0;

			if (contador_retardo_aceleracion < retardo_aceleracion) {
				contador_retardo_aceleracion++;
			}
		} else if (pulsos == 0) { // Si están los pedales parados.
			// Desacelera al parar los pedales.
			if (contador_retardo_aceleracion > 0 && desacelera_al_parar_pedal == true) {
				contador_retardo_aceleracion = contador_retardo_aceleracion - 2;
				if (contador_retardo_aceleracion < 0) {
					contador_retardo_aceleracion = 0;
				}
			}

		}

		// Asistencia desde parado a 6 km/h mientras se use el acelerador.
		if (pulsos == 0 && analogRead(pin_acelerador) > a0_min_value + 10 && contador_retardo_aceleracion == 0 && contador_retardo_paro_motor >= retardo_paro_motor && ayuda_salida) {
			ayudaArranque();
		}
		
		p_pulsos = 0;
	}

	mandaAcelerador();
}
