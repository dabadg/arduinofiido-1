#include <Arduino.h>

void nivelarRango(int &value, int minValue, int maxValue){
	// Nivelamos los valores de la media para que no se salgan del rango de mínimo.
	if (value < minValue) {
		value = minValue;
	// Nivelamos los valores de la media para que no se salgan del rango de máximo.
	} else if (value > maxValue) {
		value=maxValue;
	}
}

void nivelarRango(long &value, long minValue, long maxValue){
	// Nivelamos los valores de la media para que no se salgan del rango de mínimo.
	if (value < minValue) {
		value = minValue;
	// Nivelamos los valores de la media para que no se salgan del rango de máximo.
	} else if (value > maxValue) {
		value=maxValue;
	}
}

void nivelarRango(float &value, float minValue, float maxValue){
	// Nivelamos los valores de la media para que no se salgan del rango de mínimo.
	if (value < minValue) {
		value = minValue;
	// Nivelamos los valores de la media para que no se salgan del rango de máximo.
	} else if (value > maxValue) {
		value=maxValue;
	}
}

// Calcula si el valor se encuantra entre el rango de valores con
// tolerancia calculados con el valor2.
// valor2-tolerancia > valor < valor2+tolerancia
boolean comparaConTolerancia(int valor, int valorReferencia, byte toleranciaValor2) {
	return (valor > (valorReferencia - toleranciaValor2)) && (valor < (valorReferencia + toleranciaValor2));
}

// EOF
