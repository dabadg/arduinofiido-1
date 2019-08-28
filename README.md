Programa para Arduino en las bicicletas Fiido.

- Version con acelerador y DAC.
- La deteccion de pulsos se hace con millis().
- Progresivos y autoprogresivos no lineales.
- Posibilidad de usar asistencia a 6 km/h desde parado.

CHANGELOG 1.5 --> 1.6:
- Mejorada respuesta en fijacion del crucero.

CHANGELOG 1.4 --> 1.5:
- Corta crucero al dejar de usar la asistencia a 6 km/h.

CHANGELOG 1.3 --> 1.4:
- Lectura del acelerador sin afectacion del tiempo de cadencia.
- Establece crucero controlado por millis().

CHANGELOG 1.2 --> 1.3:
- Eliminado nivel inicial del progresivo.
- Actualizados comentarios.

CHANGELOG 1.1 --> 1.2:
-  Activado de nuevo desacelera_al_parar_pedal por defecto y corregido su comportamiento.
- Pequeña modificacion en calculo del autoprogresivo.

CHANGELOG 1.0 --> 1.1:
- Añadida posibilidad de activar ayuda de 6km/h desde parado mientras se usa el acelerador, accionando el freno al arrancar.
- Corregido temporizador del retardo_paro_motor.
- Por defecto retardo_paro_motor = 0.50.
- Desactivado temporalmente desacelera_al_parar_pedal.

------------------------------------------------------------------------------------------------------------------------------------------

Ayuda, sugerencias, preguntas, etc. en el grupo Fiido Telegram: http://t.me/FiidoD1Spain

Grupo Telegram de Desarrollo privado, si vas a montar el circuito y necesitas ayuda o colaborar pide acceso en el general de arriba.

Canal con montaje, enlaces, programas, etc. http://t.me/fiidolegal
