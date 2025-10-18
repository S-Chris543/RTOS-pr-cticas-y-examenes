// Práctica 4: Semáforo con peticiones (4 botones, solo rojo y verde).cpp 
// Christian Emmanuel Castruita Alaniz 
// Manuel García Torres 
// Axel Perea Pinedo 
// 3MM6

#include <Arduino.h>

#define LED_ROJO     19
#define LED_AMARILLO 18
#define LED_VERDE    2

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

static TimerHandle_t semaforoTimer = NULL;


// Variables de control del estado del semáforo
volatile int estado = 0;
volatile int parpadeoCount = 0;

// Función de callback del temporizador
// Se ejecuta automáticamente cada vez que el timer vence su periodo
// Controla el cambio de luces del semáforo según el estado actual

void semaforoCallback(TimerHandle_t xTimer) {
  switch (estado) {

    case 0: // Encender luz ROJA durante 2 segundos
      digitalWrite(LED_ROJO, HIGH);
      digitalWrite(LED_AMARILLO, LOW);
      digitalWrite(LED_VERDE, LOW);
      estado = 1; // siguiente estado
      // Ajusta el periodo del timer a 2 segundos
      xTimerChangePeriod(semaforoTimer, 2000 / portTICK_PERIOD_MS, 0);
      break;

    case 1: // Apagar luz roja e iniciar parpadeo amarillo
      digitalWrite(LED_ROJO, LOW);
      parpadeoCount = 0;  // reinicia contador de parpadeos
      estado = 2;
      // Cambia el periodo del timer a 500ms para parpadear
      xTimerChangePeriod(semaforoTimer, 500 / portTICK_PERIOD_MS, 0);
      break;

    case 2: // Parpadeo de la luz amarilla durante 2 segundos (4 ciclos de 500ms)
      digitalWrite(LED_AMARILLO, !digitalRead(LED_AMARILLO));  // alterna estado
      parpadeoCount++;
      if (parpadeoCount >= 4) { // 4 parpadeos de 500ms = 2s
        digitalWrite(LED_AMARILLO, LOW);
        estado = 3; // siguiente fase: verde
        // Cambia el periodo del timer a 2 segundos
        xTimerChangePeriod(semaforoTimer, 2000 / portTICK_PERIOD_MS, 0);
      }
      break;

    case 3: // Encender luz VERDE durante 2 segundos
      digitalWrite(LED_VERDE, HIGH);
      estado = 4;
      xTimerChangePeriod(semaforoTimer, 2000 / portTICK_PERIOD_MS, 0);
      break;

    case 4: // Apagar luz verde y volver al inicio del ciclo
      digitalWrite(LED_VERDE, LOW);
      estado = 0;
      // Reinicia el ciclo con un breve periodo de 10ms
      xTimerChangePeriod(semaforoTimer, 10 / portTICK_PERIOD_MS, 0);
      break;
  }
}


void setup() {

  pinMode(LED_ROJO, OUTPUT);
  pinMode(LED_AMARILLO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);


  semaforoTimer = xTimerCreate(
    "SemaforoTimer",
    10 / portTICK_PERIOD_MS,
    pdTRUE,
    (void *)0,
    semaforoCallback
  );

  if (semaforoTimer != NULL) {
    xTimerStart(semaforoTimer, 0);
  }
}

void loop() {
  // El control del semáforo se realiza completamente desde el timer

}
