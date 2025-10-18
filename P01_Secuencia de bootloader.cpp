// Práctica 1: Secuencia de bootloader que corre al mismo tiempo con 3 leds parpadeando a diferentes frecuencias.
// Christian Emmanuel Castruita Alaniz 
// Manuel García Torres 
// Axel Perea Pinedo 
// 3MM6

//Nota: Este fue el primer codigo y se hizo sin freeRTOS
#include <Arduino.h>

// --------------------------- Configuración de pines ---------------------------
#define LED1    16   // LED 1 (parpadeo cada 500 ms)
#define LED2    17   // LED 2 (parpadeo cada 325 ms)
#define LED3    18   // LED 3 (parpadeo cada 100 ms)
#define PB1     15   
#define PB2     4    
#define LED_OK  19   

// --------------------------- Variables globales -------------------------------
bool st1 = 0, st2 = 0, st3 = 0;  // Estados lógicos de los tres LEDs

unsigned long t1 = 0, t2 = 0, t3 = 0;  // Tiempos de referencia para los LEDs
int estado = 0;                        // Variable para controlar la máquina de estados
unsigned long tInicio = 0;             // Almacena el tiempo de inicio de una acción

// -----------------------------------------------------------------------------
// Configuración inicial del sistema
// -----------------------------------------------------------------------------
void setup() {
  // Configurar pines de salida para los LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED_OK, OUTPUT);


  pinMode(PB1, INPUT_PULLUP);
  pinMode(PB2, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {
  unsigned long now = millis(); 
  // Actividad 01: LEDs parpadeantes
  // LED1 cambia cada 500 ms
  if (now - t1 >= 500) {
    digitalWrite(LED1, st1);
    st1 = !st1;
    t1 = now;
  }

  // LED2 cambia cada 325 ms
  if (now - t2 >= 325) {
    digitalWrite(LED2, st2);
    st2 = !st2;
    t2 = now;
  }

  // LED3 cambia cada 100 ms
  if (now - t3 >= 100) {
    digitalWrite(LED3, st3);
    st3 = !st3;
    t3 = now;
  }

  //                    Máquina de estados
  // Controla el encendido del LED_OK según secuencia de botones

  switch (estado) {
    case 0:
      // Estado 0: Espera que PB1 sea presionado por 1 segundo
      if (digitalRead(PB1) == LOW) {
        if (tInicio == 0) tInicio = now;
        if (now - tInicio > 1000) {
          estado = 1;     // Avanza al siguiente estado
          tInicio = 0;
        }
      } else {
        tInicio = 0;       // Reinicia si se suelta antes del tiempo requerido
      }
      break;

    case 1:
      // Estado 1: Espera que PB1 y PB2 estén presionados simultáneamente por 1 s
      if (digitalRead(PB1) == LOW && digitalRead(PB2) == LOW) {
        if (tInicio == 0) tInicio = now;
        if (now - tInicio > 1000) {
          estado = 2;
          tInicio = 0;
        }
      } else {
        tInicio = 0;
      }
      break;

    case 2:
      // Estado 2: Espera que PB1 esté presionado y PB2 suelto por 1 s
      if (digitalRead(PB1) == LOW && digitalRead(PB2) == HIGH) {
        if (tInicio == 0) tInicio = now;
        if (now - tInicio > 1000) {
          estado = 3;
          tInicio = 0;
        }
      } else {
        tInicio = 0;
      }
      break;

    case 3:
      // Estado 3: Espera que PB1 sea soltado por 1 s
      if (digitalRead(PB1) == HIGH) {
        if (tInicio == 0) tInicio = now;
        if (now - tInicio > 1000) {
          estado = 4;   // Llega al estado final
          tInicio = 0;
        }
      } else {
        tInicio = 0;
      }
      break;

    case 4:
      // Estado 4: Enciende LED_OK indicando que la secuencia se completó
      digitalWrite(LED_OK, HIGH);
      break;
  }
}