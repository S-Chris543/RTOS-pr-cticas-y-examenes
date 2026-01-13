// Práctica 5: Intercambio de mensajes entre tareas FreeRTOS utilizando colas
// Christian Emmanuel Castruita Alaniz 
// Manuel García Torres 
// Axel Perea Pinedo 
// 3MM6

#include <Arduino.h> // Esta libreria se pone porque lo trabajamos en platformio

// CONFIGURACIÓN DEL NÚCLEO (CORE) DEL ESP32
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t nucleo_aplicacion = 0;
#else
  static const BaseType_t nucleo_aplicacion = 1;
#endif

// CONFIGURACIÓN DE COLAS

// Número máximo de elementos que puede almacenar cada cola
static const uint8_t longitud_cola = 5;

// Colas para comunicación bidireccional entre tareas
static QueueHandle_t cola_envio_A_a_B;
static QueueHandle_t cola_envio_B_a_A;

// Envía datos a la Tarea B y recibe respuesta
void tareaEmisoraA(void *parameter) {
  int dato_enviado_A = 0;     // Valor que se envía a la Tarea B
  int dato_recibido_B = 0;    // Valor recibido desde la Tarea B

  while (1) {
    // Envío de mensaje hacia la Tarea B
    if (xQueueSend(cola_envio_A_a_B, (void *)&dato_enviado_A, 10) != pdTRUE) {
      // La cola está llena y no se pudo enviar el mensaje
      Serial.println("Cola A -> B llena");
    } else {
      Serial.print("Tarea A envía: ");
      Serial.println(dato_enviado_A);
      dato_enviado_A++; // Incrementa el valor enviado
    }

    // Espera la respuesta proveniente de la Tarea B
    if (xQueueReceive(cola_envio_B_a_A,
                       (void *)&dato_recibido_B,
                       1000 / portTICK_PERIOD_MS) == pdTRUE) {
      Serial.print("Tarea A recibe de B: ");
      Serial.println(dato_recibido_B);
    }

    // Retardo para evitar saturar el procesador
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Recibe datos de la Tarea A y responde
  void tareaReceptoraB(void *parameter) {
  int dato_recibido_A = 0;    // Valor recibido desde la Tarea A
  int dato_respuesta_B = 100; // Valor que se envía como respuesta

  while (1) {
    // Espera un mensaje proveniente de la Tarea A
    if (xQueueReceive(cola_envio_A_a_B,(void *)&dato_recibido_A,1000 / portTICK_PERIOD_MS) == pdTRUE) {

      Serial.print("Tarea B recibe de A: ");
      Serial.println(dato_recibido_A);

      // Envío de respuesta hacia la Tarea A
      if (xQueueSend(cola_envio_B_a_A,(void *)&dato_respuesta_B,10) != pdTRUE) {
        // La cola está llena
        Serial.println("Cola B -> A llena");
      } else {
        Serial.print("Tarea B envía: ");
        Serial.println(dato_respuesta_B);
        dato_respuesta_B++; // Incrementa la respuesta
      }
    }

    // Retardo periódico
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200); // Inicialización del puerto serial
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Inicio del programa con FreeRTOS");

  // Creación de las colas (buffers circulares)
  cola_envio_A_a_B = xQueueCreate(longitud_cola, sizeof(int));
  cola_envio_B_a_A = xQueueCreate(longitud_cola, sizeof(int));

  // Verificación de creación correcta de las colas
  if (cola_envio_A_a_B == NULL || cola_envio_B_a_A == NULL) {
    Serial.println("Error al crear las colas");
    while (1); // Detiene el programa
  }

  // Creación de tareas ancladas a un núcleo específico
  xTaskCreatePinnedToCore(tareaEmisoraA,"Tarea Emisora A",2048,NULL,1,NULL,nucleo_aplicacion);
  xTaskCreatePinnedToCore(tareaReceptoraB,"Tarea Receptora B",2048,NULL,1,NULL,nucleo_aplicacion);
}


void loop() {
  // Vacío pq free rtos se avienta la chamba
}
