// Práctica 3: Secuencia de vaciado
// Christian Emmanuel Castruita Alaniz 
// Manuel García Torres 
// Axel Perea Pinedo 
// 3MM6

#include <Arduino.h>

#define LED_VA 15
#define LED_VB 2
#define LED_V2 4
#define LED_V1 16
#define LED_MOTOR 18
#define LED_VACIADO 19 


#define PB_SP1 13
#define PB_SP2 12
#define PB_DL1 14
#define PB_DL2 27
#define PB_MOTOR 26
#define PB_VA 25


int VA_FL,VB_FL,V1_FL,V2_FL,MOTOR_FL= 0;


// Configuacion de nucleos a utilizar
#if CONFIG_FREERTOS_UNICORE
// Si FreeRTOS esta configurado para usar un solo nucleo, se asigna el nucleo 0
static const BaseType_t app_cpu = 0;
#else
// Si se usan dos nucleos, se asigna el nucleo 1
static const BaseType_t app_cpu = 1;
#endif

void VA(void *parameter){
  while(1){
  if((digitalRead(PB_SP1)==0) && VA_FL == 0){
    digitalWrite(LED_VA,1);
    delay(1000);
    digitalWrite(LED_VA,0);
    VA_FL = 1; 
  }
      digitalWrite(LED_VA,0);
  }
}

void VB(void *parameter){
while(1){
  if((digitalRead(PB_SP2)==0) && (VA_FL == 1) && (VB_FL = 0)){
    digitalWrite(LED_VA,1);
    vTaskDelay(1000 /portTICK_PERIOD_MS);
    digitalWrite(LED_VA,0);
    VB_FL = 1; 
  }
  }
}

void V1(void *parameter){
  if((digitalRead(PB_DL1)==0) && (VB_FL == 1) && (V1_FL = 0)){
    digitalWrite(LED_V1,1);
    vTaskDelay(1000 /portTICK_PERIOD_MS);
    digitalWrite(LED_V1,0);
    V1_FL = 1; 
  }
  }

void V2(void *parameter){
  if((digitalRead(PB_DL2)==0) && (V1_FL == 1) && (V2_FL = 0)){
    digitalWrite(LED_V2,1);
    vTaskDelay(2000 /portTICK_PERIOD_MS);
    digitalWrite(LED_V2,0);
    V2_FL = 1; 
  }
  }

void MOTOR(void *parameter){
  if((digitalRead(PB_MOTOR)==0) && (V2_FL == 1) && (MOTOR_FL= 0)){
    digitalWrite(LED_MOTOR,1);
    vTaskDelay(2000 /portTICK_PERIOD_MS);
    digitalWrite(LED_MOTOR,0);
    MOTOR_FL = 1; 
  }
  }

void VACIADO(void *parameter){
  if((digitalRead(PB_VA)==0) && (MOTOR_FL == 1)){
    digitalWrite(LED_VACIADO,1);
    vTaskDelay(3000 /portTICK_PERIOD_MS);
    digitalWrite(LED_VACIADO,0);
   VA_FL,VB_FL,V1_FL,V2_FL,MOTOR_FL= 0;
  }
  }
  
  void setup()
{
  pinMode(LED_VA, OUTPUT);
  pinMode(LED_VB, OUTPUT);
  pinMode(LED_V1, OUTPUT);
  pinMode(LED_V2, OUTPUT);
  pinMode(LED_MOTOR, OUTPUT);
  pinMode(LED_VACIADO, OUTPUT);

  pinMode(PB_SP1, INPUT_PULLUP);
  pinMode(PB_SP2, INPUT_PULLUP);
  pinMode(PB_DL1, INPUT_PULLUP);
  pinMode(PB_DL2, INPUT_PULLUP);
  pinMode(PB_MOTOR, INPUT_PULLUP);
  pinMode(PB_VA, INPUT_PULLUP);

// Creacion de una tarra que se ejecutara de forma indefinida
  xTaskCreatePinnedToCore( //En ESP32 SE utiliza esta funcion, en FreeRTOS normal seria xTaskCreate()

    VA,    // Funcion que implementa la tarea
    "VA", //  Nombre descriptivo de la funcion
    1024,         //  Tamaño de la pila asignada
    NULL,         //  Parametro a pasar de la funcion
    1,            //  Prioridad de la funcion
    NULL,         //  Handle de la tarea
    app_cpu);     //  Núcleo en el que se ejecutará la tarea (0 o 1)
  
  xTaskCreatePinnedToCore( 

    VB,    // Funcion que implementa la tarea
    "VB", //  Nombre descriptivo de la funcion
    1024,         //  Tamaño de la pila asignada
    NULL,         //  Parametro a pasar de la funcion
    1,            //  Prioridad de la funcion
    NULL,         //  Handle de la tarea
    app_cpu);     //  Núcleo en el que se ejecutará la tarea (0 o 1)

  xTaskCreatePinnedToCore( 

    V1,    // Funcion que implementa la tarea
    "V1", //  Nombre descriptivo de la funcion
    1024,         //  Tamaño de la pila asignada
    NULL,         //  Parametro a pasar de la funcion
    1,            //  Prioridad de la funcion
    NULL,         //  Handle de la tarea
    app_cpu);     //  Núcleo en el que se ejecutará la tarea (0 o 1)
  xTaskCreatePinnedToCore( 

    V2,    // Funcion que implementa la tarea
    "V2", //  Nombre descriptivo de la funcion
    1024,         //  Tamaño de la pila asignada
    NULL,         //  Parametro a pasar de la funcion
    1,            //  Prioridad de la funcion
    NULL,         //  Handle de la tarea
    app_cpu);     //  Núcleo en el que se ejecutará la tarea (0 o 1)

  xTaskCreatePinnedToCore( 

    MOTOR,    // Funcion que implementa la tarea
    "MOTOR", //  Nombre descriptivo de la funcion
    1024,         //  Tamaño de la pila asignada
    NULL,         //  Parametro a pasar de la funcion
    1,            //  Prioridad de la funcion
    NULL,         //  Handle de la tarea
    app_cpu);     //  Núcleo en el que se ejecutará la tarea (0 o 1)

  xTaskCreatePinnedToCore( 

    VACIADO,    // Funcion que implementa la tarea
    "VACIADO", //  Nombre descriptivo de la funcion
    1024,         //  Tamaño de la pila asignada
    NULL,         //  Parametro a pasar de la funcion
    1,            //  Prioridad de la funcion
    NULL,         //  Handle de la tarea
    app_cpu);     //  Núcleo en el que se ejecutará la tarea (0 o 1)
  }

void loop()
{


}

