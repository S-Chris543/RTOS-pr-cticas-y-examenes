// Práctica 6: PID Motor DC con encoder
// Christian Emmanuel Castruita Alaniz 
// Manuel García Torres 
// Axel Perea Pinedo 
// 3MM6

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// CONFIGURACIÓN DEL NÚCLEO (CORE) DEL ESP32

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t CORE_APP = 0;
#else
  static const BaseType_t CORE_APP = 1;
#endif

// DEFINICIÓN DE PINES

// Encoder incremental
const int PIN_ENCODER_A = 18;   // Canal A del encoder
const int PIN_ENCODER_B = 19;   // Canal B del encoder

// Motor (puente H)
const int PIN_MOTOR_POS = 25;   // Salida motor sentido positivo
const int PIN_MOTOR_NEG = 26;   // Salida motor sentido negativo

// CONFIGURACIÓN PWM
const int PWM_FREQUENCY = 20000; // Frecuencia PWM: 20 kHz
const int PWM_RESOLUTION = 8;    // Resolución PWM: 8 bits (0–255)

// VARIABLES GLOBALES
volatile long encoderCount = 0; // Conteo de pulsos del encoder

// Variables compartidas entre tareas
float positionSetpoint = 0.0;   // Consigna de posición
float positionDebug = 0.0;      // Posición actual (debug)
float pwmDebug = 0.0;           // PWM calculado (debug)

// Sincronización
SemaphoreHandle_t mutexSetpoint;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// PARÁMETROS DEL SISTEMA
const float encoderResolution = 0.02406; // Resolución del encoder
const float derivFilterAlpha = 0.05;     // Filtro derivativo

// Ganancias del PID
const float Kp = 2.8;
const float Kd = 0.4;
const float Ki = 0.06;

// INTERRUPCIONES DEL ENCODER 

// ISR para el canal A
void IRAM_ATTR ISR_Encoder_A() {
    portENTER_CRITICAL_ISR(&encoderMux);
    if (digitalRead(PIN_ENCODER_B) == LOW) {
        encoderCount++;
    } else {
        encoderCount--;
    }
    portEXIT_CRITICAL_ISR(&encoderMux);
}

// ISR para el canal B
void IRAM_ATTR ISR_Encoder_B() {
    portENTER_CRITICAL_ISR(&encoderMux);
    if (digitalRead(PIN_ENCODER_A) == HIGH) {
        encoderCount++;
    } else {
        encoderCount--;
    }
    portEXIT_CRITICAL_ISR(&encoderMux);
}

// CONTROL PID 
void TaskPID(void *pvParameters) {

    float position = 0.0, prevPosition = 0.0;
    float velocityRaw = 0.0, velocityFiltered = 0.0;
    float error = 0.0, errorDeriv = 0.0, errorInt = 0.0;
    float controlSignal = 0.0, controlSat = 0.0;
    float pwmValue = 0.0;

    long encoderLocal = 0;

    const int period_ms = 2;
    const float dt = period_ms * 0.001;

    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(period_ms));

        // Lectura segura del encoder
        portENTER_CRITICAL(&encoderMux);
        encoderLocal = encoderCount;
        portEXIT_CRITICAL(&encoderMux);

        // Lectura segura del setpoint
        float targetPosition = position;
        if (xSemaphoreTake(mutexSetpoint, pdMS_TO_TICKS(10)) == pdTRUE) {
            targetPosition = positionSetpoint;
            xSemaphoreGive(mutexSetpoint);
        }

        // Cálculo de posición y velocidad
        position = encoderResolution * encoderLocal;
        velocityRaw = (position - prevPosition) / dt;
        velocityFiltered = derivFilterAlpha * velocityRaw +
                           (1.0 - derivFilterAlpha) * velocityFiltered;

        // Control PID
        error = targetPosition - position;
        errorDeriv = -velocityFiltered;
        errorInt += error * dt;

        controlSignal = (Kp * error) +
                        (Kd * errorDeriv) +
                        (Ki * errorInt);

        // Saturación y conversión a PWM
        controlSat = constrain(controlSignal, -12.0, 12.0);
        pwmValue = controlSat * 21.25;

        int pwmOutput = abs(pwmValue);
        if (pwmOutput > 255) pwmOutput = 255;

        // Accionamiento del motor
        if (pwmValue > 0) {
            ledcWrite(PIN_MOTOR_POS, pwmOutput);
            ledcWrite(PIN_MOTOR_NEG, 0);
        } else {
            ledcWrite(PIN_MOTOR_POS, 0);
            ledcWrite(PIN_MOTOR_NEG, pwmOutput);
        }

        // Actualización
        prevPosition = position;
        positionDebug = position;
        pwmDebug = pwmValue;
    }
}

// COMUNICACIÓN SERIAL
void TaskSerial(void *pvParameters) {

    String inputLine = "";

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20));

        if (Serial.available() > 0) {
            inputLine = Serial.readStringUntil('\n');
            float newSetpoint = inputLine.toFloat();

            if (xSemaphoreTake(mutexSetpoint, pdMS_TO_TICKS(10)) == pdTRUE) {
                positionSetpoint = newSetpoint;
                xSemaphoreGive(mutexSetpoint);
            }
        }

        Serial.print(positionDebug);
        Serial.print(",");
        Serial.println(pwmDebug);
    }
}


void setup() {

    Serial.begin(115200);

    // Configuración del encoder
    pinMode(PIN_ENCODER_A, INPUT_PULLUP);
    pinMode(PIN_ENCODER_B, INPUT_PULLUP);

    // Configuración del PWM
    if (!ledcAttach(PIN_MOTOR_POS, PWM_FREQUENCY, PWM_RESOLUTION)) {
        Serial.println("Error configurando PWM motor positivo");
    }
    if (!ledcAttach(PIN_MOTOR_NEG, PWM_FREQUENCY, PWM_RESOLUTION)) {
        Serial.println("Error configurando PWM motor negativo");
    }

    // Asociación de interrupciones
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), ISR_Encoder_A, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_B), ISR_Encoder_B, RISING);

    mutexSetpoint = xSemaphoreCreateMutex();

    // Creación de tareas
    xTaskCreatePinnedToCore(TaskPID, "PID_Task", 4096, NULL, 2, NULL, CORE_APP);
    xTaskCreatePinnedToCore(TaskSerial, "Serial_Task", 4096, NULL, 1, NULL, CORE_APP);

    Serial.println("Sistema PID con FreeRTOS iniciado");
}

void loop() {
      // Vacío pq free rtos se avienta la chamba
}