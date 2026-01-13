// Proyecto final: Levitador doble con hmi basada en un teclado matricial y una pantalla LCD 16x2
// Christian Emmanuel Castruita Alaniz 
// Manuel García Torres 
// Axel Perea Pinedo 
// 3MM6

// NOTA: LA INFORMACIÓN DEL CONTROLADOR DIFUSO SE ENCUENTRA EN EL REPORTE
// Librerias
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

/* ================= FreeRTOS ================= */
SemaphoreHandle_t i2cMutex;
volatile bool sensoresListos = false;

/* ================= WIFI - FIREBASE ================= */
#define WIFI_SSID "Robotica_cc"
#define WIFI_PASSWORD "12345678"
#define DATABASE_URL "levitadores-a8ad9-default-rtdb.firebaseio.com"
#define API_KEY "AIzaSyBmKjxngC3TG3VclJ9_rcsJnJnwcas3iz8"

/* ================= OBJETOS DE LA BASE DE DATOS, PANTALLA, SENSORES Y MOTOR BRUSHLESS ================= */
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

LiquidCrystal_I2C lcd(0x27, 16, 2);
VL53L0X sensor1, sensor2;
Servo EDF;

/* ================= PINES ================= */
#define SHT_LOX1 4
#define SHT_LOX2 2
#define PWM_PIN_A 16
#define PWM_PIN_B 17
#define DAC_INT 34

/* ================= PID ================= */
float Kp = 1.5, Ki = 0.1, Kd = 0.4;
float error_acumulado = 0, ultimo_error = 0;
unsigned long tiempo_anterior = 0;
float error = 0;

/* ================= TECLADO ================= */
const byte FILAS = 4, COLUMNAS = 4;
char teclas[FILAS][COLUMNAS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte pinesFilas[FILAS] = {13,12,14,27};
byte pinesColumnas[COLUMNAS] = {5,19,33,32};
Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas, FILAS, COLUMNAS);

/* ================= VARIABLES ================= */
int setpoint1 = 0, setpoint2 = 30;
int d1 = 0, d2 = 0;
int Potencia_motor = 0;

/* ================= MENU DE LA INTERFAZ ================= */
enum MenuState {
  MENU_PRINCIPAL,
  VER_SENSORES,
  AJUSTE_SP1,
  AJUSTE_SP2,
  VER_SETPOINTS,
  ERROR_RANGO
};

volatile MenuState menuActual = MENU_PRINCIPAL;
String buffer = "";

/* ================= RANGOS ================= */
#define SP1_MIN 0
#define SP1_MAX 800
#define SP2_MIN 30
#define SP2_MAX 300

/* ================= PID ================= */
void actualizarPID(int lecturaActual) {
  if (setpoint1 == 0) {
    analogWrite(PWM_PIN_A, 0);
    return;
  }

  unsigned long ahora = millis();
  float dt = (ahora - tiempo_anterior) / 1000.0;
  if (dt <= 0) return;

  error = setpoint1+100 - lecturaActual;
  error_acumulado += error * dt;
  error_acumulado = constrain(error_acumulado, -100, 100);

  float salida = Kp * error + Ki * error_acumulado + Kd * (error - ultimo_error) / dt;
  analogWrite(PWM_PIN_A, constrain(salida, 0, 255));

  ultimo_error = error;
  tiempo_anterior = ahora;
}

/* ================= TAREAS ================= */

// Lectura de los sensores y PID
void TaskSensoresPID(void *pvParameters) {
  for (;;) {
    if (!sensoresListos) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    int r1 = sensor1.readRangeContinuousMillimeters();
    int r2 = sensor2.readRangeContinuousMillimeters();
    xSemaphoreGive(i2cMutex);

    if (r1 > 50 && r1 < 1000) d1 = abs(r1 - 810);
    if (r2 > 50 && r2 < 500) d2 = r2 - 70;

    actualizarPID(d1);
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

// Interfaz
void TaskInterfaz(void *pvParameters) {
  static unsigned long lastUpdate = 0;

  for (;;) {
    char tecla = teclado.getKey();

    if (tecla) {
      if (tecla == '*') {
        menuActual = MENU_PRINCIPAL;
        buffer = "";
      }

      switch (menuActual) {

        case MENU_PRINCIPAL:
          if (tecla == 'B') menuActual = VER_SENSORES;
          else if (tecla == '1') { menuActual = AJUSTE_SP1; buffer = ""; }
          else if (tecla == '2') { menuActual = AJUSTE_SP2; buffer = ""; }
          else if (tecla == 'A') menuActual = VER_SETPOINTS;
          break;

        case AJUSTE_SP1:
          if (isdigit(tecla)) buffer += tecla;
          else if (tecla == '#') {
            int v = buffer.toInt();
            menuActual = (v >= SP1_MIN && v <= SP1_MAX) ? (setpoint1 = v, MENU_PRINCIPAL) : ERROR_RANGO;
            buffer = "";
          }
          break;

        case AJUSTE_SP2:
          if (isdigit(tecla)) buffer += tecla;
          else if (tecla == '#') {
            int v = buffer.toInt();
            menuActual = (v >= SP2_MIN && v <= SP2_MAX) ? (setpoint2 = v, MENU_PRINCIPAL) : ERROR_RANGO;
            buffer = "";
          }
          break;

        default:
          break;
      }
    }

    if (millis() - lastUpdate > 300) {
      lastUpdate = millis();

      xSemaphoreTake(i2cMutex, portMAX_DELAY);
      lcd.clear();

      switch (menuActual) {
        case MENU_PRINCIPAL:
          lcd.setCursor(0,0); lcd.print("1:SP1 2:SP2");
          lcd.setCursor(0,1); lcd.print("A:SP B:Sens");
          break;

        case VER_SENSORES:
          lcd.setCursor(0,0); lcd.print("D1:"); lcd.print(d1);
          lcd.setCursor(0,1); lcd.print("D2:"); lcd.print(d2);
          break;

        case AJUSTE_SP1:
          lcd.setCursor(0,0); lcd.print("SP1 (0-800)");
          lcd.setCursor(0,1); lcd.print(buffer);
          break;

        case AJUSTE_SP2:
          lcd.setCursor(0,0); lcd.print("SP2 (30-300)");
          lcd.setCursor(0,1); lcd.print(buffer);
          break;

        case VER_SETPOINTS:
          lcd.setCursor(0,0); lcd.print("SP1:"); lcd.print(setpoint1);
          lcd.setCursor(0,1); lcd.print("SP2:"); lcd.print(setpoint2);
          break;

        case ERROR_RANGO:
          lcd.setCursor(0,0); lcd.print("Valor fuera");
          lcd.setCursor(0,1); lcd.print("de rango");
          break;
      }

      xSemaphoreGive(i2cMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(80));
  }
}

// Salidas hacía la DAC y control del motor brushless
void TaskDAC_Servo(void *pvParameters) {
  for (;;) {
    Potencia_motor = analogRead(DAC_INT);
    Serial.println(d2);
    if(d2>200){
      d2=200;
    }
    dacWrite(25, d2);

    dacWrite(26, setpoint2);
    

    EDF.writeMicroseconds(Potencia_motor * 0.2442 + 1000);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Envio de datos a Firebase
void TaskFirebase(void *pvParameters) {
  for (;;) {
    FirebaseJson json;
    json.set("SP1", setpoint1);
    json.set("D1", d1);
    json.set("E1", (d1-setpoint1));
    json.set("SP2", setpoint2);
    json.set("D2", d2);
    json.set("E2", (d2-setpoint2));
    Firebase.set(fbdo, "/lecturas", json);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void setup() {
  Serial.begin(115200);

  // Calibración del ESC
  EDF.attach(18);
  EDF.writeMicroseconds(1000);
  delay(3000);


  Wire.begin();
  Wire.setClock(100000);
  i2cMutex = xSemaphoreCreateMutex();

  lcd.init();
  lcd.backlight();

  // Configuración y inicialización del wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
  
  // Configuración y inicialización de la base de datos en Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  Firebase.signUp(&config, &auth, "", "");
  Firebase.begin(&config, &auth);

  // Configuración de GPIOs
  pinMode(PWM_PIN_A, OUTPUT);
  pinMode(PWM_PIN_B, OUTPUT);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  // Inicialización de los sensores
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  digitalWrite(SHT_LOX1, HIGH);
  delay(50);
  sensor1.init();
  sensor1.setAddress(0x30);
  sensor1.startContinuous(50);

  digitalWrite(SHT_LOX2, HIGH);
  delay(50);
  sensor2.init();
  sensor2.setAddress(0x31);
  sensor2.startContinuous(50);

  sensoresListos = true;

  //Asignación de las tareas por prioridad y nucleo
  xTaskCreatePinnedToCore(TaskSensoresPID, "PID", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskInterfaz, "UI", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskDAC_Servo, "DAC", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskFirebase, "Firebase", 8192, NULL, 1, NULL, 0);
}

void loop() {
  // Vacio porque la chamba se la avienta free rtos
}
