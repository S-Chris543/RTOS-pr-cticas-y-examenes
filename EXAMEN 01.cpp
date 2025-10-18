// Examen 1: Simon Dice
// Christian Emmanuel Castruita Alaniz 
// Manuel García Torres 
// Axel Perea Pinedo 
// 3MM6


#include <Arduino.h>
#include <Wire.h>                 //Libreria I2C
#include <LiquidCrystal_I2C.h>    //Libreria para la pantalla LCD
#include <Keypad.h>               //Libreria para el teclado matricial

// Configuacion de nucleos a utilizar
#if CONFIG_FREERTOS_UNICORE
// Si FreeRTOS esta configurado para usar un solo nucleo, se asigna el nucleo 0
static const BaseType_t app_cpu = 0;
#else
// Si se usan dos nucleos, se asigna el nucleo 1
static const BaseType_t app_cpu = 1;
#endif


// Variables y configuracion de la LCD I2C
#define I2C_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_ROWS);

// Variables y configuracion del teclado matricial
const byte FILAS = 4;
const byte COLUMNAS = 4;

char teclas[FILAS][COLUMNAS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte pinesFilas[FILAS] = {19, 18, 5, 17};
byte pinesColumnas[COLUMNAS] = {16, 4, 23, 15};

Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas, FILAS, COLUMNAS);

// Tareas de FreeRTOS 
TaskHandle_t tareaMostrar;
TaskHandle_t tareaLeer;

// Variables del juego
#define MAX_NIVEL 10
char secuencia[MAX_NIVEL];
int nivel = 1;
int indiceJugador = 0;
bool jugando = false;
bool mostrando = false;

// Teclas posibles 
char teclasDisponibles[] = {
  '1','2','3','A',
  '4','5','6','B',
  '7','8','9','C',
  '*','0','#','D'
};
const int totalTeclas = sizeof(teclasDisponibles) / sizeof(teclasDisponibles[0]);

// Funciones para generar la secuencia aleatoria del Simon dice
void generarSecuencia() {
  for (int i = 0; i < MAX_NIVEL; i++) {
    int indice = random(0, totalTeclas);
    secuencia[i] = teclasDisponibles[indice];
  }
}

// Tarea 1: Mostrar secuencia 
void mostrarSecuencia(void *parameter) {
  while (true) {
    if (jugando && mostrando) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Simon dice:");

      for (int i = 0; i < nivel; i++) {
        lcd.setCursor(0, 1);
        lcd.print(secuencia[i]);
        vTaskDelay(pdMS_TO_TICKS(500));  // muestra tecla
        lcd.setCursor(0, 1);
        lcd.print(" ");
        vTaskDelay(pdMS_TO_TICKS(200));  // pausa
      }

      lcd.clear();
      lcd.print("Tu turno...");
      mostrando = false;  // pasa al jugador
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Tarea 2: Leer teclado
void leerTeclado(void *parameter) {
  while (true) {
    char tecla = teclado.getKey();
    if (tecla && !mostrando) {
      lcd.setCursor(indiceJugador, 1);
      lcd.print(tecla);

      if (tecla == secuencia[indiceJugador]) {
        indiceJugador++;
        if (indiceJugador == nivel) {
          nivel++;
          if (nivel > MAX_NIVEL) nivel = MAX_NIVEL;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Correcto!");
          vTaskDelay(pdMS_TO_TICKS(1000));
          indiceJugador = 0;
          mostrando = true; // siguiente ronda
        }
      } else {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Incorrecto!");
        vTaskDelay(pdMS_TO_TICKS(1500));
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Intentalo otra");
        lcd.setCursor(0, 1);
        lcd.print("vez!");
        vTaskDelay(pdMS_TO_TICKS(1500));
        nivel = 1;
        indiceJugador = 0;
        generarSecuencia();
        mostrando = true;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();
  randomSeed(analogRead(0));
  
  //Inicio del Juego donde muestra el nombre del Juego y los nombres de los integrantes
  lcd.setCursor(0, 0);
  lcd.print("Juego Simon Dice");
  lcd.setCursor(0, 1);
  lcd.print("Free RTOS");
  vTaskDelay(pdMS_TO_TICKS(2000));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Christian");
  lcd.setCursor(0, 1);
  lcd.print("Castruita");
  vTaskDelay(pdMS_TO_TICKS(1000));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Manuel Garcia");
  vTaskDelay(pdMS_TO_TICKS(1000));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Axel Perea");
  vTaskDelay(pdMS_TO_TICKS(1000));
  lcd.clear();

  generarSecuencia();
  jugando = true;
  mostrando = true;

  // Crear tareas FreeRTOS
  xTaskCreatePinnedToCore(mostrarSecuencia, "Mostrar", 4096, NULL, 1, &tareaMostrar, 1);
  xTaskCreatePinnedToCore(leerTeclado, "Leer", 4096, NULL, 1, &tareaLeer, 1);
}

void loop() {

}