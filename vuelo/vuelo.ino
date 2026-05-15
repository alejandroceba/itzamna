// ==========================================
// BME280 + Lector de RPM con 1 Sensor TCRT5000
// Placa: Seeed Studio XIAO ESP32
// ==========================================

#include <Wire.h>
#include <Adafruit_BME280.h>

// Pin donde conectaste el Colector del sensor (y la resistencia de 10k)
#define SENSOR1 D0

#define PULSOS_POR_REV 2
#define TIEMPO_MUESTREO 500   // ms (Medio segundo)
#define DEBOUNCE_US 2000      // Antirrebote de 2 milisegundos

#define N_PROMEDIO 5
#define BME280_ADDR_1 0x76
#define BME280_ADDR_2 0x77

Adafruit_BME280 bme;

volatile unsigned long pulsos1 = 0;
volatile unsigned long t1 = 0;

float buffer1[N_PROMEDIO];
int indice = 0;

// ===== INTERRUPCIONES =====
void IRAM_ATTR isr1() {
  unsigned long ahora = micros();
  // Filtro antirrebote (Debounce)
  if (ahora - t1 > DEBOUNCE_US) {
    pulsos1++;
    t1 = ahora;
  }
}

// ===== PROMEDIO =====
float promedio(float *buffer) {
  float suma = 0;
  for (int i = 0; i < N_PROMEDIO; i++) {
    suma += buffer[i];
  }
  return suma / N_PROMEDIO;
}

bool iniciarBME280() {
  if (bme.begin(BME280_ADDR_1)) {
    Serial.println("BME280 encontrado en 0x76");
    return true;
  }

  if (bme.begin(BME280_ADDR_2)) {
    Serial.println("BME280 encontrado en 0x77");
    return true;
  }

  return false;
}

void setup() {
  // Inicializamos la comunicación serie a 115200 baudios
  Serial.begin(115200);
  delay(1500);
  Serial.println("Sistema iniciado.");

  // Iniciamos I2C para el BME280
  Wire.begin();

  if (!iniciarBME280()) {
    Serial.println("ERROR: No se pudo iniciar el BME280");
    while (1) {
      delay(1000);
    }
  }

  // Configuramos el pin del sensor como entrada
  pinMode(SENSOR1, INPUT);

  // Adjuntamos la interrupción para que se active cuando la señal caiga a 0V (marca blanca detectada)
  attachInterrupt(digitalPinToInterrupt(SENSOR1), isr1, FALLING);

  // Limpiamos el buffer inicializándolo en ceros
  for (int i = 0; i < N_PROMEDIO; i++) {
    buffer1[i] = 0.0;
  }

  Serial.println("Leyendo altura y RPM cada 500 ms...");
}

// ===== CONTROL DE TIEMPO =====
unsigned long t_anterior = 0;

void loop() {
  unsigned long t_actual = millis();

  // Se ejecuta cada 500 ms
  if (t_actual - t_anterior >= TIEMPO_MUESTREO) {
    t_anterior = t_actual;

    // Leemos altura del BME280
    float altitude = bme.readAltitude(1013.25f);

    // Pausamos interrupciones un instante para no perder datos al leer
    noInterrupts();
    unsigned long c1 = pulsos1;
    pulsos1 = 0;
    interrupts();

    // Cálculo de RPM con la fórmula original
    float rpm1 = ((c1 * 60000.0) / (PULSOS_POR_REV * TIEMPO_MUESTREO)) / 2.0;

    // Guardamos el dato en el buffer
    buffer1[indice] = rpm1;

    indice++;
    if (indice >= N_PROMEDIO) indice = 0;

    // Suavizamos el resultado con el promedio móvil
    float rpm1_suave = promedio(buffer1);

    // Enviamos los datos al Monitor Serie o Serial Plotter
    Serial.print("Tiempo(ms):");
    Serial.print(t_actual);
    Serial.print("\tAltitud(m):");
    Serial.print(altitude, 2);
    Serial.print("\tRPM:");
    Serial.println(rpm1_suave);
  }
}