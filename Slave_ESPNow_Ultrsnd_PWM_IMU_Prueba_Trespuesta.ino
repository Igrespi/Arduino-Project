//*****************************************************************************
//******************************SLAVE******************************************
//**********************ESP32 EN LA DRONE***************************************
//*****************************************************************************

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include "esp_log.h"

unsigned long lastPulsoPrueba = 0;
int16_t AcXprue, AcYprue, AcZprue;
#define PeriodoPrueba  1000000

#define TRIG_PIN   5
#define ECHO_PIN   18
#define PRUEBA_PIN 17

unsigned long lastTriggerTime = 0;
unsigned long echoStartTime   = 0;
unsigned long echoEndTime     = 0;
enum EstadosUltrsnd { INICIAL, PULSO_TRIGGER, ECHO_START, ECHO_END };
EstadosUltrsnd Estado_ultrsnd = INICIAL;
#define PeriodMedUltrsn 15000UL // 20000UL

#define MOTORdd  32
#define MOTORdi  33
#define MOTORtd  25
#define MOTORti  26

const int resolucion = 10;
const int max_valor  = 1023;

const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GX, GY, GZ;
volatile float AcX_filtr, AcY_filtr, AcZ_filtr, Tmp_filtr, GX_filtr, GY_filtr, GZ_filtr;
unsigned long LastMedIMU;
const unsigned long PeriodMedIMU = 2;
#define alpha 0.9

const unsigned long commTimeout = 1000;

float inclinacion;
const float ANGULO_MAX = 45.0f;
// ============================================================
//  TEST DE LATENCIA (MEDIDO DESDE SLAVE)
//  Slave envía periódicamente data_sensores.tmp = SENTINEL_TMP_VALUE
//  y guarda T1.
//  Alumno rebota la señal con un salto en PWM.
//  Cuando Slave recibe un Pulso_dd mayor a pwmBaseline + UMBRAL,
//  calcula T2 y muestra la latencia.

#define SENTINEL_TMP_VALUE   30000   // mismo valor que SENTINEL_TMP en Alumno
#define UMBRAL_PWM_CAMBIO    100     // cambio minimo en us para considerar escalon nuevo
#define LATENCY_STEP_MS      2000UL  // periodo del escalon en ms

int16_t pwmBaseline          = 0;
bool    baselineInicializado = false; // ultimo Pulso_dd recibido (referencia)
volatile bool          stepActivo         = false;
volatile unsigned long lastStepMs         = 0;
volatile unsigned long T1_latencia        = 0;
volatile bool          esperandoRespuesta = false;

uint32_t lat_min   = UINT32_MAX;
uint32_t lat_max   = 0;
uint32_t lat_suma  = 0;
uint32_t lat_count = 0;
// ============================================================

typedef struct __attribute__((packed))
{
  float          Dist_ultrasonicos;
  unsigned long  echoStartTime;
  unsigned long  echoEndTime;
  int16_t ax, ay, az, tmp, gx, gy, gz;
  bool           estado;
} SensorPacket;
SensorPacket data_sensores;

typedef struct __attribute__((packed))
{
  int16_t Period_dd, Pulso_dd;
  int16_t Period_di, Pulso_di;
  int16_t Period_td, Pulso_td;
  int16_t Period_ti, Pulso_ti;
  bool    estado;
} ControlPacket;
ControlPacket paqueteRecibido;

uint8_t master_mac[] = { 0xA8, 0x42, 0xE3, 0xCD, 0x69, 0x18 };
unsigned long lastCommTime = 0;
uint32_t Periodo_ant[]     = {100, 100, 100, 100};

volatile bool nuevoPaquete  = false;
volatile bool nuevaMedida  = false;

// Variables globales para almacenar la última lectura limpia antes de armar el paquete
volatile float         ultrasonico_distancia = 0;
volatile unsigned long ultrasonico_echoStart = 0;
volatile unsigned long ultrasonico_echoEndTime = 0;

// Añade estas dos líneas junto al resto de globales
SemaphoreHandle_t semRespuesta;

// NUEVO: Variable volatil para registrar el tiempo exacto de recepcion de red
volatile unsigned long T2_capturado = 0; 

// ============================================================
void ActualizarPWM(uint8_t pwmPin, uint32_t Periodo, uint32_t* Periodo_ant, uint32_t Pulso)
{
  if ((Periodo > 1000) & (Periodo < 100000) & (Pulso < Periodo))
  {
    if ((Periodo > *Periodo_ant + 100) | (Periodo < *Periodo_ant - 100))
    {
      uint32_t frec = 1E6 / Periodo;
      ledcDetach(pwmPin);
      ledcAttach(pwmPin, frec, resolucion);
      *Periodo_ant = Periodo;
    }
    uint32_t Ciclotrabajo = (max_valor * Pulso) / Periodo;
    ledcWrite(pwmPin, Ciclotrabajo);
  }
  else ledcWrite(pwmPin, 0);
}

void tareaRespuesta(void *pvParameters)
{
  for (;;) 
  {
    // La tarea se queda "dormida" aquí hasta que OnDataRecv avise
    if (xSemaphoreTake(semRespuesta, portMAX_DELAY) == pdTRUE) 
    {
      // ARMAR PAQUETE
      // Tomamos la información fresca que el loop() ha ido guardando
      data_sensores.Dist_ultrasonicos = ultrasonico_distancia;
      data_sensores.echoStartTime     = ultrasonico_echoStart;
      data_sensores.echoEndTime       = ultrasonico_echoEndTime;
      
      data_sensores.ax = AcX_filtr;
      data_sensores.ay = AcY_filtr;
      data_sensores.az = AcZ_filtr;
      data_sensores.gx = GX_filtr;
      data_sensores.gy = GY_filtr;
      data_sensores.gz = GZ_filtr;
      data_sensores.estado = true;

      // Control del Sentinel para el test de latencia
      if (stepActivo) 
      {
        if (esperandoRespuesta && T1_latencia == 0) T1_latencia = micros();
        data_sensores.tmp = SENTINEL_TMP_VALUE;
      } 
      else data_sensores.tmp = (int16_t)Tmp_filtr;

      // enviamos
      esp_now_send(master_mac, (uint8_t *)&data_sensores, sizeof(data_sensores));
    }
  }
}

// ============================================================
// OnDataRecv LIMPIO: Solo valida, copia datos y levanta bandera
// ============================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len)
{
  if (memcmp(info->src_addr, master_mac, 6) != 0) return;
  if (len != sizeof(ControlPacket)) return;

  T2_capturado = micros();
  memcpy(&paqueteRecibido, incomingData, sizeof(ControlPacket));
  lastCommTime = millis();
  nuevoPaquete = true; // paquete con nuevos PWM
  xSemaphoreGive(semRespuesta);  // Despertar a la Tarea FreeRTOS al instante
}

// ============================================================
void IRAM_ATTR Ultrasnd_PWM_int()
{
  unsigned long now = micros();
  switch (Estado_ultrsnd)
  {
    case ECHO_START:
      if (digitalRead(ECHO_PIN) == HIGH)
      {
        echoStartTime  = now - lastTriggerTime;
        Estado_ultrsnd = ECHO_END;
      }
      else if (now - lastTriggerTime > 1000) Estado_ultrsnd = INICIAL;
      break;

    case ECHO_END:
      Estado_ultrsnd = INICIAL;
      if (digitalRead(ECHO_PIN) == LOW)
      {
        nuevaMedida = true;
        if (now - lastTriggerTime < 3000) echoEndTime = now - lastTriggerTime - echoStartTime;
        else { echoStartTime = 100;
        echoEndTime = 2800; }
      }
      break;

    default: Estado_ultrsnd = INICIAL;
  }
}

// ============================================================
void setup()
{
  // Primero el semáforo, luego la tarea
  semRespuesta = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore
  (
    tareaRespuesta,   // función
    "resp",           // nombre
    4096,             // stack bytes
    NULL,             // parámetro
    10,               // prioridad (> Arduino loop que va a 1)
    NULL,             // handle (no lo necesitamos)
    1                 // Core 1 — mismo que Arduino loop, prioridad mayor
  );

  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  attachInterrupt(ECHO_PIN, Ultrasnd_PWM_int, CHANGE);

  pinMode(PRUEBA_PIN, OUTPUT);

  ledcAttachChannel(MOTORdd, 50, 10, 0); ledcWrite(MOTORdd, 0);
  ledcAttachChannel(MOTORdi, 50, 10, 1); ledcWrite(MOTORdi, 0);
  ledcAttachChannel(MOTORtd, 50, 10, 2); ledcWrite(MOTORtd, 0);
  ledcAttachChannel(MOTORti, 50, 10, 3); ledcWrite(MOTORti, 0);
  ledcWrite(MOTORdd, 0.05 * max_valor);
  ledcWrite(MOTORdi, 0.05 * max_valor);
  ledcWrite(MOTORtd, 0.05 * max_valor);
  ledcWrite(MOTORti, 0.05 * max_valor);
  delay(3000);

  esp_log_level_set("*", ESP_LOG_NONE);  // elimina ~1-2ms de jitter del driver WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  if (esp_now_init() != ESP_OK) { Serial.println("Error ESP-NOW"); return; }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, master_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(master_mac))
    if (esp_now_add_peer(&peerInfo) != ESP_OK) 
    { 
      Serial.println("Error peer"); return;
    }

  Serial.println("Slave listo. Esperando mensajes del master...");
  Serial.println(F("MODO: Midiendo tiempo desde SLAVE (envia tmp = 30000)"));

  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

// ============================================================
void loop()
{

  // ---- PROCESAMIENTO DE PAQUETE RECIBIDO (Movido desde OnDataRecv) ----
  if (nuevoPaquete)
  {
    nuevoPaquete = false; // Resetear la bandera inmediatamente

    // ----------------------------------------------------------
    // DETECCION DE RESPUESTA DE ALUMNO (SALTO PWM)
    // ----------------------------------------------------------
    if (!baselineInicializado && paqueteRecibido.Pulso_dd > 0)
    {
      pwmBaseline          = paqueteRecibido.Pulso_dd;
      baselineInicializado = true;
      Serial.printf("Baseline inicializado: %d us\n", pwmBaseline);
    }
    else if (baselineInicializado)
    {
      int16_t diff = paqueteRecibido.Pulso_dd - pwmBaseline;
      if (diff > UMBRAL_PWM_CAMBIO)
      {
        // Detectamos que el PWM subió porque el Alumno rebotó el tmp
        if (esperandoRespuesta)
        {
          unsigned long T2 = T2_capturado; // Usamos el tiempo exacto capturado en OnDataRecv
          uint32_t latencia_us = (uint32_t)(T2 - T1_latencia);
          esperandoRespuesta = false;

          if (latencia_us < lat_min) lat_min = latencia_us;
          if (latencia_us > lat_max) lat_max = latencia_us;
          lat_suma  += latencia_us;
          lat_count++;

          Serial.printf("Resultado #%lu\n", lat_count);
          Serial.printf("PWM: %d us (diff=+%d us)\n", 
                        (int)paqueteRecibido.Pulso_dd, (int)diff);
          Serial.printf("   Round-trip: %lu ms (%lu us)\n", latencia_us/1000, latencia_us);
          Serial.printf("   Min: %lu ms | Max: %lu ms | Media: %lu ms\n",
                        lat_min/1000, lat_max/1000, (lat_count > 0) ? (lat_suma / lat_count / 1000) : 0);
          Serial.println();
        }
        pwmBaseline = paqueteRecibido.Pulso_dd; // Actualizamos a la nueva altura
      }
      else if (diff < -UMBRAL_PWM_CAMBIO)
      {
        // El PWM ha vuelto a bajar al valor base
        pwmBaseline = paqueteRecibido.Pulso_dd;
      }
    }

    // Aplicar PWM a los ESC
    ActualizarPWM(MOTORdd, paqueteRecibido.Period_dd, &Periodo_ant[0], paqueteRecibido.Pulso_dd);
    ActualizarPWM(MOTORdi, paqueteRecibido.Period_di, &Periodo_ant[1], paqueteRecibido.Pulso_di);
    ActualizarPWM(MOTORtd, paqueteRecibido.Period_td, &Periodo_ant[2], paqueteRecibido.Pulso_td);
    ActualizarPWM(MOTORti, paqueteRecibido.Period_ti, &Periodo_ant[3], paqueteRecibido.Pulso_ti);
  }

  // ---- Sensor ultrasonico ----
  if ((Estado_ultrsnd == INICIAL) && (micros() - lastTriggerTime >= PeriodMedUltrsn))
  {
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(TRIG_PIN, LOW);
    lastTriggerTime = micros();
    Estado_ultrsnd  = ECHO_START;
  }

  if (nuevaMedida)
  {
    nuevaMedida = false;
    // El loop "añade la info" a las variables seguras
    ultrasonico_echoStart   = echoStartTime;
    ultrasonico_echoEndTime = echoEndTime;
    ultrasonico_distancia   = echoEndTime * 0.0343f / 2.0f;
  }

  // ---- IMU ----
  if (millis() - LastMedIMU > PeriodMedIMU)
  {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    Tmp = Wire.read() << 8 | Wire.read();
    GX  = Wire.read() << 8 | Wire.read();
    GY  = Wire.read() << 8 | Wire.read();
    GZ  = Wire.read() << 8 | Wire.read();

    AcX_filtr = AcX_filtr * alpha + AcX * (1 - alpha);
    AcY_filtr = AcY_filtr * alpha + AcY * (1 - alpha);
    AcZ_filtr = AcZ_filtr * alpha + AcZ * (1 - alpha);
    Tmp_filtr = Tmp_filtr * alpha + Tmp * (1 - alpha);
    GX_filtr  = GX_filtr  * alpha + GX  * (1 - alpha);
    GY_filtr  = GY_filtr  * alpha + GY  * (1 - alpha);
    GZ_filtr  = GZ_filtr  * alpha + GZ  * (1 - alpha);
    inclinacion = (atan2f(AcY_filtr, -AcZ_filtr)) * 180.0f / PI;
    LastMedIMU  = millis();
  }

  // ---- GENERADOR DE ESCALON (SLAVE MIDE TIEMPO) ----
  unsigned long ahoraMs = millis();
  // --- TIEMPO DE CORTESÍA (EJ: 8000 ms) ---
  if (ahoraMs < 8000) 
  {
    lastStepMs = ahoraMs;
  }
  else
  {
    // Pasados los 8 segundos, arranca el test normalmente
    if (ahoraMs - lastStepMs >= LATENCY_STEP_MS)
    {
      lastStepMs = ahoraMs;
      stepActivo = !stepActivo;

      if (stepActivo)
      {
        T1_latencia = 0;
        esperandoRespuesta = true;

        Serial.println(" >>> ESCALON ENVIADO");
      }
    }

    // Timeout: si en 1 segundo no llega respuesta PWM, resetear
    if (esperandoRespuesta && T1_latencia > 0 && (micros() - T1_latencia > 1000000UL)) // guard para T1 no inicializado
    {
      esperandoRespuesta = false;
      Serial.println("TIMEOUT - PWM no cambio en 1s");
    }
  }

  // ---- Timeout comunicaciones ----
  if (millis() - lastCommTime > commTimeout)
  {
    ledcWrite(MOTORdd, 0.045 * max_valor);
    ledcWrite(MOTORdi, 0.045 * max_valor);
    ledcWrite(MOTORtd, 0.045 * max_valor);
    ledcWrite(MOTORti, 0.045 * max_valor);
    // Rate-limit: solo imprime una vez por 1000ms
    static unsigned long lastWarnMs = 0;
    if (millis() - lastWarnMs >= 1000) 
    {
      Serial.println("Sin comunicaciones");
      lastWarnMs = millis();
    }
  }
}
