#include <Servo.h>
#include <Wire.h>

// ============================================================
//  SISTEMA DE ESTADÍSTICAS
// ============================================================
#define STATS_INTERVAL_MSA 4000
#define STATS_INTERVAL_MSB 2000
#define STATS_GUARD_MS 200

uint32_t st_loopMaxUs   = 0;
uint32_t st_loopMinUs   = UINT32_MAX;

uint32_t st_imuPeriodMaxUs  = 0;
uint32_t st_imuPeriodMinUs  = UINT32_MAX;
unsigned long st_imuLastUs  = 0;
uint32_t st_imuWorkMaxUs  = 0;
uint32_t st_imuWorkMinUs  = UINT32_MAX;

uint32_t st_ultraPeriodMaxUs = 0;
uint32_t st_ultraPeriodMinUs = UINT32_MAX;
unsigned long st_ultraLastUs = 0;
uint32_t st_ultraWorkMaxUs  = 0;
uint32_t st_ultraWorkMinUs  = UINT32_MAX;

uint32_t st_ctrlPeriodMaxUs  = 0;
uint32_t st_ctrlPeriodMinUs  = UINT32_MAX;
unsigned long st_ctrlLastUs  = 0;
uint32_t st_ctrlWorkMaxUs  = 0;
uint32_t st_ctrlWorkMinUs  = UINT32_MAX;

uint32_t st_motorPeriodMaxUs = 0;
uint32_t st_motorPeriodMinUs = UINT32_MAX;
unsigned long st_motorLastUs = 0;
uint32_t st_motorWorkMaxUs  = 0;
uint32_t st_motorWorkMinUs  = UINT32_MAX;

unsigned long statsLastPrintMsA = 0;
unsigned long statsLastPrintMsB = 0;

void resetStatsA()
{
  st_imuPeriodMaxUs = st_ultraPeriodMaxUs = st_ctrlPeriodMaxUs = st_motorPeriodMaxUs = 0;
  st_imuPeriodMinUs = st_ultraPeriodMinUs = st_ctrlPeriodMinUs = st_motorPeriodMinUs = UINT32_MAX;
}

void resetStatsB()
{
  st_loopMaxUs  = st_imuWorkMaxUs = st_ultraWorkMaxUs = st_ctrlWorkMaxUs = st_motorWorkMaxUs = 0;
  st_loopMinUs  = st_imuWorkMinUs = st_ultraWorkMinUs = st_ctrlWorkMinUs = st_motorWorkMinUs = UINT32_MAX;
}

void printStatsA()
{
  Serial.print(F("A| IMU: "));      Serial.print(st_imuPeriodMinUs);
  Serial.print(F(" / ")); Serial.print(st_imuPeriodMaxUs);   Serial.print(F(" us | "));
  Serial.print(F("Ultra: "));       Serial.print(st_ultraPeriodMinUs); Serial.print(F(" / ")); Serial.print(st_ultraPeriodMaxUs); Serial.print(F(" us | "));
  Serial.print(F("PID: "));         Serial.print(st_ctrlPeriodMinUs);  Serial.print(F(" / ")); Serial.print(st_ctrlPeriodMaxUs);  Serial.print(F(" us | "));
  Serial.print(F("Motor: "));       Serial.print(st_motorPeriodMinUs); Serial.print(F(" / ")); Serial.print(st_motorPeriodMaxUs); Serial.println(F(" us"));
}

void printStatsB()
{
  Serial.print(F("B| Loop: "));     Serial.print(st_loopMinUs);    Serial.print(F(" / ")); Serial.print(st_loopMaxUs);    Serial.print(F(" us | "));
  Serial.print(F("IMU: "));         Serial.print(st_imuWorkMinUs);
  Serial.print(F(" / ")); Serial.print(st_imuWorkMaxUs); Serial.print(F(" us | "));
  Serial.print(F("Ultra: "));       Serial.print(st_ultraWorkMinUs); Serial.print(F(" / ")); Serial.print(st_ultraWorkMaxUs); Serial.print(F(" us | "));
  Serial.print(F("PID: "));         Serial.print(st_ctrlWorkMinUs); Serial.print(F(" / ")); Serial.print(st_ctrlWorkMaxUs); Serial.print(F(" us | "));
  Serial.print(F("Motor: "));       Serial.print(st_motorWorkMinUs); Serial.print(F(" / ")); Serial.print(st_motorWorkMaxUs); Serial.println(F(" us"));
}

// ============================================================
//  TEST DE LATENCIA
//  Funcionamiento:
//    - Slave envia IMU.tmp = SENTINEL_TMP periodicamente
//    - Alumno lee ese valor y cambia su PWM al escalon alto
//    - Slave detecta el cambio de PWM y calcula la latencia
// ============================================================
#define SENTINEL_TMP        30000    // valor de tmp imposible en condiciones normales
#define PWM_BASE_PCT        25.0f    // PWM base durante el test (%)
#define PWM_STEP_PCT        50.0f    // PWM del escalon alto (%)

bool modoTestLatencia = true; // true = test activo, false = PID normal

// ============================================================
#define OSC_PIN 9

#define Periodo_Ctrl   10000 // 20000
#define Periodo_Actlz  5000 // 10000
unsigned long LastCtrl  = 0;
unsigned long LastActlz = 0;

float Ref_incl           = 0;
float Ref_altura         = 20.0f;
float Consgn_motor_ascnd = 0;
float Consgn_motor_incl  = 0;
float Integral_asc       = 0;
float Integral_incl      = 0;
float Error_alt_ant      = 0;
float Error_incl_ant     = 0;

#define Ref_dd 0
#define Ref_di 1
#define Ref_td 2
#define Ref_ti 3

#define KI_asc   0.5f
#define KP_asc   6.0f
#define KD_asc   3.0f
#define KI_incl  0.6f
#define KP_incl  5.0f
#define KD_incl  500000.0f

#define Max_Consg_motor  70
#define Min_Consg_motor  20

float throttleTargetPct[]  = {0, 0, 0, 0};
float throttleCurrentPct[] = {0, 0, 0, 0};
unsigned long lastUpdateMs[] = {0, 0, 0, 0};

#define TRIG_PIN        10
#define ECHO_PIN        11
#define PeriodMedUltsnd 15000 // 20000
unsigned long lastTriggerTime = 0;
unsigned long echoStartTime   = 0;
unsigned long echoEndTime     = 0;
volatile bool nuevaMedidaUltrasonido = false;
float Dist_ultrasonicos       = 0;

enum EstadosUltrsnd { INICIAL, PULSO_TRIGGER, ECHO_START, ECHO_END };
EstadosUltrsnd Estado_ultrsnd = INICIAL;

void Ultrasnd_PWM_int()
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
        if (now - lastTriggerTime < 3000) echoEndTime = now - lastTriggerTime - echoStartTime;
        else { echoStartTime = 100; echoEndTime = 2800; }
        nuevaMedidaUltrasonido = true; // Añade una bandera booleana (volatile bool)
      }
      break;

    default: Estado_ultrsnd = INICIAL;
  }
}

Servo esc_dd, esc_di, esc_td, esc_ti;
const int PIN_ESCdd  = 6;
const int PIN_ESCdi  = 5;
const int PIN_ESCtd  = 4;
const int PIN_ESCti  = 3;

const int US_MIN     = 1000;
const int US_MAX     = 2000;
const int STARTUP_MS = 6000;
const float SLEW_PCT_PER_S = 2000.0f;

int pctToMicros(float pct)
{
  pct = constrain(pct, 0.0f, 100.0f);
  return (int)(US_MIN + (US_MAX - US_MIN) * (pct / 100.0f));
}

void setThrottlePercent(float pct, unsigned int Motor)
{
  throttleTargetPct[Motor] = constrain(pct, 0.0f, 100.0f);
  lastUpdateMs[Motor]      = millis();
}

void updateThrottle(Servo esc, unsigned int Motor)
{
  unsigned long now = millis();
  float dt = (now - lastUpdateMs[Motor]) / 1000.0f;
  lastUpdateMs[Motor] = now;
  float maxStep = SLEW_PCT_PER_S * dt;

  if (throttleCurrentPct[Motor] < throttleTargetPct[Motor])
    throttleCurrentPct[Motor] = min(throttleCurrentPct[Motor] + maxStep, throttleTargetPct[Motor]);
  else
    throttleCurrentPct[Motor] = max(throttleCurrentPct[Motor] - maxStep, throttleTargetPct[Motor]);

  esc.writeMicroseconds(pctToMicros(throttleCurrentPct[Motor]));
}

const int MPU_addr   = 0x68;
unsigned long LastMedIMU = 0;
#define PeriodMedIMU 2000 // 7500

typedef struct __attribute__((packed)) { int16_t ax, ay, az, tmp, gx, gy, gz;
} Sensor_IMU;
Sensor_IMU IMU;

// ============================================================
void setup()
{
  Serial.begin(115200);

  pinMode(OSC_PIN, OUTPUT);
  digitalWrite(OSC_PIN, LOW);

  pinMode(PIN_ESCdd, OUTPUT); digitalWrite(PIN_ESCdd, HIGH);
  pinMode(PIN_ESCdi, OUTPUT); digitalWrite(PIN_ESCdi, HIGH);
  pinMode(PIN_ESCtd, OUTPUT); digitalWrite(PIN_ESCtd, HIGH);
  pinMode(PIN_ESCti, OUTPUT); digitalWrite(PIN_ESCti, HIGH);

  esc_dd.attach(PIN_ESCdd, US_MIN, US_MAX);
  esc_di.attach(PIN_ESCdi, US_MIN, US_MAX);
  esc_td.attach(PIN_ESCtd, US_MIN, US_MAX);
  esc_ti.attach(PIN_ESCti, US_MIN, US_MAX);

  setThrottlePercent(0, Ref_dd); setThrottlePercent(0, Ref_di);
  setThrottlePercent(0, Ref_td); setThrottlePercent(0, Ref_ti);
  updateThrottle(esc_dd, Ref_dd); updateThrottle(esc_di, Ref_di);
  updateThrottle(esc_td, Ref_td); updateThrottle(esc_ti, Ref_ti);
  lastUpdateMs[Ref_dd] = lastUpdateMs[Ref_di] = lastUpdateMs[Ref_td] = lastUpdateMs[Ref_ti] = millis();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  attachInterrupt(ECHO_PIN, Ultrasnd_PWM_int, CHANGE);

  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(STARTUP_MS);

  unsigned long ahora = micros();
  st_imuLastUs = st_ultraLastUs = st_ctrlLastUs = st_motorLastUs = ahora;

  resetStatsA();
  resetStatsB();

  unsigned long ahoraMs = millis();
  statsLastPrintMsA = ahoraMs;
  statsLastPrintMsB = ahoraMs - (STATS_INTERVAL_MSB / 2);

  Serial.println(F("=== TEST DE LATENCIA ACTIVO ==="));
  Serial.println(F("    Modo: SLAVE MIDE EL TIEMPO"));
  Serial.print(F("    PWM base: ")); Serial.print(PWM_BASE_PCT); Serial.println(F(" %"));
  Serial.print(F("    PWM escalon: ")); Serial.print(PWM_STEP_PCT); Serial.println(F(" %"));
  Serial.println(F("    Sentinel: IMU.tmp = 30000"));
  Serial.println(F("================================"));
}

// ============================================================
void loop()
{
  uint32_t loopStart = micros();

  // ---- 1. IMU ----
  if (micros() - LastMedIMU > PeriodMedIMU)
  {
    digitalWrite(OSC_PIN, !digitalRead(OSC_PIN));
    //uint32_t t0 = micros();
    //uint32_t periodo = t0 - (uint32_t)st_imuLastUs;
    //if (periodo > st_imuPeriodMaxUs) st_imuPeriodMaxUs = periodo;
    //if (periodo < st_imuPeriodMinUs) st_imuPeriodMinUs = periodo;
    //st_imuLastUs = t0;
    //uint32_t tWork = micros();

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    // Solo actualizamos si nos devuelven los 14 bytes correctos
    if (Wire.requestFrom(MPU_addr, 14, true) == 14)
    {
      IMU.ax  = Wire.read() << 8 | Wire.read();
      IMU.ay  = Wire.read() << 8 | Wire.read();
      IMU.az  = Wire.read() << 8 | Wire.read();
      IMU.tmp = Wire.read() << 8 | Wire.read();
      IMU.gx  = Wire.read() << 8 | Wire.read();
      IMU.gy  = Wire.read() << 8 | Wire.read();
      IMU.gz  = Wire.read() << 8 | Wire.read();
    }

    LastMedIMU = micros();

    //uint32_t work = LastMedIMU - tWork;
    //if (work > st_imuWorkMaxUs) st_imuWorkMaxUs = work;
    //if (work < st_imuWorkMinUs) st_imuWorkMinUs = work;

    // Mostrar tmp siempre para ver la transicion
    static int16_t tmpUltimo = 0;
    if (IMU.tmp != tmpUltimo) 
    {
      tmpUltimo = IMU.tmp;
      if (IMU.tmp != (int16_t)0xFFFF) 
      {
        Serial.print(F("tmp: "));
        Serial.println(IMU.tmp);
      }
    }
  }

  // ---- 2. Ultrasonidos ----
  if ((Estado_ultrsnd == INICIAL) && (micros() - lastTriggerTime >= PeriodMedUltsnd))
  {
    //uint32_t t0 = micros();
    //uint32_t periodo = t0 - (uint32_t)st_ultraLastUs;
    //if (periodo > st_ultraPeriodMaxUs) st_ultraPeriodMaxUs = periodo;
    //if (periodo < st_ultraPeriodMinUs) st_ultraPeriodMinUs = periodo;
    //st_ultraLastUs = t0;
    //uint32_t tWork = micros();

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(TRIG_PIN, LOW);
    lastTriggerTime = micros();
    Estado_ultrsnd  = ECHO_START;

    //uint32_t work = micros() - tWork;
    //if (work > st_ultraWorkMaxUs) st_ultraWorkMaxUs = work;
    //if (work < st_ultraWorkMinUs) st_ultraWorkMinUs = work;
  }

  if (nuevaMedidaUltrasonido)
  {
    nuevaMedidaUltrasonido = false;
    Dist_ultrasonicos = echoEndTime * 0.0343f / 2.0f; 
  }

  // ---- 3. REBOTE DE ESCALON ----
  if (modoTestLatencia)
  {
    float pwmTest;
    // Si el Slave inyectó el 30000, subimos el PWM. Si no, lo mantenemos base.
    if (IMU.tmp == SENTINEL_TMP) pwmTest = PWM_STEP_PCT;
    else pwmTest = PWM_BASE_PCT;
    
    // Forzamos el cambio para no depender del PID
    for (int i = 0; i < 4; i++)
    {
      throttleCurrentPct[i] = pwmTest;
      throttleTargetPct[i]  = pwmTest;
      lastUpdateMs[i]       = millis();
    }
  }

  // ---- 4. PID ----
  if (micros() - LastCtrl > Periodo_Ctrl && !modoTestLatencia)
  {
    //uint32_t t0 = micros();
    //uint32_t periodo = t0 - (uint32_t)st_ctrlLastUs;
    //if (periodo > st_ctrlPeriodMaxUs) st_ctrlPeriodMaxUs = periodo;
    //if (periodo < st_ctrlPeriodMinUs) st_ctrlPeriodMinUs = periodo;
    //st_ctrlLastUs = t0;
    //uint32_t tWork = micros();

    float inclinacion = atan2f(IMU.ay, -IMU.az);
    float Error_alt   = Ref_altura - Dist_ultrasonicos * cosf(inclinacion);
    float Error_incl  = sinf(Ref_incl - inclinacion);
    float Error_incl_derv;

    Integral_asc  += Error_alt  * Periodo_Ctrl * KI_asc  / 1000000.0f;
    Integral_asc   = constrain(Integral_asc, (float)Min_Consg_motor, (float)Max_Consg_motor);
    Integral_incl += Error_incl * Periodo_Ctrl * KI_incl / 1000000.0f;
    Integral_incl  = constrain(Integral_incl, -4.0f, 4.0f);

    Consgn_motor_ascnd = Error_alt * KP_asc + Integral_asc + (Error_alt - Error_alt_ant) * KD_asc / Periodo_Ctrl;
    Consgn_motor_ascnd = constrain(Consgn_motor_ascnd, (float)Min_Consg_motor, (float)Max_Consg_motor);

    Error_incl_derv    = (Error_incl - Error_incl_ant) * KD_incl / Periodo_Ctrl;
    Consgn_motor_incl  = Error_incl * KP_incl + Integral_incl + Error_incl_derv;
    Consgn_motor_incl  = constrain(Consgn_motor_incl, -5.0f, 5.0f);

    Error_alt_ant  = Error_alt;
    Error_incl_ant = Error_incl;

    setThrottlePercent(Consgn_motor_ascnd + Consgn_motor_incl, Ref_dd);
    setThrottlePercent(Consgn_motor_ascnd - Consgn_motor_incl, Ref_di);
    setThrottlePercent(Consgn_motor_ascnd + Consgn_motor_incl, Ref_td);
    setThrottlePercent(Consgn_motor_ascnd - Consgn_motor_incl, Ref_ti);
    
    LastCtrl = micros();
    //uint32_t work = LastCtrl - tWork;
    //if (work > st_ctrlWorkMaxUs) st_ctrlWorkMaxUs = work;
    //if (work < st_ctrlWorkMinUs) st_ctrlWorkMinUs = work;
  }

  // ---- 5. Motores ----
  if (micros() - LastActlz > Periodo_Actlz)
  {
    //uint32_t t0 = micros();
    //uint32_t periodo = t0 - (uint32_t)st_motorLastUs;
    //if (periodo > st_motorPeriodMaxUs) st_motorPeriodMaxUs = periodo;
    //if (periodo < st_motorPeriodMinUs) st_motorPeriodMinUs = periodo;
    //st_motorLastUs = t0;
    //uint32_t tWork = micros();

    updateThrottle(esc_dd, Ref_dd);
    updateThrottle(esc_di, Ref_di);
    updateThrottle(esc_td, Ref_td);
    updateThrottle(esc_ti, Ref_ti);
    LastActlz = micros();

    //uint32_t work = LastActlz - tWork;
    //if (work > st_motorWorkMaxUs) st_motorWorkMaxUs = work;
    //if (work < st_motorWorkMinUs) st_motorWorkMinUs = work;
  }

  /*
  // ---- 6. Duración de iteración ----
  uint32_t loopDur = micros() - loopStart;
  if (loopDur > st_loopMaxUs) st_loopMaxUs = loopDur;
  if (loopDur < st_loopMinUs) st_loopMinUs = loopDur;

  // ---- 7. Estadísticas por Serie ----
  unsigned long ahoraMs = millis();
  if (ahoraMs - statsLastPrintMsA >= STATS_INTERVAL_MSA)
  {
    if (abs((long)(ahoraMs - statsLastPrintMsB)) > STATS_GUARD_MS)
    {
      printStatsA();
      resetStatsA();
      statsLastPrintMsA = ahoraMs;
    }
  }

  if (ahoraMs - statsLastPrintMsB >= STATS_INTERVAL_MSB)
  {
    if (abs((long)(ahoraMs - statsLastPrintMsA)) > STATS_GUARD_MS)
    {
      printStatsB();
      resetStatsB();
      statsLastPrintMsB = ahoraMs;
    }
  } 
  */
}
