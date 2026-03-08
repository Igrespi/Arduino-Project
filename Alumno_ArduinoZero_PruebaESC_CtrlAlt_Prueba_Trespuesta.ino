#include <Servo.h>
#include<Wire.h>

//**********Definiciones control***************
#define Periodo_Ctrl 20000  //Periodo en microsegundos
#define Periodo_Actlz  10000
unsigned long LastCtrl;
unsigned long LastActlz;
float Ref_incl = 0; //Magnitud comparada con ay
float Ref_altura = 35.0; //Altura en cm
float Consgn_motor_ascnd = 0;   //()
float Consgn_motor_incl = 0;   //()
float Integral_asc = 0;
float Integral_incl = 0;
float Error_alt_ant;
float Error_incl_ant;

#define Ref_dd 0
#define Ref_di 1
#define Ref_td 2
#define Ref_ti 3
#define KI_asc 0.5
#define KP_asc 6
#define KD_asc 3
#define KI_incl 0.6 //0.2
#define KP_incl 5
#define KD_incl 500000.

#define Max_Consg_motor 70
#define Min_Consg_motor 20

float throttleTargetPct[] = {0,0,0,0};
float throttleCurrentPct[] = {0,0,0,0};
unsigned long lastUpdateMs[] = {0,0,0,0};

//***********Definiciones ultrasonidos**************
#define TRIG_PIN 10
#define ECHO_PIN 11
#define MAX_DISTANCE 100 // en cm
#define PeriodMedUltsnd 15000 //Periodo en microsegundos
unsigned long lastTriggerTime = 0;
unsigned long echoStartTime = 0;
unsigned long echoEndTime = 0;
float Dist_ultrasonicos;

enum EstadosUltrsnd 
{
  INICIAL,
  PULSO_TRIGGER,
  ECHO_START,
  ECHO_END
};
EstadosUltrsnd Estado_ultrsnd = INICIAL;
volatile unsigned int distance = 0; // variable actualizada en la interrupción

//Interrupción del sensor de ultrasonidos
void  Ultrasnd_PWM_int() 
{
  unsigned long now = micros();
  
  switch (Estado_ultrsnd)
  {
    case ECHO_START:  // Paso 2: Esperar a que ECHO pase a HIGH
      if (digitalRead(ECHO_PIN) == HIGH) 
      {
        echoStartTime = now - lastTriggerTime;
        Estado_ultrsnd = ECHO_END; //Espera finalizado pulso en ECHO
      }
      else
      if (now - lastTriggerTime > 1000)   //No Detecta pulso ECHO. Toma medida anterior
       {
        Estado_ultrsnd = INICIAL;
       }
       break;

    case ECHO_END: // Paso 3: Esperar a que ECHO pase a LOW
      Estado_ultrsnd = INICIAL;
      if (digitalRead(ECHO_PIN) == LOW) 
      {  
        if (now - lastTriggerTime < 3000)  //Máxima distancia 1m
          echoEndTime = now - lastTriggerTime - echoStartTime;
        else
        {
         echoStartTime = 100;
         echoEndTime = 2800;
        } 
        // Paso 4: Calcular duración y distancia
        Dist_ultrasonicos = echoEndTime * 0.0343 / 2.0;
        echoStartTime = echoStartTime;
        echoEndTime = echoEndTime;
      }
      break;
    default: Estado_ultrsnd = INICIAL;
  }
}


//*************Definiciones control ESC*****************
Servo esc_dd;
Servo esc_di;
Servo esc_td;
Servo esc_ti;

// ==== AJUSTA A TU ESC ====
const int PIN_ESCdd = 6;  // pin de señal hacia el ESC
const int PIN_ESCdi = 5;
const int PIN_ESCtd = 4;
const int PIN_ESCti = 3;


const int US_MIN = 1000;    // mínimo (idle/parado). Muchos ESC: 1000 µs
const int US_MAX = 2000;    // máximo (full throttle). Muchos ESC: 2000 µs
const int US_ARM = US_MIN;  // pulso para armar (normalmente el mínimo)
const int STARTUP_MS = 6000;   // tiempo de espera inicial para que el ESC “pite” y se inicie
// ==========================

// limitador de rampa (porcentaje por segundo)
const float SLEW_PCT_PER_S = 2000.0;  


// Mapea 0..100% -> microsegundos (US_MIN..US_MAX)
int pctToMicros(float pct) 
{
  pct = constrain(pct, 0.0f, 100.0f);
  return (int)(US_MIN + (US_MAX - US_MIN) * (pct / 100.0f));
}


// Cambia objetivo de gas en %
void setThrottlePercent(float pct, unsigned int Motor) 
{
  throttleTargetPct[Motor] = constrain(pct, 0.0f, 100.0f);
  lastUpdateMs[Motor] = millis();
}


// Aplica rampa y escribe al ESC
void updateThrottle(Servo esc, unsigned int Motor) 
{
  unsigned long now = millis();
  float dt = (now - lastUpdateMs[Motor]) / 1000.0f;
  lastUpdateMs[Motor] = now;

  // calcular cambio máximo permitido por rampa
  float maxStep = SLEW_PCT_PER_S * dt;

  if (throttleCurrentPct[Motor] < throttleTargetPct[Motor])
    throttleCurrentPct[Motor] = min(throttleCurrentPct[Motor] + maxStep, throttleTargetPct[Motor]);
  else
    throttleCurrentPct[Motor] = max(throttleCurrentPct[Motor] - maxStep, throttleTargetPct[Motor]);

  esc.writeMicroseconds(pctToMicros(throttleCurrentPct[Motor]));
}



//*************Definiciones IMU****************************
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
unsigned long LastMedIMU;
#define PeriodMedIMU  15000  //microsegundos entre medidas de la IMU

typedef struct __attribute__((packed)) 
{
  //IMU ACELERÓMETRO
  int16_t ax;
  int16_t ay;
  int16_t az;
  //IMU temperatura:
  int16_t tmp;
  //IMU GIROSCOPIO
  int16_t gx;
  int16_t gy;
  int16_t gz;
} Sensor_IMU;

Sensor_IMU IMU;




//**********************************************************
void setup() 
{
  //Configuración puerto serie
  Serial.begin(115200);

  //**************Configuración control ESC******************  
  pinMode(PIN_ESCdd, OUTPUT);
  digitalWrite(PIN_ESCdd, HIGH);

  pinMode(PIN_ESCdi, OUTPUT);
  digitalWrite(PIN_ESCdi, HIGH);

  pinMode(PIN_ESCtd, OUTPUT);
  digitalWrite(PIN_ESCtd, HIGH);

  pinMode(PIN_ESCti, OUTPUT);
  digitalWrite(PIN_ESCti, HIGH);

  esc_dd.attach(PIN_ESCdd, US_MIN, US_MAX);
  esc_di.attach(PIN_ESCdi, US_MIN, US_MAX);
  esc_td.attach(PIN_ESCtd, US_MIN, US_MAX);
  esc_ti.attach(PIN_ESCti, US_MIN, US_MAX);
  

  // Armado seguro
  setThrottlePercent(0, Ref_dd);           // objetivo 60%
  setThrottlePercent(0, Ref_di); 
  setThrottlePercent(0, Ref_td); 
  setThrottlePercent(0, Ref_ti);
  
  updateThrottle(esc_dd, Ref_dd);
  updateThrottle(esc_di, Ref_di);
  updateThrottle(esc_td, Ref_td);
  updateThrottle(esc_ti, Ref_ti);

  lastUpdateMs[Ref_dd] = millis();
  lastUpdateMs[Ref_di] = lastUpdateMs[Ref_dd];
  lastUpdateMs[Ref_td] = lastUpdateMs[Ref_dd];
  lastUpdateMs[Ref_ti] = lastUpdateMs[Ref_dd];
 
  //***************Configuración Sensor ultrasonico*******************
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);  // Asegura que TRIG comience en LOW
  attachInterrupt(ECHO_PIN, Ultrasnd_PWM_int, CHANGE);

  //*********Configurar I2C para IMU**************
  Wire.begin();
  Wire.setClock(400000); 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true); 
  
  delay(STARTUP_MS);
}

void loop() 
{

//****************MEDIDA DE LA IMU****************************

  if (micros() - LastMedIMU > PeriodMedIMU) 
  {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  IMU.ax = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  IMU.ay = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  IMU.az = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  IMU.tmp = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  IMU.gx = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  IMU.gy = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  IMU.gz = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  LastMedIMU = micros();

/*
Serial.print("Tiempo = "); 
Serial.print(millis());

Serial.print("s  Ay = "); 
Serial.println(IMU.ay);
*/
/*
  Serial.print(" | AcX = "); Serial.print(IMU.ax);
  Serial.print(" | AcY = "); Serial.print(IMU.ay);
  Serial.print(" | AcZ = "); Serial.print(IMU.az);
  Serial.print(" | Tmp = "); Serial.print(IMU.tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(IMU.gx);
  Serial.print(" | GyY = "); Serial.print(IMU.gy);
  Serial.print(" | GyZ = "); Serial.print(IMU.gz);
  Serial.println();
 */
  
  } 

//****************SENSOR ULTRASONICO****************************
  // Paso 1: Enviar pulso TRIG cada 20 ms
  if ((Estado_ultrsnd == INICIAL) && micros() - lastTriggerTime >= PeriodMedUltsnd) 
  {
//    Serial.print("Altura: ");
//    Serial.println(Dist_ultrasonicos, 2); // 2 decimales
    // Enviar pulso de 20 us   
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);  //Pulso de 20us
    digitalWrite(TRIG_PIN, LOW);
    lastTriggerTime = micros();
    Estado_ultrsnd = ECHO_START;      //Espera señal ECHO  
  }

//*******************Control************************
  if (micros() - LastCtrl > Periodo_Ctrl) 
  {
  // Calculo de errores
  float Error_alt;
  float Error_incl;
  float Error_incl_derv;
  float inclinacion;
  LastCtrl = micros();
  inclinacion = atan2f(IMU.ay, -IMU.az);
  Error_alt = Ref_altura - Dist_ultrasonicos*cosf(inclinacion);
  Error_incl = sin(Ref_incl - inclinacion);

  // Lazo de control Proporcional Integral Derivativo
  Integral_asc += Error_alt*Periodo_Ctrl*KI_asc/1000000.;
  Integral_asc= constrain(Integral_asc,Min_Consg_motor,Max_Consg_motor);

  Integral_incl += Error_incl*Periodo_Ctrl*KI_incl/1000000.;
  Integral_incl = constrain(Integral_incl,-4,4);


  Consgn_motor_ascnd = Error_alt*KP_asc+Integral_asc+(Error_alt-Error_alt_ant)*KD_asc/Periodo_Ctrl;  
  Consgn_motor_ascnd = constrain(Consgn_motor_ascnd,Min_Consg_motor,Max_Consg_motor);

  Error_incl_derv = (Error_incl-Error_incl_ant)*KD_incl/Periodo_Ctrl;
  Consgn_motor_incl = Error_incl*KP_incl+Integral_incl+Error_incl_derv;  
  Consgn_motor_incl = constrain(Consgn_motor_incl,-5,5);

  //Consgn_motor_incl=0;
  //Consgn_motor_ascnd=20;

  Error_alt_ant = Error_alt;
  Error_incl_ant = Error_incl;
  //Establece consigna de motores
  Consgn_motor_ascnd = Min_Consg_motor;   //Descarga motor con mínima potencia
  Consgn_motor_incl = 0;
    
  setThrottlePercent(Consgn_motor_ascnd + Consgn_motor_incl, Ref_dd);           
  setThrottlePercent(Consgn_motor_ascnd - Consgn_motor_incl, Ref_di); 
  setThrottlePercent(Consgn_motor_ascnd + Consgn_motor_incl, Ref_td); 
  setThrottlePercent(Consgn_motor_ascnd - Consgn_motor_incl, Ref_ti); 
  
  
  
  Serial.print("Tiempo = "); 
  Serial.print(millis());

  Serial.print("s  Distancia = "); 
  Serial.print(Dist_ultrasonicos);
  
  Serial.print("cm  Integral = "); 
  Serial.print(Integral_asc);

  Serial.print(" Consigna_asc = "); 
  Serial.print(Consgn_motor_ascnd);
  
  Serial.print(" Motor_dd = "); 
  Serial.println(throttleCurrentPct[Ref_dd]);

/* 
Serial.print("Tiempo = "); 
Serial.print(millis());

Serial.print("ms  Ay = "); 
Serial.print(IMU.ay);

Serial.print("  Az = "); 
Serial.print(IMU.az);


Serial.print("  Error_incl = "); 
Serial.print(Error_incl);

Serial.print("  Integral = "); 
Serial.print(Integral_incl);

Serial.print("  Derivativo = "); 
Serial.print(Error_incl_derv);


Serial.print(" Consigna_incl = "); 
 Serial.print(Consgn_motor_incl);

Serial.print(" Trhole_dd = "); 
 Serial.print(throttleCurrentPct[0]);

 Serial.print(" Trhole_di = "); 
 Serial.println(throttleCurrentPct[1]);

*/  
  }
  

//Actualiza estado motores
  if (micros() - LastActlz > Periodo_Actlz) 
  {
  LastActlz = micros();
  updateThrottle(esc_dd, Ref_dd);
  updateThrottle(esc_di, Ref_di);
  updateThrottle(esc_td, Ref_td);
  updateThrottle(esc_ti, Ref_ti);
  }

 

}
