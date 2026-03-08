//*****************************************************************************
//******************************SLAVE******************************************
//**********************ESP32 EN LA BASE***************************************
//***************************************************************************** 

#include <WiFi.h>
#include <esp_now.h>
#include<Wire.h>
//****Prueba tiempo respuesta*********
unsigned long lastPulsoPrueba=0;
int16_t AcXprue,AcYprue,AcZprue;
#define PeriodoPrueba  1000000

//**********Sensor ultrasonico*********
// Definición de pines
#define TRIG_PIN 5
#define ECHO_PIN 18
// Pines de prueba
#define PRUEBA_PIN 17
// Variables de control de tiempo y estado para el sensor ultrasónico
unsigned long lastTriggerTime = 0;
unsigned long echoStartTime = 0;
unsigned long echoEndTime = 0;

enum EstadosUltrsnd 
{
  INICIAL,
  PULSO_TRIGGER,
  ECHO_START,
  ECHO_END
};
EstadosUltrsnd Estado_ultrsnd = INICIAL;
#define PeriodMedUltrsn 20   //Periodo de medida de ultrasonidos milisegundos

//**********Control PWM de motores**********
//Definición de pines
#define MOTORdd  32
#define MOTORdi  33
#define MOTORtd  25
#define MOTORti  26

//Definiciones
const int resolucion = 10; //10 bits
const int max_valor = 1023;   //Valora máximo para 10 bits

//*************IMU****************************
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GX,GY,GZ;
float AcX_filtr, AcY_filtr, AcZ_filtr,Tmp_filtr,GX_filtr, GY_filtr, GZ_filtr;
unsigned long LastMedIMU;
const unsigned long PeriodMedIMU = 2;  //milisegundos entre medidas de la IMU
#define alpha 0.9
//*************Comunicaciones*******************
const unsigned long commTimeout = 1000; // milisegundos sin comunicación para activar apagado

//Estructura de datos de comunicaciones ESP32 Now
typedef struct __attribute__((packed)) 
{
  //ULTRASONIDOS
  float Dist_ultrasonicos;
  unsigned long echoStartTime;
  unsigned long echoEndTime;

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
  //ESTADO
   bool estado;   //true es encendio, false es apagado  
} SensorPacket;

typedef struct __attribute__((packed)) 
{
  //MOTOR DELANTERO DERECHO
  int16_t Period_dd;   //Microsegundos
  int16_t Pulso_dd;    //Microsegundos

  //MOTOR DELANTERO IZQUIERDO
  int16_t Period_di;   //Microsegundos
  int16_t Pulso_di;    //Microsegundos

  //MOTOR TRASERO DERECHO
  int16_t Period_td;   //Microsegundos
  int16_t Pulso_td;    //Microsegundos

  //MOTOR TRASERO IZQUIERDO
  int16_t Period_ti;   //Microsegundos
  int16_t Pulso_ti;    //Microsegundos
  //ESTADO
  bool estado;   //true es encendio, falase es apagado
} ControlPacket;

SensorPacket data_sensores;

// ✅ MAC del MASTER autorizado (reemplázala con la real)
uint8_t master_mac[] = { 0xA8, 0x42, 0xE3, 0xCD, 0x69, 0x18 };  // Ejemplo

//Variables para detectar desconexión del master y parar los motores
unsigned long lastCommTime = 0; // tiempo en micros o millis


//Funciones para el control de motores
uint32_t Periodo_ant[] = {100,100,100,100};

void ActualizarPWM(uint8_t pwmPin, uint32_t Periodo, uint32_t* Periodo_ant, uint32_t Pulso )
  {
    if((Periodo > 1000) & (Periodo < 100000) & (Pulso < Periodo))
    {
      if((Periodo > *Periodo_ant + 100) | (Periodo < *Periodo_ant - 100))
      {
        uint32_t frec = 1E6/Periodo;
        //ledcChangeFrequency(pwmPin, frec, resolucion);
        ledcDetach(pwmPin);
        ledcAttach(pwmPin,frec,resolucion);
        *Periodo_ant=Periodo;
      }
      
      uint32_t Ciclotrabajo=(max_valor*Pulso)/Periodo;
      ledcWrite(pwmPin, Ciclotrabajo);

    }
    else ledcWrite(pwmPin,0);  
  } 


void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) 
{
  // Verificar si el mensaje viene del master autorizado
  if (memcmp(info->src_addr, master_mac, 6) != 0) 
  {
    Serial.println("Mensaje de MAC no autorizada, ignorado.");
    return;
  }

  //Serial.printf("Mensaje recibido del master: %s\n", incomingData);
  if (len != sizeof(ControlPacket)) 
  {
    Serial.println("❌ Tamaño de paquete incorrecto");
    return;
  }

  ControlPacket received;
  memcpy(&received, incomingData, sizeof(received));

 // Serial.println("📥 Paquete recibido:");
/* 
  Serial.printf("  Periodo_dd: %d", received.Period_dd);
  Serial.printf("  Periodo_di: %d", received.Period_di);
  Serial.printf("  Periodo_td: %d", received.Period_td);
  Serial.printf("  Periodo_ti: %d", received.Period_ti);
  Serial.printf("  Pulso_dd: %d", received.Pulso_dd);
  Serial.printf("  Pulso_di: %d", received.Pulso_di);
  Serial.printf("  Pulso_td: %d", received.Pulso_td);
  Serial.printf("  Pulso_ti: %d\n", received.Pulso_ti);
//  Serial.printf("  Estado: %s\n", received.estado ? "Encendido" : "Apagado");

*/


  // ✅ Actualizar tiempo de última comunicación válida
  lastCommTime = millis();

  //Actualizar señales PWM de control

  Serial.printf("Distancia: %.2f   ",  data_sensores.Dist_ultrasonicos);
  Serial.printf("AcYfiltr: %.2f   ", AcY_filtr);
  Serial.printf("Actualiza PWMdd: %d ",  received.Pulso_dd);
  
  Serial.printf(" Tiempo: %d \n", millis());
  ActualizarPWM(MOTORdd,received.Period_dd,&Periodo_ant[0], received.Pulso_dd);
  ActualizarPWM(MOTORdi,received.Period_di,&Periodo_ant[1], received.Pulso_di);
  ActualizarPWM(MOTORtd,received.Period_td,&Periodo_ant[2], received.Pulso_td);
  ActualizarPWM(MOTORti,received.Period_ti,&Periodo_ant[3], received.Pulso_ti);


  // Responder con un mensaje inmediato con las medidas de los sensores en el drone

  data_sensores.ax=AcX_filtr;
  data_sensores.ay=AcY_filtr;
  data_sensores.az=AcZ_filtr;
  data_sensores.gx=GX_filtr;
  data_sensores.gy=GY_filtr;
  data_sensores.gz=GZ_filtr;
  data_sensores.tmp=Tmp_filtr;
  data_sensores.estado=true;


  //data_sensores.echoStartTime=100;
  //data_sensores.echoEndTime=300;

  esp_err_t result = esp_now_send(master_mac, (uint8_t *)&data_sensores, sizeof(data_sensores));
  if (result == ESP_OK) 
  {
    //Serial.println("Respuesta enviada al master");
  } 
  else 
  {
    Serial.println("Error al enviar respuesta");
  }
}

//Interrupción del sensor de ultrasonidos
void IRAM_ATTR Ultrasnd_PWM_int() 
{
  unsigned long now = micros();
  
  
  switch (Estado_ultrsnd)
  {
    case ECHO_START:   // Paso 2: Esperar a que ECHO pase a HIGH
      if (digitalRead(ECHO_PIN) == HIGH) 
      {
        echoStartTime = now - lastTriggerTime;
        Estado_ultrsnd = ECHO_END;  //Espera finalizado pulso en ECHO
      }
      else
       if(now - lastTriggerTime > 1000)   //No Detecta pulso ECHO. Toma medida anterior
       {
        Estado_ultrsnd = INICIAL;
       }
       break;

    case ECHO_END: // Paso 3: Esperar a que ECHO pase a LOW
      Estado_ultrsnd = INICIAL;
      if (digitalRead(ECHO_PIN) == LOW) 
      {     
        if (now-lastTriggerTime<3000)   //Máxima distancia 1m
          echoEndTime = now-lastTriggerTime-echoStartTime;
        else
        {
         echoStartTime = 100;
         echoEndTime = 2800;
        } 
        // Paso 4: Calcular duración y distancia
        data_sensores.Dist_ultrasonicos = echoEndTime * 0.0343 / 2.0;
        data_sensores.echoStartTime = echoStartTime;
        data_sensores.echoEndTime = echoEndTime;
      }
      break;
    default:
      Estado_ultrsnd=INICIAL;
  }
}



void setup() 
{
//Configuración puerto serie
  Serial.begin(115200);

//Configuración Sensor ultrasonico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);  // Asegura que TRIG comience en LOW
  attachInterrupt(ECHO_PIN, Ultrasnd_PWM_int, CHANGE);
//Configurciones pins pruebas
  pinMode(PRUEBA_PIN, OUTPUT);
  
//Configura puertos PWM
  //ledcAttach(MOTORdd, 50, 10);
  ledcAttachChannel(MOTORdd, 50, 10, 0);
  ledcWrite(MOTORdd, 0);

  //ledcAttach(MOTORdi, 50, 10);
  ledcAttachChannel(MOTORdi, 50, 10, 1);
  ledcWrite(MOTORdi, 0);

  //ledcAttach(MOTORtd, 50, 10);
  ledcAttachChannel(MOTORtd, 50, 10, 2);
  ledcWrite(MOTORtd, 0);

  //ledcAttach(MOTORti, 50, 10);
  ledcAttachChannel(MOTORti, 50, 10, 3);
  ledcWrite(MOTORti, 0);


//Arma ESC

  ledcWrite(MOTORdd, 0.05*max_valor);
  ledcWrite(MOTORdi, 0.05*max_valor);
  ledcWrite(MOTORtd, 0.05*max_valor);
  ledcWrite(MOTORti, 0.05*max_valor);
  delay(3000);

//Configuración comunicación ESP32 Now
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error al inicializar ESP-NOW");
    return;
  }

  // Registrar callback actualizado
  esp_now_register_recv_cb(OnDataRecv);

  // Registrar al master como peer válido
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, master_mac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(master_mac)) 
  {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) 
    {
      Serial.println("Error al registrar al master");
      return;
    }
  }

  Serial.println("Slave listo. Esperando mensajes del master...");

  //*********Configurar I2C para IMU**************
  Wire.begin();
  Wire.setClock(400000); 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop() 
{
 // digitalWrite(PRUEBA_PIN, !digitalRead(PRUEBA_PIN)); //Prueba de tiempo de refresco del loop
//**************Prueba de tiempo de respuesta
/*
  if (micros() - lastPulsoPrueba >= PeriodoPrueba) {
    if(AcYprue==7000)
      {
        AcYprue=-7000;
        AcZprue-10000;
      }  
      else{
        AcYprue=7000;
        AcZprue-10000;
      }
    Serial.printf("       Cambio angulo: %d \n", micros());  
    lastPulsoPrueba=micros();
  }
*/
//****************ESTADO DEL SENSOR ULTRASONICO****************************
  // Paso 1: Enviar pulso TRIG cada 20 ms
  if ((Estado_ultrsnd == INICIAL) && millis() - lastTriggerTime >= PeriodMedUltrsn) 
  {
    // Enviar pulso de 10 us
    
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);  //Pulso de 20us
    digitalWrite(TRIG_PIN, LOW);
    lastTriggerTime = micros();
    Estado_ultrsnd = ECHO_START;      //Espera señal ECHO
  }

  //**************Control de motores*******************
  //Desactivar si se ha perdido comunicaciones con el Master********
  if (millis() - lastCommTime > commTimeout) 
  {
    // Apagar PWM de todos los motores
    ledcWrite(MOTORdd, 0.045*max_valor);
    ledcWrite(MOTORdi, 0.045*max_valor);
    ledcWrite(MOTORtd, 0.045*max_valor);
    ledcWrite(MOTORti, 0.045*max_valor);
    Serial.println("Sin comunicaciones");
  }

  
  //Nueva medida de la IMU
  if (millis() - LastMedIMU > PeriodMedIMU) 
  {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

/*
  AcX=AcXprue;  //Prueba de tiempo de respuesta
  AcY=AcYprue;  //Prueba de tiempo de respuesta
  AcZ=AcZprue;  //Prueba de tiempo de respuesta
*/

  AcX_filtr = AcX_filtr*alpha+AcX*(1-alpha);
  AcY_filtr = AcY_filtr*alpha+AcY*(1-alpha);
  AcZ_filtr = AcZ_filtr*alpha+AcZ*(1-alpha);
  
  Tmp_filtr = Tmp_filtr*alpha+Tmp*(1-alpha);
  
  GX_filtr = GX_filtr*alpha+GX*(1-alpha);
  GY_filtr = GY_filtr*alpha+GY*(1-alpha);
  GZ_filtr = GZ_filtr*alpha+GZ*(1-alpha);
/*  
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcY_Filtr = "); Serial.print(AcY_filtr);
  Serial.print(" | AcY = "); Serial.print(data_sensores.ay);
  Serial.print(" | Tmp = "); Serial.print(data_sensores.tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(data_sensores.gx);
  Serial.print(" | GyY = "); Serial.print(data_sensores.gy);
  Serial.print(" | GyZ = "); Serial.print(data_sensores.gz);
  Serial.println(); 
 */

  LastMedIMU = millis();
  }
  
  
      // Mostrar la distancia medida
      //  Serial.print("Echo End Time: ");
      //  Serial.print(data_sensores.echoEndTime);
      //  Serial.println(" us");
  
}
