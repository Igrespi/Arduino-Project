//*****************************************************************************
//******************************MASTER******************************************
//**********************ESP32 EN EL DRONE***************************************
//*****************************************************************************


#include <WiFi.h>
#include <esp_now.h>
#include "driver/i2c.h"
#include "esp_timer.h"

// ============================================================
//  SISTEMA DE ESTADÍSTICAS (Basado en Alumno1)
// ============================================================
#define STATS_INTERVAL_MSA 4000
#define STATS_INTERVAL_MSB 2000

// Variables para Loop
uint32_t st_loopMaxUs = 0, st_loopMinUs = UINT32_MAX;

// Variables para Transmisión al Slave (ESP-NOW)
uint32_t st_txPeriodMaxUs = 0, st_txPeriodMinUs = UINT32_MAX;
uint32_t st_txWorkMaxUs = 0, st_txWorkMinUs = UINT32_MAX;
unsigned long st_txLastUs = 0;

// Variables para Comunicación I2C (IMU)
uint32_t st_i2cPeriodMaxUs = 0, st_i2cPeriodMinUs = UINT32_MAX;
uint32_t st_i2cWorkMaxUs = 0, st_i2cWorkMinUs = UINT32_MAX;
unsigned long st_i2cLastUs = 0;

unsigned long statsLastPrintMsA = 0;
unsigned long statsLastPrintMsB = 0;

// Funciones de utilidad para estadísticas [cite: 246, 247, 248]
void resetStatsA() 
{
  st_txPeriodMaxUs = st_i2cPeriodMaxUs = 0;
  st_txPeriodMinUs = st_i2cPeriodMinUs = UINT32_MAX;
}

void resetStatsB() 
{
  st_loopMaxUs = st_txWorkMaxUs = st_i2cWorkMaxUs = 0;
  st_loopMinUs = st_txWorkMinUs = st_i2cWorkMinUs = UINT32_MAX;
}

void printStatsA() 
{
  Serial.printf("A| TX Slave: %u / %u us ", st_txPeriodMinUs, st_txPeriodMaxUs);
  Serial.printf("| I2C IMU: %u / %u us\n", st_i2cPeriodMinUs, st_i2cPeriodMaxUs);
}

void printStatsB() {
  Serial.printf("B| Loop: %u / %u us ", st_loopMinUs, st_loopMaxUs);
  Serial.printf("| TX Work: %u / %u us ", st_txWorkMinUs, st_txWorkMaxUs);
  Serial.printf("| I2C Work: %u / %u us\n", st_i2cWorkMinUs, st_i2cWorkMaxUs);
}

//Definiciones comunicaciones wifi
#define Transmdatos_Period 15000 // antes 15000

//Definiciones de I2C para el control de la IMU
#define I2C_SLAVE_NUM I2C_NUM_0
#define I2C_SLAVE_SDA 21  //Pin del SDA
#define I2C_SLAVE_SCL 22  //Pin del SCL
#define I2C_SLAVE_ADDR 0x68  // Dirección del MPU-6050
#define I2C_SLAVE_RX_BUF_LEN 128
#define I2C_SLAVE_TX_BUF_LEN 128

uint8_t i2c_register_pointer = 0x00;
#define I2C_Period 2000  // Periodo entre lecturas microsegundos (3000 antes)
unsigned long I2C_time = 0;

// Pines del sensor ultrasónico
#define TRIG_PIN 5
#define ECHO_PIN 18
// Pines de las señales PWM para el control de motores
#define MOTORdd  32
#define MOTORdi  33
#define MOTORtd  25
#define MOTORti  26
// Pines de prueba
#define PRUEBA_PIN 17
#define PRUEBA_PIN2 35


// Variables de control de tiempo y estado
unsigned long pulseStartTime = 0;
esp_timer_handle_t oneShotTimer;

enum EstadosUltrsnd 
{
  INICIAL,
  PULSO_TRIGGER,
  ECHO_START,
  ECHO_END
};

EstadosUltrsnd Estado_ultrsnd=INICIAL;

//Estructura de datos
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
   bool estado; //true es encendio, false es apagado  
} DataSensores;

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


DataSensores received;
ControlPacket Control;
bool Act_PWM[]={false,false, false, false};
uint8_t slave_mac[] = { 0x3C, 0x8A, 0x1F, 0xA0, 0xE2, 0xE8 }; // Reemplaza con MAC real del slave

unsigned long send_time = 0;

volatile bool nuevoDato = false;
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len)
{
  if (len != sizeof(DataSensores)) return;

  memcpy(&received, incomingData, sizeof(DataSensores));
  nuevoDato = true;
}
  

//*************Control PWM de motorores*****************
// Variables para medir PWM
volatile unsigned long pwm_lastRiseTime[] = {0,0,0,0};
volatile unsigned long pwm_pulseStartTime[] = {0,0,0,0};
volatile unsigned long pwm_pulseEndTime[] = {0,0,0,0};
volatile unsigned long pwmPeriod[] = {0,0,0,0};
volatile unsigned long pwmHighTime[] = {0,0,0,0};
volatile bool estado_anterior[]={HIGH, HIGH, HIGH, HIGH};  //true ->High, false->Low
const uint8_t  MOTOR[]={MOTORdd, MOTORdi, MOTORtd, MOTORti}; 

// Interrupción
void IRAM_ATTR onPwmChange() 
{
  bool currentState;
  for(int i=0; i<4; i++)
  {
    currentState = digitalRead(MOTOR[i]);
    unsigned long now = micros();
    if(currentState != estado_anterior[i])
    {
      if (currentState == HIGH) 
      {
      // Flanco de subida
      pwmPeriod[i] = now - pwm_lastRiseTime[i];  // Periodo = tiempo entre flancos de subida
      pwm_lastRiseTime[i] = now;
      pwm_pulseStartTime[i] = now;
      
      } 
      else 
      {
      // Flanco de bajada
        pwm_pulseEndTime[i] = now;
        pwmHighTime[i] = pwm_pulseEndTime[i] - pwm_pulseStartTime[i];  // Duración del pulso alto
        Act_PWM[i]=true;
      }
    }
    else
        if(now - pwm_lastRiseTime[i] > 120000) pwmHighTime[i] =0;
    
    estado_anterior[i] = currentState;
  }
}

//************Interrupciones del detector del Trigger del ultrasonidos*************************
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void  onTimer(void* arg) 
{
  unsigned long echoStart, echoEnd;
  noInterrupts();
  echoStart = received.echoStartTime;
  echoEnd   = received.echoEndTime;
  interrupts();

   switch (Estado_ultrsnd) 
   {
    case ECHO_START:
      digitalWrite(ECHO_PIN, HIGH);
      esp_timer_start_once(oneShotTimer, received.echoEndTime);
      Estado_ultrsnd = ECHO_END;
      break;

    case ECHO_END:
      digitalWrite(ECHO_PIN, LOW);
      Estado_ultrsnd=INICIAL;
    break;

    default:
      digitalWrite(ECHO_PIN, LOW);
      Estado_ultrsnd=INICIAL;
  }
}


void IRAM_ATTR Ultrasnd_PWM_int() 
{
unsigned long ancho_pulso;
unsigned long now = micros();
//Esperar a un nuevo pulso
  switch (Estado_ultrsnd) 
  {
    case INICIAL:
      if(digitalRead(TRIG_PIN) == HIGH)
      {
        pulseStartTime = micros(); 
        Estado_ultrsnd=PULSO_TRIGGER;
      }
      break;

    case PULSO_TRIGGER:
      if(digitalRead(TRIG_PIN) == LOW)
      {
        ancho_pulso = now - pulseStartTime;

        if ((ancho_pulso>10)&(ancho_pulso<5000))
        {
          Estado_ultrsnd=ECHO_START;
          esp_timer_start_once(oneShotTimer, received.echoStartTime);
        }
        else Estado_ultrsnd=INICIAL;
      }  
      break;

    default:
      Estado_ultrsnd=INICIAL;      
  }
}


void setup() 
{
  Serial.begin(115200);

  //Configuración Sensor ultrasonico
  pinMode(TRIG_PIN, INPUT);
  pinMode(ECHO_PIN, OUTPUT);
  pinMode(PRUEBA_PIN, OUTPUT);
  pinMode(PRUEBA_PIN2, OUTPUT);
  digitalWrite(ECHO_PIN, LOW);  // Asegura que TRIG comience en LOW
  attachInterrupt(TRIG_PIN, Ultrasnd_PWM_int, CHANGE);
  
  // Definir los argumentos del temporizador
  esp_timer_create_args_t oneShotTimerConfig = 
  {
    .callback = &onTimer,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "myOneShotTimer"
  };
  // Crear el temporizador
  esp_timer_create(&oneShotTimerConfig, &oneShotTimer);


  //Configuración puertos PWM para el control de motores
  pinMode(MOTORdd, INPUT);
  pinMode(MOTORdi, INPUT);
  pinMode(MOTORtd, INPUT);
  pinMode(MOTORti, INPUT);

  pinMode(MOTORdd, INPUT_PULLDOWN);
  pinMode(MOTORdi, INPUT_PULLDOWN);
  pinMode(MOTORtd, INPUT_PULLDOWN);
  pinMode(MOTORti, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(MOTORdd), onPwmChange, CHANGE); //// Interrupción por cualquier cambio (subida o bajada)
  attachInterrupt(digitalPinToInterrupt(MOTORdi), onPwmChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTORtd), onPwmChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTORti), onPwmChange, CHANGE);


  //Configuración ESP32 NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, slave_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(slave_mac)) 
  {
    esp_now_add_peer(&peerInfo);
  }

  Serial.println("Master listo");

  //Configuración del I2C para comunicaciones con la IMU
  i2c_config_t conf = 
  {
    .mode = I2C_MODE_SLAVE,
    .sda_io_num = (gpio_num_t)I2C_SLAVE_SDA,
    .scl_io_num = (gpio_num_t)I2C_SLAVE_SCL,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .slave = 
    {
      .slave_addr = I2C_SLAVE_ADDR,
      .maximum_speed = 400000
    }
  };
  i2c_param_config(I2C_SLAVE_NUM, &conf);
  i2c_driver_install(I2C_SLAVE_NUM, I2C_MODE_SLAVE, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);

  // Inicializar timestamps de estadísticas
  unsigned long ahora = micros();
  st_txLastUs = st_i2cLastUs = ahora;
  resetStatsA();
  resetStatsB();
}

void loop() 
{
  /*
  if (nuevoDato)
  {
    nuevoDato = false;

    Serial.printf("Distancia: %.2f\n", received.Dist_ultrasonicos);
  }
  */

  unsigned long now = micros();
  //uint32_t loopStart = now;

  bool Sincr_PWM = true;

  //Sincronización de las comunicaciones con el Slave con las señales PWM  para el control de motores
  for(int i=0;i<4;i++)
    Sincr_PWM &=((now-pwm_lastRiseTime[i]>2000)&(now-pwm_lastRiseTime[i]<18000))|(pwm_lastRiseTime[i]==0);

  // --- 1. PROCESO ENVÍO AL SLAVE (ESP-NOW) ---
  //Hacer un nueva solicitud a Slave
  if((now-send_time>Transmdatos_Period)&(Sincr_PWM==true))
  {
    digitalWrite(PRUEBA_PIN2, !digitalRead(PRUEBA_PIN2));

    //uint32_t t0 = micros();
    //uint32_t periodo = t0 - (uint32_t)st_txLastUs;
    //if (periodo > st_txPeriodMaxUs) st_txPeriodMaxUs = periodo;
    //if (periodo < st_txPeriodMinUs) st_txPeriodMinUs = periodo;
    //st_txLastUs = t0;

    for(int i=0; i<4; i++)
      if(now-pwm_lastRiseTime[i]>300000) Act_PWM[i]=false;

      //Actualizar medidas de los periodos y pulsos  
      if(Act_PWM[0]==true)
      {
        Control.Period_dd=pwmPeriod[0];
        Control.Pulso_dd=pwmHighTime[0];
        Act_PWM[0]=false;
      }
      else
      {
        Control.Period_dd=20000;
        Control.Pulso_dd=950;
      }

      if(Act_PWM[1]==true)
      {
        Control.Period_di=pwmPeriod[1];
        Control.Pulso_di=pwmHighTime[1];
        Act_PWM[1]=false;
      }
      else
      {
        Control.Period_di=20000;
        Control.Pulso_di=950;
      }
    
      if(Act_PWM[2]==true)
      {
        Control.Period_td=pwmPeriod[2];
        Control.Pulso_td=pwmHighTime[2];
        Act_PWM[2]=false;
      }
      else
      {
        Control.Period_td=20000;
        Control.Pulso_td=950;
      }

      if(Act_PWM[3]==true)
      {
        Control.Period_ti=pwmPeriod[3];
        Control.Pulso_ti=pwmHighTime[3];
        Act_PWM[3]=false;
      }
      else
      {
        Control.Period_ti=20000;
        Control.Pulso_ti=950;
      }
    Control.estado=true;      
    send_time = micros(); // Marcar tiempo justo antes de enviar

    // Medición del trabajo (Work)
    //uint32_t tWorkStart = micros();
    esp_now_send(slave_mac, (uint8_t *)&Control, sizeof(Control));
    //uint32_t work = micros() - tWorkStart; 
    //if (work > st_txWorkMaxUs) st_txWorkMaxUs = work;
    //if (work < st_txWorkMinUs) st_txWorkMinUs = work;
    
    Serial.printf("Pulso_dd: %d \n", Control.Pulso_dd);
  }

  // --- 2. COMUNICACIONES I2C con la IMU ---
  uint8_t data[32];
  
  if((now - I2C_time > I2C_Period)&(Sincr_PWM == true))
  {
    //uint32_t t0 = micros();
    //uint32_t periodo = t0 - (uint32_t)st_i2cLastUs;
    //if (periodo > st_i2cPeriodMaxUs) st_i2cPeriodMaxUs = periodo;
    //if (periodo < st_i2cPeriodMinUs) st_i2cPeriodMinUs = periodo;
    //st_i2cLastUs = t0;

    digitalWrite(PRUEBA_PIN, HIGH);

    //uint32_t tWorkStart = micros();
    uint8_t data[32];
    int len = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, sizeof(data), 0);

    if (len > 0) 
    {
      // El maestro ha escrito algo (probablemente la dirección del registro)
      i2c_register_pointer = data[0]; // guardamos dirección de registro
    }

    // Si se ha solicitado una lectura
    if (i2c_register_pointer == 0x3B) 
    {
      uint8_t response[14];
      int16_t values[] = { received.ax, received.ay, received.az, received.tmp, received.gx, received.gy, received.gz };

      for (int i = 0; i < 7; i++) 
      {
        response[2 * i] = (values[i] >> 8) & 0xFF;     // MSB
        response[2 * i + 1] = values[i] & 0xFF;        // LSB
      }

      i2c_slave_write_buffer(I2C_SLAVE_NUM, response, sizeof(response), 0 / portTICK_PERIOD_MS);
      i2c_register_pointer = 0x00; // Reset después de escribir
      I2C_time = now;
    }

    //uint32_t work = micros() - tWorkStart;
    //if (work > st_i2cWorkMaxUs) st_i2cWorkMaxUs = work;
    //if (work < st_i2cWorkMinUs) st_i2cWorkMinUs = work;
  }
  else digitalWrite(PRUEBA_PIN, LOW);


  /*
  // --- DURACIÓN DEL LOOP ---
  uint32_t loopDur = micros() - loopStart;
  if (loopDur > st_loopMaxUs) st_loopMaxUs = loopDur;
  if (loopDur < st_loopMinUs) st_loopMinUs = loopDur;

  // --- IMPRESIÓN PERIÓDICA ---
  unsigned long ahoraMs = millis();


  if (ahoraMs - statsLastPrintMsA >= STATS_INTERVAL_MSA) 
  {
    printStatsA();
    resetStatsA();
    statsLastPrintMsA = ahoraMs;
  }

  if (ahoraMs - statsLastPrintMsB >= STATS_INTERVAL_MSB) 
  {
    printStatsB();
    resetStatsB();
    statsLastPrintMsB = ahoraMs;
  }
  */
}
