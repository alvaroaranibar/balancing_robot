//GANANCIAS:
//k1=  -47, -47.5
//k2=  -38, -38.5
//k3= -89, -88.5
//k4=  -44, -44.5, -45
//Se sintonizo con: voltaje real: 11.61V, programacion: vm = 12.33V
//Segunda prueba: vreal= 12.03, vm=12.776



#include <Arduino.h>
#include <XSpaceIoT.h>
#include <cmath>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

/******************************PIN OUT*************************************/
//Encoder 1
#define encoder_CHA_1 19
#define encoder_CHB_1 18
//Encoder 2
#define encoder_CHA_2 4
#define encoder_CHB_2 5
//Driver
const int PWMA= 25;
const int A01= 26;
const int A02= 27;
const int PWMB= 32;
const int A01_B= 14;
const int A02_B= 12; 

/******************CONSTANTES Y GLOB VARIABLES******************************/
/********ENCODERS********/
#define CTE_MOTOR 378.4 //Rel transformación*pulsos por vuelta de encoder
#define RADIO 0.034   //-m, radio de la rueda

volatile double T = 0;
volatile double Periodo = 1000000;
volatile double Periodo_0 = 1000000;
volatile double Periodo_1 = 1000000;
volatile double Periodo_2 = 1000000;
volatile double Tant = 0;
volatile double counter = 0;

volatile double T_B = 0;
volatile double Periodo_B = 1000000;
volatile double Periodo_0_B = 1000000;
volatile double Periodo_1_B = 1000000;
volatile double Periodo_2_B = 1000000;
volatile double Tant_B = 0;
volatile double counter_B = 0;

/********DRIVER********/
double vm = 12.776; // Voltaje maximo de motor

/********MPU6050********/
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// Variables de control/estado del MPU
bool dmpReady = false;  // se establece en true si la inicialización del DMP fue exitosa
uint8_t devStatus;      // estado de retorno después de cada operación del dispositivo (0 = éxito, !0 = error)
uint16_t packetSize;    // tamaño esperado del paquete DMP (por defecto es 42 bytes)
uint8_t fifoBuffer[64]; // buffer de almacenamiento FIFO

// Variables de orientación/movimiento
Quaternion q;           // [w, x, y, z]         contenedor de cuaterniones
VectorInt16 gy;         // [x, y, z]            mediciones del sensor giroscópico
VectorFloat gravity;    // [x, y, z]            vector de gravedad
float ypr[3];           // [yaw, pitch, roll]   contenedor de yaw/pitch/roll 
  

/********MQTT*******/
double x_ref=0;
double x_ref_gen=0;
double u_ref = 0;

double k1=0;
double k2=0;
double k3=0;
double k4=0;
double teta_ref = 0;

/******************FUNCIONES ENCODER******************************/
void IRAM_ATTR ISR_encoder1(){
    T = micros();
    if(digitalRead(encoder_CHA_1) == HIGH){
        Periodo_0 = T - Tant;
        Tant = T;
        if(digitalRead(encoder_CHB_1) == LOW){
            Periodo_0 = (-1)*Periodo_0;
            counter--;
        }
        else {
            counter++;
        }
        //Filtro mediana  
        if (((Periodo_0>Periodo_1)&&(Periodo_0<Periodo_2))||
            ((Periodo_0<Periodo_1)&&(Periodo_0>Periodo_2))){
              Periodo=Periodo_0;
            }
        if (((Periodo_1>Periodo_0)&&(Periodo_1<Periodo_2))||
            ((Periodo_1<Periodo_0)&&(Periodo_1>Periodo_2))){
              Periodo=Periodo_1;
            }
        if (((Periodo_2>Periodo_0)&&(Periodo_2<Periodo_1))||
            ((Periodo_2<Periodo_0)&&(Periodo_2>Periodo_1))){
              Periodo=Periodo_2;
            }
        Periodo_2=Periodo_1;
        Periodo_1=Periodo_0;
    }
}

void IRAM_ATTR ISR_encoder2(){
    T_B = micros();
    if(digitalRead(encoder_CHA_2) == HIGH){
        Periodo_0_B = T_B - Tant_B;
        Tant_B = T_B;
        if(digitalRead(encoder_CHB_2) == LOW){
            Periodo_0_B = (-1)*Periodo_0_B;
            counter_B--;
        }
        else {
            counter_B++;
        }
        //Filtro mediana  
        if (((Periodo_0_B>Periodo_1_B)&&(Periodo_0_B<Periodo_2_B))||
            ((Periodo_0_B<Periodo_1_B)&&(Periodo_0_B>Periodo_2_B))){
              Periodo_B=Periodo_0_B;
            }
        if (((Periodo_1_B>Periodo_0_B)&&(Periodo_1_B<Periodo_2_B))||
            ((Periodo_1_B<Periodo_0_B)&&(Periodo_1_B>Periodo_2_B))){
              Periodo_B=Periodo_1_B;
            }
        if (((Periodo_2_B>Periodo_0_B)&&(Periodo_2_B<Periodo_1_B))||
            ((Periodo_2_B<Periodo_0_B)&&(Periodo_2_B>Periodo_1_B))){
              Periodo=Periodo_2;
            }
        Periodo_2_B=Periodo_1_B;
        Periodo_1_B=Periodo_0_B;
    }
}

void config_encoders(){
  pinMode(encoder_CHA_1,INPUT_PULLDOWN);
  pinMode(encoder_CHB_1,INPUT_PULLDOWN);
  attachInterrupt(encoder_CHA_1, ISR_encoder1, HIGH);

  pinMode(encoder_CHA_2,INPUT_PULLDOWN);
  pinMode(encoder_CHB_2,INPUT_PULLDOWN);
  attachInterrupt(encoder_CHA_2, ISR_encoder2, HIGH);
}

// Devuelve velocidad del encoder en °/s
// Entradas: -1: encoder1    -2: encoder2
double GetEncoderSpeed(int encoder){
    double vel=0;   
    if (encoder == 1){
      vel = 360000000.0/(CTE_MOTOR*Periodo);
      vel = vel* 3.1416 / 180; //en radianes/s
    }
    else if(encoder ==2){
      vel = 360000000.0/(CTE_MOTOR*Periodo_B);
      vel = vel* 3.1416 / 180; //en radianes/s
    }
    else
      vel =  -1; //En caso de error
    return vel;
    vel = -vel; // Se cambia sentido para mantener la misma referencia de otros estados
}

double GetEncoderPosition(int encoder){
  double pos=0;
  if (encoder==1){
    pos = counter/CTE_MOTOR*360.0;
    pos = pos* 3.1416 / 180;
  }
  else if (encoder==2){
    pos = counter_B/CTE_MOTOR*360.0;
    pos = pos* 3.1416 / 180;
  }
  else
    pos = 1; //En caso de error

  pos = -pos; // Se cambia sentido para mantener la misma referencia de otros estados
  return pos;
}

//Devuelve posicion horizontal
double GetPosition(){
  double pos=0;
  double x = 0;
  pos = counter/CTE_MOTOR*360.0;
  pos = pos* M_PI / 180;
  x = pos*RADIO;
  x = -x; // Se cambia sentido para mantener la misma referencia de otros estados
  return x;
}

double GetSpeed(){
  double vel=0;
  double x_p = 0;
  vel = 360000000.0/(CTE_MOTOR*Periodo);
  vel = vel* M_PI / 180; //en radianes/s
  x_p = vel * RADIO;
  x_p = -x_p; // Se cambia sentido para mantener la misma referencia de otros estados
  return x_p;
}

/******************FUNCIONES DRIVER******************************/
void config_driver(){
  int pwmFrequency = 20000;
  int pwmResolution = 10;
  pinMode(A01, OUTPUT);
  pinMode(A02, OUTPUT);
  pinMode(A01_B, OUTPUT);
  pinMode(A02_B, OUTPUT);
  ledcSetup(1, pwmFrequency, pwmResolution);
  ledcSetup(2, pwmFrequency, pwmResolution);
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 2);
}

void escribir_voltaje(double voltaje){
  int dutyCycle= 0; // 0-1023
  if (voltaje > 0){
    digitalWrite(A01, HIGH);  
    digitalWrite(A02, LOW);  
    digitalWrite(A01_B, HIGH); 
    digitalWrite(A02_B, LOW);  
  }
  else {
    digitalWrite(A01, LOW); 
    digitalWrite(A02, HIGH); 
    digitalWrite(A01_B, LOW);  
    digitalWrite(A02_B, HIGH);  
    voltaje = -1 * voltaje;
  }
  dutyCycle= map(voltaje, 0, vm, 0, 1023); //Mapea voltaje promedio a dutycycle
  ledcWrite(1, dutyCycle);
  ledcWrite(2, dutyCycle);
}

/*******************FUNCIONES MQTT******************************/
 XSThing Thing;
// Constantes para las credenciales de WiFi
const char* WIFI_SSID = "A54 de Alvaro"; // Cambiar red PUCP
const char* WIFI_PASSWORD = "cenicienta123"; 
// ID único del cliente MQTT
const char* MQTT_ID = "sr_roque"; 

/*void Mqtt_Callback(char* topicx, byte* Data, unsigned int DataLen){
    // Convierte el tema y los datos recibidos en cadenas
    String ReceivedData = String((char*)Data, DataLen);
    String Topic = String((char*)topicx);

    // Verifica si los datos recibidos son del tópico 
    if(Topic == "control/sr_roque/ref"){
      u_ref = ReceivedData.toDouble();
    }
} */


void Mqtt_Callback(char* topicx, byte* Data, unsigned int DataLen){
    // Convierte el tema y los datos recibidos en cadenas
    String ReceivedData = String((char*)Data, DataLen);
    String Topic = String((char*)topicx);

    // Verifica si los datos recibidos son del tópico 
    if(Topic == "control/sr_roque/k1"){
      k1 = ReceivedData.toDouble();
    }
    if(Topic == "control/sr_roque/k2"){
      k2 = ReceivedData.toDouble();
    }
    if(Topic == "control/sr_roque/k3"){
      k3 = ReceivedData.toDouble();
    }
    if(Topic == "control/sr_roque/k4"){
      k4 = ReceivedData.toDouble();
    }
    if(Topic == "control/sr_roque/teta_ref"){
      teta_ref = ReceivedData.toDouble();
    }
    if(Topic == "control/sr_roque/x_ref"){
      x_ref = ReceivedData.toDouble();
    }
} 

void config_mqtt(){
  Thing.Mqtt_SerialInfo(true); // Habilita la salida de información serial
  // Inicializa la conexión MQTT con el servidor en el puerto 1883 usando la función de callback especificada
  // El último parámetro establece el intervalo en milisegundos para revisar automáticamente el búfer de datos entrantes
  Thing.Mqtt_init("www.xspace.pe", 1883, Mqtt_Callback, 100); 
  // Conéctate al broker MQTT usando las credenciales proporcionadas
  Thing.Mqtt_Connect(WIFI_SSID, WIFI_PASSWORD, MQTT_ID); 
  // Suscríbete al tópico
  Thing.Mqtt_Suscribe("control/sr_roque/k1"); 
  Thing.Mqtt_Suscribe("control/sr_roque/k2"); 
  Thing.Mqtt_Suscribe("control/sr_roque/k3"); 
  Thing.Mqtt_Suscribe("control/sr_roque/k4"); 
  Thing.Mqtt_Suscribe("control/sr_roque/teta_ref"); 
  Thing.Mqtt_Suscribe("control/sr_roque/x_ref"); 
}

/******************FUNCIONES MPU6050******************************/
//Sentidos de Y,P, R con flechas en impreso
//r = rotar hacia y
//p = rotar hacia x
//y = rotar en plano xy (medida erronea con el tiempo)


void config_MPU6050() {
  // Unirse al bus I2C (la librería I2Cdev no hace esto automáticamente)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // Reloj I2C de 400kHz. Comenta esta línea si tienes dificultades de compilación
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // Inicializar el dispositivo
  Serial.println(F("Inicializando dispositivos I2C..."));
  mpu.initialize();

  // Verificar conexión
  Serial.println(F("Probando conexiones del dispositivo..."));
  Serial.println(mpu.testConnection() ? F("Conexión MPU6050 exitosa") : F("Conexión MPU6050 fallida"));

  // Cargar y configurar el DMP
  Serial.println(F("Inicializando DMP..."));
  devStatus = mpu.dmpInitialize();

  // Proporcionar tus propios offsets de giroscopio aquí, escalados para sensibilidad mínima
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);

  // Asegurarse de que funcionó (devuelve 0 si es así)
  if (devStatus == 0) {
    // Tiempo de calibración: generar offsets y calibrar nuestro MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();

    // Encender el DMP, ahora que está listo
    Serial.println(F("Activando DMP..."));
    mpu.setDMPEnabled(true);

    // Establecer nuestra bandera DMP Ready para que la función principal loop() sepa que está bien usarlo
    Serial.println(F("DMP listo!"));
    dmpReady = true;

    // Obtener el tamaño esperado del paquete DMP para comparación posterior
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = carga inicial de memoria fallida
    // 2 = actualizaciones de configuración DMP fallidas
    Serial.print(F("Inicialización DMP fallida (código "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void leer_mpu6050() {
  // Si la programación falló, no intentar hacer nada
  if (!dmpReady) return;

  // Verificar si hay nuevos datos disponibles
  if (mpu.dmpPacketAvailable()) {
    // Leer un paquete del FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Obtener el paquete más reciente
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetGyro(&gy, fifoBuffer);
    }
  }
}

/****************************TAREAS******************************/
//Para probar que todos los estados tienen la misma referencia (sentido)
void prueba_sensores(void *pvParameters){
  double x;
  double x_p;
  double teta;
  double teta_p;
  while(1){
    x= GetPosition();
    x_p = GetSpeed();
    leer_mpu6050();
    teta = ypr[2] * 180/M_PI;
    teta_p = gy.x;
    teta_p = (gy.x / 131.0); //A °
    Serial.print(">x: ");
    Serial.println(x);
    Serial.print(">x_p: ");
    Serial.println(x_p);
    Serial.print(">teta: ");
    Serial.println(teta);
    Serial.print(">teta_p: ");
    Serial.println(teta_p);
    vTaskDelay(pdMS_TO_TICKS(25));
    
  }
}
//Para probar que voltaje de motor tiene misma referencia(sentido) que x, x_p
void prueba_motor(void *pvParameters){
  double x;
  double x_p;
  while(1){
    escribir_voltaje(u_ref);
    x= GetPosition();
    x_p = GetSpeed();
    Serial.print(">x: ");
    Serial.println(x);
    Serial.print(">x_p: ");
    Serial.println(x_p);
    Serial.print(">u_ref: ");
    Serial.println(u_ref);
    Thing.Mqtt_CheckBuffer(); // Revisa si se recibió dato y entra a Mqtt_Callback
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

void servo_sistema(void *pvParameters){
  double K1 = -1.737019946251197;
  double K2[4] = {-15.684719748134254,-8.848089122788284,-47.074347097336286,-2.585802755941737};
  
  int T=30;

  double uk;
  double ek;
  double Iek;
  double ek_1=0;
  double Iek_1 =0;

  double x, x_p, teta, teta_p;
  double D2;

  while(1){
    x = GetPosition();
    x_p = GetSpeed();
    leer_mpu6050();
    teta = ypr[2]; //en rad
    teta_p = (gy.x/131.0)*(M_PI/180); //en rad

    ek=x_ref-x;
    Iek=Iek_1+ek;
    uk = K1 * Iek - K2[0] * x - K2[1] * x_p - K2[2] * teta - K2[3] * teta_p;
    Iek_1=Iek;
    ek_1=ek;

    if (uk>11)
      uk = 11;
    if (uk<-11)
      uk = -11;
    escribir_voltaje(uk);
    
    Serial.print(x);
    Serial.print(" ");
    Serial.println(teta*180/M_PI);

    Thing.Mqtt_Publish("control/sr_roque/uk", uk); // Aquí se publica en el tópico (es diferente al tópico para recibir referencia al que me suscribí antes)
    Thing.Mqtt_CheckBuffer(); // Revisa si se recibió dato y entra a Mqtt_Callback
    vTaskDelay(pdMS_TO_TICKS(T));
  }
  vTaskDelete(NULL);
}

//Funcion para suavizar escalon de x_ref
void genera_x_ref(void *pvParameters){
  while(1){
    Thing.Mqtt_CheckBuffer();

    if (x_ref_gen<x_ref)
      x_ref_gen = x_ref_gen + 0.01;
    
    else if (x_ref_gen>x_ref)
      x_ref_gen = x_ref_gen - 0.01;
    
    Thing.Mqtt_Publish("control/sr_roque/x_ref_eco", x_ref_gen);
    vTaskDelay(pdMS_TO_TICKS(50)); // el suavizado es 0.01m cada 50ms
  }
}

void ss_error (void *pvParameters){
  int T=30;
  double uk;
  double ex;
  double ex_p;
  double eteta;
  double eteta_p;

  double x, x_p, teta, teta_p;

  while(1){
    x = GetPosition();
    x_p = GetSpeed();
    leer_mpu6050();
    teta = ypr[2]; //en rad
    teta_p = (gy.x/131.0)*(M_PI/180); //en rad

    ex=x_ref_gen-x;
    ex_p=0.0-x_p;
    eteta=teta_ref-teta;
    eteta_p=0.0-teta_p;

    uk = k1 * ex + k2 * ex_p + k3 * eteta + k4 * eteta_p;

    if (uk>11)
      uk = 11;
    if (uk<-11)
      uk = -11;

    escribir_voltaje(uk);
    vTaskDelay(pdMS_TO_TICKS(T));
  }
  vTaskDelete(NULL);
}

void valida (void *pvParameters){
  int T=30;
  double x, x_p, teta, teta_p;

  while(1){
    x = GetPosition();
    x_p = GetSpeed();
    leer_mpu6050();
    teta = ypr[2]; //en rad
    teta_p = (gy.x/131.0)*(M_PI/180); //en rad

    Serial.print(">teta: ");
    Serial.println(teta);
    Serial.print(">teta_p: ");
    Serial.println(teta_p);
    vTaskDelay(pdMS_TO_TICKS(T));
  }
}


void setup() {
  Serial.begin(115200);
  config_encoders();
  config_driver();
  config_mqtt();
  while(k4==0){
    Thing.Mqtt_CheckBuffer(); // Revisa si se recibió dato y entra a Mqtt_Callback
    delay(100);
  }
  Thing.Mqtt_Publish("control/sr_roque/k1_eco", k1);
  Thing.Mqtt_Publish("control/sr_roque/k2_eco", k2);
  Thing.Mqtt_Publish("control/sr_roque/k3_eco", k3);
  Thing.Mqtt_Publish("control/sr_roque/k4_eco", k4);
  config_MPU6050();
  xTaskCreatePinnedToCore(ss_error,"",8000,NULL,1,NULL,1);
  xTaskCreatePinnedToCore(genera_x_ref,"",4000,NULL,2,NULL,1);
}

void loop() {

}

