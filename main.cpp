#include <Arduino.h>
#include <XSpaceV2.h>
#include <XSpaceIoT.h>
#include <cmath>
XSpaceV2Board XSB;
XSThing Thing;

// Constantes para las credenciales de WiFi
const char* WIFI_SSID = "A54 de Alvaro"; // Cambiar red PUCP
const char* WIFI_PASSWORD = "cenicienta123"; 
// ID único del cliente MQTT
const char* MQTT_ID = "sr_roque"; 

//double vel_ref=100;
double ang_ref=70;

void ControlAngulo_EE_disc(void *pvParameters){
  double K1 = 0.0225;
  double K21 = 0.1698;
  double K22 = 0.0282;
  int T=62;

  double uk;
  double ek;
  double Iek;
  double ek_1=0;
  double Iek_1 =0;
  double ang;
  double vel_ang;
  double D2;

  XSB.DRV8837_Wake();

  while(1){
    ang = XSB.GetEncoderPosition(DEGREES);
    vel_ang = XSB.GetEncoderSpeed(DEGREES_PER_SECOND);

    ek=ang_ref-ang;
    Iek=Iek_1+ek;
    uk = K1*Iek - K21*ang - K22*vel_ang;

    Iek_1=Iek;
    ek_1=ek;

    XSB.DRV8837_Voltage(uk);

    Serial.println(ang);
    Thing.Mqtt_Publish("control/sr_roque/ang", ang); // Aquí se publica en el tópico (es diferente al tópico para recibir referencia al que me suscribí antes)
    Thing.Mqtt_CheckBuffer(); // Revisa si se recibió dato y entra a Mqtt_Callback
    vTaskDelay(T);
  }
  vTaskDelete(NULL);
}

void Mqtt_Callback(char* topicx, byte* Data, unsigned int DataLen){
    // Convierte el tema y los datos recibidos en cadenas
    String ReceivedData = String((char*)Data, DataLen);
    String Topic = String((char*)topicx);

    // Verifica si los datos recibidos son del tópico 
    if(Topic == "control/sr_roque/ref"){
      ang_ref = ReceivedData.toDouble();
    }
}

void setup() {
  Serial.begin(115200);
  XSB.init(20000,1280);

  Thing.Mqtt_SerialInfo(true); // Habilita la salida de información serial
  // Inicializa la conexión MQTT con el servidor en el puerto 1883 usando la función de callback especificada
  // El último parámetro establece el intervalo en milisegundos para revisar automáticamente el búfer de datos entrantes
  Thing.Mqtt_init("www.xspace.pe", 1883, Mqtt_Callback, 100); 
  // Conéctate al broker MQTT usando las credenciales proporcionadas
  Thing.Mqtt_Connect(WIFI_SSID, WIFI_PASSWORD, MQTT_ID); 
  // Suscríbete al tópico
  Thing.Mqtt_Suscribe("control/sr_roque/ref"); 

  xTaskCreatePinnedToCore(ControlAngulo_EE_disc,"",4000,NULL,1,NULL,0);
}

void loop() {
 
}
