//-----------------------------------------------------
//Incluimos librerías para el WiFi de la placa Arduino ESP8266, para MQTT y para poder crear y recibir JSON
#include <ESP8266WiFi.h>
#include <ESP8266httpUpdate.h>
#include <SimpleTimer.h> //repetitive tasks
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
//-----------------------------------------------------
//Todo esto es para el sensor de temperatura DS18B20, el cual se encarga de leer la temperatura que hay fuera de la cabina
#include <OneWire.h>
#include <DallasTemperature.h>

// Instancia al timer creado para rutina de actualización
SimpleTimer timerManager; //timer declaration
#define OTA_URL             "https://huertociencias.uma.es/esp8266-ota-update"
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu"
// Pin donde se conecta el bus 1-Wire
const int pinDatosTempExt = 9;
 
// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosTempExt);
DallasTemperature sensorDS18B20(&oneWireObjeto);
//-----------------------------------------------------
//Este apartado corresponde al sensor de temperatura que está situado dentro de la cabina DHT22
#include "DHTesp.h"
DHTesp dht;
ADC_MODE(ADC_VCC);
#include <Wire.h>
//-----------------------------------------------------
//La siguiente librería nos permite utilizar el sensor INA219 para la letura de los voltajes
#include <Adafruit_INA219.h>
//A continuación, presentamos los dos sensores INA219 que vamos a utilizar, ya que vamos a usar un único PIN de la placa Arduino
//para estos sensores, esto va a ser prosible gracias a I2C 
Adafruit_INA219 ina219_1(0x40);
Adafruit_INA219 ina219_2(0x44);
//-----------------------------------------------------
//Definimos a cotinuación el PIN al cual conectaremos el sensor TCRT5000, que es el encargado de detectar si la puerta de la cabina
//se encuentra abierta o cerrada
const int pinTCRT5000 = 10;
//-----------------------------------------------------
//Ahora vamos a definir el PIN de la placa Arduino al que vamos a conectar el sensor TCRT5000, que es el Relé cuya función va a ser
//la de reiniciar la placa RaspBerry del Huerto
int PIN_Rele1=12;
int control_1=1;
int control_1_ant=1;
//-----------------------------------------------------
//Declaramos e inicializamos la variable que controlará el ventilador (modo automatico o manual)
int control_ventilador=1; //Inicializamos el control en modo automatico
float porcentaje_ventilador=0; //Inicializamos la intensidad del ventilador en modo manual al 0%
int baremos_temperatura=2; //Inicializamos el baremo en 2 grados centígrados
//-----------------------------------------------------
//Pin por donde saldrá la señal PWM para regular el ventilador
int PWMPin = 14;
//-----------------------------------------------------
//Ahora creamos una estructura que será un registro de datos donde incorporaremos varios datos que observamos a continuación
struct registro_datos {
        float caida_tension_12V;
        float caida_tension_24V;
        float temperatura_interior;
        float temperatura_exterior;
        float humedad;
        int movimiento;
        String IP;
        float porcentaje_ventilador;
        float PWM;
        int baremo;
        float dif;
        };


struct registro_datos_puerta{
        int movimiento;
        };

        
WiFiClientSecure wClient;
PubSubClient mqtt_client(wClient);

#define Huerto

#ifdef Huerto
#define ssid "huerticawifi"
#define password "4cc3sshu3rt1c4"
#define mqtt_server "huertociencias.uma.es"
#define mqtt_user "huerta" 
#define mqtt_pass "accesohuertica"
#define MQTT_PORT 8163

#endif

char ID_PLACA[16];
//-----------------------------------------------------
// Esta funcion va a ser la que usemos para que se establezca la conexion WiFi
void conecta_wifi() {
  Serial.printf("\nConnecting to %s:\n", ssid);
 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  wClient.setInsecure();
  wClient.setTimeout(12);
  Serial.printf("\nWiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());
}
//-----------------------------------------------------
// Esta funcion va a ser la que usemos para que se establezca la conexion MQTT
  void conecta_mqtt() {
  char cadenaConectado[128];  // cadena de 128 caracteres 
  char cadenaDesconectado[128];  // cadena de 128 caracteres
  char prueba_mqtt[128]; 
  // A continuación, creamos una cadena de caracteres de 128 de longitud donde guardaremos 
  // el mensaje de ultima voluntad indicando que nos hemos desconectado repentinamente
  snprintf(cadenaConectado, 128,"{\"online\": false}");
  snprintf(cadenaDesconectado, 128,"{\"online\": true}");
  snprintf(prueba_mqtt, 128,"Conexion establecida");
  // Mienrtas que no establezcamos la conexion no saldremos del bucle
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(ID_PLACA,mqtt_user,mqtt_pass,"II3/ESP3097139/conexion",2,true,cadenaDesconectado)) {
      Serial.printf(" conectado a broker: %s\n",mqtt_server);
      mqtt_client.subscribe("huerta/Control_reles");
      mqtt_client.subscribe("huerta/Control_ventilador");
      mqtt_client.publish("huerta/DHT22_conexion_establecida",prueba_mqtt);
    } else {
      Serial.printf("failed, rc=%d  try again in 5s\n", mqtt_client.state());
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//-----------------------------------------------------
//Con la siguiente funcion conseguiremos crear un Json para poder enviarlo por MQTT 
String serializa_JSON (struct registro_datos datos)
    {
      StaticJsonDocument<512> jsonRoot;
      String jsonString;

      //jsonRoot["Conexion"] = "prueba";
      //jsonRoot["WiFi"] = "prueba2";
      jsonRoot["IP_Emisor"] = datos.IP;

      JsonObject Ventilador=jsonRoot.createNestedObject("Ventilador");
      Ventilador["%Intensidad"] = datos.porcentaje_ventilador;
      Ventilador["PWM"] = datos.PWM;
      Ventilador["Baremo"]=datos.baremo;
      Ventilador["dif"]=datos.dif;

      JsonObject DHT22=jsonRoot.createNestedObject("DHT22");
      DHT22["temp_interior"] = datos.temperatura_interior;
      DHT22["hum"] = datos.humedad;

      JsonObject DS18B20=jsonRoot.createNestedObject("DS18B20");
      DS18B20["temp_exterior"] = datos.temperatura_exterior;
      
      JsonObject INA219=jsonRoot.createNestedObject("INA219");
      INA219["Tension_12V"] = datos.caida_tension_12V;
      INA219["Tension_24V"] = datos.caida_tension_24V;

      JsonObject TCRT5000=jsonRoot.createNestedObject("TCRT5000");
      TCRT5000["Puerta"] = datos.movimiento;
      
      serializeJson(jsonRoot,jsonString);
      return jsonString;
    }
//-----------------------------------------------------
String serializa_JSON_puerta(struct registro_datos_puerta datos_puerta)
{
      StaticJsonDocument<512> jsonRoot;
      String jsonString;
      
      JsonObject Puerta=jsonRoot.createNestedObject("Puerta");
      Puerta["Movimiento"] = datos_puerta.movimiento;

      serializeJson(jsonRoot,jsonString);
      return jsonString;
}
//-----------------------------------------------------
  void callback(char* topic, byte* payload, unsigned int length) {
  char *mensaje = (char *)malloc(length+1); // reservo memoria para copia del mensaje
  strncpy(mensaje, (char*)payload,length); // copio el mensaje en cadena de caracteres
  Serial.printf("Mensaje recibido [%s] %s\n", topic, mensaje);
  if(strcmp(topic, "huerta/Control_reles")==0) {

    StaticJsonDocument<512> root; // el tamaño tiene que ser adecuado para el mensaje
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje,length);

    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    else
    if(root.containsKey("control_1"))  // comprobar si existe el campo/clave que estamos buscando
    { 
      digitalWrite(PIN_Rele1, root["control_1"]);
      if(root["control_1"]==0)
      {
      delay(1500);
      digitalWrite(PIN_Rele1, 1);
      const char* apagar="desconectar rele";
      mqtt_client.publish("huerta/Reinicializacion_done",apagar);
      }
    }
    else
    {
      Serial.print("Error : ");
      Serial.println("\"control_!\" or \"control_!\" key not found in JSON");
    }

  }
//Ahora pasamos al control del ventilador por MQTT
  if(strcmp(topic, "huerta/Control_ventilador")==0) {

    StaticJsonDocument<512> root; // el tamaño tiene que ser adecuado para el mensaje
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(root, mensaje,length);

    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
    else
    if(root.containsKey("ventilador"))  // comprobar si existe el campo/clave que estamos buscando
    { 
      //Si el valor que llega por "ventilador" es '1', el modo automatico está activo, si es '0', el manual
      control_ventilador=root["ventilador"];
      if(control_ventilador==1)
      {
        const char* mensaje_ventilador="modo AUTOMATICO activo";
        mqtt_client.publish("huerta/Modo_Control_Ventilador",mensaje_ventilador);
        baremos_temperatura=root["baremos_temperatura"];
        Serial.println(" "); Serial.print("Baremo de:   "); Serial.print(baremos_temperatura); Serial.println("ºC");
      }
      if(control_ventilador==0)
      {
        porcentaje_ventilador=root["porcentaje_ventilador"];
        const char* mensaje_ventilador="modo MANUAL activo";
        mqtt_client.publish("huerta/Modo_Control_Ventilador",mensaje_ventilador);
        Serial.println(" "); Serial.print("Intensidad al:   "); Serial.print(porcentaje_ventilador); Serial.println("%");
      }
      
    }
    else
    {
      Serial.print("Error : ");
      Serial.println("\"ventilador\" key not found in JSON");
    }

  }
  free(mensaje);
  }
//-----------------------------------------------------
//     SETUP
//-----------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Empieza setup...");
  pinMode(pinTCRT5000,INPUT);
  dht.setup(0, DHTesp::DHT22); // Connect DHT22 sensor to GPIO 0
  sprintf(ID_PLACA, "%d", ESP.getChipId());
  conecta_wifi();
  mqtt_client.setServer(mqtt_server, MQTT_PORT);
  mqtt_client.setBufferSize(512); // para poder enviar mensajes de hasta X bytes

  mqtt_client.setCallback(callback);

  conecta_mqtt();

  Serial.printf("Identificador placa: %s\n", ID_PLACA );
  Serial.printf("Termina setup en %lu ms\n\n",millis());
  // Iniciamos el timer  y buscamos actualización OTA
  checkForUpdates();
  timerManager.setInterval(10L * 60000L, checkForUpdates); // Look for update each 10'
  // Iniciamos el bus 1-Wire
  sensorDS18B20.begin(); 
  //Inicializamos pines de los reles
  pinMode(PIN_Rele1, OUTPUT);
  digitalWrite(PIN_Rele1, HIGH);
}
//-----------------------------------------------------
#define TAMANHO_MENSAJE 128
unsigned long ultimo_mensaje=0;
unsigned long ultimo_mensaje_puerta=0;
//-----------------------------------------------------
//     LOOP
//-----------------------------------------------------
void loop() {
  delay(dht.getMinimumSamplingPeriod());
  if (!mqtt_client.connected()) conecta_mqtt();
  mqtt_client.loop(); // esta llamada para que la librería recupere el control
  unsigned long ahora = millis();
  uint32_t currentFrequency;

  if (! ina219_1.begin()) {
    Serial.println("Failed to find INA219_1 chip");
    while (1) { delay(10); }
  }
  if (! ina219_2.begin()) {
    Serial.println("Failed to find INA219_2 chip");
    while (1) { delay(10); }
  }
  

if (ahora - ultimo_mensaje >= 5000) {
    char mensaje[TAMANHO_MENSAJE];
    ultimo_mensaje = ahora;
    //snprintf (mensaje, TAMANHO_MENSAJE, "Mensaje enviado desde %s en %6lu ms", ID_PLACA, ahora);
    //Serial.println(mensaje); 

    struct registro_datos datos;

    datos.IP=WiFi.localIP().toString().c_str();

    datos.movimiento = digitalRead(pinTCRT5000);
    
    datos.humedad = dht.getHumidity();
    datos.temperatura_interior = dht.getTemperature();
    
    datos.caida_tension_12V= ina219_1.getBusVoltage_V();
    datos.caida_tension_24V= ina219_2.getBusVoltage_V();
    
    Serial.printf("Temperatura interior: %f\n",datos.temperatura_interior);  
    Serial.printf("Humedad: %f\n",datos.humedad); 
    
    Serial.print("Tension VCC_12V:   "); Serial.print(datos.caida_tension_12V); Serial.println(" V");

    Serial.print("Tension VCC_24V:   "); Serial.print(datos.caida_tension_24V); Serial.println(" V");

    Serial.print("Puerta ('0' cerrada '1' abierta)= ");
    Serial.print(datos.movimiento);
    
    Serial.println("\n");

    // Mandamos comandos para toma de temperatura a los sensores (temperatura exterior)
    sensorDS18B20.requestTemperatures();
    // Leemos y mostramos los datos del sensor DS18B20
    datos.temperatura_exterior=sensorDS18B20.getTempCByIndex(0);
    Serial.print("Temperatura exterior: ");
    Serial.print(datos.temperatura_exterior);
    Serial.println(" ºC");
    float diferencia=datos.temperatura_interior-datos.temperatura_exterior;
    
    Serial.print("Diferencia:   "); Serial.print(diferencia); Serial.println(" ºC");
    //Ahora vamos a comenzar con el desarrollo del ventilador, tanto en modo automático como en modo manual:
    datos.baremo=baremos_temperatura; //El baremo es una variable que aunque en un modo no se use, no podemos eliminar:
    //Si control_ventilador==0 el modo es MANUAL
    //MODO MANUAL
    Serial.print("Ventilador (0 manual): ");
    Serial.print(control_ventilador);
    if(control_ventilador==0)
    {
      datos.porcentaje_ventilador=porcentaje_ventilador; 
      datos.PWM=(porcentaje_ventilador/100)*255;
      analogWrite(PWMPin, datos.PWM);
    }
    //Si control_ventilador==0 el modo es AUTOMATICO
    //MODO AUTOMATICO
    //Para este modo, destacamos que tenemos dos sensores, uno fuera y otro dentro, de manera que existe una diferencia de temperatura
    //esta diferencia va a ser usada para establecer la intensidad de funcionamiento del ventilador
    else if(control_ventilador==1)
    {
      if (diferencia>0)
      {
        if (diferencia>baremos_temperatura)
        {
          datos.PWM=255;
          Serial.print("PWM: ");
          Serial.print(datos.PWM);
        }
        else
        {
          datos.PWM= (diferencia*255)/(baremos_temperatura);
          Serial.print("PWM:  ");
          Serial.print(datos.PWM);
        }
        
      }
      else
      {
        datos.PWM=0;
      }
      analogWrite(PWMPin, datos.PWM);
      datos.porcentaje_ventilador=datos.PWM/255;
    }
    datos.dif=diferencia;    
    mqtt_client.publish("huerta/Informacion",serializa_JSON(datos).c_str());
  }
  if (ahora - ultimo_mensaje_puerta >= 1000) 
  {
     ultimo_mensaje_puerta=ahora;
     struct registro_datos_puerta datos_puerta;
     datos_puerta.movimiento = digitalRead(pinTCRT5000);
     mqtt_client.publish("huerta/Informacion_puerta",serializa_JSON_puerta(datos_puerta).c_str());
  }  
  timerManager.run();
}

void checkForUpdates() {
  // wait for WiFi connection
  if ((WiFi.status() == WL_CONNECTED)) {
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);
    t_httpUpdate_return ret = ESPhttpUpdate.update(wClient, OTA_URL, HTTP_OTA_VERSION);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  }
}

/***********************************************************************
   UTILITY FUNCTIONS
***********************************************************************/

void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}
