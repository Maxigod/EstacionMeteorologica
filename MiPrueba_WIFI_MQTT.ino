/*Este código lo utilizaremos como ejemplo para enviar datos por MQTT.
A nivel de ejemplo, enviaremos un dato y la recibiremos en un servidor.
Primero vamos a probar que nos podemos conectar a la red de la casa.
Posteriormente probaremos que podemos conectarnos al servidor MQTT.
Finalmente, revisaremos que podamos enviar el dato.
Elaboró: Enrique Preza 060223 v0.1
Referencia: https://www.luisllamas.es/como-usar-mqtt-en-el-esp8266-esp32/
            https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/#3
            http://www.steves-internet-guide.com/using-arduino-pubsub-mqtt-client/


*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_AHTX0.h>

#define INTENTOS_WIFI 10

//Variables globales.
Adafruit_AHTX0 aht;
bool successAHT=false;
sensors_event_t humidity, temp;
char *cad={"{\"Temperatura\":**}"};

WiFiClient ConexionWifi;
PubSubClient client(ConexionWifi); //Recordar aquí que client es un objeto que después vamos a utilizar.

int ConectarWifi(const char* ssid, const char* password){
  
  int intentosConexionWifi = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  Serial.print("Conectando a WiFI ");
  Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED){
    Serial.print(" .");
    delay(1000);
    if(intentosConexionWifi > INTENTOS_WIFI){
      Serial.println("");
      Serial.println("Falló la conexión a Wifi");
      return 0; //Regresa un 0 si falló la conexión. 
    }
    intentosConexionWifi++;
  }
  Serial.println("");
  Serial.print("Conectado a red: ");
  Serial.println(WiFi.SSID());
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
  return 1; //Regresamos 1 si la conexión fue exitosa.
}

int IniciarMQTT(const char *MQTT_BROKER_ADDRESS, const uint16_t MQTT_PORT, const char *MQTT_CLIENT_NAME){

  client.setServer(MQTT_BROKER_ADDRESS, MQTT_PORT);     //Aquí llamamos una función del objeto client.

  if(client.connect("esp32_id","esp32_user","esp32_password"))
  {
    Serial.println("Conexión exitosa a servidor MQTT");
    return 1;
  }
    return 0;
}
void completa(){
    int aux =  temp.temperature;
    cad[15] = aux/10+'0';
    cad[16] = aux%10+'0';
}
void leerTemperatura(void *pvParameters)
{
  while (1) {
    successAHT = aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    if(!successAHT){
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    // Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    // Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
    completa();
    Serial.print("Enviando el siguiente dato: ");
    Serial.println(cad);
    if(client.publish("esp32/telemetry",cad))
    {
      Serial.println("Se envío datos a servidor MQTT");
    }else {
      Serial.println("Problema para enviar datos por MQTT");
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // Espera 5 s
    
  }
}

void init_AHT20(){
  // check AHT
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    Serial.println("Don't you want me?");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

  xTaskCreate(leerTemperatura, "Lectura sensor", 2048, NULL, 1, NULL); // Crea la tarea
}

void setup() {
  //Variables del WIFI
  const char* ssid = "INFINITUMB6A5_2.4";
  const char* password = "43YBDGRu2Y";

  //Variables de MQTT
  const char *MQTT_BROKER_ADDRESS = "137.184.157.109"; //Dirección del servidor de MQTT.
  const uint16_t MQTT_PORT = 1883; //Este es el puerto default de MQTT.
  const char *MQTT_CLIENT_NAME = "esp32";

  Serial.begin(115200);

  if(!ConectarWifi(ssid,password))
  {
    Serial.println("Terminando el programa. Revise la configuración de su WiFi");
    esp_deep_sleep_start();
  }
  delay(1500); //Supongo que para estabilizar o esperar que realmente haya una conexión. Se podría revisar.


  Serial.println("Conectando a MQTT");
  
  if(!IniciarMQTT(MQTT_BROKER_ADDRESS, MQTT_PORT, MQTT_CLIENT_NAME))
  {
    Serial.println("Terminando el programa. Revise la configuración del Broker MQTT");
    esp_deep_sleep_start();
  }

  // Iniciar tarea de leer AHT 20 y enviar lectura a Broker MQTT
  init_AHT20();

  // char *cadena_datos = "{\"Temperatura\":18}";
  // Serial.print("Enviando el siguiente dato: ");
  // Serial.println(cadena_datos);

  // if(client.publish("esp32/telemetry",cadena_datos))
  // {
  // Serial.println("Se envió datos a servidor MQTT");
  // }else {
  //   Serial.println("Problema para enviar datos por MQTT");
  // }  
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
