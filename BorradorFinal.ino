#include <WiFi.h>
#include <PubSubClient.h> // Para MQTT
#include <Adafruit_AHTX0.h> // Para sensor de temperatura y humedad
#include <Wire.h> // Para I2C
#include <Adafruit_BMP280.h> // Para sensor de presión
#include <BH1750.h> // Para sensor de luz

// <E-paper>
// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>

// ESP32 SS=5,SCL(SCK)=18,SDA(MOSI)=23,BUSY=15,RST=2,DC=0

// 2.13'' EPD Module
GxEPD2_BW<GxEPD2_213_BN, GxEPD2_213_BN::HEIGHT> display(GxEPD2_213_BN(/*CS=5*/ 5, /*DC=*/0, /*RST=*/2, /*BUSY=*/15));  // DEPG0213BN 122x250, SSD1680
// </E-paper>

#define INTENTOS_BMP280 4
#define INTENTOS_AHT20 4
#define INTENTOS_BH1750 4
#define INTENTOS_WIFI 10
#define INTENTOS_MQTT 5
#define PIN_ANEMOMETRO 24

Adafruit_AHTX0 aht;   // Objeto para trabajar con el sensor de temperatura y humedad
Adafruit_BMP280 bmp;  // Objeto para trabajar con el sensor de presión
BH1750 lightMeter;    // Objeto para trabajar con el sensor de luz

sensors_event_t humidity, temp;

WiFiClient ConexionWifi;
PubSubClient client(ConexionWifi);

//Variables del WIFI
const char *ssid = "AlumnosUP";
//ssid de la red de la universidad: AlumnosUP
const char *password = "";

//Variables de MQTT
const char *MQTT_BROKER_ADDRESS = "20.168.52.21";  //Dirección del servidor de MQTT.
const uint16_t MQTT_PORT = 1883;                   //Este es el puerto default de MQTT.
const char *MQTT_CLIENT_NAME = "esp32";

int ConectarWifi(const char *ssid, const char *password) {

  int intentosConexionWifi = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFI ");
  Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(" .");
    delay(1000);
    if (intentosConexionWifi > INTENTOS_WIFI) {
      Serial.println("");
      Serial.println("Falló la conexión a Wifi");
      return 0;  //Regresa un 0 si falló la conexión.
    }
    intentosConexionWifi++;
  }
  Serial.println("");
  Serial.print("Conectado a red: ");
  Serial.println(WiFi.SSID());
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
  return 1;  //Regresamos 1 si la conexión fue exitosa.
}

int IniciarMQTT(const char *MQTT_BROKER_ADDRESS, const uint16_t MQTT_PORT, const char *MQTT_CLIENT_NAME) {
  client.setServer(MQTT_BROKER_ADDRESS, MQTT_PORT);  //Aquí llamamos una función del objeto client.

  int k = 0;
  while (!client.connected() && k < INTENTOS_MQTT) {
    Serial.println("MQTT no conectado");
    Serial.print("Intentando conexión a MQTT...");

    if (client.connect("esp32", "equipo3", "equipo3")) {
      Serial.println("Conexión exitosa a servidor MQTT");
      return 1; // regresa 1 en caso de éxito
    }
    k += 1;
    delay(10000); 
  }
  return 0; // regresa 0 en caso de fracaso
}

volatile int cont_anem=0;

// Se incrementa el contador cuando el encoder registra un paso
void IRAM_ATTR isr(){
  cont_anem++;
}

// Iniciar comunicación con los sensores del clima
void iniciarSensores() {
  pinMode(PIN_ANEMOMETRO, INPUT_PULLUP); 
  attachInterrupt(PIN_ANEMOMETRO, isr, RISING); // enlazar la función isr con la interrupción del flanco de subida en el pin del anemómetro
  
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();

  // Iniciar sensor BMP280, presión
  int i = INTENTOS_BMP280;
  unsigned status;
  do {
    status = bmp.begin(0x76);
    if (!status) {
      Serial.println("No pudo iniciar el sensor BMP280");
      if (i > 1) {
        Serial.println("Intentando conexion sensor BMP280...");
        displayText("Intentando conexion sensor BMP280...");
      }
      i -= 1;
      delay(10000);
    }
  } while (i > 0 && status == 0);
  // Validar sensor
  if (status) {
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    displayText("Conectado sensor BMP280");
    delay(1000);
  } else {
    Serial.println("ERROR: No se pudo conectar a sensor BMP280");
    displayText("ERROR: No se pudo conectar a sensor BMP280");
    delay(1000);
  }

  // Iniciar sensor AHT20, temperatura y humedad
  i = INTENTOS_AHT20;
  bool ready;
  do {
    ready = aht.begin();
    if (!ready) {
      Serial.println("No pudo iniciar el sensor AHT20");
      if (i > 1) {
        Serial.println("Intentando conexion sensor AHT20...");
        displayText("Intentando conexion sensor AHT20...");
      }
      i -= 1;
      delay(10000);
    }
  } while (i > 0 && !ready);
  // Validar sensor
  if (!ready) {
    Serial.println("ERROR: No se pudo conectar a sensor AHT20");
    displayText("ERROR: No se pudo conectar a sensor AHT20");
    delay(1000);
  }else{
    displayText("Conectado sensor AHT20");
    delay(1000);
  }

  // Iniciar sensor BH1750, luz
  i = INTENTOS_BH1750;
  do {
    ready = lightMeter.begin();
    if (!ready) {
      Serial.println("No pudo iniciar el sensor BH1750");
      if (i > 1) {
        Serial.println("Intentando conexion sensor BH1750...");
        displayText("Intentando conexion sensor BH1750...");
      }
      i -= 1;
      delay(10000);
    }
  } while (i > 0 && !ready);
  // Validar sensor
  if (!ready) {
    Serial.println("ERROR: No se pudo conectar a sensor BH1750");
    displayText("ERROR: No se pudo conectar a sensor BH1750");
    delay(1000);
  }else{
    displayText("Conectado sensor BH1750");
  }
}

double leer_anemometro(){
  //Reiniciar contador
  cont_anem=0;
  //comienza a contar en el delay
  vTaskDelay(pdMS_TO_TICKS(1000));
  //A continuación, mandar el dato de cont al servidor como velocidad
  double vel = 2*(3.1416)*(0.05)*(cont_anem);
  return vel;
}

// Funcion que se ejecuta recurrentemente como tarea de RTOS
// Aquí se validan y leen los sensores, se validan las conexiones con WiFi y MQTT, y se envían los datos al servidor
void leerSensores(void *pvParameters) {
  float tempr=0.0, hum=0.0, pres=0.0, luz=0.0,vel_viento = -1.0;
  int k = 0;
  while (1) {
    vel_viento=leer_anemometro();

    if (!aht.getEvent(&humidity, &temp)) {  // populate temp and humidity objects with fresh data
      Serial.println("Error al leer el sensor AHT20");
      displayText("Error al leer el sensor AHT20");
      vTaskDelay(pdMS_TO_TICKS(1000));
      tempr=-273.0;
      hum=-1.0;
    }
    else{
      tempr = temp.temperature;
      hum = humidity.relative_humidity;
    }

    pres = bmp.readPressure();
    luz = lightMeter.readLightLevel();
    // TODO
    //vel_viento=-1.0;

    if (isnan(pres)) {
      Serial.println("Error al leer el sensor BMP280");
      displayText("Error al leer el sensor BMP280");
      vTaskDelay(pdMS_TO_TICKS(1000));
      pres=-1.0;
    }

    if (luz < 0) {
      Serial.println("Error al leer el sensor BH1750");
      displayText("Error al leer el sensor BH1750");
      vTaskDelay(pdMS_TO_TICKS(1000));
      luz=-1.0;
    }

    if (vel_viento < 0) {
      Serial.println("Error al leer el sensor de viento");
      displayText("Error al leer el sensor de viento");
      vTaskDelay(pdMS_TO_TICKS(1000));
      vel_viento=-1.0;
    }

    Serial.print("Temperatura: ");
    Serial.print(tempr);
    Serial.println(" grados C");
    Serial.print("Humedad: ");
    Serial.print(hum);
    Serial.println("% rH");
    Serial.print("Presión: ");
    Serial.print(pres);
    Serial.println(" Pa");
    Serial.print("Luz: ");
    Serial.print(luz);
    Serial.println(" lx");
    Serial.print("Vel. viento: ");
    Serial.print(vel_viento);
    Serial.println(" km/h");

    // No funciona cuando recién carga el programa
    displayText("\nTemperatura: " + String(tempr) + String(" C") + String("\nHumedad: ") + String(hum) + String("% rH") + String("\nLuz: ") + String(luz) + String(" lx") + String("\nPresión: ") + String(pres) + String(" Pa") + String("\nVel. viento: ") + String(vel_viento) + String(" km/h"));
    display.hibernate();

    String message = "{\"Temperature\":" + String(tempr) + ",\"Humidity\":" + String(hum) + ",\"Pressure\":" + String(pres) + ",\"Light\":" + String(luz) + ",\"WindSpeed\":" + String(vel_viento) + "}";
    Serial.println("Enviando JSON:");
    Serial.println(message);

    // Longitud del string + 1 por el caracter nulo
    int str_len = message.length() + 1;
    // char* del tamaño de la cadena + 1
    char out[str_len];
    // copiar el String a char* para usarlo en la función de publish
    message.toCharArray(out, str_len);

    k = 0;

    if(WiFi.status()!=WL_CONNECTED){
      WiFi.begin(ssid,password);
      Serial.println("Sin conexión a WiFi");
      displayText("Sin conexion a WiFi");
      delay(1000);
    }

    while (WiFi.status()!=WL_CONNECTED && k<INTENTOS_WIFI) {
      Serial.println("Intentanto conectar a WiFi...");
      displayText("Intentanto conectar a WiFi...");
      delay(10000);
      k+=1;
    }
    if(WiFi.status()!=WL_CONNECTED){
      Serial.println("Error al conectar a WiFi");
      displayText("Error al conectar a WiFi");
      delay(1000);
    }else{
      displayText("Conexion exitosa a WiFi");
      delay(1000);
    }

    k=0;

    while (!client.connected() && k < INTENTOS_MQTT) {
      Serial.println("Sin conexión a broker MQTT.");
      Serial.print("Intentando conexión con broker MQTT");
      displayText("Intentando conexion con broker MQTT");

      if (client.connect("esp32", "equipo3", "equipo3")) {
        Serial.println("Conexión exitosa con broker MQTT");
        displayText("Conexion exitosa con broker MQTT");
        delay(1000);
        break;
      }
      k += 1;
      delay(10000);
    }
    if (client.connected()) {
      if (client.publish("estacion", out)) {
        Serial.println("Se enviaron datos a servidor MQTT");
        displayText("Se enviaron datos a servidor MQTT");
      } else {
        Serial.println("Problema para enviar datos por MQTT");
        displayText("Problema para enviar datos por MQTT");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(60000));  // Espera 60 s
  }
}


void setup() {
  // put your setup code here, to run once:
  display.init(115200, true, 50, false); // Iniciar la epaper
  displayText("Hello world!");
  
  Serial.begin(115200);

  iniciarSensores();

  Serial.println("Conectando a WiFi");

  if (!ConectarWifi(ssid, password)) {
    Serial.println("Terminando el programa. Revise la configuración de su WiFi");
    displayText("Fallo la conexion con WiFi. Terminando programa.");
    esp_deep_sleep_start();
  }
  delay(1500);

  Serial.print("WiFi status: ");
  Serial.println(WiFi.status());

  Serial.println("Conectando a MQTT");

  if (!IniciarMQTT(MQTT_BROKER_ADDRESS, MQTT_PORT, MQTT_CLIENT_NAME)) {
    Serial.println("Terminando el programa. Revise la configuración del Broker MQTT");
    displayText("Fallo la conexion con MQTT. Terminando programa.");
    esp_deep_sleep_start();
  }

  client.setCallback(recibir_mensajes);
  client.subscribe("estacion/resumen_temp");

  // xTaskCreate(leerSensores, "Lectura sensores", 2048, NULL, 1, NULL);  // Crea la tarea
}
// callback cuando el cliente mqtt recibe un nuevo mensaje
void recibir_mensajes(char* topic, byte* payload, unsigned int length)
{
   // Convertir el mensaje recibido a una cadena de caracteres.
  char mensaje[length + 1];
  for (int i = 0; i < length; i++) {
    mensaje[i] = (char)payload[i];
  }
  mensaje[length] = '\0';

  // Imprimir el mensaje por el puerto serial.
  Serial.println("Mensaje recibido:");
  Serial.println(mensaje);
}
// Función para mostrar una cadena de texto en la epaper
void displayText(String text) {
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(text, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center the bounding box by transposition of the origin:
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y);
    display.print(text);

  } while (display.nextPage());
}
void loop() {
  client.loop();
  float vel_viento=leer_anemometro();
  Serial.print("Vel viento: ");
  Serial.println(vel_viento);
}
