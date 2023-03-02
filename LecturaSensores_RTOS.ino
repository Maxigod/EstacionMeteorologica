#include <Adafruit_AHTX0.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp; // I2C

const int adcPin = 34; // Pin del ADC en el ESP32
// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
sensors_event_t humidity, temp;

void task1(void *pvParameters)
{
  while (1) {
    aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
//    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
//    Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Espera .1 segundo
    
  }
}

void task2(void *pvParameters)
{
  while(1){
    // set cursor to first column, first row
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temp.temperature,3);
    lcd.print(" *C");
    // set cursor to first column, second row
    lcd.setCursor(0, 1);
    lcd.print("Hum: ");
    lcd.print(humidity.relative_humidity,3);
    lcd.print(" %");
    // set cursor to first column, third row
    lcd.setCursor(0, 2);
    lcd.print("Press: ");
    lcd.print(bmp.readPressure(),3);
    lcd.print(" Pa");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1 segundo
    
    lcd.clear();
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Medir temperatura y mostrarla con RTOS");
  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();

  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // check AHT
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    Serial.println("Don't you want me?");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

  xTaskCreate(task1, "Lectura sensor ", 2048, NULL, 1, NULL); // Crea la tarea
  xTaskCreate(task2, "Escribir en serial", 2048, NULL, 1, NULL); // Tarea 2
}

void loop()
{
  // No se utiliza en este ejemplo.
}
