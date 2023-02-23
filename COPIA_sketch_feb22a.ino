#include <Adafruit_AHTX0.h>
#include <LiquidCrystal_I2C.h>

Adafruit_AHTX0 aht;

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
    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
    
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Espera 1 segundo
    
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
    vTaskDelay(pdMS_TO_TICKS(500)); // Espera 0.5 segundos
    lcd.clear();
  }
}

void setup()
{
  Serial.begin(9600);

  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();

  // check AHT
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

  xTaskCreate(task1, "Lectura sensor ", 2048, NULL, 1, NULL); // Crea la tarea
  xTaskCreate(task2, "Escribir en serial", 1024, NULL, 1, NULL); // Tarea 2
}

void loop()
{
  // No se utiliza en este ejemplo.
}
