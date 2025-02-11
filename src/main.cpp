#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h> //Влажность и температура 
#include <Adafruit_BMP280.h> //Давление
#include <ESP8266WiFi.h>
#include <espnow.h>

// Create objects for sensors BMP280 - 0x77 and AHT20 - 0x38
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

uint8_t broadcastAddress1[] = {0x34, 0x98, 0x7A, 0xB9, 0xF2, 0x39};

typedef struct climate {
  float temperature_esp8266;
  float humidity_esp8266;
  float pressure_esp8266;
} climate;
 
climate Data_climate;

bool status_AHT20;
bool status_BMP280;
bool status_signal;

#define MY_PERIOD 300000  // период в мс
uint32_t tmr1;         // переменная таймера

typedef struct synhron {
  int y;
} synhron;
synhron connect;

void OnDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  memcpy(&connect, data, sizeof(connect));
  Serial.println(connect.y);
}

// Проверка отправки данных
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0){
    Serial.println("Delivery success");
    status_signal=1;
    digitalWrite(2,HIGH);
  }
  else{
    Serial.println("Delivery fail");
    status_signal=0;
    digitalWrite(2,LOW);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2,OUTPUT);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (! aht.begin()) {
    Serial.println(F("AHT20 not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    status_AHT20 = 1;
    delay(3000);
  }
  Serial.println(F("AHT20 OK"));

  
  if(!bmp.begin()) 
  { // Если датчик BMP280 не найден
    status_BMP280 = 1;
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    delay(3000);
  }
  Serial.println(F("BMP280 OK"));

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Режим работы
                Adafruit_BMP280::SAMPLING_X2,     // Точность изм. температуры
                Adafruit_BMP280::SAMPLING_X16,    // Точность изм. давления
                Adafruit_BMP280::FILTER_X16,      // Уровень фильтрации
                Adafruit_BMP280::STANDBY_MS_500); // Период просыпания, мСек

  // Инициализируем протокол ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  esp_now_register_send_cb(OnDataSent);
  
  esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  
  esp_now_register_recv_cb(OnDataRecv);
  //tmr1=millis();
}

void loop() {
  while (millis()-tmr1 <= MY_PERIOD){
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    if (status_AHT20 == 1){
      Data_climate.temperature_esp8266=0;
      Data_climate.humidity_esp8266=0;
    }else{
      Data_climate.temperature_esp8266=temp.temperature;
      Data_climate.humidity_esp8266=humidity.relative_humidity;
    }

    if (status_BMP280 == 1){
      Data_climate.pressure_esp8266=0;
    }else{
      Data_climate.pressure_esp8266=bmp.readPressure();
    }

    esp_now_send(0, (uint8_t *) &Data_climate, sizeof(Data_climate));
  }

    ESP.deepSleep(3300e6);  //55 минут
  
}






