#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "DFRobot_ENS160.h"
#include <Adafruit_AHTX0.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h> // Added for Structured Payloads

// --- CONFIGURATION ---
const char* ssid = "EDA-IOT";
const char* password = "3aB1J27M";
const char* serverUrl = "http://10.150.46.105/update";
const char* API_KEY = "EDA_Secure_Key_2024";

// --------------------------------------------------------
// [NODE 2 SETTINGS]
const char* nodeID = "node2";       
#define SOUND_ATTENUATION ADC_6db   
float tempOffset = -2;            
float humidOffset = 6.0;            
int cycleTime = 2300;               
float alpha = 0.25;                 

const unsigned long WARMUP_DURATION = 180000; 
// --------------------------------------------------------

#define SOUND_PIN 0
#define TRIG_PIN 2
#define ECHO_PIN 3
#define CALIB_BTN 18 
#define SDA_PIN 15
#define SCL_PIN 14

Adafruit_NeoPixel pixels(1, 8, NEO_GRB + NEO_KHZ800);

void setPixel(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
DFRobot_ENS160_I2C ENS160(&Wire, (uint8_t)0x53);

unsigned long lastCycle = 0;
volatile bool flag_calibrate = false;

// --- SOUND LATCHING VARIABLES ---
int peakSoundEnergy = 0; 
float currentMidpoint = 2048.0;
float baseline = 2048.0;

// --- HISTORICAL TRACKING REGISTERS FOR EMA SMOOTHING ---
float smoothTemp = 25.0;
float smoothHumidity = 50.0;
float smoothPressure = 1013.25;
bool firstRun = true;

void IRAM_ATTR handleCalibPress() {
  flag_calibrate = true; 
}

float getDistanceBurst() {
  float readings[5];
  for (int i = 0; i < 5; i++) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    readings[i] = (duration == 0) ? 400.0 : (duration * 0.034 / 2); 
    delay(10); 
  }
  for(int i=0; i<4; i++) {
    for(int j=i+1; j<5; j++) {
      if(readings[i] > readings[j]) {
        float temp = readings[i]; readings[i] = readings[j]; readings[j] = temp;
      }
    }
  }
  return (readings[1] + readings[2] + readings[3]) / 3.0;
}

void setup() {
  Serial.begin(115200);
  delay(50);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pixels.begin();       
  setPixel(150, 0, 0); 

  Wire.begin(SDA_PIN, SCL_PIN);
  
  pinMode(SOUND_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(CALIB_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CALIB_BTN), handleCalibPress, FALLING);
  
  analogReadResolution(12);
  analogSetPinAttenuation(SOUND_PIN, SOUND_ATTENUATION); 

  if (!bmp.begin(0x76)) {
    setPixel(150, 0, 0);  
    while (1) { delay(10); } 
  }

  if (!aht.begin()) { 
    setPixel(150, 0, 0);  
    while (1) { delay(10); } 
  }

  while (ENS160.begin() != 0){
    setPixel(150, 150, 0); 
    delay(1000);
  }
  ENS160.setPWRMode(ENS160_STANDARD_MODE);

  long sum = 0;
  for(int i=0; i<300; i++) { sum += analogRead(SOUND_PIN); delay(2); }
  baseline = sum / 300.0;
  currentMidpoint = baseline;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

void loop() {
  int rawSound = analogRead(SOUND_PIN);
  int amplitude = abs(rawSound - (int)baseline); 
  
  if (amplitude > peakSoundEnergy) {
      peakSoundEnergy = amplitude;
  }
  
  currentMidpoint = (currentMidpoint * 0.999) + (rawSound * 0.001); 

  if (millis() - lastCycle >= cycleTime) { 
    lastCycle = millis();

    digitalWrite(LED_BUILTIN, HIGH); 
    
    bool isWarmingUp = (millis() < WARMUP_DURATION);
    
    if (isWarmingUp) {
      setPixel(100, 40, 0); 
    } else {
      setPixel(0, 0, 60);   
    }

    baseline = currentMidpoint;

    float distance = getDistanceBurst();
    sensors_event_t h_ev, t_ev;
    bool aht_ok = aht.getEvent(&h_ev, &t_ev);
    float rawPressure = bmp.readPressure() / 100.0F;

    if (aht_ok) {
        float rawTempAdjusted = t_ev.temperature + tempOffset;
        float rawHumidity = h_ev.relative_humidity + humidOffset;
        rawHumidity = constrain(rawHumidity, 0.0f, 100.0f);

        if (firstRun) {
            smoothTemp = rawTempAdjusted;
            smoothHumidity = rawHumidity;
            smoothPressure = rawPressure;
            firstRun = false;
        } else {
            smoothTemp = (alpha * rawTempAdjusted) + ((1.0f - alpha) * smoothTemp);
            smoothHumidity = (alpha * rawHumidity) + ((1.0f - alpha) * smoothHumidity);
            smoothPressure = (alpha * rawPressure) + ((1.0f - alpha) * smoothPressure);
        }

        ENS160.setTempAndHum(smoothTemp, smoothHumidity);
    }

    int txAqi  = 0;
    int txTvoc = 0;
    int txEco2 = 0;

    if (!isWarmingUp) {
        txAqi  = ENS160.getAQI();
        txTvoc = ENS160.getTVOC();
        txEco2 = ENS160.getECO2();
    } else {
        Serial.printf("[WARMUP] Stabilizing gas sensor. %lu seconds remaining...\n", (WARMUP_DURATION - millis()) / 1000);
    }

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");
      
      int sendCalFlag = flag_calibrate ? 1 : 0;
      flag_calibrate = false; 
      
      JsonDocument doc;
      doc["key"] = API_KEY;
      doc["id"] = nodeID;
      doc["t"] = smoothTemp;
      doc["h"] = smoothHumidity;
      doc["p"] = smoothPressure;
      doc["aqi"] = txAqi;
      doc["tvoc"] = txTvoc;
      doc["eco2"] = txEco2;
      doc["snd"] = peakSoundEnergy;
      doc["dst"] = distance;
      doc["cal"] = sendCalFlag;

      String jsonPayload;
      serializeJson(doc, jsonPayload);
               
      int httpResponseCode = http.POST(jsonPayload);
      http.end();
      
      if (httpResponseCode > 0) {
        if (!isWarmingUp) setPixel(0, 40, 0); 
      } else {
        setPixel(80, 0, 0); 
      }
      
    } else {
      setPixel(80, 0, 0); 
      WiFi.begin(ssid, password); 
    }
    
    peakSoundEnergy = 0; 
    digitalWrite(LED_BUILTIN, LOW); 
  }
}