#include <WiFi.h>              // Handles Wi-Fi connectivity for the ESP32
#include <HTTPClient.h>        // Enables sending HTTP POST requests to the server
#include <Wire.h>              // I2C communication library for sensors
#include <Adafruit_BMP280.h>   // Library for the BMP280 barometric pressure sensor
#include "DFRobot_ENS160.h"    // Library for the ENS160 air quality sensor
#include <Adafruit_AHTX0.h>    // Library for the AHT20/AHT10 temperature & humidity sensor
#include <Adafruit_NeoPixel.h> // Library for the WS2812 RGB LED (status indicator)
#include <ArduinoJson.h>       // Added for Structured Payloads (JSON formatting)

//  CONFIGURATION 
const char* ssid = "EDA-IOT";                 // Wi-Fi network SSID
const char* password = "3aB1J27M";            // Wi-Fi network password
const char* serverUrl = "http://10.150.46.120/update"; // Endpoint for data ingestion
const char* API_KEY = "EDA_Secure_Key_2024";  // Security key for API authentication


// NODE 1 SETTINGS
const char* nodeID = "node1";       // Unique identifier for this specific hardware node
#define SOUND_ATTENUATION ADC_6db   // Sets analogue-to-digital converter (ADC) gain for the sound pin
float tempOffset = -6;              // Heavy offset due to ENS160 heater affecting ambient temperature
int cycleTime = 2000;               // 2.0 second update interval between transmissions

// BOOTUP WARM-UP SETTINGS 
// 180000 ms = 3 minutes. The ENS160 internal hotplate requires this time to stabilise.
const unsigned long WARMUP_DURATION = 180000; 


// HARDWARE PINS 
#define SOUND_PIN 0                 // Analogue pin connected to the sound sensor
#define TRIG_PIN 2                  // Digital pin to trigger the ultrasonic sensor
#define ECHO_PIN 3                  // Digital pin to read the ultrasonic sensor echo
#define CALIB_BTN 18                // Digital pin for the physical calibration button
#define SDA_PIN 15                  // I2C Data pin
#define SCL_PIN 14                  // I2C Clock pin

// NEOPIXEL SETUP 
// Initialise the NeoPixel string (1 pixel, connected to pin 8)
Adafruit_NeoPixel pixels(1, 8, NEO_GRB + NEO_KHZ800);

// Helper function to set the colour of the status NeoPixel
void setPixel(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show(); // Push the updated colour to the hardware
}

// Instantiate sensor objects
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
DFRobot_ENS160_I2C ENS160(&Wire, (uint8_t)0x53); // ENS160 requires the I2C address (0x53)

// Timing and state variables
unsigned long lastCycle = 0;             // Tracks the last time data was transmitted
volatile bool flag_calibrate = false;    // Volatile flag to safely share state between the ISR and main loop

//  SOUND LATCHING VARIABLES 
int peakSoundEnergy = 0;                 // Holds the maximum sound amplitude detected within a cycle
float currentMidpoint = 2048.0;          // Running midpoint of the sound waveform (12-bit ADC: max 4095)
float baseline = 2048.0;                 // Fixed baseline to calculate amplitude against

// Interrupt Service Routine (ISR) triggered by pressing the calibration button
void IRAM_ATTR handleCalibPress() {
  flag_calibrate = true; // Set flag to true to notify the main loop
}

//  MATHEMATICAL HUMIDITY COMPENSATION (Magnus-Tetens Formula) 
// Calculates the true relative humidity, compensating for the artificial temperature offset
float calculateTrueRH(float t_sensor, float rh_sensor, float t_ambient) {
  // Calculate saturation vapour pressure at the sensor's temperature
  float ps_sensor = 6.112 * exp((17.67 * t_sensor) / (t_sensor + 243.5));
  // Calculate saturation vapour pressure at the true ambient temperature
  float ps_ambient = 6.112 * exp((17.67 * t_ambient) / (t_ambient + 243.5));
  // Adjust the relative humidity based on the ratio of vapour pressures
  float true_rh = rh_sensor * (ps_sensor / ps_ambient);

  // Constrain the output to logical percentages (0-100%)
  if (true_rh > 100.0) true_rh = 100.0;
  if (true_rh < 0.0) true_rh = 0.0;

  return true_rh;
}

// Function to get a reliable distance measurement using an ultrasonic burst technique
float getDistanceBurst() {
  float readings[5];
  // Take 5 distinct measurements to form a statistical sample
  for (int i = 0; i < 5; i++) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);   // Ensure trigger is low
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10); // Send 10 microsecond acoustic pulse
    digitalWrite(TRIG_PIN, LOW);
    
    // Measure how long it takes for the echo to return (timeout after 30ms)
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    // If timeout (duration == 0), default to max distance (400 cm), else calculate distance
    readings[i] = (duration == 0) ? 400.0 : (duration * 0.034 / 2); 
    delay(10); // Brief pause before next ping
  }
  
  // Sort the array of 5 readings in ascending order (simple bubble sort)
  for(int i=0; i<4; i++) {
    for(int j=i+1; j<5; j++) {
      if(readings[i] > readings[j]) {
        float temp = readings[i]; readings[i] = readings[j]; readings[j] = temp;
      }
    }
  }
  
  // Discard the highest and lowest readings (outliers) and return the average of the middle three
  return (readings[1] + readings[2] + readings[3]) / 3.0;
}

void setup() {
  Serial.begin(115200); // Initialise serial communication for debugging
  delay(50);
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialise the status NeoPixel 
  pixels.begin();       
  setPixel(150, 0, 0); // Start Red until connected to WiFi

  // Initialise I2C bus with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Configure sensor pins
  pinMode(SOUND_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Configure calibration button with internal pull-up and attach hardware interrupt
  pinMode(CALIB_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CALIB_BTN), handleCalibPress, FALLING); // Trigger when button pulls voltage to ground
  
  // Configure ESP32 analogue settings
  analogReadResolution(12); // Set ADC resolution to 12 bits (0-4095 range)
  analogSetPinAttenuation(SOUND_PIN, SOUND_ATTENUATION); // Set signal attenuation for the mic

  // Initialise BMP280 Pressure Sensor. Halt and stay red if it fails.
  if (!bmp.begin(0x76)) {
    setPixel(150, 0, 0);  
    while (1) { delay(10); } 
  }

  // Initialise AHT20 Temp/Humidity Sensor. Halt and stay red if it fails.
  if (!aht.begin()) { 
    setPixel(150, 0, 0);  
    while (1) { delay(10); } 
  }

  // Initialise ENS160 Gas Sensor. Flash yellow whilst trying to connect.
  while (ENS160.begin() != 0){
    setPixel(150, 150, 0); // Yellow if retrying air sensor
    delay(1000);
  }
  ENS160.setPWRMode(ENS160_STANDARD_MODE); // Set sensor to standard operation mode

  // Initial Sound Baseline Calibration
  long sum = 0;
  for(int i=0; i<300; i++) { sum += analogRead(SOUND_PIN); delay(2); }
  baseline = sum / 300.0;       // Average of 300 readings
  currentMidpoint = baseline;   // Initialise the running midpoint

  // Automatically Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

void loop() {
  // FAST LOOP (Executes as fast as possible to catch sound peaks) 
  int rawSound = analogRead(SOUND_PIN); // Read analogue sound level
  int amplitude = abs(rawSound - (int)baseline); // Calculate distance from the DC baseline
  
  // Latch onto the highest amplitude detected during this cycle
  if (amplitude > peakSoundEnergy) {
      peakSoundEnergy = amplitude;
  }
  
  // Slowly adjust the running midpoint to account for DC drift over time
  currentMidpoint = (currentMidpoint * 0.999) + (rawSound * 0.001); 

  // TIMED CYCLE (Executes only once every 'cycleTime' milliseconds) 
  if (millis() - lastCycle >= cycleTime) { 
    lastCycle = millis(); // Reset the cycle timer

    digitalWrite(LED_BUILTIN, HIGH); // Flash the built-in LED to indicate data processing
    
    // Check if the ENS160 is still in its mandatory 3-minute warm-up phase
    bool isWarmingUp = (millis() < WARMUP_DURATION);
    
    // Update the NeoPixel status colour
    if (isWarmingUp) {
      setPixel(100, 40, 0);  // Orange: Warming up
    } else {
      setPixel(0, 0, 60);    // Blue: Processing data
    }

    // Synchronise the baseline to the slowly drifting midpoint
    baseline = currentMidpoint;

    // Gather basic sensor readings
    float distance = getDistanceBurst();
    sensors_event_t h_ev, t_ev;
    bool aht_ok = aht.getEvent(&h_ev, &t_ev);
    
    float trueTemp = 0.0;
    float trueHumidity = 0.0;

    if (aht_ok) {
        // Apply manual calibration offset for temperature
        trueTemp = t_ev.temperature + tempOffset;
        // Calculate the compensated true humidity using the Magnus-Tetens formula
        trueHumidity = calculateTrueRH(t_ev.temperature, h_ev.relative_humidity, trueTemp);
        
        // Feed the uncompensated (raw internal) temperature and humidity into the ENS160 
        // for accurate internal gas compensation
        ENS160.setTempAndHum(t_ev.temperature, h_ev.relative_humidity);
    }

    // Initialise gas reading variables to zero
    int txAqi  = 0; // Air Quality Index
    int txTvoc = 0; // Total Volatile Organic Compounds
    int txEco2 = 0; // Equivalent CO2

    // Only fetch gas readings if the sensor has finished warming up
    if (!isWarmingUp) {
        txAqi  = ENS160.getAQI();
        txTvoc = ENS160.getTVOC();
        txEco2 = ENS160.getECO2();
    } else {
        Serial.printf("[WARMUP] Stabilizing gas sensor. %lu seconds remaining...\n", (WARMUP_DURATION - millis()) / 1000);
    }

    // NETWORK TRANSMISSION 
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverUrl); // Prepare POST to the endpoint
      
      // Switch from URL encoded to JSON
      http.addHeader("Content-Type", "application/json");
      
      // Determine if the calibration button was pressed, then clear the flag
      int sendCalFlag = flag_calibrate ? 1 : 0;
      flag_calibrate = false; 
      
      // JSON Serialization: Construct the payload dictionary
      JsonDocument doc;
      doc["key"] = API_KEY;
      doc["id"] = nodeID;
      doc["t"] = trueTemp;
      doc["h"] = trueHumidity;
      doc["p"] = bmp.readPressure() / 100.0F; // Convert Pascals to hPa (millibars)
      doc["aqi"] = txAqi;
      doc["tvoc"] = txTvoc;
      doc["eco2"] = txEco2;
      doc["snd"] = peakSoundEnergy; // Send the latched maximum sound amplitude
      doc["dst"] = distance;
      doc["cal"] = sendCalFlag;

      // Serialize JSON document into a String for transmission
      String jsonPayload;
      serializeJson(doc, jsonPayload);

      // Send the POST request with the JSON payload
      int httpResponseCode = http.POST(jsonPayload);
      http.end(); // Free network resources
      
      // Provide visual feedback via the NeoPixel
      if (httpResponseCode > 0) {
        if (!isWarmingUp) setPixel(0, 40, 0); // Clean Green: Successfully transmitted
      } else {
        setPixel(80, 0, 0); // Red on Server Error
      }
      
    } else {
      setPixel(80, 0, 0); // Red on Disconnect
      WiFi.begin(ssid, password); // Attempt to reconnect
    }
    
    // Reset variables for the next cycle
    peakSoundEnergy = 0;  // Clear the latched sound peak
    digitalWrite(LED_BUILTIN, LOW); // Turn off the processing indicator
  }
}