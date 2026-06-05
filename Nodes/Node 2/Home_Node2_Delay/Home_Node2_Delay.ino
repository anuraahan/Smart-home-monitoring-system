 
#include <WiFi.h>              // Handles Wi-Fi connectivity for the ESP32
#include <HTTPClient.h>        // Enables sending HTTP POST requests to the server
#include <Wire.h>              // I2C communication library for sensors
#include <Adafruit_BMP280.h>   // Library for the BMP280 barometric pressure sensor
#include "DFRobot_ENS160.h"    // Library for the ENS160 air quality sensor
#include <Adafruit_AHTX0.h>    // Library for the AHT20/AHT10 temperature & humidity sensor
#include <Adafruit_NeoPixel.h> // Library for the WS2812 RGB LED (status indicator)
#include <ArduinoJson.h>       // Added for Structured Payloads (JSON formatting)

// CONFIGURATION 
const char* ssid = "EDA-IOT";                 // Wi-Fi network SSID
const char* password = "3aB1J27M";            // Wi-Fi network password
const char* serverUrl = "http://10.150.46.120/update"; // Endpoint for data ingestion
const char* API_KEY = "EDA_Secure_Key_2024";  // Security key for API authentication


// NODE 2 SETTINGS
const char* nodeID = "node2";       // Unique identifier for this specific hardware node
#define SOUND_ATTENUATION ADC_6db   // Sets analogue-to-digital converter (ADC) gain for the sound pin
float tempOffset = -2;              // Calibration offset for temperature readings (in Celsius)
float humidOffset = 6.0;            // Calibration offset for humidity readings (in %)
int cycleTime = 2300;               // Interval between data transmissions (in milliseconds)
float alpha = 0.25;                 // Smoothing factor for the Exponential Moving Average (EMA) filter

const unsigned long WARMUP_DURATION = 180000; // 3-minute warm-up time required for the ENS160 gas sensor


// HARDWARE PINS 
#define SOUND_PIN 0                 // Analogue pin connected to the sound sensor
#define TRIG_PIN 2                  // Digital pin to trigger the ultrasonic sensor
#define ECHO_PIN 3                  // Digital pin to read the ultrasonic sensor echo
#define CALIB_BTN 18                // Digital pin for the physical calibration button
#define SDA_PIN 15                  // I2C Data pin
#define SCL_PIN 14                  // I2C Clock pin

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

// SOUND LATCHING VARIABLES 
int peakSoundEnergy = 0;                 // Holds the maximum sound amplitude detected within a cycle
float currentMidpoint = 2048.0;          // Running midpoint of the sound waveform (12-bit ADC: max 4095)
float baseline = 2048.0;                 // Fixed baseline to calculate amplitude against

// HISTORICAL TRACKING REGISTERS FOR EMA SMOOTHING 
float smoothTemp = 25.0;                 // Smoothed temperature value
float smoothHumidity = 50.0;             // Smoothed humidity value
float smoothPressure = 1013.25;          // Smoothed barometric pressure value (standard sea-level pressure default)
bool firstRun = true;                    // Flag to bypass smoothing on the very first reading

// Interrupt Service Routine (ISR) triggered by pressing the calibration button
void IRAM_ATTR handleCalibPress() {
  flag_calibrate = true; // Set flag to true to notify the main loop
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
  
  // Initialise the status NeoPixel to red (indicating setup phase)
  pixels.begin();       
  setPixel(150, 0, 0); 

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
    setPixel(150, 150, 0); 
    delay(1000);
  }
  ENS160.setPWRMode(ENS160_STANDARD_MODE); // Set sensor to standard operation mode

  // Calculate the initial analogue baseline for the sound sensor
  long sum = 0;
  for(int i=0; i<300; i++) { sum += analogRead(SOUND_PIN); delay(2); }
  baseline = sum / 300.0;       // Average of 300 readings
  currentMidpoint = baseline;   // Initialise the running midpoint

  // Connect to the Wi-Fi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

void loop() {
  // --- FAST LOOP (Executes as fast as possible to catch sound peaks) ---
  int rawSound = analogRead(SOUND_PIN);
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

    // Gather sensor readings
    float distance = getDistanceBurst();
    sensors_event_t h_ev, t_ev;
    bool aht_ok = aht.getEvent(&h_ev, &t_ev);
    float rawPressure = bmp.readPressure() / 100.0F; // Convert Pascals to hPa (millibars)

    if (aht_ok) {
        // Apply manual calibration offsets
        float rawTempAdjusted = t_ev.temperature + tempOffset;
        float rawHumidity = h_ev.relative_humidity + humidOffset;
        rawHumidity = constrain(rawHumidity, 0.0f, 100.0f); // Ensure humidity stays within logical bounds

        // Apply Exponential Moving Average (EMA) to smooth out noise
        if (firstRun) {
            // Seed the EMA on the first run to prevent a slow ramp-up from zero
            smoothTemp = rawTempAdjusted;
            smoothHumidity = rawHumidity;
            smoothPressure = rawPressure;
            firstRun = false;
        } else {
            // New Smoothed Value = (Alpha * New Reading) + ((1 - Alpha) * Previous Smoothed Value)
            smoothTemp = (alpha * rawTempAdjusted) + ((1.0f - alpha) * smoothTemp);
            smoothHumidity = (alpha * rawHumidity) + ((1.0f - alpha) * smoothHumidity);
            smoothPressure = (alpha * rawPressure) + ((1.0f - alpha) * smoothPressure);
        }

        // Feed ambient temperature and humidity data into the ENS160 for accurate internal compensation
        ENS160.setTempAndHum(smoothTemp, smoothHumidity);
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
      http.addHeader("Content-Type", "application/json");
      
      // Determine if the calibration button was pressed, then clear the flag
      int sendCalFlag = flag_calibrate ? 1 : 0;
      flag_calibrate = false; 
      
      // Construct the JSON payload
      JsonDocument doc;
      doc["key"] = API_KEY;
      doc["id"] = nodeID;
      doc["t"] = smoothTemp;
      doc["h"] = smoothHumidity;
      doc["p"] = smoothPressure;
      doc["aqi"] = txAqi;
      doc["tvoc"] = txTvoc;
      doc["eco2"] = txEco2;
      doc["snd"] = peakSoundEnergy; // Send the latched maximum amplitude
      doc["dst"] = distance;
      doc["cal"] = sendCalFlag;

      // Serialize JSON document into a String for transmission
      String jsonPayload;
      serializeJson(doc, jsonPayload);
               
      // Send the POST request
      int httpResponseCode = http.POST(jsonPayload);
      http.end(); // Free network resources
      
      // Provide visual feedback via the NeoPixel
      if (httpResponseCode > 0) {
        if (!isWarmingUp) setPixel(0, 40, 0); // Green: Successfully transmitted
      } else {
        setPixel(80, 0, 0); // Red: Transmission failed
      }
      
    } else {
      setPixel(80, 0, 0); // Red: Wi-Fi disconnected
      WiFi.begin(ssid, password); // Attempt to reconnect
    }
    
    // Reset variables for the next cycle
    peakSoundEnergy = 0; // Clear the latched sound peak
    digitalWrite(LED_BUILTIN, LOW); // Turn off the processing indicator
  }
}