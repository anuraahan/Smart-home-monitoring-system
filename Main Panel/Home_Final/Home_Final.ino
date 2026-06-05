/*
Smart Home Area Network (HAN) - Master Display & Server Node
Target Hardware: ESP32-S3 with TFT_eSPI Display, FT6336U Touch, and SD Card
This programme acts as a central hub, receiving JSON data from sensor nodes,
logging it to an SD card, displaying real-time metrics on a touchscreen GUI, 
and serving a web dashboard via an asynchronous web server.
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <TFT_eSPI.h>
#include "FT6336U.h"
#include "FS.h"
#include "SD.h" 
#include "SPI.h"
#include "time.h" 
#include <ArduinoJson.h> 
#include "AsyncJson.h" // Required for ESPAsyncWebServer JSON extraction

// WiFi Configuration  
const char* ssid = "EDA-IOT";
const char* password = "3aB1J27M";
const char* API_KEY = "EDA_Secure_Key_2024"; // Security key for incoming JSON payloads

// Real-Time Clock (NTP) Synchronisation 
const char* ntpServer = "pool.ntp.org"; 
const long   gmtOffset_sec = 0;       
const int    daylightOffset_sec = 3600; // 1-hour offset for Daylight Saving Time (BST)

// Delay to allow gas/air quality sensors on the nodes to stabilise before reporting
const unsigned long WARMUP_DELAY_MS = 180000; // 3 minute warmup delay

// Custom SD Card SPI Pin Definitions
#define SD_SCK  38 
#define SD_MISO 39 
#define SD_MOSI 40 
#define SD_CS   47 
SPIClass spiSD(FSPI); // Initialise hardware SPI for the SD card

// Environmental & Tactical Threshold Configuration
// Index 0 = Node 1, Index 1 = Node 2
int soundThresholds[2]   = {500, 300}; 
float tempLow[2]         = {18.0, 18.0}; 
float tempHigh[2]        = {28.0, 30.0}; 
float humidLow[2]        = {30.0, 30.0}; 
float humidHigh[2]       = {70.0, 75.0}; 
float pressLow[2]        = {980.0, 980.0};   
float pressHigh[2]       = {1030.0, 1030.0}; 
int aqiThreshold[2]      = {3, 3}; 
float distChangeThreshold[2] = {15.0, 15.0}; // Trigger delta for ultrasonic obstacles

// Hardware Instantiation 
TFT_eSPI tft = TFT_eSPI(); // Instantiates the TFT display using User_Setup settings
FT6336U ft6336u(16, 15, 18, 17); // Touchscreen driver pins (SDA, SCL, RST, INT)
AsyncWebServer server(80); // Asynchronous web server running on port 80

//  Data Structures 
struct NodeData {  // Holds incoming data and evaluation states for each node
    float t=0, h=0, p=0, dst=0; // Temperature, humidity, pressure, distance
    int aqi=0, tvoc=0, eco2=0, snd=0; // Air quality indices and sound level
    // Detection flags (-1 = low alert, 0 = normal, 1 = high alert)
    int det_snd=0, det_temp=0, det_humid=0, det_press=0, det_aqi=0, det_dst=0; 
    float base_dst = 0; // Baseline distance for ultrasonic calibration
    int calib_count = 0; // Number of calibration readings taken
    unsigned long lastSeen = 0; // Timestamp of the last received packet
};

NodeData nodes[2]; // Array to store data for Node 1 and Node 2
int activeNode = 0, activePage = 0; // UI tracking variables

// UI State Tracking 
int lastPage = -1;
int lastNode = -1;
volatile bool forceFullRedraw = true; // Flag to force a complete screen refresh

volatile bool newDataReady = false;  // Flag set when new JSON data arrives
unsigned long lastStatusCheck = 0;
bool sdAvailable = false; // Tracks if the SD card successfully mounted

// Functions 

// Logs current node data to a CSV file on the SD card
void logDataToSD(int idx) { 
    if (!sdAvailable) return;
    struct tm timeinfo;
    char timeStr[30];
    
    // Attempt to get NTP time; fallback to millis() if time isn't synchronised yet
    if(!getLocalTime(&timeinfo)){
        sprintf(timeStr, "%lu", millis());
    } else {
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    }

    String fileName = (idx == 0) ? "/node1_log.csv" : "/node2_log.csv";
    File file = SD.open(fileName.c_str(), FILE_APPEND);
    if (!file) return; // Exit if file failed to open

    // Append a comma-separated row with timestamps and metrics
    file.printf("%s, %.2f, %.2f, %.2f, %d, %d, %d, %.1f, %d, %d\n", 
                timeStr, nodes[idx].t, nodes[idx].h, nodes[idx].p, 
                nodes[idx].aqi, nodes[idx].tvoc, nodes[idx].eco2, nodes[idx].dst, nodes[idx].snd, nodes[idx].det_dst);
    file.close();
}

// Evaluates incoming node data against predefined thresholds to set alert flags
void evaluateThresholds(int idx) { 
    NodeData &n = nodes[idx];
    
    // Evaluate sound levels
    n.det_snd = (n.snd > soundThresholds[idx]) ? 1 : 0;
    
    // Evaluate temperature (allows for cold and hot alerts)
    if (n.t < tempLow[idx]) {
        n.det_temp = -1;
    } else if (n.t > tempHigh[idx]) {
        n.det_temp = 1;
    } else {
        n.det_temp = 0;
    }
    
    // Evaluate humidity, pressure and AQI
    n.det_humid = (n.h < humidLow[idx] || n.h > humidHigh[idx]) ? 1 : 0;
    n.det_press = (n.p < pressLow[idx] || n.p > pressHigh[idx]) ? 1 : 0; 
    n.det_aqi = (n.aqi >= aqiThreshold[idx]) ? 1 : 0;
    
    // Evaluate ultrasonic distance and handle calibration
    if (n.calib_count < 5) { // Calibration stage: average the first 5 readings
        n.base_dst += n.dst;
        n.calib_count++; 
        if (n.calib_count == 5) {
            n.base_dst /= 5.0; // Calculate average baseline
            Serial.printf("Node %d Distance Calibrated: %.1f cm\n", idx + 1, n.base_dst);
            forceFullRedraw = true; // Update UI once calibrated
        }
    } else {
        // Compare current distance against the calibrated baseline
        if (abs(n.dst - n.base_dst) > distChangeThreshold[idx]) {
            n.det_dst = 1; // Obstacle detected
        } else {
            n.det_dst = 0; // Clear
        }
    }
}

// Generates the HTML, CSS, and JS for the web server dashboard
String getDashboardHTML() {
    // Basic HTML setup and styling (mimics the hardware TFT interface)
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'><style>";
    html += "body { font-family: 'Courier New', monospace; background: #000000; color: #FFFFFF; padding: 15px; margin: 0; }";
    html += ".screen-container { max-width: 600px; margin: 20px auto; border: 3px solid #333; background: #000000; box-shadow: 0 0 15px rgba(0,230,118,0.1); }";
    
    html += ".tft-header { background: #102008; border-bottom: 2px solid #333; padding: 10px; display: flex; justify-content: space-between; color: #7BEF; font-size: 14px; font-weight: bold; }";
    
    html += ".node-card { border: 1px solid #222; margin: 15px; background: #050505; }";
    html += ".node-title { background: #081018; padding: 8px 12px; font-size: 16px; border-bottom: 1px solid #222; display: flex; justify-content: space-between; }";
    html += ".section-title { color: #FFA500; font-size: 13px; font-weight: bold; letter-spacing: 1px; padding: 10px 15px 0 15px; text-transform: uppercase; text-align: left;}";
    html += ".grid-3 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; padding: 10px 15px 15px 15px; text-align: left; }";
    html += ".grid-2 { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; padding: 10px 15px 15px 15px; text-align: left; }";
    
    html += ".val-group { background: #111; padding: 10px; border-radius: 2px; border-left: 3px solid #555; }";
    html += ".lbl { font-size: 11px; color: #888; text-transform: uppercase; margin-bottom: 4px; }";
    html += ".val { font-size: 18px; font-weight: bold; color: #FFFFFF; }";
    html += ".thresh-val { font-size: 14px; font-weight: bold; color: #FFFF00; }"; 
    
    // Dynamic styling for alerts and states
    html += ".online { color: #00FFFF; } .offline { color: #FF0000; }";
    html += ".alert-bg { border-left-color: #FF0000 !important; background: #200000 !important; }";
    html += ".alert-bg .val { color: #FF0000 !important; }";
    html += ".cold-bg { border-left-color: #0088FF !important; background: #001133 !important; }";
    html += ".cold-bg .val { color: #0088FF !important; }";
    html += ".alert-text { color: #FF0000 !important; font-weight: bold; animation: blink 1.5s infinite; }";
    html += ".safe-text { color: #008000; font-weight: bold; }";
    html += ".calib-text { color: #FFFF00; font-weight: bold; }";
    html += "@keyframes blink { 0% {opacity: 0.4;} 50% {opacity: 1;} 100% {opacity: 0.4;} }";
    
    // Sound bar graph CSS
    html += ".snd-bar-container { background: #222; border: 1px solid #FFF; height: 16px; width: 100%; position: relative; margin-top: 5px; }";
    html += ".snd-bar { height: 100%; width: 0%; transition: width 0.3s ease; }";
    html += "</style></head><body>";
    
    html += "<div class='screen-container'>";
    
    // Header containing IP and Time
    html += "  <div class='tft-header'>";
    html += "    <div>IP: " + WiFi.localIP().toString() + "</div>";
    html += "    <div id='sys-time'>00:00</div>";
    html += "  </div>";

    // Loop through both nodes to generate their respective dashboard cards
    for(int i=0; i<2; i++) {
        String num = String(i+1);
        int idx = i;
        html += "  <div class='node-card'>";
        html += "    <div class='node-title'><span>NODE " + num + "</span><span id='status-" + num + "' class='offline'>OFFLINE</span></div>";
        
        // Climate Section
        html += "    <div class='section-title'>CLIMATE</div>";
        html += "    <div class='grid-3'>";
        html += "      <div class='val-group' id='card-t-" + num + "'><div class='lbl'>Temp</div><div class='val' id='temp-" + num + "'>-</div></div>";
        html += "      <div class='val-group' id='card-h-" + num + "'><div class='lbl'>Humid</div><div class='val' id='humid-" + num + "'>-</div></div>";
        html += "      <div class='val-group' id='card-p-" + num + "'><div class='lbl'>Baro</div><div class='val' id='press-" + num + "'>-</div></div>";
        html += "    </div>";

        // Air Quality Section
        html += "    <div class='section-title'>AIR QUALITY</div>";
        html += "    <div class='grid-3'>";
        html += "      <div class='val-group' id='card-aqi-" + num + "'><div class='lbl'>AQI</div><div class='val' id='aqi-" + num + "'>-</div></div>";
        html += "      <div class='val-group'><div class='lbl'>TVOC</div><div class='val' id='tvoc-" + num + "'>-</div></div>";
        html += "      <div class='val-group'><div class='lbl'>eCO2</div><div class='val' id='eco2-" + num + "'>-</div></div>";
        html += "    </div>";

        // Tactical Sensors Section
        html += "    <div class='section-title'>TACTICAL SENSORS</div>";
        html += "    <div class='grid-3'>";
        html += "      <div class='val-group' id='card-dst-" + num + "' style='grid-column: span 2;'><div class='lbl'>Ultrasonic Distance</div><div class='val' id='dist-" + num + "'>-</div><div id='dist-msg-" + num + "' style='font-size:11px; margin-top:4px;'></div></div>";
        html += "      <div class='val-group' id='card-snd-" + num + "'><div class='lbl'>Sound</div><div class='val' id='sound-" + num + "'>-</div></div>";
        html += "    </div>";
        
        // Sound Graph Sub-section
        html += "    <div style='padding: 0 15px 15px 15px;'>";
        html += "      <div class='lbl'>Sound Decibel Matrix Graph</div>";
        html += "      <div class='snd-bar-container'><div class='snd-bar' id='sbar-" + num + "'></div></div>";
        html += "      <div id='sound-msg-" + num + "' style='font-size:11px; margin-top:5px; text-align:left;'></div>";
        html += "    </div>";

        // Threshold configuration display
        html += "    <div class='section-title' style='color:#FFFF00;'>THRESHOLDS MATRIX (PAGE 4 CONFIG)</div>";
        html += "    <div class='grid-2' style='background:#0a0a0a; border-top: 1px dashed #222;'>";
        html += "      <div class='val-group' style='border-left: 2px solid #FFFF00;'><div class='lbl'>Sound Max</div><div class='thresh-val'>" + String(soundThresholds[idx]) + "</div></div>";
        html += "      <div class='val-group' style='border-left: 2px solid #FFFF00;'><div class='lbl'>Temp Limits</div><div class='thresh-val'>" + String(tempLow[idx],1) + " - " + String(tempHigh[idx],1) + " C</div></div>";
        html += "      <div class='val-group' style='border-left: 2px solid #FFFF00;'><div class='lbl'>Humid Limits</div><div class='thresh-val'>" + String(humidLow[idx],0) + " - " + String(humidHigh[idx],0) + "%</div></div>";
        html += "      <div class='val-group' style='border-left: 2px solid #FFFF00;'><div class='lbl'>Baro Limits</div><div class='thresh-val'>" + String(pressLow[idx],0) + " - " + String(pressHigh[idx],0) + " hPa</div></div>";
        html += "      <div class='val-group' style='border-left: 2px solid #FFFF00; grid-column: span 2;'><div class='lbl'>Distance Delta Change Trigger</div><div class='thresh-val'>" + String(distChangeThreshold[idx],1) + " cm</div></div>";
        html += "    </div>";
        
        html += "  </div>";
    }
    
    // Footer containing system status
    html += "  <div class='tft-header' style='border-top:2px solid #333; border-bottom:none; color:#888; font-size:12px;'>";
    html += "    <div id='sd-log-status'>SD LOGGING: " + String(sdAvailable ? "ACTIVE" : "FAILED") + "</div>";
    html += "    <div>TARGET PROFILE: HYBRID MULTI-NODE</div>";
    html += "  </div>";
    html += "</div>";

    // JavaScript for asynchronous data fetching and DOM updates
    html += "<script>";
    html += "async function updateDashboard() {";
    html += "  try {";
    html += "    let res = await fetch('/api/data');"; // Fetch data from local API endpoint
    html += "    let data = await res.json();";
    html += "    let d = new Date();";
    html += "    document.getElementById('sys-time').innerText = String(d.getHours()).padStart(2,'0') + ':' + String(d.getMinutes()).padStart(2,'0');";
    html += "    data.forEach((node, i) => {";
    html += "      let idx = i + 1;";
    html += "      let statusEl = document.getElementById('status-'+idx);";
    html += "      statusEl.innerText = node.online ? 'ONLINE' : 'OFF';";
    html += "      statusEl.className = node.online ? 'online' : 'offline';";
    html += "      ";
    html += "      document.getElementById('temp-'+idx).innerText = node.t.toFixed(1) + ' C';";
    
    // Apply background classes based on detection states
    html += "      let tClass = 'val-group';";
    html += "      if (node.det_t === -1) tClass += ' cold-bg';";
    html += "      else if (node.det_t === 1) tClass += ' alert-bg';";
    html += "      document.getElementById('card-t-'+idx).className = tClass;";
    
    html += "      ";
    html += "      document.getElementById('humid-'+idx).innerText = node.h.toFixed(0) + '%';";
    html += "      document.getElementById('card-h-'+idx).className = 'val-group' + (node.det_h ? ' alert-bg' : '');";
    html += "      ";
    html += "      document.getElementById('press-'+idx).innerText = node.p.toFixed(0) + ' hPa';";
    html += "      document.getElementById('card-p-'+idx).className = 'val-group' + (node.det_p ? ' alert-bg' : '');";
    html += "      ";
    html += "      document.getElementById('aqi-'+idx).innerText = node.aqi;";
    html += "      document.getElementById('card-aqi-'+idx).className = 'val-group' + (node.det_aqi ? ' alert-bg' : '');";
    html += "      ";
    html += "      document.getElementById('tvoc-'+idx).innerText = node.tvoc + ' ppb';";
    html += "      document.getElementById('eco2-'+idx).innerText = node.eco2 + ' ppm';";
    html += "      ";
    
    // Manage Ultrasonic calibration UI state
    html += "      let dMsg = document.getElementById('dist-msg-'+idx);";
    html += "      if(node.cal < 5){";
    html += "         document.getElementById('dist-'+idx).innerText = 'CALIB';";
    html += "         dMsg.innerText = 'CALIBRATING...'; dMsg.className='calib-text';";
    html += "      } else {";
    html += "         document.getElementById('dist-'+idx).innerText = node.dst.toFixed(1) + ' cm';";
    html += "         dMsg.innerText = node.det_d ? '! OBSTACLE DETECTED !' : 'STATUS: CLEAR';";
    html += "         dMsg.className = node.det_d ? 'alert-text' : 'safe-text';";
    html += "      }";
    html += "      document.getElementById('card-dst-'+idx).className = 'val-group' + (node.det_d ? ' alert-bg' : '');";
    html += "      ";
    
    // Manage Sound Bar width and colour
    html += "      document.getElementById('sound-'+idx).innerText = node.snd;";
    html += "      document.getElementById('card-snd-'+idx).className = 'val-group' + (node.det_s ? ' alert-bg' : '');";
    html += "      let pct = Math.min((node.snd / 1000) * 100, 100);";
    html += "      let sBar = document.getElementById('sbar-'+idx); sBar.style.width = pct + '%';";
    html += "      sBar.style.background = node.det_s ? '#FF0000' : '#008000';";
    html += "      let sMsg = document.getElementById('sound-msg-'+idx);";
    html += "      sMsg.innerText = node.det_s ? '! SOUND DETECTED !' : 'STATUS: QUIET';";
    html += "      sMsg.className = node.det_s ? 'alert-text' : 'safe-text';";
    html += "    });";
    html += "  } catch(e) { console.error('Data pull error', e); }";
    html += "}";
    html += "setInterval(updateDashboard, 2000); updateDashboard();"; // Poll every 2 seconds
    html += "</script></body></html>";
    return html;
}

// Main UI rendering function for the TFT display
void refreshUI() { 
    // Only perform a slow full-screen clear/draw if the page or node has changed, or forced
    bool doFullDraw = (activePage != lastPage || activeNode != lastNode || forceFullRedraw);

    if (doFullDraw) {
        tft.fillScreen(TFT_BLACK);
        
        // Draw Header bar
        tft.fillRect(0, 0, 320, 32, 0x2104);
        tft.setTextSize(2); tft.setTextColor(0x7BEF, 0x2104);
        tft.setCursor(10, 8); tft.print("IP: "); tft.print(WiFi.localIP().toString());
        
        // Draw Page Titles
        const char* titles[] = {"CLIMATE", "AIR QUALITY", "ULTRASONIC", "NOISE", "THRESHOLDS"};
        tft.setCursor(15, 45); tft.setTextColor(TFT_ORANGE, TFT_BLACK); 
        tft.print(titles[activePage]);

        // Draw Footer bar & Pagination indicators
        tft.fillRect(0, 205, 320, 35, 0x1082);
        for(int i=0; i<5; i++) tft.fillCircle(140 + (i * 10), 223, 2, (i == activePage) ? TFT_WHITE : 0x52AA);
        // Draw left/right navigation arrows
        tft.fillTriangle(240, 222, 255, 212, 255, 232, (activePage > 0) ? TFT_YELLOW : TFT_DARKGREY);
        tft.fillTriangle(305, 222, 290, 212, 290, 232, (activePage < 4) ? TFT_YELLOW : TFT_DARKGREY);

        if (activePage == 3) {
            // Draw outline for the sound decibel bar graph
            tft.drawRect(20, 115, 280, 20, TFT_WHITE); 
        }
        // Threshold Page Matrix Rendering
        else if (activePage == 4) { 
            tft.setTextSize(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setCursor(20, 60);  tft.printf("Sound Max: %-4d", soundThresholds[activeNode]);
            tft.setCursor(20, 85);  tft.printf("Temp: %.1f - %.1f", tempLow[activeNode], tempHigh[activeNode]);
            tft.setCursor(20, 110); tft.printf("Humid: %.0f - %.0f", humidLow[activeNode], humidHigh[activeNode]);
            tft.setCursor(20, 135); tft.printf("Press: %.0f - %.0f", pressLow[activeNode], pressHigh[activeNode]);
            tft.setCursor(20, 160); tft.printf("Dist Chg: %.1f cm", distChangeThreshold[activeNode]);
            tft.setTextColor(0x7BEF, TFT_BLACK);
            tft.setCursor(20, 185); tft.print(sdAvailable ? "SD Logging: ACTIVE" : "SD Logging: FAILED");
        }

        // Update tracking states
        lastPage = activePage;
        lastNode = activeNode;
        forceFullRedraw = false;
    }

    // Dynamic Data Updates (Runs constantly without clearing the screen)
    NodeData &d = nodes[activeNode];
    
    // Determine node health (timeout if not seen in 15 seconds)
    bool online = (millis() - d.lastSeen < 15000);

    // Update real-time clock in header
    struct tm timeinfo; 
    if(getLocalTime(&timeinfo, 10)) { 
        char timeStr[10];
        strftime(timeStr, sizeof(timeStr), "%H:%M", &timeinfo);
        tft.setTextSize(2); 
        tft.setTextColor(TFT_WHITE, 0x2104); 
        tft.setCursor(250, 8); 
        tft.print(timeStr);
    }

    // Node Active Indicator in footer
    tft.setTextSize(2); 
    tft.setTextColor(online ? TFT_CYAN : TFT_RED, 0x1082); 
    tft.setCursor(15, 215); 
    tft.printf("NODE %d %-4s", activeNode + 1, online ? "   " : "OFF"); 

    // Temperature, humidity and pressure page rendering
    if (activePage == 0) { 
        tft.setTextSize(4); tft.setCursor(20, 80);  
        
        // Dynamic colour formatting for temperature limits
        uint16_t tempColor = TFT_WHITE;
        if (d.det_temp == -1) tempColor = TFT_BLUE; // Cold warning
        else if (d.det_temp == 1) tempColor = TFT_RED; // Hot warning
        
        tft.setTextColor(tempColor, TFT_BLACK); 
        tft.printf("%-5.1f C    ", d.t); 
        
        tft.setTextSize(2); tft.setCursor(20, 125); 
        tft.setTextColor(d.det_humid ? TFT_RED : TFT_WHITE, TFT_BLACK); 
        tft.printf("Humid: %-3.0f%%      ", d.h);
        tft.setTextColor(d.det_press ? TFT_RED : TFT_WHITE, TFT_BLACK); tft.setCursor(20, 155);  
        tft.printf("Baro: %-4.0f hPa   ", d.p);
    } 
    // Air quality page rendering
    else if (activePage == 1) { 
        if (millis() < WARMUP_DELAY_MS) { // Show boot sequence warning to allow gas sensors to heat up
            tft.setTextSize(3); tft.setCursor(20, 90); 
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.print("SENSOR BOOTING");
            tft.setTextSize(2); tft.setCursor(20, 130);
            tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
            tft.print("Please wait 3 mins...  ");
        } else { // Active data stream
            tft.setTextSize(4); tft.setCursor(20, 80); 
            tft.setTextColor(d.det_aqi ? TFT_RED : TFT_WHITE, TFT_BLACK); 
            tft.printf("AQI %-4d  ", d.aqi);
            tft.setTextSize(2); tft.setCursor(20, 125); tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.printf("TVOC: %-5d ppb   ", d.tvoc);
            tft.setCursor(20, 155); 
            tft.printf("eCO2: %-5d ppm   ", d.eco2);
        }
    } 
    // Ultrasonic Tactical page rendering
    else if (activePage == 2) { 
        if (d.calib_count < 5) { // Calibration stage feedback
            tft.setTextSize(3); tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.setCursor(20, 100); tft.print("CALIBRATING...      ");
        } else {   // Live distance rendering
            tft.setTextSize(5); tft.setCursor(20, 75); 
            tft.setTextColor(d.det_dst ? TFT_RED : TFT_WHITE, TFT_BLACK);
            tft.printf("%-5.1f", d.dst);
            tft.setTextSize(2); tft.print(" cm  ");
            
            tft.setTextColor(0x7BEF, TFT_BLACK); tft.setCursor(20, 125); 
            tft.printf("Base: %-5.1f cm   ", d.base_dst);
            
            tft.setCursor(20, 155); 
            tft.setTextColor(d.det_dst ? TFT_RED : TFT_DARKGREEN, TFT_BLACK);
            tft.print(d.det_dst ? "! OBSTACLE DETECTED !   " : "STATUS: CLEAR           ");
        }
    } 
    // Sound Level matrix page rendering
    else if (activePage ==  3) { 
        tft.setTextSize(3); tft.setCursor(20, 75); 
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.printf("SND: %-4d   ", d.snd);
        
        // Map sound value (0-1000) to the physical bar width (0-276 pixels)
        int barWidth = map(constrain(d.snd, 0, 1000), 0, 1000, 0, 276); 
        tft.fillRect(22, 117, barWidth, 16, d.det_snd ? TFT_RED : TFT_GREEN); // Draw active bar
        if (276 - barWidth > 0) tft.fillRect(22 + barWidth, 117, 276 - barWidth, 16, TFT_BLACK); // Blank remaining area
        
        tft.setTextSize(2); tft.setCursor(20, 155); 
        tft.setTextColor(d.det_snd ? TFT_RED : TFT_DARKGREEN, TFT_BLACK);
        tft.print(d.det_snd ? "! SOUND DETECTED !    " : "STATUS: QUIET         "); // No sound detected 
    }
}

void setup() {
    Serial.begin(115200);
    
    // Initialise hardware components
    tft.init(); tft.setRotation(1); // Set display to landscape
    ft6336u.begin(); // Initialise I2C touchscreen controller

    // Connect to WiFi network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) delay(500); 

    // Initialise Time Protocol
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); 

    // Initialise SD Card Module
    spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS); 
    if (!SD.begin(SD_CS, spiSD)) {
        Serial.println("SD Card Mount Failed.");
        sdAvailable = false;
    } else {  // SD card successfully mounted
        sdAvailable = true;
        const char* files[2] = {"/node1_log.csv", "/node2_log.csv"};
        // Generate CSV headers if files do not exist
        for(int i = 0; i < 2; i++) {
            if (!SD.exists(files[i])) {
                File file = SD.open(files[i], FILE_WRITE);
                if(file) {
                    file.println("Timestamp,Temp,Humid,Press,AQI,TVOC,eCO2,Dist,Sound,ObstacleDet");
                    file.close();
                }
            }
        }
    }

    // Serve HTML dashboard at root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ 
        request->send(200, "text/html", getDashboardHTML());
    });

    // Provide a JSON endpoint for the dashboard to fetch real-time updates
    server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc; 
        JsonArray array = doc.to<JsonArray>();
        
        for (int i = 0; i < 2; i++) {
            JsonObject nodeObj = array.add<JsonObject>();
            nodeObj["online"] = (millis() - nodes[i].lastSeen < 15000);
            nodeObj["t"] = nodes[i].t;
            nodeObj["h"] = nodes[i].h;
            nodeObj["p"] = nodes[i].p;
            nodeObj["aqi"] = nodes[i].aqi;
            nodeObj["tvoc"] = nodes[i].tvoc;
            nodeObj["eco2"] = nodes[i].eco2;
            nodeObj["dst"] = nodes[i].dst;
            nodeObj["snd"] = nodes[i].snd;
            nodeObj["cal"] = nodes[i].calib_count;
            
            nodeObj["det_t"] = nodes[i].det_temp;
            nodeObj["det_h"] = nodes[i].det_humid;
            nodeObj["det_p"] = nodes[i].det_press;
            nodeObj["det_aqi"] = nodes[i].det_aqi;
            nodeObj["det_d"] = nodes[i].det_dst;
            nodeObj["det_s"] = nodes[i].det_snd;
        }
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // Handle incoming JSON data (POST requests) from sensor nodes
    AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/update", [](AsyncWebServerRequest *request, JsonVariant &json) {
        JsonObject jsonObj = json.as<JsonObject>();
        
        // Enforce API Key Security
        if (!jsonObj.containsKey("key") || jsonObj["key"] != API_KEY) {
            request->send(403, "text/plain", "Forbidden - Invalid API Key");
            Serial.println("Security Alert: Unauthorized POST request blocked!");
            return;
        }

        if (jsonObj.containsKey("id")) {
            String id = jsonObj["id"].as<String>();
            int idx = (id == "node1") ? 0 : 1;
            
            // Allow remote calibration resets
            if (jsonObj.containsKey("cal") && jsonObj["cal"] == 1) {
                nodes[idx].calib_count = 0;
                nodes[idx].base_dst = 0;
                nodes[idx].det_dst = 0;
                Serial.printf("Recalibration triggered remotely for Node %d\n", idx + 1);
                forceFullRedraw = true; 
            }
            
            // Parse environmental data
            if(jsonObj.containsKey("t"))   nodes[idx].t = jsonObj["t"];
            if(jsonObj.containsKey("h"))   nodes[idx].h = jsonObj["h"];
            if(jsonObj.containsKey("p"))   nodes[idx].p = jsonObj["p"];
            if(jsonObj.containsKey("dst")) nodes[idx].dst = jsonObj["dst"];
            if(jsonObj.containsKey("snd")) nodes[idx].snd = jsonObj["snd"];
            
            // Apply 3-Minute Warmup rule to gas sensors (prevent false alerts)
            if (millis() < WARMUP_DELAY_MS) {
                nodes[idx].aqi = 0;
                nodes[idx].tvoc = 0;
                nodes[idx].eco2 = 0;
            } else {
                if(jsonObj.containsKey("aqi"))  nodes[idx].aqi = jsonObj["aqi"];
                if(jsonObj.containsKey("tvoc")) nodes[idx].tvoc = jsonObj["tvoc"];
                if(jsonObj.containsKey("eco2")) nodes[idx].eco2 = jsonObj["eco2"];
            }
            
            evaluateThresholds(idx);
            logDataToSD(idx); // Log reading to SD storage
            
            nodes[idx].lastSeen = millis();
            newDataReady = true; // Signal main loop that UI needs updating
        }
        request->send(200, "text/plain", "OK");
    });
    
    server.addHandler(handler);
    server.begin();
    refreshUI(); // Draw initial screen
}

void loop() {
    // Refresh the UI if new JSON data arrived or periodically every 1 second
    if (millis() - lastStatusCheck > 1000 || newDataReady) { 
        refreshUI(); lastStatusCheck = millis(); newDataReady = false; 
    }
    
    // Poll the FT6336U touchscreen for user input
    FT6336U_TouchPointType tp = ft6336u.scan(); 
    if (tp.touch_count > 0) {
        // Map touch coordinates (display rotation must be accounted for)
        int tx = tp.tp[0].y; 
        int ty = 240 - tp.tp[0].x;
        
        // Touch zone detection: bottom of the screen
        if (ty > 200) {
            // Node toggle button area (bottom left)
            if (tx < 140) activeNode = (activeNode == 0) ? 1 : 0;
            // Page left arrow area
            else if (tx > 230 && tx < 270 && activePage > 0) activePage--;
            // Page right arrow area
            else if (tx > 275 && activePage < 4) activePage++;
            
            forceFullRedraw = true; // Input registered, redraw layout
            refreshUI(); 
            delay(250); // Debounce delay
        }
    }
}