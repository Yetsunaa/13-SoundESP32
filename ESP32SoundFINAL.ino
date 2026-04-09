/*
 * ESP32 Acoustic Data Logger
 * Hardware: ESP32 (CPU, WiFi Radio, ADC1)
 * Sensor: Analog Microphone connected to Pin 34
 */

#include <WebServer.h>      // Runs on CPU: Handles HTTP requests
#include <WiFi.h>           // Runs on WiFi Radio: Manages Access Point (AP)
#include "arduinoFFT.h"     // Runs on CPU: Performs math for frequency analysis
#include <vector>           // Runs on RAM: Dynamic storage for the data log

// --- CONFIGURATION ---
WebServer server(80);       // Initialize Web Server on Port 80

#define SAMPLES 512              // FFT Sample count (Must be power of 2)
#define SAMPLING_FREQUENCY 8000  // Sample rate in Hz (Capture up to 4kHz audio)
const int SOUND_PIN = 34;        // Hardware: Analog Input Pin (ADC1_CH6)

unsigned int sampling_period_us;
double vReal[SAMPLES];           // Buffer for raw audio data
double vImag[SAMPLES];           // Buffer for FFT imaginary component
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// --- TIMING LOGIC ---
const unsigned long CYCLE_TOTAL = 30000;  // 30 second full cycle
const unsigned long RECORD_WINDOW = 5000; // 5 second recording "burst"
unsigned long lastCycleStart = 0;
unsigned long lastSecondTick = 0;

float latestDb = 0;
float latestHz = 0;
bool isRecording = false;

// --- DATA STRUCTURES ---
struct LogEntry {
  String timestamp;
  float dbVals[5];   // Stores 5 individual 1-second samples
  float hzVals[5];   // Stores 5 individual 1-second samples
  bool isOutlier;    // Flag for values > 85dB
};

// Global buffer and storage
std::vector<LogEntry> dataLog; // Historical log kept in RAM (limited to 20 entries)
int samplesCaptured = 0;
float tmpDb[5];
float tmpHz[5];

// --- UTILITY FUNCTIONS ---

// Calculates uptime in HH:MM:SS format using the ESP32 internal clock
String getUptimeStamp() {
  unsigned long sec = millis() / 1000;
  int h = sec / 3600;
  int m = (sec % 3600) / 60;
  int s = sec % 60;
  char buf[15];
  sprintf(buf, "%02d:%02d:%02d", h, m, s);
  return String(buf);
}

// Endpoint: /data -> Returns current readings and history as a JSON object
void handleData() {
  String json = "{";
  json += "\"db\":" + String(latestDb, 1) + ",";
  json += "\"hz\":" + String(latestHz, 0) + ",";
  json += "\"history\": [";
  
  for (int i = dataLog.size() - 1; i >= 0; i--) {
    json += "{";
    json += "\"time\":\"" + dataLog[i].timestamp + "\",";
    json += "\"outlier\":" + String(dataLog[i].isOutlier ? "true" : "false") + ",";
    json += "\"dbs\": [";
    for(int j=0; j<5; j++) {
      json += String(dataLog[i].dbVals[j], 1) + (j < 4 ? "," : "");
    }
    json += "],";
    json += "\"hzs\": [";
    for(int j=0; j<5; j++) {
      json += String(dataLog[i].hzVals[j], 0) + (j < 4 ? "," : "");
    }
    json += "]}";
    if (i > 0) json += ",";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// Endpoint: / -> Serves the main HTML dashboard with CSS styling and Auto-refresh JS
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<style>body{font-family:sans-serif; text-align:center; background:#f4f7f6; padding:20px;}";
  html += ".card{background:white; max-width:950px; margin:auto; padding:25px; border-radius:15px; box-shadow:0 10px 20px rgba(0,0,0,0.05);}";
  html += "table{width:100%; border-collapse:collapse; margin-top:20px; font-size:12px;} th{background:#eee;} td,th{padding:8px; border-bottom:1px solid #ddd;}";
  html += ".outlier{color:#d9534f; font-weight:bold; background:#fff1f0;} .status-bar{padding:10px; margin-bottom:10px; border-radius:8px; font-weight:bold; background:#e8f5e9; color:#2e7d32;} .rec-active{background:#ffebee; color:#c62828;}</style>";
  
  html += "<script>setInterval(function() {";
  html += "  fetch('/data').then(r => r.json()).then(d => {";
  html += "    document.getElementById('db').innerText = d.db > 0 ? d.db + ' dB' : 'SLEEPING';";
  html += "    document.getElementById('hz').innerText = d.hz > 0 ? d.hz + ' Hz' : '--';";
  html += "    let tableBody = '';";
  html += "    d.history.forEach(entry => {";
  html += "      let rowClass = entry.outlier ? 'class=\"outlier\"' : '';";
  html += "      tableBody += '<tr ' + rowClass + '><td>' + entry.time + '</td>';";
  html += "      tableBody += '<td>' + entry.dbs.join(' | ') + '</td>';";
  html += "      tableBody += '<td>' + entry.hzs.join(' | ') + ' Hz</td>';";
  html += "      tableBody += '<td>' + (entry.outlier ? '⚠ OUTLIER' : 'Normal') + '</td></tr>';";
  html += "    });";
  html += "    document.getElementById('logBody').innerHTML = tableBody;";
  html += "  });";
  html += "}, 2000);</script></head><body><div class='card'><h1>Acoustic Data Log</h1>";
  html += "<div class='status-bar'>Live Monitor (Updates automatically)</div>";
  html += "<h3>Live: <span id='db'>-- | <span id='hz'>--</span></h3>"</span>;
  html += "<table><thead><tr><th>Uptime</th><th>5s Decibel Samples (dB)</th><th>5s Frequency Samples (Hz)</th><th>Status</th></tr></thead>";
  html += "<tbody id='logBody'></tbody></table></div></body></html>";
  
  server.send(200, "text/html", html);
}

// --- CORE PROCESSING ---

// Runs on CPU & ADC: Captures sound wave and performs FFT math
void runFFTAnalysis() {
  int sMax = 0, sMin = 4095;
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long start = micros();
    int val = analogRead(SOUND_PIN); // Hardware ADC Read
    vReal[i] = val; 
    vImag[i] = 0;
    if (val > sMax) sMax = val;
    if (val < sMin) sMin = val;
    // Maintains precise sampling timing
    while (micros() < (start + sampling_period_us));
  }
  
  // Math: Compute dominant frequency
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  vReal[0] = 0; vReal[1] = 0; vReal[2] = 0; // Filter low-end noise/DC offset
  
  latestHz = FFT.majorPeak();
  // Math: Estimate dB level based on Peak-to-Peak amplitude
  latestDb = 20 * log10((sMax - sMin) + 1) + 9.0;
}

void setup() {
  Serial.begin(115200);
  
  // Hardware: Start Access Point. Connect PC/Phone to "SoundScape_Monitor"
  WiFi.softAP("SoundScape_Monitor", "password123"); 
  
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  
  lastCycleStart = millis();
}

void loop() {
  server.handleClient(); // CPU: Listen for browser requests
  unsigned long now = millis();

  // STATE: RECORDING (Active for the first 5 seconds of the 30s cycle)
  if (now - lastCycleStart <= RECORD_WINDOW) {
    if (!isRecording) {
      isRecording = true;
      samplesCaptured = 0;
      lastSecondTick = now;
    }

    runFFTAnalysis(); // Heavy CPU Processing

    // Capture one snapshot every second
    if (now - lastSecondTick >= 1000 && samplesCaptured < 5) {
      tmpDb[samplesCaptured] = latestDb;
      tmpHz[samplesCaptured] = latestHz;
      samplesCaptured++;
      lastSecondTick = now;
    }
  } 
  // STATE: SLEEPING (Active for the remaining 25 seconds)
  else {
    if (isRecording) {
      isRecording = false; // Recording window just ended
      if (samplesCaptured > 0) {
        LogEntry entry;
        entry.timestamp = getUptimeStamp();
        entry.isOutlier = false;
        for(int i=0; i<5; i++) {
          entry.dbVals[i] = tmpDb[i];
          entry.hzVals[i] = tmpHz[i];
          if(tmpDb[i] > 85.0) entry.isOutlier = true; // Check for loud noise events
        }
        dataLog.push_back(entry);
        if (dataLog.size() > 20) dataLog.erase(dataLog.begin()); // Prevent RAM overflow
      }
    }
    latestDb = 0; latestHz = 0; // Reset live display indicators
  }

  // Restart the 30-second loop
  if (now - lastCycleStart >= CYCLE_TOTAL) {
    lastCycleStart = now;
  }
}