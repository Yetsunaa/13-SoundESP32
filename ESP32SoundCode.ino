#include <WebServer.h>
#include <WiFi.h>
#include "arduinoFFT.h"
#include <vector>

// --- CONFIGURATION ---
WebServer server(80);

#define SAMPLES 512
#define SAMPLING_FREQUENCY 8000 
const int SOUND_PIN = 34;
unsigned int sampling_period_us;
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Timing
const unsigned long CYCLE_TOTAL = 30000; //30 second cycle
const unsigned long RECORD_WINDOW = 5000; //record for 5 seconds
unsigned long lastCycleStart = 0;
unsigned long lastSecondTick = 0;

float latestDb = 0;
float latestHz = 0;
bool isRecording = false;

// Set up for log of readings
struct LogEntry {
  String timestamp;
  float dbVals[5];
  float hzVals[5];
  bool isOutlier;
};

// Limit log to 20 entries to prevent RAM crashes
std::vector<LogEntry> dataLog;

// Counters for the number of smaples capture per cycle
int samplesCaptured = 0;

// --- FUNCTIONS ---

String getUptimeStamp() {
  unsigned long sec = millis() / 1000;
  int h = sec / 3600;
  int m = (sec % 3600) / 60;
  int s = sec % 60;
  char buf[15];
  sprintf(buf, "%02d:%02d:%02d", h, m, s);
  return String(buf);
}

void handleData() {
  String json = "{";
  json += "\"db\":" + String(latestDb, 1) + ",";
  json += "\"hz\":" + String(latestHz, 0) + ",";
  json += "\"history\": [";

  for (int i = dataLog.size() - 1; i >= 0; i--) {
    json += "{";
    json += "\"time\":\"" + dataLog[i].timestamp + "\",";
    json += "\"outlier\":" + String(dataLog[i].isOutlier ? "true" : "false") + ",";

    // Add Decibel Array
    json += "\"dbs\": [";
    for(int j=0; j<5; j++) {
      json += String(dataLog[i].dbVals[j], 1) + (j < 4 ? "," : "");
    }
    json += "],";

    // Add Frequency Array
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

  // Logic to rebuild the table dynamically
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

void setup() {
  Serial.begin(115200);
  WiFi.softAP("SoundScape_Monitor", "password123"); 
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  lastCycleStart = millis();
}

// Global buffer to hold current session data
float tmpDb[5];
float tmpHz[5];

void loop() {
  server.handleClient();
  unsigned long now = millis();

  if (now - lastCycleStart <= RECORD_WINDOW) {
    if (!isRecording) {
      isRecording = true;
      samplesCaptured = 0;
      lastSecondTick = now; // Start the first second immediately
    }

    runFFTAnalysis();

    // Check if 1 second has passed AND we haven't hit 5 samples yet
    if (now - lastSecondTick >= 1000 && samplesCaptured < 5) {
      tmpDb[samplesCaptured] = latestDb;
      tmpHz[samplesCaptured] = latestHz;
      samplesCaptured++;
      lastSecondTick = now;
    }
  } 
  else {
    if (isRecording) {
      isRecording = false;
      // Push to log only if we actually got samples
      if (samplesCaptured > 0) {
        LogEntry entry;
        entry.timestamp = getUptimeStamp();
        entry.isOutlier = false;
        for(int i=0; i<5; i++) {
          entry.dbVals[i] = tmpDb[i];
          entry.hzVals[i] = tmpHz[i];
          if(tmpDb[i] > 85.0) entry.isOutlier = true;
        }
        dataLog.push_back(entry);
        if (dataLog.size() > 20) dataLog.erase(dataLog.begin());
      }
    }
    latestDb = 0; latestHz = 0;
  }

  if (now - lastCycleStart >= CYCLE_TOTAL) {
    lastCycleStart = now;
  }
}

void runFFTAnalysis() {
  int sMax = 0, sMin = 4095;
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long start = micros();
    int val = analogRead(SOUND_PIN);
    vReal[i] = val; vImag[i] = 0;
    if (val > sMax) sMax = val;
    if (val < sMin) sMin = val;
    while (micros() < (start + sampling_period_us));
  }
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  vReal[0] = 0; vReal[1] = 0; vReal[2] = 0; 
  latestHz = FFT.majorPeak();
  latestDb = 20 * log10((sMax - sMin) + 1) + 9.0;
}