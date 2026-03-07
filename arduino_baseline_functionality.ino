/*
   CAPSTONE VEST - FINAL FIRMWARE V12 (Activity Logging + Speed Smoothing)

   Hardware:
   - Arduino Uno R4 WiFi
   - SparkFun Qwiic OpenLog (Wire1)
   - SparkFun GPS (Wire1)
   - MPU6050 (Wire - Standard)
   - Adafruit ST7789 (SPI)

   V12 changes:
   - Speed smoothing: Median-of-5 + EMA + hysteresis + decay-to-zero
   - CSV: Add ACTIVITY column (STOP/WALK/JOG/RUN/SPRINT)
   - Speed decimals: LCD shows 2 dp, CSV logs 3 dp
   - Pace: corrected to Min/Mile
   - RTC month fix: RTC.getMonth() is typically 0-based -> +1 for logging/filename
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <ArduinoBLE.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFiS3.h>
#include <RTC.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"

// ---------------- LCD CONFIGURATION ----------------
#define TFT_CS    10
#define TFT_DC     7
#define TFT_RST    6
#define ST77XX_DARKGREY 0x4208
#define ST77XX_ORANGE   0xFA60
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// ---------------- HARDWARE OBJECTS ----------------
Adafruit_MPU6050 mpu;
SFE_UBLOX_GNSS myGNSS;
WiFiServer server(80);
OpenLog myLog;

// ---------------- BLE VARIABLES ----------------
const char* polarServiceUuid = "180D";
const char* polarCharUuid = "2A37";
BLEDevice polarDevice;
BLECharacteristic hrChar;
bool polarConnected = false;

// ================================================================
// SETTINGS (User-configurable via WiFi page)
// ================================================================
float stepThreshold = 11.0;
float minSpeedThreshold = 0.8;   // used as start-moving threshold (m/s)
int hrLow = 60, hrHigh = 100;
int goalPaceMin = 8, goalPaceSec = 30;
int lcdFreqMs = 2000;
int logFreqMs = 1000;

bool setHrGoal = true;
bool setPaceGoal = true;
bool useGPS = true;
bool useBreathing = true;
bool isConfigured = false;

// ================================================================
// SPEED PROCESSING TUNING
// ================================================================
// If your GNSS library supports getFixType/getSIV, keep ON.
// If compile errors happen, set to 0.
#define USE_GNSS_QUALITY_GATING 1

static const uint8_t SPEED_MEDIAN_N = 5;
static const float   SPEED_EMA_ALPHA = 0.25f;     // lower = smoother (more lag)
static const float   HYSTERESIS_DELTA = 0.20f;    // stopThresh = startThresh - delta
static const float   STOP_DECAY_MULT = 0.85f;     // decay when stopped
static const float   DISPLAY_DEADBAND = 0.05f;    // clamp tiny values to 0
static const float   MAX_REASONABLE_SPEED = 15.0f; // m/s safety cap

static const uint32_t SPEED_UPDATE_MS = 200;      // display/log speed refresh cadence
static const uint32_t GNSS_STALE_MS   = 2000;     // if no good GNSS for this long, decay harder

// ================================================================
// DATA
// ================================================================
int currentHR = 0;
int stepCount = 0;
unsigned long lastStepTime = 0;
bool isStepHigh = false;
const int STEP_DELAY_MS = 300;

long totalBreaths = 0;
float rrHistory[3] = {0, 0, 0};

// ---------------- SPEED STATE ----------------
float displaySpeed = 0.0f;              // m/s (filtered, user-facing)
int displayPaceMin = 0, displayPaceSec = 0;

static float speedBuf[SPEED_MEDIAN_N];
static uint8_t speedBufCount = 0;
static uint8_t speedBufIdx = 0;

static float speedEma = 0.0f;
static bool  moving = false;

static unsigned long lastSpeedUpdateMillis = 0;
static unsigned long lastGoodGnssMillis = 0;

// ---------------- LOGGING ----------------
String logFileName = "LOG.CSV";
unsigned long arduinoStartMillis = 0;
unsigned long lastLogMillis = 0;
unsigned long lastLcdMillis = 0;

// ================================================================
// PROTOTYPES
// ================================================================
void startWiFiConfig();
void parseConfig(String req);
String getParam(String req, String tag);
void sendSetupPage(WiFiClient c);
void sendSuccessPage(WiFiClient c);

void drawDashboardLayout();
void updateDashboard();
void processPolarData();
void runStepCounter();
void pollGPS();
void logData(unsigned long ms);

// Speed helpers
float medianOfBuffer(const float* buf, uint8_t count);
void  pushSpeedSample(float s);
void  updateSpeedFromPipeline(unsigned long nowMillis, bool gnssGood);

// Activity classification
const char* classifyActivity(float speedMS);

// ================================================================
// SETUP
// ================================================================
void setup() {
  tft.init(240, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  RTC.begin();

  // 1) WiFi config & RTC sync
  startWiFiConfig();

  // 2) Switch radio
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(20, 100); tft.setTextColor(ST77XX_YELLOW); tft.setTextSize(3);
  tft.println("SWITCHING");
  tft.setCursor(20, 140); tft.println("RADIO...");

  WiFi.disconnect();
  WiFi.end();
  delay(3000);

  // 3) Sensor checks
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);

  Wire.begin();
  Wire1.begin();

  // GPS
  tft.setCursor(10, 10); tft.setTextColor(ST77XX_WHITE); tft.print("GPS: ");
  if (useGPS) {
    while (!myGNSS.begin(Wire1)) {
      tft.fillRect(60, 10, 140, 20, ST77XX_BLACK);
      tft.setCursor(60, 10); tft.setTextColor(ST77XX_RED); tft.print("RETRY...");
      delay(1000);
    }
    tft.fillRect(60, 10, 140, 20, ST77XX_BLACK);
    tft.setCursor(60, 10); tft.setTextColor(ST77XX_GREEN); tft.println("OK");
    myGNSS.setI2COutput(COM_TYPE_UBX);
    myGNSS.setNavigationFrequency(5); // 5 Hz
  } else {
    tft.setTextColor(ST77XX_ORANGE); tft.println("DISABLED");
  }

  // IMU
  tft.setCursor(10, 40); tft.setTextColor(ST77XX_WHITE); tft.print("IMU: ");
  while (!mpu.begin()) {
    tft.fillRect(60, 40, 140, 20, ST77XX_BLACK);
    tft.setCursor(60, 40); tft.setTextColor(ST77XX_RED); tft.print("RETRY...");
    delay(1000);
  }
  tft.fillRect(60, 40, 140, 20, ST77XX_BLACK);
  tft.setCursor(60, 40); tft.setTextColor(ST77XX_GREEN); tft.println("OK");

  // LOG
  tft.setCursor(10, 70); tft.setTextColor(ST77XX_WHITE); tft.print("LOG: ");
  if (!myLog.begin(QOL_DEFAULT_ADDRESS, Wire1)) {
    tft.setTextColor(ST77XX_RED); tft.println("FAIL");
  } else {
    tft.setTextColor(ST77XX_GREEN); tft.println("OK");

    RTCTime now;
    RTC.getTime(now);

    // FIX: Month is typically 0-based with RTC.h -> add +1 for human-readable
    int month1 = (int)now.getMonth() + 1;

    char fileNameBuff[13];
    sprintf(fileNameBuff, "%02d%02d%02d%02d.CSV",
            month1, now.getDayOfMonth(), now.getHour(), now.getMinutes());
    logFileName = String(fileNameBuff);

    myLog.append(logFileName);

    myLog.print("RTC_START_DATE,");
    myLog.print(String(now.getDayOfMonth()) + "/" + String(month1) + "/" + String(now.getYear()));
    myLog.println();

    myLog.print("RTC_START_TIME,");
    myLog.print(String(now.getHour()) + ":" + String(now.getMinutes()) + ":" + String(now.getSeconds()));
    myLog.println();

    arduinoStartMillis = millis();
    myLog.print("ARDUINO_START_MILLIS,"); myLog.println(arduinoStartMillis);

    // NEW: ACTIVITY column
    myLog.println("MILLIS,HR,PACE_MIN,PACE_SEC,SPEED_MS,ACTIVITY,BREATHS,STEPS");
  }

  // BLE
  tft.setCursor(10, 100); tft.setTextColor(ST77XX_WHITE); tft.print("BLE: ");
  if (!BLE.begin()) {
    tft.setTextColor(ST77XX_RED); tft.println("FAIL!");
    while (1);
  }
  tft.setTextColor(ST77XX_GREEN); tft.println("OK");

  // 4) Polar pairing
  tft.setCursor(10, 140); tft.setTextColor(ST77XX_BLUE); tft.println("Scan Polar H10...");

  BLE.scanForUuid(polarServiceUuid);
  while (!polarConnected) {
    BLEDevice peripheral = BLE.available();
    if (peripheral) {
      BLE.stopScan();
      if (peripheral.connect()) {
        if (peripheral.discoverAttributes()) {
          hrChar = peripheral.characteristic(polarCharUuid);
          if (hrChar && hrChar.subscribe()) {
            polarConnected = true;
            polarDevice = peripheral;

            tft.fillRect(10, 140, 240, 30, ST77XX_BLACK);
            tft.setCursor(10, 140); tft.setTextColor(ST77XX_GREEN); tft.println("Polar Paired!");
            delay(1000);
          }
        }
      }
      if (!polarConnected) BLE.scanForUuid(polarServiceUuid);
    }
  }

  drawDashboardLayout();
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  unsigned long currentMillis = millis();

  if (polarConnected) {
    BLE.poll();
    if (hrChar.valueUpdated()) processPolarData();
    if (!polarDevice.connected()) polarConnected = false;
  }

  runStepCounter();
  if (useGPS) pollGPS();

  if (currentMillis - lastLogMillis >= (unsigned long)logFreqMs) {
    logData(currentMillis);
    lastLogMillis = currentMillis;
  }

  if (currentMillis - lastLcdMillis >= (unsigned long)lcdFreqMs) {
    updateDashboard();
    lastLcdMillis = currentMillis;
  }
}

// ================================================================
// LOGGING (CSV)
// ================================================================
void logData(unsigned long ms) {
  String dataRow = String(ms) + ",";
  dataRow += String(currentHR) + ",";

  if (useGPS) {
    // Pace (Min/Mile)
    dataRow += String(displayPaceMin) + ",";
    dataRow += String(displayPaceSec) + ",";

    // Speed: log 3 decimals (matches mm/s resolution best)
    dataRow += String(displaySpeed, 3) + ",";

    // Activity label from filtered speed
    dataRow += String(classifyActivity(displaySpeed)) + ",";
  } else {
    dataRow += "0,0,0,NA,";
  }

  dataRow += String(totalBreaths) + ",";
  dataRow += String(stepCount);

  myLog.append(logFileName);
  myLog.println(dataRow);
}

// ================================================================
// ACTIVITY CLASSIFICATION
// ================================================================
const char* classifyActivity(float speedMS) {
  // You can tune these cutoffs any time without touching the filter pipeline.
  // Based on your ranges screenshot (m/s):
  // Jogging: 1.8–4.9
  // Running: 4.5–6.5 (we use 4.5+)
  // Sprinting: > 7.0
  // Walking: below 1.8 but above "stopped"
  const float STOP_CUTOFF = 0.50f; // treat near-zero as STOP for analysis

  if (speedMS < STOP_CUTOFF) return "STOP";
  if (speedMS < 1.80f)       return "WALK";
  if (speedMS < 4.50f)       return "JOG";
  if (speedMS < 7.00f)       return "RUN";
  return "SPRINT";
}

// ================================================================
// WIFI SETUP / CONFIG
// ================================================================
void startWiFiConfig() {
  WiFi.config(IPAddress(192, 168, 4, 1));
  WiFi.beginAP("Vest_Setup", "12345678");
  server.begin();

  tft.setTextColor(ST77XX_CYAN); tft.setTextSize(2);
  tft.println("WI-FI SETUP");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("\nConnect to:");
  tft.setTextColor(ST77XX_GREEN); tft.println("Vest_Setup");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("Pass: 12345678");
  tft.println("\nGo to URL:");
  tft.setTextColor(ST77XX_GREEN); tft.println("192.168.4.1");

  while (!isConfigured) {
    WiFiClient client = server.available();
    if (client) {
      String request = "";
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          request += c;
          if (c == '\n') {
            if (request.indexOf("GET /START") >= 0) {
              parseConfig(request);
              isConfigured = true;
              sendSuccessPage(client);
              break;
            } else {
              sendSetupPage(client);
              break;
            }
          }
        }
      }
      client.stop();
    }
  }
}

void parseConfig(String req) {
  // set RTC from browser epoch (seconds)
  int tIndex = req.indexOf("time=");
  if (tIndex != -1) {
    int endT = req.indexOf("&", tIndex);
    if (endT == -1) endT = req.indexOf(" ", tIndex);
    String tStr = req.substring(tIndex + 5, endT);
    unsigned long epoch = strtoul(tStr.c_str(), NULL, 10);
    RTCTime timeToSet(epoch);
    RTC.setTime(timeToSet);
  }

  setHrGoal = (req.indexOf("hrGoal=yes") >= 0);
  setPaceGoal = (req.indexOf("paceGoal=yes") >= 0);

  if (req.indexOf("useGPS=yes") == -1) useGPS = false;
  if (req.indexOf("useBreath=yes") == -1) useBreathing = false;

  hrLow = getParam(req, "hrL=").toInt();
  hrHigh = getParam(req, "hrH=").toInt();
  goalPaceMin = getParam(req, "pM=").toInt();
  goalPaceSec = getParam(req, "pS=").toInt();
  stepThreshold = getParam(req, "stepT=").toFloat();

  float tempMinS = getParam(req, "minS=").toFloat();
  if (tempMinS > 0.0f) minSpeedThreshold = tempMinS;

  lcdFreqMs = getParam(req, "lcdF=").toInt() * 1000;

  float pointsPerSec = getParam(req, "logR=").toFloat();
  if (pointsPerSec > 10.0f) pointsPerSec = 10.0f;
  if (pointsPerSec < 0.1f)  pointsPerSec = 0.1f;
  logFreqMs = (int)(1000.0f / pointsPerSec);
}

String getParam(String req, String tag) {
  int i = req.indexOf(tag);
  if (i == -1) return "0";
  int end = req.indexOf("&", i);
  if (end == -1) end = req.indexOf(" ", i);
  return req.substring(i + tag.length(), end);
}

void sendSetupPage(WiFiClient c) {
  c.println("HTTP/1.1 200 OK\nContent-type:text/html\n");
  c.println("<!DOCTYPE HTML><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>");
  c.println("<style>body{font-family:sans-serif;background:#f2f2f7;padding:20px;text-align:center;}");
  c.println(".card{background:white;padding:15px;border-radius:12px;margin-bottom:15px;text-align:left;box-shadow:0 2px 5px rgba(0,0,0,0.1);}");
  c.println("input,select{width:100%;padding:10px;margin-top:5px;border:1px solid #ccc;border-radius:8px;}");
  c.println(".row{display:flex;gap:10px;} .col{flex:1;}");
  c.println("input[type=submit]{background:#007AFF;color:white;font-weight:bold;border:none;cursor:pointer;}");
  c.println("</style></head><body>");

  c.println("<h2>Vest Setup</h2>");

  c.println("<script>");
  c.println("function togglePace() {");
  c.println("  var gps = document.getElementById('gpsCheck');");
  c.println("  var pace = document.getElementById('paceCheck');");
  c.println("  if(!gps.checked) { pace.checked = false; pace.disabled = true; }");
  c.println("  else { pace.disabled = false; }");
  c.println("}");
  c.println("function sendData() {");
  c.println("  var now = new Date();");
  c.println("  var utc = Math.floor(now.getTime()/1000);");
  c.println("  var off = now.getTimezoneOffset() * 60;");
  c.println("  var local = utc - off;");
  c.println("  document.getElementById('tField').value = local;");
  c.println("  document.forms[0].submit();");
  c.println("}");
  c.println("</script>");

  c.println("<form action='/START' onsubmit='event.preventDefault(); sendData();'>");
  c.println("<input type='hidden' name='time' id='tField'>");

  c.println("<div class='card'><h3>Sensor Settings</h3>");
  c.println("<label><input type='checkbox' id='gpsCheck' name='useGPS' value='yes' checked onclick='togglePace()'> Enable GPS</label><br>");
  c.println("<label><input type='checkbox' name='useBreath' value='yes' checked> Track Breathing</label><br><br>");
  c.println("<label>Step Threshold (G-Force)</label><input type='number' name='stepT' value='11' step='0.1'>");
  c.println("<br><br><label>Min Speed Threshold (m/s)</label>");
  c.println("<input type='number' name='minS' value='0.8' step='0.1'>");
  c.println("<br><br><label>Data Points per Second</label>");
  c.println("<input type='number' name='logR' value='1' step='0.1' max='10'>");
  c.println("<small style='color:red'>Max: 10</small>");
  c.println("</div>");

  c.println("<div class='card'><h3>Goals</h3>");
  c.println("<label><input type='checkbox' name='hrGoal' value='yes' checked> HR Goal (Range)</label>");
  c.println("<div class='row'><div class='col'><label>Low</label><input type='number' name='hrL' value='60'></div>");
  c.println("<div class='col'><label>High</label><input type='number' name='hrH' value='100'></div></div>");
  c.println("<br><label><input type='checkbox' id='paceCheck' name='paceGoal' value='yes' checked> Pace Goal (Min/Mile)</label>");
  c.println("<div class='row'><div class='col'><label>Min</label><input type='number' name='pM' value='8'></div>");
  c.println("<div class='col'><label>Sec</label><input type='number' name='pS' value='30'></div></div></div>");

  c.println("<div class='card'><h3>Display Settings</h3>");
  c.println("<div class='row'><div class='col'><label>Screen Refresh (s)</label><input type='number' name='lcdF' value='2'></div>");
  c.println("</div></div>");

  c.println("<input type='submit' value='START WORKOUT'>");
  c.println("</form></body></html>");
}

void sendSuccessPage(WiFiClient c) {
  c.println("HTTP/1.1 200 OK\nContent-type:text/html\n");
  c.println("<body><h1>Ready!</h1><p>Starting...</p></body>");
}

// ================================================================
// DISPLAY
// ================================================================
void drawDashboardLayout() {
  tft.fillScreen(ST77XX_BLACK);
  tft.drawFastHLine(0, 100, 320, ST77XX_DARKGREY);
  tft.drawFastHLine(0, 170, 320, ST77XX_DARKGREY);
  tft.drawFastVLine(160, 0, 170, ST77XX_DARKGREY);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(10, 10);  tft.print("HR");
  tft.setCursor(170, 10); tft.print("PACE");
  tft.setCursor(10, 105); tft.print("SPEED (m/s)");
  tft.setCursor(170, 105); tft.print("BREATHS");
  tft.setCursor(250, 105); tft.print("STEPS");
}

void updateDashboard() {
  // HR
  tft.setTextSize(4);
  tft.fillRect(10, 25, 100, 35, ST77XX_BLACK);
  tft.setCursor(10, 25);
  tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
  if (polarConnected) {
    tft.print(currentHR);
  } else {
    tft.setTextSize(3); tft.print("???");
  }

  tft.setTextSize(1); tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
  tft.setCursor(10, 65);
  tft.print("(Goal: "); tft.print(hrLow); tft.print("-"); tft.print(hrHigh); tft.print(")");

  // Pace
  tft.fillRect(170, 25, 140, 35, ST77XX_BLACK);
  tft.setTextSize(4);
  tft.setCursor(170, 25); tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);

  if (useGPS) {
    if (displayPaceMin > 0) {
      tft.print(displayPaceMin); tft.print(":");
      if (displayPaceSec < 10) tft.print("0");
      tft.print(displayPaceSec);
    } else {
      tft.setTextSize(3); tft.print("--:--");
    }
  } else {
    tft.setTextSize(3); tft.print("OFF");
  }

  if (useGPS) {
    tft.setTextSize(1); tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
    tft.setCursor(170, 65);
    tft.print("(Goal: <"); tft.print(goalPaceMin); tft.print(":");
    if (goalPaceSec < 10) tft.print("0");
    tft.print(goalPaceSec); tft.print(")");
  }

  // Speed (LCD shows 2 decimals)
  tft.fillRect(10, 130, 130, 25, ST77XX_BLACK);
  tft.setTextSize(2); tft.setCursor(10, 130); tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  if (useGPS) tft.print(displaySpeed, 2);
  else { tft.setTextSize(1); tft.print("OFF"); }

  // Breaths
  tft.fillRect(170, 130, 60, 25, ST77XX_BLACK);
  tft.setTextSize(2); tft.setCursor(170, 130); tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
  if (useBreathing) tft.print(totalBreaths);
  else { tft.setTextSize(1); tft.print("OFF"); }

  // Steps
  tft.fillRect(250, 130, 60, 25, ST77XX_BLACK);
  tft.setTextSize(2); tft.setCursor(250, 130); tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.print(stepCount);

  // Status row
  tft.setTextSize(2);

  tft.setCursor(10, 190);
  if (!polarConnected) {
    tft.setTextColor(ST77XX_DARKGREY, ST77XX_BLACK); tft.print("NO SIGNAL  ");
  } else {
    if (currentHR > hrHigh) { tft.setTextColor(ST77XX_RED, ST77XX_BLACK); tft.print("HR: HIGH!  "); }
    else if (currentHR < hrLow) { tft.setTextColor(ST77XX_BLUE, ST77XX_BLACK); tft.print("HR: LOW    "); }
    else { tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK); tft.print("HR: GOOD   "); }
  }

  tft.setCursor(170, 190);
  if (useGPS && setPaceGoal) {
    if (displayPaceMin == 0 && displayPaceSec == 0) {
      tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
      tft.print("READY      ");
    } else {
      int currentSec = (displayPaceMin * 60) + displayPaceSec;
      int goalSec = (goalPaceMin * 60) + goalPaceSec;
      if (currentSec > goalSec) {
        tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
        tft.print("SPEED UP!  ");
      } else {
        tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        tft.print("ON PACE    ");
      }
    }
  } else {
    tft.setTextColor(ST77XX_DARKGREY, ST77XX_BLACK);
    tft.print("           ");
  }
}

// ================================================================
// BLE / HR / BREATHING
// ================================================================
void processPolarData() {
  const uint8_t* data = hrChar.value();
  int length = hrChar.valueLength();
  byte flags = data[0];
  int idx = 1;

  if (bitRead(flags, 0) == 0) {
    currentHR = data[idx];
    idx += 1;
  } else {
    currentHR = data[idx] | (data[idx + 1] << 8);
    idx += 2;
  }

  if (useBreathing && bitRead(flags, 4) == 1) {
    if (bitRead(flags, 3) == 1) idx += 2;
    while (idx + 1 < length) {
      uint16_t rr_raw = data[idx] | (data[idx + 1] << 8);
      idx += 2;
      float rr_ms = (rr_raw / 1024.0f) * 1000.0f;

      rrHistory[0] = rrHistory[1];
      rrHistory[1] = rrHistory[2];
      rrHistory[2] = rr_ms;

      if (rrHistory[0] > 0 && rrHistory[1] > rrHistory[0] + 5 && rrHistory[1] > rrHistory[2]) {
        totalBreaths++;
      }
    }
  }
}

// ================================================================
// STEP COUNTER
// ================================================================
void runStepCounter() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float magnitude = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
  if (magnitude > stepThreshold) {
    if ((millis() - lastStepTime) > (unsigned long)STEP_DELAY_MS) {
      if (!isStepHigh) {
        stepCount++;
        lastStepTime = millis();
        isStepHigh = true;
      }
    }
  } else {
    isStepHigh = false;
  }
}

// ================================================================
// GPS / SPEED PIPELINE
// ================================================================
void pollGPS() {
  unsigned long nowMillis = millis();

  if (!myGNSS.getPVT()) {
    bool gnssGood = (nowMillis - lastGoodGnssMillis) <= GNSS_STALE_MS;
    updateSpeedFromPipeline(nowMillis, gnssGood);
    return;
  }

  long speedMM = myGNSS.getGroundSpeed(); // mm/s
  float speedMS = speedMM / 1000.0f;

  if (speedMS < 0) speedMS = 0;
  if (speedMS > MAX_REASONABLE_SPEED) speedMS = MAX_REASONABLE_SPEED;

  bool gnssGood = true;

#if USE_GNSS_QUALITY_GATING
  uint8_t fixType = myGNSS.getFixType();
  uint8_t siv     = myGNSS.getSIV();
  if (fixType < 3 || siv < 6) gnssGood = false; // tighten/loosen as needed
#endif

  if (gnssGood) {
    lastGoodGnssMillis = nowMillis;
    pushSpeedSample(speedMS);
  }

  updateSpeedFromPipeline(nowMillis, gnssGood);
}

void pushSpeedSample(float s) {
  speedBuf[speedBufIdx] = s;
  speedBufIdx = (speedBufIdx + 1) % SPEED_MEDIAN_N;
  if (speedBufCount < SPEED_MEDIAN_N) speedBufCount++;
}

float medianOfBuffer(const float* buf, uint8_t count) {
  if (count == 0) return 0.0f;

  float temp[SPEED_MEDIAN_N];
  for (uint8_t i = 0; i < count; i++) temp[i] = buf[i];

  // insertion sort
  for (uint8_t i = 1; i < count; i++) {
    float key = temp[i];
    int j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j--;
    }
    temp[j + 1] = key;
  }

  if (count % 2 == 1) return temp[count / 2];
  return 0.5f * (temp[count / 2 - 1] + temp[count / 2]);
}

void updateSpeedFromPipeline(unsigned long nowMillis, bool gnssGood) {
  if (nowMillis - lastSpeedUpdateMillis < SPEED_UPDATE_MS) return;
  lastSpeedUpdateMillis = nowMillis;

  bool stale = (nowMillis - lastGoodGnssMillis) > GNSS_STALE_MS;
  if (stale) gnssGood = false;

  // Build linear list of last samples for median
  float linear[SPEED_MEDIAN_N];
  for (uint8_t i = 0; i < speedBufCount; i++) {
    uint8_t idx = (speedBufIdx + SPEED_MEDIAN_N - speedBufCount + i) % SPEED_MEDIAN_N;
    linear[i] = speedBuf[idx];
  }

  float med = medianOfBuffer(linear, speedBufCount);

  // EMA update (or decay if no good GNSS)
  if (gnssGood) {
    speedEma = (SPEED_EMA_ALPHA * med) + ((1.0f - SPEED_EMA_ALPHA) * speedEma);
  } else {
    speedEma *= 0.90f;
  }

  // Hysteresis thresholds
  float startMovingThresh = minSpeedThreshold;
  float stopMovingThresh  = minSpeedThreshold - HYSTERESIS_DELTA;
  if (stopMovingThresh < 0.0f) stopMovingThresh = 0.0f;

  if (!moving && speedEma >= startMovingThresh) moving = true;
  if ( moving && speedEma <= stopMovingThresh)  moving = false;

  // Decay-to-zero when stopped
  float out = speedEma;
  if (!moving) {
    out *= STOP_DECAY_MULT;
    speedEma = out;
  }

  if (out < DISPLAY_DEADBAND) out = 0.0f;
  displaySpeed = out;

  // Pace (Min/Mile) from speed (m/s)
  if (displaySpeed > 0.1f) {
    // minutes per mile = (1609.344/speed)/60 = 26.8224/speed
    float pace_decimal = 26.8224f / displaySpeed;
    displayPaceMin = (int)pace_decimal;
    displayPaceSec = (int)((pace_decimal - displayPaceMin) * 60.0f);
    if (displayPaceSec >= 60) { displayPaceSec -= 60; displayPaceMin += 1; }
  } else {
    displayPaceMin = 0;
    displayPaceSec = 0;
  }
}
