/*
 * PROJECT: ESP32 SMART ASTRO BOX
 * VERSION: 6.3 (Ultimate Edition)
 * AUTHOR: Slemrijan-Gislev
 * DATE: 2023-10-21
 * * DESCRIPTION:
 * ASCOM Alpaca compatible Smart Box controlling:
 * 1. Stepper Motor Focuser (Absolute positioning)
 * 2. Dual Channel Dew Heaters (PWM with Auto-Dew algorithm)
 * 3. Weather Station (Temp, Humidity, Pressure, Dew Point)
 * 4. OLED Status Display (IP, Status, Data)
 * * HARDWARE:
 * - ESP32 Dev Kit V1
 * - Stepper Driver (DRV8825/A4988)
 * - BME280 Sensor (I2C)
 * - SH1107 OLED 1.5" (I2C) - 128x128
 * - 12V DC Input
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h> 
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h> 
#include <Preferences.h> 
#include <math.h>

// ======================= WIFI CONFIGURATION =======================
const char* ssid = "ssid";
const char* password = "password";

// STATIC IP SETUP (Change to match your network)
IPAddress local_IP(192, 168, 8, 10);
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
// ==================================================================

const int serverPort = 4567; // Alpaca Standard Port

// --- PIN DEFINITIONS ---
#define MOTOR_STEP_PIN  14  
#define MOTOR_DIR_PIN   12  
#define MOTOR_EN_PIN    13  

#define HEATER_1_PIN    25 
#define HEATER_2_PIN    26 

#define I2C_SDA         21 
#define I2C_SCL         22 

// --- OBJECT INITIALIZATION ---
WebServer server(serverPort);
WiFiUDP udp;
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);
Adafruit_BME280 bme;
Preferences preferences; 
Adafruit_SH1107 display = Adafruit_SH1107(128, 128, &Wire);

// --- GLOBAL VARIABLES ---
volatile long sharedTargetPosition = 0;   
volatile long sharedCurrentPosition = 0;  
volatile bool sharedIsMoving = false;     
volatile bool cmdStop = false;
volatile unsigned long lastMoveTime = 0; 

// Sensor Data
float cachedTemp = 0.0;
float cachedHum = 0.0;
double cachedDewPoint = 0.0;
double cachedPress = 0.0; 
bool sensorOnline = false;

// Heater Settings
int currentPower1 = 0;
int currentPower2 = 0;
bool autoDewEnabled = true;

uint32_t transactionId = 0;
TaskHandle_t TaskCore0;

// ============================================================================
//   CORE 0: NETWORK, SENSORS, DISPLAY & MEMORY TASK
// ============================================================================

void sendResponse(JsonDocument& doc) {
  doc["ClientTransactionID"] = server.arg("ClientTransactionID").toInt();
  doc["ServerTransactionID"] = transactionId++;
  doc["ErrorNumber"] = 0;
  doc["ErrorMessage"] = "";
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

void handleDiscovery() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    packetBuffer[len] = 0;
    String pkt = String(packetBuffer);
    if (pkt.indexOf("alpacadiscovery") >= 0) {
      String msg = "{\"AlpacaPort\":" + String(serverPort) + "}";
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print(msg);
      udp.endPacket();
    }
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.println("ASTRO BOX v6.3");
  display.println("---------------------");

  display.print("IP: ");
  if (WiFi.status() == WL_CONNECTED) {
    display.println(WiFi.localIP());
  } else {
    display.println("No WiFi!");
  }
  display.println("");

  display.print("Temp: "); display.print(cachedTemp, 1); display.println(" C");
  display.print("Hum : "); display.print(cachedHum, 0); display.println(" %");
  display.print("Pres: "); display.print(cachedPress, 0); display.println(" hPa");
  display.print("DewP: "); display.print(cachedDewPoint, 1); display.println(" C");
  display.println("");

  display.println("Focuser Pos:");
  display.setTextSize(2); 
  display.println(sharedCurrentPosition);
  
  display.display();
}

void NetworkLoop(void * pvParameters) {
  
  // --- ALPACA API ENDPOINTS ---
  // Management
  server.on("/management/apiversions", HTTP_GET, []() { JsonDocument doc; JsonArray arr = doc.createNestedArray("Value"); arr.add(1); sendResponse(doc); });
  server.on("/management/v1/configureddevices", HTTP_GET, []() {
    JsonDocument doc; JsonArray arr = doc.createNestedArray("Value");
    JsonObject d1 = arr.createNestedObject(); d1["DeviceName"] = "ESP32 Focuser"; d1["DeviceType"] = "Focuser"; d1["DeviceNumber"] = 0; d1["UniqueID"] = "F001";
    JsonObject d2 = arr.createNestedObject(); d2["DeviceName"] = "Dew Controller"; d2["DeviceType"] = "Switch"; d2["DeviceNumber"] = 0; d2["UniqueID"] = "S001";
    JsonObject d3 = arr.createNestedObject(); d3["DeviceName"] = "Weather Sensor"; d3["DeviceType"] = "ObservingConditions"; d3["DeviceNumber"] = 0; d3["UniqueID"] = "W001";
    sendResponse(doc);
  });

  // Focuser
  server.on("/api/v1/focuser/0/connected", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/focuser/0/connected", HTTP_PUT, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/focuser/0/name", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "ESP32 SmartBox"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/driverinfo", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "Stable Driver"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/driverversion", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "6.3"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/interfaceversion", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 1; sendResponse(doc); });
  server.on("/api/v1/focuser/0/absolute", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/focuser/0/maxstep", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 100000; sendResponse(doc); });
  server.on("/api/v1/focuser/0/stepsize", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 4.0; sendResponse(doc); });
  server.on("/api/v1/focuser/0/temperature", HTTP_GET, []() { JsonDocument doc; doc["Value"] = cachedTemp; sendResponse(doc); });
  server.on("/api/v1/focuser/0/tempcompavailable", HTTP_GET, []() { JsonDocument doc; doc["Value"] = false; sendResponse(doc); });
  server.on("/api/v1/focuser/0/tempcomp", HTTP_GET, []() { JsonDocument doc; doc["Value"] = false; sendResponse(doc); });
  server.on("/api/v1/focuser/0/tempcomp", HTTP_PUT, []() { JsonDocument doc; sendResponse(doc); });
  server.on("/api/v1/focuser/0/position", HTTP_GET, []() { JsonDocument doc; doc["Value"] = sharedCurrentPosition; sendResponse(doc); });
  server.on("/api/v1/focuser/0/ismoving", HTTP_GET, []() { JsonDocument doc; doc["Value"] = sharedIsMoving; sendResponse(doc); });
  server.on("/api/v1/focuser/0/move", HTTP_PUT, []() { if(server.hasArg("Position")) { sharedTargetPosition = server.arg("Position").toInt(); lastMoveTime = millis(); } JsonDocument doc; sendResponse(doc); });
  server.on("/api/v1/focuser/0/halt", HTTP_PUT, []() { cmdStop = true; JsonDocument doc; sendResponse(doc); });

  // Switch (Heaters)
  server.on("/api/v1/switch/0/connected", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/switch/0/connected", HTTP_PUT, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/switch/0/maxswitch", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 3; sendResponse(doc); });
  server.on("/api/v1/switch/0/canwrite", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/switch/0/getswitchname", HTTP_GET, []() { int id = server.arg("Id").toInt(); JsonDocument doc; if (id == 0) doc["Value"] = "Heater 1"; else if (id == 1) doc["Value"] = "Heater 2"; else doc["Value"] = "Auto Dew"; sendResponse(doc); });
  server.on("/api/v1/switch/0/getswitchdescription", HTTP_GET, []() { int id = server.arg("Id").toInt(); JsonDocument doc; if (id == 0) doc["Value"] = "Heater 1 Power"; else if (id == 1) doc["Value"] = "Heater 2 Power"; else doc["Value"] = "Enable Auto Dew"; sendResponse(doc); });
  server.on("/api/v1/switch/0/maxswitchvalue", HTTP_GET, []() { int id = server.arg("Id").toInt(); JsonDocument doc; doc["Value"] = (id < 2) ? 100.0 : 1.0; sendResponse(doc); });
  server.on("/api/v1/switch/0/minswitchvalue", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 0.0; sendResponse(doc); });
  server.on("/api/v1/switch/0/switchstep", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 1.0; sendResponse(doc); });
  server.on("/api/v1/switch/0/getswitchvalue", HTTP_GET, []() { int id = server.arg("Id").toInt(); JsonDocument doc; if (id == 0) doc["Value"] = currentPower1; if (id == 1) doc["Value"] = currentPower2; if (id == 2) doc["Value"] = autoDewEnabled ? 1.0 : 0.0; sendResponse(doc); });
  server.on("/api/v1/switch/0/setswitchvalue", HTTP_PUT, []() { 
    int id = server.arg("Id").toInt(); double val = server.arg("Value").toDouble();
    if (id == 0) { currentPower1 = (int)val; ledcWrite(HEATER_1_PIN, map(currentPower1, 0, 100, 0, 255)); autoDewEnabled = false; }
    if (id == 1) { currentPower2 = (int)val; ledcWrite(HEATER_2_PIN, map(currentPower2, 0, 100, 0, 255)); }
    if (id == 2) { autoDewEnabled = (val > 0.5); }
    JsonDocument doc; sendResponse(doc);
  });

  // Weather (ObservingConditions)
  server.on("/api/v1/observingconditions/0/connected", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/connected", HTTP_PUT, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/temperature", HTTP_GET, []() { JsonDocument doc; doc["Value"] = cachedTemp; sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/humidity", HTTP_GET, []() { JsonDocument doc; doc["Value"] = cachedHum; sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/dewpoint", HTTP_GET, []() { JsonDocument doc; doc["Value"] = cachedDewPoint; sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/pressure", HTTP_GET, []() { JsonDocument doc; doc["Value"] = cachedPress; sendResponse(doc); });

  server.begin();

  unsigned long lastSensorCheck = 0;
  unsigned long retryTimer = 0;
  long lastSavedPosition = -999; 

  if (bme.begin(0x76)) { sensorOnline = true; } else { sensorOnline = false; }
  
  display.begin(0x3C, true); 
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextColor(SH110X_WHITE);
  display.println("Booting...");
  display.display();

  // TASK LOOP
  while(true) {
    server.handleClient();
    handleDiscovery();

    unsigned long now = millis();

    // SENSOR & DISPLAY LOGIC (Every 2 seconds)
    if (now - lastSensorCheck > 2000) {
      lastSensorCheck = now;
      if (sensorOnline) {
        float t = bme.readTemperature();
        float h = bme.readHumidity();
        float p = bme.readPressure() / 100.0F; 
        if (!isnan(t)) {
           cachedTemp = t; cachedHum = h; cachedPress = p;
           double a = 17.27, b = 237.7; 
           double alpha = ((a * t) / (b + t)) + log(h / 100.0);
           cachedDewPoint = (b * alpha) / (a - alpha);

           if (autoDewEnabled) {
             float delta = t - cachedDewPoint; 
             int newPower = 0;
             if (delta < 3.5) {
                float calc = (3.5 - delta) * (100.0 / 3.5);
                if (calc > 100) calc = 100; if (calc < 10) calc = 10;
                newPower = (int)calc;
             }
             currentPower1 = newPower;
             ledcWrite(HEATER_1_PIN, map(currentPower1, 0, 100, 0, 255));
           }
        }
      } else {
        if (now - retryTimer > 5000) {
           retryTimer = now;
           Wire.begin(I2C_SDA, I2C_SCL); 
           if (bme.begin(0x76)) { sensorOnline = true; }
        }
      }
      updateDisplay();
    }

    // LAZY MEMORY LOGIC (Prevents crashing)
    if (!sharedIsMoving) {
        unsigned long timeSinceMove = now - lastMoveTime;
        if (timeSinceMove > 10000 && sharedCurrentPosition != lastSavedPosition) {
             preferences.putLong("savedPos", sharedCurrentPosition);
             lastSavedPosition = sharedCurrentPosition;
        }
    } else {
        lastMoveTime = now;
    }
    
    delay(2); 
  }
}

// ============================================================================
//   CORE 1: STEPPER MOTOR CONTROL
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW); 
  
  stepper.setMaxSpeed(1000);    
  stepper.setAcceleration(500);

  // MEMORY RECALL
  preferences.begin("astrobox", false); 
  long storedPos = preferences.getLong("savedPos", 0); 
  
  stepper.setCurrentPosition(storedPos);
  sharedCurrentPosition = storedPos;
  sharedTargetPosition = storedPos;
  lastMoveTime = millis(); 
  
  ledcAttach(HEATER_1_PIN, 5000, 8); ledcWrite(HEATER_1_PIN, 0);
  ledcAttach(HEATER_2_PIN, 5000, 8); ledcWrite(HEATER_2_PIN, 0);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000); 

  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  
  udp.begin(32227); 

  xTaskCreatePinnedToCore(NetworkLoop, "NetworkTask", 10000, NULL, 1, &TaskCore0, 0);
}

void loop() {
  if (cmdStop) {
    stepper.stop();
    sharedTargetPosition = stepper.currentPosition(); 
    stepper.moveTo(sharedTargetPosition);
    cmdStop = false;
    lastMoveTime = millis(); 
  }

  if (sharedTargetPosition != stepper.targetPosition()) {
    stepper.moveTo(sharedTargetPosition);
    lastMoveTime = millis(); 
  }

  bool moving = stepper.run();
  sharedIsMoving = moving;
  sharedCurrentPosition = stepper.currentPosition();
  
  if(moving) {
      lastMoveTime = millis(); 
  }
}
