/*
 * ------------------------------------------------------------------------
 * SMART ASTRO BOX - ASCOM ALPACA EDITION
 * ------------------------------------------------------------------------
 * Devices: Focuser, Dual Dew Heater (PWM), Weather Sensor (BME280)
 * Protocol: ASCOM Alpaca (WiFi/UDP)
 * Platform: ESP32 (WROOM)
 * ------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h> 
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <math.h>

// ======================= WIFI & IP SETUP =======================
// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "ssid";
const char* password = "password";

// STATIC IP CONFIGURATION
// Useful to ensure N.I.N.A. always finds the device at the same address.
// Adjust these IPs to match your router's subnet (e.g., 192.168.1.x or 192.168.0.x)
IPAddress local_IP(192, 168, 8, 10);     // The fixed IP for this device
IPAddress gateway(192, 168, 8, 1);       // Your router's IP
IPAddress subnet(255, 255, 255, 0);      // Standard subnet mask
// ===============================================================

const int serverPort = 4567; // Default Alpaca Port

// --- PIN CONFIGURATION (ESP32 WROOM) ---
#define MOTOR_STEP_PIN  14  
#define MOTOR_DIR_PIN   12  
#define MOTOR_EN_PIN    13  

#define HEATER_1_PIN    25  // Primary Heater (Auto-Dew Capable)
#define HEATER_2_PIN    26  // Secondary Heater (Manual)

#define I2C_SDA         21  // BME280 Data
#define I2C_SCL         22  // BME280 Clock

WebServer server(serverPort);
WiFiUDP udp;
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);
Adafruit_BME280 bme;

long targetPosition = 0;
bool isMoving = false;

// Heater Variables
int currentPower1 = 0;
int currentPower2 = 0;
bool autoDewEnabled = true;

uint32_t transactionId = 0;

// Helper to create Alpaca JSON responses
void sendResponse(JsonDocument& doc) {
  doc["ClientTransactionID"] = server.arg("ClientTransactionID").toInt();
  doc["ServerTransactionID"] = transactionId++;
  doc["ErrorNumber"] = 0;
  doc["ErrorMessage"] = "";
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// Alpaca Discovery Protocol (UDP)
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

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- Motor Setup ---
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW); // Enable motor
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);

  // --- PWM Heater Setup ---
  ledcAttach(HEATER_1_PIN, 5000, 8); // 5 kHz, 8-bit resolution
  ledcWrite(HEATER_1_PIN, 0);
  
  ledcAttach(HEATER_2_PIN, 5000, 8); 
  ledcWrite(HEATER_2_PIN, 0);

  // --- Sensor Setup ---
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bme.begin(0x76)) {
     if (!bme.begin(0x77)) Serial.println("ERROR: No BME280 sensor found! Check wiring.");
  }

  // --- WiFi Setup with Static IP ---
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("ERROR: STA Failed to configure Static IP");
  }
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  udp.begin(32227); // Alpaca Discovery Port

  // --- API DEFINITIONS ---
  
  // Management
  server.on("/management/apiversions", HTTP_GET, []() {
    JsonDocument doc; JsonArray arr = doc.createNestedArray("Value"); arr.add(1); sendResponse(doc);
  });
  
  server.on("/management/v1/configureddevices", HTTP_GET, []() {
    JsonDocument doc; JsonArray arr = doc.createNestedArray("Value");
    JsonObject d1 = arr.createNestedObject(); d1["DeviceName"] = "ESP32 Focuser"; d1["DeviceType"] = "Focuser"; d1["DeviceNumber"] = 0; d1["UniqueID"] = "F001";
    JsonObject d2 = arr.createNestedObject(); d2["DeviceName"] = "Dew Controller"; d2["DeviceType"] = "Switch"; d2["DeviceNumber"] = 0; d2["UniqueID"] = "S001";
    JsonObject d3 = arr.createNestedObject(); d3["DeviceName"] = "Weather Sensor"; d3["DeviceType"] = "ObservingConditions"; d3["DeviceNumber"] = 0; d3["UniqueID"] = "W001";
    sendResponse(doc);
  });

  // 1. FOCUSER API
  server.on("/api/v1/focuser/0/connected", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/focuser/0/connected", HTTP_PUT, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/focuser/0/name", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "ESP32 Focuser"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/description", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "ESP32 Alpaca Driver"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/driverinfo", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "Custom ESP32 Driver"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/driverversion", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "2.0"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/interfaceversion", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 1; sendResponse(doc); });
  
  server.on("/api/v1/focuser/0/absolute", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/focuser/0/maxstep", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 100000; sendResponse(doc); });
  server.on("/api/v1/focuser/0/stepsize", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 4.0; sendResponse(doc); });
  
  // Reports ambient temp to N.I.N.A. for autofocus triggers
  server.on("/api/v1/focuser/0/temperature", HTTP_GET, []() { JsonDocument doc; doc["Value"] = bme.readTemperature(); sendResponse(doc); });
  
  server.on("/api/v1/focuser/0/tempcompavailable", HTTP_GET, []() { JsonDocument doc; doc["Value"] = false; sendResponse(doc); });
  server.on("/api/v1/focuser/0/tempcomp", HTTP_GET, []() { JsonDocument doc; doc["Value"] = false; sendResponse(doc); });
  server.on("/api/v1/focuser/0/tempcomp", HTTP_PUT, []() { JsonDocument doc; sendResponse(doc); });

  server.on("/api/v1/focuser/0/position", HTTP_GET, []() { JsonDocument doc; doc["Value"] = stepper.currentPosition(); sendResponse(doc); });
  server.on("/api/v1/focuser/0/ismoving", HTTP_GET, []() { JsonDocument doc; doc["Value"] = (stepper.distanceToGo() != 0); sendResponse(doc); });
  
  server.on("/api/v1/focuser/0/move", HTTP_PUT, []() { 
    if(server.hasArg("Position")) { targetPosition = server.arg("Position").toInt(); stepper.moveTo(targetPosition); }
    JsonDocument doc; sendResponse(doc); 
  });
  server.on("/api/v1/focuser/0/halt", HTTP_PUT, []() { stepper.stop(); targetPosition = stepper.currentPosition(); stepper.moveTo(targetPosition); JsonDocument doc; sendResponse(doc); });

  // 2. SWITCH API (Dew Heaters)
  server.on("/api/v1/switch/0/connected", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/switch/0/connected", HTTP_PUT, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/switch/0/maxswitch", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 3; sendResponse(doc); }); // 3 switches
  server.on("/api/v1/switch/0/canwrite", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  
  server.on("/api/v1/switch/0/getswitchname", HTTP_GET, []() { 
    int id = server.arg("Id").toInt(); JsonDocument doc; 
    if (id == 0) doc["Value"] = "Heater 1 (Primary)";
    else if (id == 1) doc["Value"] = "Heater 2 (Aux)";
    else doc["Value"] = "Auto Dew Control"; 
    sendResponse(doc); 
  });
  
  server.on("/api/v1/switch/0/getswitchdescription", HTTP_GET, []() { 
    int id = server.arg("Id").toInt(); JsonDocument doc; 
    if (id == 0) doc["Value"] = "Pin 25 Power (0-100%)";
    else if (id == 1) doc["Value"] = "Pin 26 Power (0-100%)";
    else doc["Value"] = "Enable Automatic Dew Control based on Dew Point"; 
    sendResponse(doc); 
  });

  server.on("/api/v1/switch/0/maxswitchvalue", HTTP_GET, []() { 
    int id = server.arg("Id").toInt(); JsonDocument doc; 
    doc["Value"] = (id < 2) ? 100.0 : 1.0; // Sliders vs Toggle
    sendResponse(doc); 
  });
  server.on("/api/v1/switch/0/minswitchvalue", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 0.0; sendResponse(doc); });
  server.on("/api/v1/switch/0/switchstep", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 1.0; sendResponse(doc); });

  server.on("/api/v1/switch/0/getswitchvalue", HTTP_GET, []() { 
    int id = server.arg("Id").toInt(); JsonDocument doc; 
    if (id == 0) doc["Value"] = currentPower1;
    if (id == 1) doc["Value"] = currentPower2;
    if (id == 2) doc["Value"] = autoDewEnabled ? 1.0 : 0.0;
    sendResponse(doc);
  });
  
  server.on("/api/v1/switch/0/setswitchvalue", HTTP_PUT, []() { 
    int id = server.arg("Id").toInt(); double val = server.arg("Value").toDouble();
    
    if (id == 0) { // Heater 1
      if (val > 100) val = 100; if (val < 0) val = 0; 
      currentPower1 = (int)val; 
      ledcWrite(HEATER_1_PIN, map(currentPower1, 0, 100, 0, 255)); 
      autoDewEnabled = false; // Disable auto if manual override
    }
    if (id == 1) { // Heater 2
      if (val > 100) val = 100; if (val < 0) val = 0; 
      currentPower2 = (int)val; 
      ledcWrite(HEATER_2_PIN, map(currentPower2, 0, 100, 0, 255)); 
    }
    if (id == 2) { // Auto Toggle
      autoDewEnabled = (val > 0.5); 
    }
    JsonDocument doc; sendResponse(doc);
  });

  // 3. WEATHER SENSOR API
  server.on("/api/v1/observingconditions/0/connected", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/connected", HTTP_PUT, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/temperature", HTTP_GET, []() { JsonDocument doc; doc["Value"] = bme.readTemperature(); sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/humidity", HTTP_GET, []() { JsonDocument doc; doc["Value"] = bme.readHumidity(); sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/dewpoint", HTTP_GET, []() { 
    float t = bme.readTemperature(); float h = bme.readHumidity();
    // Magnus formula for dew point
    double a = 17.27, b = 237.7; double alpha = ((a * t) / (b + t)) + log(h / 100.0);
    double dew = (b * alpha) / (a - alpha);
    JsonDocument doc; doc["Value"] = dew; sendResponse(doc); 
  });

  server.begin();
}

void loop() {
  server.handleClient();
  handleDiscovery();
  stepper.run();

  // Auto Dew Logic (Runs every 2 seconds)
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 2000) {
    lastUpdate = millis();
    if (autoDewEnabled) {
       float t = bme.readTemperature(); float h = bme.readHumidity();
       
       // Calculate Dew Point
       double a = 17.27, b = 237.7; double alpha = ((a * t) / (b + t)) + log(h / 100.0);
       double dew = (b * alpha) / (a - alpha);
       
       float delta = t - dew; 
       float targetOffset = 3.5; // Maintain temp 3.5C above dew point
       int newPower = 0;
       
       if (delta < targetOffset) {
          // Calculate power (Closer to dew point = more power)
          float calc = (targetOffset - delta) * (100.0 / targetOffset);
          if (calc > 100) calc = 100; if (calc < 10) calc = 10;
          newPower = (int)calc;
       }
       currentPower1 = newPower;
       ledcWrite(HEATER_1_PIN, map(currentPower1, 0, 100, 0, 255));
       // Note: Heater 2 is manual only and not affected by auto logic
    }
  }
}
