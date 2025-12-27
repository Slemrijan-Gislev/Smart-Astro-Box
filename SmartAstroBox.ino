/*
 * ------------------------------------------------------------------------
 * SMART ASTRO BOX - DUAL CORE EDITION (FIXED & COMPLETE)
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
const char* ssid = "ssid";
const char* password = "password";

IPAddress local_IP(192, 168, 8, 10);
IPAddress gateway(192, 168, 8, 1);
IPAddress subnet(255, 255, 255, 0);
// ===============================================================

const int serverPort = 4567; 

// --- PIN CONFIGURATION ---
#define MOTOR_STEP_PIN  14  
#define MOTOR_DIR_PIN   12  
#define MOTOR_EN_PIN    13  

#define HEATER_1_PIN    25 
#define HEATER_2_PIN    26 

#define I2C_SDA         21 
#define I2C_SCL         22 

// --- OBJECTS ---
WebServer server(serverPort);
WiFiUDP udp;
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);
Adafruit_BME280 bme;

// --- SHARED VARIABLES ---
volatile long sharedTargetPosition = 0;   
volatile long sharedCurrentPosition = 0;  
volatile bool sharedIsMoving = false;     
volatile bool cmdStop = false;            

// Heater Variables
int currentPower1 = 0;
int currentPower2 = 0;
bool autoDewEnabled = true;

uint32_t transactionId = 0;
TaskHandle_t TaskCore0;

// ============================================================================
//   CORE 0 CODE: NETWORK & SENSORS
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

// Netværks-loopet på Core 0
void NetworkLoop(void * pvParameters) {
  Serial.print("Network Task running on Core: ");
  Serial.println(xPortGetCoreID());

  // --- MANAGEMENT API ---
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

  // --- FOCUSER API ---
  server.on("/api/v1/focuser/0/connected", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/focuser/0/connected", HTTP_PUT, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/focuser/0/name", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "ESP32 DualCore"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/driverinfo", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "DualCore Driver"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/driverversion", HTTP_GET, []() { JsonDocument doc; doc["Value"] = "3.2 FINAL"; sendResponse(doc); });
  server.on("/api/v1/focuser/0/interfaceversion", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 1; sendResponse(doc); });
  
  server.on("/api/v1/focuser/0/absolute", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/focuser/0/maxstep", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 100000; sendResponse(doc); });
  server.on("/api/v1/focuser/0/stepsize", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 4.0; sendResponse(doc); });
  
  server.on("/api/v1/focuser/0/temperature", HTTP_GET, []() { JsonDocument doc; doc["Value"] = bme.readTemperature(); sendResponse(doc); });
  server.on("/api/v1/focuser/0/tempcompavailable", HTTP_GET, []() { JsonDocument doc; doc["Value"] = false; sendResponse(doc); });
  server.on("/api/v1/focuser/0/tempcomp", HTTP_GET, []() { JsonDocument doc; doc["Value"] = false; sendResponse(doc); });
  server.on("/api/v1/focuser/0/tempcomp", HTTP_PUT, []() { JsonDocument doc; sendResponse(doc); });

  server.on("/api/v1/focuser/0/position", HTTP_GET, []() { JsonDocument doc; doc["Value"] = sharedCurrentPosition; sendResponse(doc); });
  server.on("/api/v1/focuser/0/ismoving", HTTP_GET, []() { JsonDocument doc; doc["Value"] = sharedIsMoving; sendResponse(doc); });
  
  server.on("/api/v1/focuser/0/move", HTTP_PUT, []() { 
    if(server.hasArg("Position")) { 
      sharedTargetPosition = server.arg("Position").toInt(); 
    }
    JsonDocument doc; sendResponse(doc); 
  });
  
  server.on("/api/v1/focuser/0/halt", HTTP_PUT, []() { cmdStop = true; JsonDocument doc; sendResponse(doc); });

  // --- SWITCH API ---
  server.on("/api/v1/switch/0/connected", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/switch/0/connected", HTTP_PUT, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/switch/0/maxswitch", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 3; sendResponse(doc); });
  server.on("/api/v1/switch/0/canwrite", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  
  server.on("/api/v1/switch/0/getswitchname", HTTP_GET, []() { 
    int id = server.arg("Id").toInt(); JsonDocument doc; 
    if (id == 0) doc["Value"] = "Heater 1"; else if (id == 1) doc["Value"] = "Heater 2"; else doc["Value"] = "Auto Dew"; 
    sendResponse(doc); 
  });
  
  // FIX: Den manglende beskrivelse
  server.on("/api/v1/switch/0/getswitchdescription", HTTP_GET, []() { 
    int id = server.arg("Id").toInt(); JsonDocument doc; 
    if (id == 0) doc["Value"] = "Heater 1 Power (0-100)";
    else if (id == 1) doc["Value"] = "Heater 2 Power (0-100)";
    else doc["Value"] = "Enable Auto Dew Control"; 
    sendResponse(doc); 
  });

  server.on("/api/v1/switch/0/maxswitchvalue", HTTP_GET, []() { int id = server.arg("Id").toInt(); JsonDocument doc; doc["Value"] = (id < 2) ? 100.0 : 1.0; sendResponse(doc); });
  server.on("/api/v1/switch/0/minswitchvalue", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 0.0; sendResponse(doc); });
  server.on("/api/v1/switch/0/switchstep", HTTP_GET, []() { JsonDocument doc; doc["Value"] = 1.0; sendResponse(doc); });
  
  server.on("/api/v1/switch/0/getswitchvalue", HTTP_GET, []() { 
    int id = server.arg("Id").toInt(); JsonDocument doc; 
    if (id == 0) doc["Value"] = currentPower1; if (id == 1) doc["Value"] = currentPower2; if (id == 2) doc["Value"] = autoDewEnabled ? 1.0 : 0.0;
    sendResponse(doc);
  });
  
  server.on("/api/v1/switch/0/setswitchvalue", HTTP_PUT, []() { 
    int id = server.arg("Id").toInt(); double val = server.arg("Value").toDouble();
    if (id == 0) { currentPower1 = (int)val; ledcWrite(HEATER_1_PIN, map(currentPower1, 0, 100, 0, 255)); autoDewEnabled = false; }
    if (id == 1) { currentPower2 = (int)val; ledcWrite(HEATER_2_PIN, map(currentPower2, 0, 100, 0, 255)); }
    if (id == 2) { autoDewEnabled = (val > 0.5); }
    JsonDocument doc; sendResponse(doc);
  });

  // --- OBSERVING CONDITIONS API ---
  server.on("/api/v1/observingconditions/0/connected", HTTP_GET, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/connected", HTTP_PUT, []() { JsonDocument doc; doc["Value"] = true; sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/temperature", HTTP_GET, []() { JsonDocument doc; doc["Value"] = bme.readTemperature(); sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/humidity", HTTP_GET, []() { JsonDocument doc; doc["Value"] = bme.readHumidity(); sendResponse(doc); });
  server.on("/api/v1/observingconditions/0/dewpoint", HTTP_GET, []() { 
    float t = bme.readTemperature(); float h = bme.readHumidity();
    double a = 17.27, b = 237.7; double alpha = ((a * t) / (b + t)) + log(h / 100.0);
    double dew = (b * alpha) / (a - alpha);
    JsonDocument doc; doc["Value"] = dew; sendResponse(doc); 
  });

  server.begin();

  unsigned long lastDewCheck = 0;
  while(true) {
    server.handleClient();
    handleDiscovery();

    if (millis() - lastDewCheck > 2000) {
      lastDewCheck = millis();
      if (autoDewEnabled) {
         float t = bme.readTemperature(); 
         float h = bme.readHumidity();
         double a = 17.27, b = 237.7; double alpha = ((a * t) / (b + t)) + log(h / 100.0);
         double dew = (b * alpha) / (a - alpha);
         float delta = t - dew; 
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
    delay(1); 
  }
}

// ============================================================================
//   MAIN SETUP (Runs on Core 1)
// ============================================================================

void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_EN_PIN, OUTPUT);
  digitalWrite(MOTOR_EN_PIN, LOW); 
  
  stepper.setMaxSpeed(1000);    
  stepper.setAcceleration(500);

  ledcAttach(HEATER_1_PIN, 5000, 8); ledcWrite(HEATER_1_PIN, 0);
  ledcAttach(HEATER_2_PIN, 5000, 8); ledcWrite(HEATER_2_PIN, 0);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bme.begin(0x76)) { if (!bme.begin(0x77)) Serial.println("ERROR: No BME280 sensor found!"); }

  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected!");
  Serial.println(WiFi.localIP());
  
  udp.begin(32227); 

  // HER GIK DET GALT SIDST - SIKR DIG AT DENNE LINJE ER KOMPLET:
  xTaskCreatePinnedToCore(NetworkLoop, "NetworkTask", 10000, NULL, 1, &TaskCore0, 0);
  
  Serial.println("Dual Core System Started.");
}

// ============================================================================
//   CORE 1 LOOP: MOTOR ONLY
// ============================================================================

void loop() {
  if (cmdStop) {
    stepper.stop();
    sharedTargetPosition = stepper.currentPosition(); 
    stepper.moveTo(sharedTargetPosition);
    cmdStop = false;
  }

  if (sharedTargetPosition != stepper.targetPosition()) {
    stepper.moveTo(sharedTargetPosition);
  }

  bool moving = stepper.run();
  sharedIsMoving = moving;
  sharedCurrentPosition = stepper.currentPosition();
}
