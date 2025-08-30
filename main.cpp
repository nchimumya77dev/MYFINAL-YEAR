#include <Arduino.h>
#include <LiquidCrystal.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ====== WiFi + MQTT Setup ======
const char* ssid = "Wifi 7";       
const char* password = "12345678900";   
const char* mqtt_server = "192.168.91.54";   // 🔹 change to your broker IP

WiFiClient espClient;
PubSubClient client(espClient);

// ====== LCD PIN SETUP (ESP32 pins) ======
const int rs = 19, en = 23, d4 = 18, d5 = 17, d6 = 16, d7 = 15;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// ====== Ultrasonic Sensor Pins ======
const int trigPins[4] = {4, 5, 12, 13};
const int echoPins[4] = {14, 27, 26, 25};
const char* ultrasonicShort[4] = {"F-L", "F-R", "B-L", "B-R"};
const char* ultrasonicFull[4]  = {"Front-Left", "Front-Right", "Back-Left", "Back-Right"};

// ====== Vibration Sensor Pins ======
const int vibrationPins[4] = {32, 33, 34, 35};
const char* vibrationShort[4] = {"V-L", "V-R", "V-B", "V-F"};
const char* vibrationFull[4]  = {"Vibration-Left", "Vibration-Right", "Vibration-Back", "Vibration-Front"};

// ====== Alert Output Pins ======
const int buzzerPin = 2;
const int ledPin = 22;

// ====== Settings ======
const int distanceThreshold = 40; // cm

// ====== System Status Variables ======
unsigned long lastDebugTime = 0;
unsigned long lastSystemStatusTime = 0;
unsigned long lastHeartbeatTime = 0;
const unsigned long debugInterval = 5000; // Print debug info every 5 seconds
const unsigned long systemStatusInterval = 30000; // Send system status every 30 seconds
const unsigned long heartbeatInterval = 10000; // Send heartbeat every 10 seconds
String deviceId = "FenceModule_001";
unsigned long bootTime = 0;
int totalAlerts = 0;
int ultrasonicAlerts = 0;
int vibrationAlerts = 0;

// ====== Functions ======
long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  long distance = duration * 0.034 / 2; // cm
  
  // Log ultrasonic readings for debugging
  Serial.print("Ultrasonic - Trig:");
  Serial.print(trigPin);
  Serial.print(" Echo:");
  Serial.print(echoPin);
  Serial.print(" Duration:");
  Serial.print(duration);
  Serial.print(" Distance:");
  Serial.print(distance);
  Serial.println("cm");
  
  return distance;
}

void sendSystemStatus() {
  DynamicJsonDocument doc(1024);
  doc["deviceId"] = deviceId;
  doc["timestamp"] = millis();
  doc["uptime"] = millis() - bootTime;
  doc["ip"] = WiFi.localIP().toString();
  doc["rssi"] = WiFi.RSSI();
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["wifiStatus"] = (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
  doc["mqttStatus"] = client.connected() ? "connected" : "disconnected";
  doc["totalAlerts"] = totalAlerts;
  doc["ultrasonicAlerts"] = ultrasonicAlerts;
  doc["vibrationAlerts"] = vibrationAlerts;
  doc["distanceThreshold"] = distanceThreshold;
  doc["macAddress"] = WiFi.macAddress();
  doc["chipId"] = ESP.getChipRevision();
  doc["sdkVersion"] = ESP.getSdkVersion();
  
  String payload;
  serializeJson(doc, payload);
  
  if (client.publish("farm/fence/system/status", payload.c_str())) {
    Serial.println("System status sent successfully");
  } else {
    Serial.println("Failed to send system status");
  }
}

void sendHeartbeat() {
  DynamicJsonDocument doc(256);
  doc["deviceId"] = deviceId;
  doc["timestamp"] = millis();
  doc["status"] = "online";
  doc["ip"] = WiFi.localIP().toString();
  doc["uptime"] = millis() - bootTime;
  
  String payload;
  serializeJson(doc, payload);
  
  if (client.publish("farm/fence/system/heartbeat", payload.c_str())) {
    Serial.println("Heartbeat sent successfully");
  } else {
    Serial.println("Failed to send heartbeat");
  }
}

void sendSensorReading(const char* sensorType, const char* position, int value, const char* unit = "") {
  DynamicJsonDocument doc(512);
  doc["deviceId"] = deviceId;
  doc["timestamp"] = millis();
  doc["sensorType"] = sensorType;
  doc["position"] = position;
  doc["value"] = value;
  doc["unit"] = unit;
  doc["ip"] = WiFi.localIP().toString();
  doc["alertTriggered"] = (sensorType == "ultrasonic" && value > 2 && value < distanceThreshold) || 
                          (sensorType == "vibration" && value == HIGH);
  
  String payload;
  serializeJson(doc, payload);
  
  String topic = "farm/fence/" + String(sensorType);
  
  if (client.publish(topic.c_str(), payload.c_str())) {
    Serial.println("Sensor reading sent successfully");
  } else {
    Serial.println("Failed to send sensor reading");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(deviceId.c_str())) {
      Serial.println("connected");
      // Send initial system status on connection
      sendSystemStatus();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 sec");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== FARM FENCE MONITOR STARTING ===");
  bootTime = millis();

  // WiFi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  // MQTT
  client.setServer(mqtt_server, 1883);
  Serial.print("MQTT server set to: ");
  Serial.println(mqtt_server);

  // Initialize ultrasonic pins
  Serial.println("Initializing ultrasonic sensors:");
  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    Serial.print("  ");
    Serial.print(ultrasonicFull[i]);
    Serial.print(" - Trig:");
    Serial.print(trigPins[i]);
    Serial.print(" Echo:");
    Serial.println(echoPins[i]);
  }

  // Initialize vibration sensor pins
  Serial.println("Initializing vibration sensors:");
  for (int i = 0; i < 4; i++) {
    pinMode(vibrationPins[i], INPUT);
    Serial.print("  ");
    Serial.print(vibrationFull[i]);
    Serial.print(" - Pin:");
    Serial.println(vibrationPins[i]);
  }

  // Output pins
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  Serial.print("Buzzer pin: ");
  Serial.println(buzzerPin);
  Serial.print("LED pin: ");
  Serial.println(ledPin);

  // LCD setup
  lcd.begin(16, 2);
  lcd.print("System Init...");
  Serial.println("LCD initialized");
  delay(2000);
  lcd.clear();
  lcd.print("Monitoring...");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(2000);
  lcd.clear();
  
  Serial.println("=== SYSTEM READY - MONITORING STARTED ===");
  Serial.print("Device ID: ");
  Serial.println(deviceId);
  Serial.print("Distance threshold: ");
  Serial.print(distanceThreshold);
  Serial.println("cm");
  Serial.println();
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  bool alert = false;
  String lcdLine1 = "All Clear";
  String lcdLine2 = "";
  String alertDetails = "";

  Serial.println("--- Sensor Scan Start ---");

  // === Check Ultrasonic Sensors ===
  Serial.println("Checking ultrasonic sensors:");
  for (int i = 0; i < 4; i++) {
    long dist = readDistance(trigPins[i], echoPins[i]);
    
    // Send sensor reading regardless of alert status
    sendSensorReading("ultrasonic", ultrasonicFull[i], dist, "cm");

    if (dist > 2 && dist < distanceThreshold) {
      alert = true;
      totalAlerts++;
      ultrasonicAlerts++;
      lcdLine1 = "MOVEMENT!";
      lcdLine2 = String(ultrasonicShort[i]) + " " + String(dist) + "cm";
      alertDetails = "ULTRASONIC ALERT: " + String(ultrasonicFull[i]) + " detected movement at " + String(dist) + "cm";
      
      Serial.println("*** " + alertDetails + " ***");
    }
    delay(50); // Small delay between sensor readings
  }

  // === Check Vibration Sensors ===
  Serial.println("Checking vibration sensors:");
  for (int i = 0; i < 4; i++) {
    int vib = digitalRead(vibrationPins[i]);
    Serial.print("  ");
    Serial.print(vibrationFull[i]);
    Serial.print(" (Pin ");
    Serial.print(vibrationPins[i]);
    Serial.print("): ");
    Serial.println(vib == HIGH ? "HIGH (TRIGGERED)" : "LOW (normal)");

    // Send sensor reading regardless of alert status
    sendSensorReading("vibration", vibrationFull[i], vib);

    if (vib == HIGH) {
      alert = true;
      totalAlerts++;
      vibrationAlerts++;
      lcdLine1 = "TOUCH ALERT!";
      lcdLine2 = vibrationShort[i];
      alertDetails = "VIBRATION ALERT: " + String(vibrationFull[i]) + " detected touch/vibration";
      
      Serial.println("*** " + alertDetails + " ***");
    }
  }

  // === LCD Display ===
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(lcdLine1);
  lcd.setCursor(0, 1);
  lcd.print(lcdLine2);

  // === Alert Outputs ===
  if (alert) {
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledPin, HIGH);
    Serial.println("BUZZER AND LED ACTIVATED!");
  } else {
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    Serial.println("Status: All sensors normal");
  }

  // === Send System Status Periodically ===
  if (millis() - lastSystemStatusTime > systemStatusInterval) {
    sendSystemStatus();
    lastSystemStatusTime = millis();
  }

  // === Send Heartbeat ===
  if (millis() - lastHeartbeatTime > heartbeatInterval) {
    sendHeartbeat();
    lastHeartbeatTime = millis();
  }

  // === Periodic Debug Info ===
  if (millis() - lastDebugTime > debugInterval) {
    Serial.println();
    Serial.println("=== DEBUG STATUS REPORT ===");
    Serial.print("Device ID: ");
    Serial.println(deviceId);
    Serial.print("WiFi Status: ");
    Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.print("MQTT Status: ");
    Serial.println(client.connected() ? "Connected" : "Disconnected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
    Serial.print("Free Heap: ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("Uptime: ");
    Serial.println((millis() - bootTime) / 1000);
    Serial.print("Total Alerts: ");
    Serial.println(totalAlerts);
    Serial.println("===========================");
    Serial.println();
    lastDebugTime = millis();
  }

  Serial.println("--- Sensor Scan Complete ---");
  Serial.println();

  delay(600);
}