# HW 7
## CODE
- `Server Code`

``` ino
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

void setup() {
  Serial.begin(115200);

  BLEDevice::init("ESP32-Server-team8");   
  BLEServer *pServer = BLEDevice::createServer();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  Serial.println("BLE Advertising Started...");
}
void loop() {
  delay(1000);
}
```

- `Client Code`

```ino
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <math.h>

BLEScan* pBLEScan;

float calculateDistance(int rssi, int txPower = -59, float n = 2.0) {
  return pow(10.0, ((float)(txPower - rssi)) / (10 * n));
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      String name = advertisedDevice.getName().c_str();
      int rssi = advertisedDevice.getRSSI();

      if (name == "ESP32-Server-team8") { 
        float distance = calculateDistance(rssi);

        Serial.print("Device: ");
        Serial.println(name.c_str());
        Serial.print("RSSI: ");
        Serial.println(rssi);
        Serial.print("Estimated Distance (m): ");
        Serial.println(distance);
        Serial.println("---------------------------");
      }
    }
};

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop() {
  pBLEScan->start(5, false);
  delay(5000);
}
```

# ADVANCED
## _1 

``` ino
//Client
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <math.h>

BLEScan* pBLEScan;

#define LED_PIN 2  

float calculateDistance(int rssi, int txPower = -59, float n = 2.0) {
  return pow(10.0, ((float)(txPower - rssi)) / (10 * n));
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      std::string name = advertisedDevice.getName();
      int rssi = advertisedDevice.getRSSI();

      if (name == "ESP32-Server-team8") {   
        float distance = calculateDistance(rssi);

        Serial.print("Device: ");
        Serial.println(name.c_str());
        Serial.print("RSSI: ");
        Serial.println(rssi);
        Serial.print("Estimated Distance (m): ");
        Serial.println(distance);
        Serial.println("---------------------------");
      }

      if (distance <= 1.0) {
        digitalWrite(LED_PIN, HIGH); 
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      } else {
        digitalWrite(LED_PIN, LOW); 
      }
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop() {
  delay(10000);
  pBLEScan->start(5, false);
}


```

## _2

```ino
//Client
#include <WiFi.h>
#include <WebServer.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

const char* ssid = "SeungJun"; 
const char* password = "38733873";

BLEScan* pBLEScan;
float currentDistance = 0;

WebServer server(80);

float calculateDistance(int rssi, int txPower = -59, float n = 2.0) {
  return pow(10.0, ((txPower - rssi) / (10 * n)));
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      String name = advertisedDevice.getName().c_str();
      int rssi = advertisedDevice.getRSSI();

      if (name == "ESP32-Server-team8") {
        currentDistance = calculateDistance(rssi);
        Serial.print("Distance: ");
        Serial.println(currentDistance);
      }
    }
};

void handleRoot() {
  String html = "<html><head><meta http-equiv='refresh' content='5'></head><body><h1>Distance to ESP32-Server</h1><p>" +
                String(currentDistance) + " meters</p></body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.begin();
}

void loop() {
  server.handleClient();
  pBLEScan->start(5, false);
  delay(5000);
}

```

