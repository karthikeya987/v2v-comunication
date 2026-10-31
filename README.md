#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// -------- VEHICLE SETTINGS --------
#define VEHICLE_ID 1

// -------- PIN CONNECTIONS --------
#define LIFI_TX 25
#define LIFI_RX 26

#define TRIG_PIN 5
#define ECHO_PIN 18

#define BUZZER 27
#define SD_CS 13

// -------- OBJECTS --------
Adafruit_MPU6050 mpu;
HardwareSerial LiFi(2);

// -------- VEHICLE DATA --------
struct VehicleData {
  int id;
  float distance;
  float acceleration;
  int collision;
};

VehicleData myData;
VehicleData receivedData;

volatile bool newESPData = false;

unsigned long lastSend = 0;
unsigned long lastLog = 0;
unsigned long lastWarning = 0;

bool sdReady = false;
bool mpuReady = false;

// -------- ESP-NOW RECEIVE --------
void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData,
                int len) {
  if (len == sizeof(VehicleData)) {
    memcpy(&receivedData, incomingData,
           sizeof(receivedData));
    newESPData = true;
  }
}

// -------- ULTRASONIC SENSOR --------
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) return 999;

  return duration * 0.0343 / 2;
}

// -------- ACCIDENT DETECTION --------
float getAcceleration() {
  if (!mpuReady) return 0;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float magnitude = sqrt(
    accel.acceleration.x * accel.acceleration.x +
    accel.acceleration.y * accel.acceleration.y +
    accel.acceleration.z * accel.acceleration.z
  );

  return magnitude / 9.81;
}

// -------- LIFI TRANSMISSION --------
void sendLiFi() {
  LiFi.print(myData.id);
  LiFi.print(",");
  LiFi.print(myData.distance);
  LiFi.print(",");
  LiFi.print(myData.acceleration);
  LiFi.print(",");
  LiFi.println(myData.collision);
}

// -------- LIFI RECEPTION --------
void receiveLiFi() {
  if (LiFi.available()) {
    String message = LiFi.readStringUntil('\n');

    int id, collision;
    float distance, acceleration;

    if (sscanf(message.c_str(), "%d,%f,%f,%d",
        &id, &distance, &acceleration,
        &collision) == 4) {

      receivedData.id = id;
      receivedData.distance = distance;
      receivedData.acceleration = acceleration;
      receivedData.collision = collision;

      newESPData = true;
    }
  }
}

// -------- ESP-NOW TRANSMISSION --------
void sendESPNow() {
  VehicleData packet = myData;

  uint8_t broadcastAddress[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  };

  esp_now_send(broadcastAddress,
               (uint8_t *)&packet,
               sizeof(packet));
}

// -------- BLACK BOX LOGGING --------
void logData() {
  if (!sdReady) return;

  File file = SD.open("/blackbox.csv", FILE_APPEND);

  if (file) {
    file.print(millis());
    file.print(",");
    file.print(myData.id);
    file.print(",");
    file.print(myData.distance);
    file.print(",");
    file.print(myData.acceleration);
    file.print(",");
    file.println(myData.collision);

    file.close();
  }
}

// -------- COLLISION WARNING --------
void collisionWarning() {
  bool danger = false;

  // Local vehicle obstacle detection
  if (myData.distance < 50) {
    danger = true;
  }

  // Other vehicle collision information
  if (newESPData &&
      receivedData.id != VEHICLE_ID &&
      receivedData.collision == 1) {
    danger = true;
  }

  if (danger) {
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
  }
}

// -------- SETUP --------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);

  LiFi.begin(1200, SERIAL_8N1,
             LIFI_RX, LIFI_TX);

  Wire.begin(21, 22);

  // MPU6050 initialization
  mpuReady = mpu.begin();

  if (mpuReady) {
    mpu.setAccelerometerRange(
      MPU6050_RANGE_8_G);
    Serial.println("MPU6050 Ready");
  } else {
    Serial.println("MPU6050 Not Found");
  }

  // SD card initialization
  SPI.begin(14, 19, 23, SD_CS);
  sdReady = SD.begin(SD_CS);

  if (sdReady) {
    Serial.println("SD Card Ready");
  } else {
    Serial.println("SD Card Failed");
  }

  // ESP-NOW initialization
  WiFi.mode(WIFI_STA);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memset(peerInfo.peer_addr, 0xFF, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_now_add_peer(&peerInfo);

    Serial.println("ESP-NOW Ready");
  } else {
    Serial.println("ESP-NOW Failed");
  }

  Serial.println("Vehicle Black Box Started");
}

// -------- MAIN LOOP --------
void loop() {

  // 1. Read vehicle sensors
  myData.id = VEHICLE_ID;
  myData.distance = getDistance();
  myData.acceleration = getAcceleration();

  // Basic prototype crash threshold.
  myData.collision =
    (myData.acceleration > 3.0) ? 1 : 0;

  // 2. Transmit Li-Fi and ESP-NOW
  if (millis() - lastSend >= 500) {
    lastSend = millis();

    sendLiFi();
    sendESPNow();
  }

  // 3. Receive vehicle information
  receiveLiFi();

  // 4. Collision warning
  collisionWarning();

  // 5. Save black box information
  if (millis() - lastLog >= 1000) {
    lastLog = millis();
    logData();
  }

  // 6. Serial monitor
  Serial.print("Vehicle ID: ");
  Serial.print(myData.id);

  Serial.print(" | Distance: ");
  Serial.print(myData.distance);

  Serial.print(" cm | Acceleration: ");
  Serial.print(myData.acceleration);

  Serial.print(" g | Collision: ");
  Serial.println(myData.collision);

  delay(50);
}
