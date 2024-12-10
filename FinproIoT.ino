#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include "soc/rtc.h"
#include "HX711.h"

#define BLYNK_TEMPLATE_ID "TMPL68cNQnfPZ"
#define BLYNK_TEMPLATE_NAME "Finpro IoT"
#define BLYNK_AUTH_TOKEN "TH9Pzx7SSQEI4XKm5AYb3DoCJdo44etZ"
#include <BlynkSimpleEsp32.h>
#include <Firebase_ESP_Client.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
static const int RXPin = 16, TXPin = 17;
// static const int RXPin = 15, TXPin = 16;
SoftwareSerial gpsSerial(RXPin, TXPin); // RX TX pin in the software serial
double latitude = 0, longitude = 0;

const int DT = 25;
const int LOADCELL_SCK_PIN = 33;
HX711 scale;
double loadCellWeight = 0; // Variabel untuk menyimpan berat dari load cell

// Virtual Pin
#define BIN_VPIN V1
#define LATITUDE_VPIN V2
#define LONGITUDE_VPIN V3
#define FULLNESS_VPIN V4

// Ultrasonic Sensor Pins
#define TRIG_PIN 4
#define ECHO_PIN 5

// Servo Initialization
static const int servoPin = 13;
Servo servo;

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
char authBlynk[] = BLYNK_AUTH_TOKEN;
#define WIFI_SSID "DTE Dosen"
#define WIFI_PASSWORD "zfla2290"

// Insert Firebase project API Key
#define API_KEY "AIzaSyCXgMvzVuCltKh819bEo2R9-tKeIKP4xXo"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://postioning-via-bluetooth-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

// Task handles
TaskHandle_t uploadTaskHandle;
TaskHandle_t GPSTaskHandle;
TaskHandle_t ultrasonicTaskHandle;
TaskHandle_t servoTaskHandle;
TaskHandle_t loadCellTaskHandle;

// Data variables
int count = 0;
bool signupOK = false;
bool closeBin = false;
float distance = 0;
double binFullness = 0;

SemaphoreHandle_t xGPSSemaphore, xFullnessSemaphore, xWeightSemaphore;

BLYNK_WRITE(BIN_VPIN) {
  closeBin = param.asInt(); // Membaca status bin (0 = buka, 1 = tutup)
  if (closeBin) {          
    Serial.println("Tutup");
  } else {
    Serial.println("Buka");
  }
}

void gpsTask(void *parameter) {
  for (;;) {
    while (gpsSerial.available() > 0) {
      int data = gpsSerial.read();
      if (gps.encode(data)) {
        if (gps.location.isUpdated()) {
          double lat, lng;
          lat = gps.location.lat();
          lng = gps.location.lng();

          if (xSemaphoreTake(xGPSSemaphore, portMAX_DELAY)) {
            latitude = lat;
            longitude = lng;
            xSemaphoreGive(xGPSSemaphore);
          }

          // Kirim data ke Blynk
          Blynk.virtualWrite(LATITUDE_VPIN, lat);
          Blynk.virtualWrite(LONGITUDE_VPIN, lng);
        }
      }
    }
  }
}

void uploadTask(void *parameter) {
  unsigned long sendDataPrevMillis = 0;
  for (;;) {
    if (Firebase.ready() && signupOK) {
      sendDataPrevMillis = millis();

      double lat, lng, fullness, weight;
      if (xSemaphoreTake(xGPSSemaphore, portMAX_DELAY)) {
        lat = latitude;
        lng = longitude;
        xSemaphoreGive(xGPSSemaphore);
      }

      if (xSemaphoreTake(xFullnessSemaphore, portMAX_DELAY)) {
        fullness = binFullness;
        xSemaphoreGive(xFullnessSemaphore);
      }

      if (xSemaphoreTake(xWeightSemaphore, portMAX_DELAY)) {
        weight = loadCellWeight;
        xSemaphoreGive(xWeightSemaphore);
      }

      if (Firebase.RTDB.setFloat(&fbdo, "tong/latitude", lat)) {
        Serial.println("PASSED - Latitude");
      } else {
        Serial.println("FAILED - Latitude");
      }

      if (Firebase.RTDB.setFloat(&fbdo, "tong/longitude", lng)) {
        Serial.println("PASSED - Longitude");
      } else {
        Serial.println("FAILED - Longitude");
      }

      if (Firebase.RTDB.setFloat(&fbdo, "tong/fullness", fullness)) {
        Serial.println("PASSED - Fullness");
      } else {
        Serial.println("FAILED - Fullness");
      }

      if (Firebase.RTDB.setFloat(&fbdo, "tong/weight", weight)) {
        Serial.println("PASSED - Weight");
      } else {
        Serial.println("FAILED - Weight");
      }
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void ultrasonicTask(void *parameter) {
  for (;;) {
    // Send pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read pulse
    long duration = pulseIn(ECHO_PIN, HIGH);
    double calculatedFullness;

    // Calculate distance
    distance = (duration * 0.034) / 2;
    if (distance >= 25)
      calculatedFullness = 0;
    else
      calculatedFullness = ((25-distance)/25) * 100;

    if (xSemaphoreTake(xFullnessSemaphore, portMAX_DELAY)) {
      binFullness = calculatedFullness;
      xSemaphoreGive(xFullnessSemaphore);
    }

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    Serial.print("Fullness: ");
    Serial.print(calculatedFullness);
    Serial.println(" %");

    // Send distance to Blynk
    Blynk.virtualWrite(FULLNESS_VPIN, calculatedFullness);

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
  }
}

void servoTask(void *parameter) {
  for (;;) {
    double currentFullness;
    if (xSemaphoreTake(xFullnessSemaphore, portMAX_DELAY)) {
      currentFullness = binFullness;
      xSemaphoreGive(xFullnessSemaphore);
    }

    if (currentFullness >= 80){
      servo.write(90);
      Serial.println("Tempat sampah PENUH");
    } else {
      if (closeBin){
        Serial.println("Tempat sampah DITUTUP PAKSA"); // fitur force close
        servo.write(90);
      } else {
        Serial.println("Tempat sampah DIBUKA");
        servo.write(0);
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loadCellTask(void *parameter) {
  for (;;) {
    loadCellWeight = scale.get_units(5); 

    Serial.print("Load Cell Weight: ");
    Serial.print(loadCellWeight);
    Serial.println(" kg");
  }
}

void setup() {
  gpsSerial.begin(9600);
  Serial.begin(9600);

  servo.attach(servoPin);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(DT, INPUT);
  pinMode(LOADCELL_SCK_PIN, OUTPUT);

  // Initialize sensors
  scale.begin(DT, LOADCELL_SCK_PIN);
  scale.set_scale(-100000.0); // Calibrate this value as needed
  scale.tare();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);

  /* Assign the API key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // See addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  xGPSSemaphore = xSemaphoreCreateMutex();
  xFullnessSemaphore = xSemaphoreCreateMutex();
  xWeightSemaphore = xSemaphoreCreateMutex();

  // Create FreeRTOS task for uploading data
  xTaskCreatePinnedToCore(
    gpsTask,           // Task function
    "GPS Task",       // Name of the task
    10000,             // Stack size in bytes
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &GPSTaskHandle,    // Task handle
    1                  // Core 1
  );

  xTaskCreatePinnedToCore(
    loadCellTask,    // Task function
    "LoadCell Task",// Name of the task
    4096,             // Stack size in bytes
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &loadCellTaskHandle, // Task handle
    1                 // Core 1
  );

  xTaskCreatePinnedToCore(
    ultrasonicTask,    // Task function
    "Ultrasonic Task",// Name of the task
    4096,             // Stack size in bytes
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &ultrasonicTaskHandle, // Task handle
    1                  // Core 1
  );

  xTaskCreatePinnedToCore(
    uploadTask,        // Task function
    "Upload Task",     // Name of the task
    10000,             // Stack size in bytes
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &uploadTaskHandle, // Task handle
    0                  // Core 0
  );

  xTaskCreatePinnedToCore(
    servoTask,    // Task function
    "Servo Task",// Name of the task
    2048,             // Stack size in bytes
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &servoTaskHandle, // Task handle
    0                 // Core 1
  );
}

void loop() {
  Blynk.run();
}