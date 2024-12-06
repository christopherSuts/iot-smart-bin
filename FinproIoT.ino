#include <SoftwareSerial.h>

#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif

#include <Firebase_ESP_Client.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SoftwareSerial gpsSerial(8,9); // TX RX
TinyGPSPlus gps;
float latitude = 0, longitude = 0;

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
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

// Data variables
int count = 0;
bool signupOK = false;

void gpsTask(void *parameter) {
  for (;;) {
    while (gpsSerial.available()) {
      int data = gpsSerial.read();
      if (gps.encode(data)) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Serial.print("Latitude: ");
        Serial.println(latitude);
        Serial.print("Longitude: ");
        Serial.println(longitude);
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay 500ms to reduce CPU usage
  }
}

void uploadTask(void *parameter) {
  unsigned long sendDataPrevMillis = 0;
  for (;;) {
    if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {
      sendDataPrevMillis = millis();

      // Write latitude to Firebase
      if (Firebase.RTDB.setFloat(&fbdo, "tong/latitude", latitude)) {
        Serial.println("PASSED");
        Serial.println("Latitude: " + String(latitude) + " PATH: " + fbdo.dataPath() + " TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }

      // Write longitude to Firebase
      if (Firebase.RTDB.setFloat(&fbdo, "tong/longitude", longitude)) {
        Serial.println("PASSED");
        Serial.println("Longitude: " + String(longitude) + " PATH: " + fbdo.dataPath() + " TYPE: " + fbdo.dataType());
      } else {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
      }
    }
    longitude++; latitude++;
    vTaskDelay(30000 / portTICK_PERIOD_MS); // Delay for 10 seconds
  }
}

void setup() {
  gpsSerial.begin(9600);
  Serial.begin(115200);

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

  // Create FreeRTOS task for uploading data
  xTaskCreate(
    uploadTask,        // Task function
    "Upload Task",     // Name of the task
    10000,             // Stack size in bytes
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &uploadTaskHandle  // Task handle
  );

  xTaskCreate(
    gpsTask,        // Task function
    "GPS Task",     // Name of the task
    10000,          // Stack size in bytes
    NULL,           // Task input parameter
    1,              // Priority of the task
    &GPSTaskHandle  // Task handle
  );
}

void loop() {

}