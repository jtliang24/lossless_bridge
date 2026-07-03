#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// put function declarations here:
typedef struct struct_message {
  char text[32];
  unsigned long packetId;
} struct_message;

struct_message incomingData;

volatile bool newPacketAvailable = false;
volatile int lastPacketLen = 0;

// Modern receiver callback signature matching ESP32 Arduino Core v3.0+ and fallback for v2.0.x
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingBytes, int len) {
  // const uint8_t *mac = recv_info->src_addr; // Sender's MAC address if needed
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingBytes, int len) {
#endif
  memcpy(&incomingData, incomingBytes, sizeof(incomingData));

  lastPacketLen = len;
  newPacketAvailable = true;

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(2000);
  Serial.println("RECEIVER: Initializing System...");

  WiFi.mode(WIFI_STA);

  // Explicitly set Wi-Fi channel to 1 to match the transmitter channel
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Disable Wi-Fi power-saving (sleep) mode to ensure the receiver's radio stays active
  // esp_wifi_set_ps(WIFI_PS_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  esp_now_register_recv_cb(OnDataRecv);
#else
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
#endif

  Serial.println("RECEIVER: Setup Complete. Listening for transmitter...");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (newPacketAvailable) {
    newPacketAvailable = false; // Reset flag immediately
    
    Serial.print("[Wireless Packet Received] Bytes: ");
    Serial.print(lastPacketLen);
    Serial.print(" | Message: ");
    Serial.print(incomingData.text);
    Serial.print(" | Packet ID: ");
    Serial.println(incomingData.packetId);
  }
  
  delay(1);
}


// put function definitions here: