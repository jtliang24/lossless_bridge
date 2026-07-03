#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

uint8_t receiverAddress[] = {0xE0, 0x72, 0xA1, 0xFA, 0x58, 0x84};

typedef struct struct_message {
  char text[32];
  unsigned long packetId;
} struct_message;

struct_message myData;
unsigned long count = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(2000);
  Serial.println("TRANSMITTER: Initializing System...");

  WiFi.mode(WIFI_STA);

  // Explicitly set Wi-Fi channel to 1 to match the receiver channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1; // Match the configured Wi-Fi channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer receiver");
    return ;
  }
  Serial.println("TRANSMITTER: Setup Complete. Target Channel: 1. Ready to ping...");
}

void loop() {
  // put your main code here, to run repeatedly:
  count ++;
  strcpy(myData.text, "PING: Hello, Receiver!");
  myData.packetId = count;

  Serial.println("Sending Packet ID: ");
  Serial.println(myData.packetId);
  Serial.print("... ");

  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
        Serial.println("Sent call accepted.");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.println("Error: ESP-NOW Not Initialized.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Error: Invalid Argument (Bad MAC/Parameters).");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        Serial.println("Error: Peer Target Destination Not Found in Registry.");
    } else {
        Serial.print("Unknown Error Code: ");
        Serial.println(result);
    }

  delay(1500);

}
