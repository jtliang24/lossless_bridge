#include <Arduino.h>
#include "AudioTools.h"
// This forces the use of the ESP32-S3 specific USB Audio driver
#include "AudioTools/Communication/USB/USBAudioDeviceESP32.h" 

using namespace audio_tools;
// Instantiate the USB Audio Stream using the S3-specific driver
USBAudioStream usbAudio;

void setup() {
    Serial.begin(115200);
    delay(2000); 
    
    // Enable logging to confirm the S3 USB hardware initialized correctly
    AudioLogger::instance().begin(Serial, AudioLogger::Info);

    Serial.println("\n--- Initializing with USBAudioDeviceESP32.h ---");

    // Configure the audio stream (RX_MODE: Host PC -> Dongle)
    auto config = usbAudio.defaultConfig(RX_MODE);
    config.sample_rate = 44100;
    config.channels = 2;
    config.bits_per_sample = 16;
    config.begin_usb = true;
    
    // Initialize the S3 USB peripheral
    usbAudio.begin(config);

    Serial.println("Driver loaded: USBAudioDeviceESP32 ready.");
}

void loop() {
    // 512-byte buffer for the incoming PCM audio packets
    uint8_t buffer[512];
    
    // The driver populates the buffer automatically from the USB hardware
    size_t bytes_read = usbAudio.readBytes(buffer, sizeof(buffer));
    
    if (bytes_read > 0) {
        // You are now capturing raw PCM data at the source.
        Serial.printf("Captured %d bytes of PCM data\n", bytes_read);
    }
}
