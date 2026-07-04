#include <Arduino.h>
#include "AudioTools.h"
// This forces the use of the ESP32-S3 specific USB Audio driver
#include "AudioTools/Communication/USB/USBAudioDeviceESP32.h" 

using namespace audio_tools;
// Instantiate the USB Audio Stream using the S3-specific driver
USBAudioStream usbAudio;

void setup() {
    // Configure the audio stream (RX_MODE: Host PC -> Dongle)
    auto config = usbAudio.defaultConfig(RX_MODE);
    config.sample_rate = 44100;
    config.channels = 2;
    config.bits_per_sample = 16;
    
    // Set custom USB device names
    config.manufacturer = "SEEED";
    config.product = "SEEED Audio Device";
    
    // Change VID, PID and Serial to force Windows to clear its device name cache
    config.vid = 0xcafe;
    config.pid = 0x4005;
    config.serial = "000002";
    
    // Start the USB stack automatically during begin()
    config.begin_usb = true;
    
    // Initialize the S3 USB peripheral
    usbAudio.begin(config);
}

void loop() {
    // 512-byte buffer for the incoming PCM audio packets
    uint8_t buffer[512];
    
    // The driver populates the buffer automatically from the USB hardware
    size_t bytes_read = usbAudio.readBytes(buffer, sizeof(buffer));
    
    if (bytes_read == 0) {
        delay(1);
    }
}
