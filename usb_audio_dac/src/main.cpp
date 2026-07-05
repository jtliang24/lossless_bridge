#include <Arduino.h>
#include <driver/i2s.h>
#include "AudioTools.h"
#include "AudioTools/Communication/USB/USBAudioDeviceESP32.h" 

using namespace audio_tools;

// Pins for I2S output (from RX_test)
#define I2S_BCK_PIN 7
#define I2S_DIN_PIN 8
#define I2S_LRCK_PIN 9

#define SAMPLE_RATE 16000
#define I2S_NUM I2S_NUM_0

// Instantiate USB Audio Stream
USBAudioStream usbAudio;

void setup_i2s() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_PIN,
    .ws_io_num = I2S_LRCK_PIN,
    .data_out_num = I2S_DIN_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // Turn LED OFF (HIGH) on boot
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Set up I2S (from RX_test)
  setup_i2s();

  // Configure the USB audio stream
  auto config = usbAudio.defaultConfig(RX_MODE);
  config.sample_rate = SAMPLE_RATE;
  config.channels = 2;
  config.bits_per_sample = 16;
  config.fifo_packets = 32; // Expanded FIFO capacity
  
  config.manufacturer = "SEEED";
  config.product = "SEEED Audio Device";
  
  config.vid = 0xcafe;
  config.pid = 0x4026; // New PID to clear caches
  config.serial = "000026"; // New Serial to clear caches
  
  config.use_linear_buffer_rx = true;
  
  config.begin_usb = true;
  usbAudio.begin(config);
}

// Buffer to store leftover bytes from the previous read that didn't form a complete 4-byte stereo frame
uint8_t leftover_buf[4];
int leftover_len = 0;

// Audio pipeline playback state
bool playing = false;

// Timestamp of the last successful USB audio packet read
unsigned long last_data_time = 0;

// Preroll threshold: wait until we accumulate 2048 bytes (~32ms of audio at 16kHz)
// before starting to write to the I2S DMA, absorbing timing jitter.
const size_t PREROLL_BYTES = 2048;

void loop() {
  if (!playing) {
    // Wait for the stream buffer to fill up to the preroll threshold
    if (usbAudio.available() >= PREROLL_BYTES) {
      playing = true;
      last_data_time = millis();
    } else {
      // Return and wait for more USB packets to accumulate
      delay(2);
      return;
    }
  }

  // Aligned stack-allocated buffer for incoming 16-bit USB PCM data
  alignas(4) uint8_t read_buf[512];
  
  // If there are leftovers, copy them to the beginning of the read buffer
  if (leftover_len > 0) {
    memcpy(read_buf, leftover_buf, leftover_len);
  }
  
  // Read incoming PCM bytes from USB Audio stack
  size_t bytes_to_read = sizeof(read_buf) - leftover_len;
  size_t bytes_read = usbAudio.readBytes(read_buf + leftover_len, bytes_to_read);
  
  if (bytes_read > 0) {
    // Reset silence timer since we successfully read audio data
    last_data_time = millis();
    
    size_t total_bytes = bytes_read + leftover_len;
    
    // Calculate complete 4-byte stereo frames
    size_t write_bytes = (total_bytes / 4) * 4;
    size_t remaining_bytes = total_bytes % 4;
    
    if (write_bytes > 0) {
      size_t bytes_written_total = 0;
      
      // Write the complete stereo frames to I2S
      while (bytes_written_total < write_bytes) {
        size_t bytes_written = 0;
        i2s_write(I2S_NUM, 
                  read_buf + bytes_written_total, 
                  write_bytes - bytes_written_total, 
                  &bytes_written, 
                  portMAX_DELAY);
        bytes_written_total += bytes_written;
      }
    }
    
    // Save any leftover bytes
    if (remaining_bytes > 0) {
      memcpy(leftover_buf, read_buf + write_bytes, remaining_bytes);
      leftover_len = remaining_bytes;
    } else {
      leftover_len = 0;
    }
  } else {
    // No new data read (bytes_read == 0).
    // Yield the CPU to allow the background TinyUSB task to run and receive packets.
    unsigned long silent_duration = millis() - last_data_time;
    if (silent_duration > 50) {
      // Feed I2S with 1ms of zeros to prevent it from underflowing and looping the old audio
      // At 16kHz, 1ms of stereo 16-bit PCM is exactly 64 bytes
      alignas(4) static const uint8_t silence[64] = {0};
      size_t bytes_written = 0;
      i2s_write(I2S_NUM, silence, sizeof(silence), &bytes_written, portMAX_DELAY);
      
      // If silent for more than 200ms, the stream has actually stopped. Reset state to require preroll.
      if (silent_duration > 200) {
        playing = false;
        i2s_zero_dma_buffer(I2S_NUM);
      }
    } else {
      // Yield CPU to the USB task
      delay(1);
    }
  }
}