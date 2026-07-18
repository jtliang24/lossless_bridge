# Receiver Implementation Guide (`lossless_rx`)

This guide outlines how to code and compile the **Receiver** (`lossless_rx`) project using the Arduino framework.

---

## 1. Project Configuration (`platformio.ini`)

Ensure your `lossless_rx/platformio.ini` is configured as follows (already set up in the workspace):
```ini
[env:seeed_xiao_esp32s3]
platform = espressif32
board = seeed_xiao_esp32s3
framework = arduino

build_flags =
    -DBOARD_HAS_PSRAM

board_build.partitions = huge_app.csv
```

---

## 2. Main Application Code (`src/main.cpp`)

Implement the receiver application in `lossless_rx/src/main.cpp`. This includes:
- A custom, thread-safe, non-allocating circular ring buffer.
- ESP-NOW receive handler to staging-assemble the 8 sub-packets (1920 bytes total) per frame.
- An independent high-priority FreeRTOS playback task assigned to **Core 1** to stream data directly into the I2S PCM5102 DAC.
- A background pairing beacon task that periodically broadcasts the receiver's MAC address to pair with the transmitter.

```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include "driver/i2s_std.h"

#define STATUS_LED_PIN GPIO_NUM_21 // active-low LED on Seeed Studio XIAO ESP32S3

// I2S Pin definitions for Seeed Studio XIAO ESP32S3
#define I2S_BCLK_PIN GPIO_NUM_4 // D3
#define I2S_WS_PIN   GPIO_NUM_5 // D4
#define I2S_DOUT_PIN GPIO_NUM_6 // D5

#define AUDIO_PAYLOAD_SIZE    240
#define SUB_PACKETS_PER_FRAME 8
#define FRAME_SIZE            (AUDIO_PAYLOAD_SIZE * SUB_PACKETS_PER_FRAME) // 1920 bytes (480 stereo samples)

typedef struct __attribute__((packed)) {
    uint32_t seq_num;          // Incremented for every 10ms frame
    uint8_t sub_packet_idx;    // 0 to 7
    uint8_t total_sub_packets; // 8
    uint16_t payload_len;      // 240
    uint8_t audio_data[AUDIO_PAYLOAD_SIZE];
} audio_packet_t;

typedef struct __attribute__((packed)) {
    char magic[8];             // "LR_BEACN"
    uint8_t mac[6];            // Receiver's MAC address
} beacon_packet_t;

// --- Thread-Safe Ring Buffer Implementation ---
class AudioRingBuffer {
private:
    uint8_t* buffer;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t size;
    portMUX_TYPE mux;

public:
    AudioRingBuffer(size_t cap) : capacity(cap), head(0), tail(0), size(0) {
        buffer = new uint8_t[capacity];
        memset(buffer, 0, capacity);
        vPortCPUInitializeMutex(&mux);
    }
    
    ~AudioRingBuffer() {
        delete[] buffer;
    }
    
    size_t write(const uint8_t* data, size_t len) {
        portENTER_CRITICAL(&mux);
        if (size + len > capacity) {
            // Buffer overflow, drop oldest frame to make space (FIFO)
            size_t overflow = (size + len) - capacity;
            head = (head + overflow) % capacity;
            size -= overflow;
        }
        size_t first_part = min(len, capacity - tail);
        memcpy(buffer + tail, data, first_part);
        memcpy(buffer, data + first_part, len - first_part);
        tail = (tail + len) % capacity;
        size += len;
        portEXIT_CRITICAL(&mux);
        return len;
    }
    
    size_t read(uint8_t* dest, size_t len) {
        portENTER_CRITICAL(&mux);
        if (len > size) {
            len = size;
        }
        if (len == 0) {
            portEXIT_CRITICAL(&mux);
            return 0;
        }
        size_t first_part = min(len, capacity - head);
        memcpy(dest, buffer + head, first_part);
        memcpy(dest + first_part, buffer, len - first_part);
        head = (head + len) % capacity;
        size -= len;
        portEXIT_CRITICAL(&mux);
        return len;
    }
    
    size_t available() {
        portENTER_CRITICAL(&mux);
        size_t s = size;
        portEXIT_CRITICAL(&mux);
        return s;
    }

    void clear() {
        portENTER_CRITICAL(&mux);
        head = 0;
        tail = 0;
        size = 0;
        portEXIT_CRITICAL(&mux);
    }
};

// Allocate a 19200 byte ring buffer (10 frames / 100ms of stereo 48kHz 16-bit)
AudioRingBuffer ring_buffer(19200);

// ESP-NOW Frame Assembly Staging
static uint8_t staging_buffer[FRAME_SIZE];
static uint32_t active_seq = 0xFFFFFFFF;
static uint8_t received_mask = 0;

static volatile uint32_t last_packet_time = 0;
static i2s_chan_handle_t tx_chan = NULL;

// ESP-NOW receive callback
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingBytes, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingBytes, int len) {
#endif
    last_packet_time = millis();

    if (len == sizeof(audio_packet_t)) {
        const audio_packet_t* pkt = (const audio_packet_t*)incomingBytes;
        
        // 1. Check if new sequence frame
        if (pkt->seq_num > active_seq || active_seq == 0xFFFFFFFF) {
            active_seq = pkt->seq_num;
            received_mask = 0;
        }
        
        // 2. Put sub-packet in correct slot
        if (pkt->seq_num == active_seq) {
            uint8_t idx = pkt->sub_packet_idx;
            if (idx < SUB_PACKETS_PER_FRAME && !(received_mask & (1 << idx))) {
                memcpy(staging_buffer + (idx * AUDIO_PAYLOAD_SIZE), pkt->audio_data, AUDIO_PAYLOAD_SIZE);
                received_mask |= (1 << idx);
                
                // 3. If all 8 pieces received, push to playback ring buffer
                if (received_mask == 0xFF) {
                    ring_buffer.write(staging_buffer, FRAME_SIZE);
                    active_seq = 0xFFFFFFFF; // Reset for next frame
                    received_mask = 0;
                }
            }
        }
    }
}

// Initialize standard I2S Master
static esp_err_t init_i2s(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    if (ret != ESP_OK) return ret;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000), // 48kHz
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)I2S_BCLK_PIN,
            .ws = (gpio_num_t)I2S_WS_PIN,
            .dout = (gpio_num_t)I2S_DOUT_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    // Force 16-bit slot width to match the transmitter updates and prevent Right-channel silence
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;

    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if (ret != ESP_OK) return ret;

    return i2s_channel_enable(tx_chan);
}

// Dedicated FreeRTOS I2S playback task pinned to Core 1
void i2s_playback_task(void *pvParameters) {
    static uint8_t out_buf[FRAME_SIZE];
    static uint8_t silence_buf[FRAME_SIZE] = {0};
    bool prebuffering = true;
    
    // Playback starts when prebuffer threshold (30ms / 3 frames) is met
    const size_t prebuffer_bytes = 3 * FRAME_SIZE; 
    
    while (1) {
        if (prebuffering) {
            if (ring_buffer.available() >= prebuffer_bytes) {
                prebuffering = false;
                Serial.println("Buffering complete. Playback started.");
            } else {
                // Write silence to prevent clock issues
                size_t written = 0;
                i2s_channel_write(tx_chan, silence_buf, FRAME_SIZE, &written, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(10)); // Wait for next UAC tick
            }
        } else {
            size_t bytes_read = ring_buffer.read(out_buf, FRAME_SIZE);
            if (bytes_read < FRAME_SIZE) {
                // Buffer underflow
                prebuffering = true;
                ring_buffer.clear();
                Serial.println("Buffer underflow. Buffering...");
                size_t written = 0;
                i2s_channel_write(tx_chan, silence_buf, FRAME_SIZE, &written, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                size_t written = 0;
                i2s_channel_write(tx_chan, out_buf, FRAME_SIZE, &written, portMAX_DELAY);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("RECEIVER: Initializing Lossless Receiver...");

    // Status LED initialization (active-low)
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH); // OFF

    // Initialize I2S
    if (init_i2s() != ESP_OK) {
        Serial.println("I2S Initialization failed!");
        while (1) delay(100);
    }
    Serial.println("I2S initialized successfully.");

    // Start Wi-Fi in Station Mode on Channel 1
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_ps(WIFI_PS_NONE); // Disable Wi-Fi sleep for low latency

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    esp_now_register_recv_cb(OnDataRecv);
#else
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
#endif

    // Add Broadcast Peer for pairing beacons
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcast_mac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add broadcast peer!");
    }

    // Launch Playback Task on Core 1
    xTaskCreatePinnedToCore(
        i2s_playback_task,
        "i2s_playback_task",
        4096,
        NULL,
        configMAX_PRIORITIES - 1, // High priority
        NULL,
        1 // Core 1
    );

    Serial.println("Receiver setup completed.");
}

void loop() {
    static uint32_t last_beacon_time = 0;
    uint32_t now = millis();

    // 1. Send broadcast pairing beacon every 1 second
    if (now - last_beacon_time >= 1000) {
        last_beacon_time = now;
        
        beacon_packet_t beacon;
        memcpy(beacon.magic, "LR_BEACN", 8);
        esp_read_mac(beacon.mac, ESP_MAC_WIFI_STA);
        
        uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        esp_now_send(broadcast_mac, (uint8_t*)&beacon, sizeof(beacon_packet_t));
    }

    // 2. Control status LED: ON when audio stream is active (packets received in last 200ms)
    if (now - last_packet_time < 200) {
        digitalWrite(STATUS_LED_PIN, LOW); // LED ON
    } else {
        digitalWrite(STATUS_LED_PIN, HIGH); // LED OFF
    }

    delay(20);
}
```
