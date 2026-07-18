#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include "driver/i2s_std.h"
#include "esp_mac.h"
#define ENABLE_DEBUG_LOGS 0 // Set to 1 to enable verbose debugging

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

typedef struct __attribute__((packed)) {
    char magic[8];             // "TX_CONF"
    uint8_t transmitter_mac[6]; // Transmitter's MAC address
    uint8_t receiver_mac[6];    // Receiver's MAC address in transmitter memory
    uint32_t seq_num;           // Sequence number
} tx_confirm_packet_t;

// --- Thread-Safe Ring Buffer Implementation ---
class AudioRingBuffer {
private:
    uint8_t* buffer;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t size;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

public:
    AudioRingBuffer(size_t cap) : capacity(cap), head(0), tail(0), size(0) {
        buffer = new uint8_t[capacity];
        memset(buffer, 0, capacity);
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

// Statistics and connection tracking
static uint32_t stats_frames_received = 0;
static uint32_t stats_sub_packets_received = 0;
static uint32_t stats_underflows = 0;
static uint8_t sender_mac[6] = {0};
static bool sender_known = false;

// ESP-NOW Frame Assembly Staging
static uint8_t staging_buffer[FRAME_SIZE];
static uint32_t active_seq = 0xFFFFFFFF;
static uint8_t received_mask = 0;

static volatile uint32_t last_packet_time = 0;
static i2s_chan_handle_t tx_chan = NULL;

// ESP-NOW receive callback
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingBytes, int len) {
    const uint8_t* mac = recv_info->src_addr;
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingBytes, int len) {
#endif
    last_packet_time = millis();

#if ENABLE_DEBUG_LOGS
    static uint32_t last_raw_recv_time = 0;
    if (millis() - last_raw_recv_time >= 1000) {
        last_raw_recv_time = millis();
        Serial.printf("Raw ESP-NOW recv: len=%d from %02X:%02X:%02X:%02X:%02X:%02X\n",
                      len, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
#endif

    if (len == sizeof(tx_confirm_packet_t)) {
        const tx_confirm_packet_t* conf = (const tx_confirm_packet_t*)incomingBytes;
#if ENABLE_DEBUG_LOGS
        Serial.printf("DEBUG: Received 24-byte packet. Magic: '%.8s' | Hex: %02X %02X %02X %02X %02X %02X %02X %02X | Seq: %lu\n",
                      conf->magic, 
                      conf->magic[0], conf->magic[1], conf->magic[2], conf->magic[3],
                      conf->magic[4], conf->magic[5], conf->magic[6], conf->magic[7],
                      (unsigned long)conf->seq_num);
        Serial.printf("DEBUG: Transmitter MAC: %02X:%02X:%02X:%02X:%02X:%02X | Receiver MAC in payload: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      conf->transmitter_mac[0], conf->transmitter_mac[1], conf->transmitter_mac[2],
                      conf->transmitter_mac[3], conf->transmitter_mac[4], conf->transmitter_mac[5],
                      conf->receiver_mac[0], conf->receiver_mac[1], conf->receiver_mac[2],
                      conf->receiver_mac[3], conf->receiver_mac[4], conf->receiver_mac[5]);
#endif
        
        bool is_magic_match = (conf->magic[0] == 'T' && conf->magic[1] == 'X' && conf->magic[2] == '_' && 
                               conf->magic[3] == 'C' && conf->magic[4] == 'O' && conf->magic[5] == 'N' && 
                               conf->magic[6] == 'F') ||
                              (conf->magic[0] == 'T' && conf->magic[1] == 'E' && conf->magic[2] == 'S' && 
                               conf->magic[3] == 'T' && conf->magic[4] == '_' && conf->magic[5] == 'U' && 
                               conf->magic[6] == 'N' && conf->magic[7] == 'I');
                               
#if ENABLE_DEBUG_LOGS
        Serial.printf("DEBUG: is_magic_match: %d | sender_known: %d\n", is_magic_match, sender_known);
#endif
        if (is_magic_match) {
            if (!sender_known || memcmp(sender_mac, conf->transmitter_mac, 6) != 0) {
                esp_now_peer_info_t peer;
                memset(&peer, 0, sizeof(peer));
                memcpy(peer.peer_addr, conf->transmitter_mac, 6);
                peer.channel = 1;
                peer.ifidx = WIFI_IF_STA;
                peer.encrypt = false;
                
                if (esp_now_is_peer_exist(conf->transmitter_mac)) {
                    esp_now_del_peer(conf->transmitter_mac);
                }
                
                esp_err_t err = esp_now_add_peer(&peer);
                if (err == ESP_OK) {
                    // Set PHY rate for audio throughput with good range
                    // 802.11b 11 Mbps (CCK): better range than OFDM, ~1200 unicast pkts/s capacity
                    esp_now_rate_config_t rate_cfg = {
                        .phymode = WIFI_PHY_MODE_11B,
                        .rate = WIFI_PHY_RATE_11M_L,
                        .ersu = false,
                        .dcm = false,
                    };
                    esp_now_set_peer_rate_config(conf->transmitter_mac, &rate_cfg);

                    memcpy(sender_mac, conf->transmitter_mac, 6);
                    sender_known = true;
#if ENABLE_DEBUG_LOGS
                    Serial.printf("Successfully registered Transmitter peer: %02X:%02X:%02X:%02X:%02X:%02X\n",
                                  sender_mac[0], sender_mac[1], sender_mac[2],
                                  sender_mac[3], sender_mac[4], sender_mac[5]);
#endif
                } else {
                    Serial.printf("Failed to register Transmitter peer: %s\n", esp_err_to_name(err));
                }
            }
        }
    }

    if (len == sizeof(audio_packet_t)) {
        stats_sub_packets_received++;
        const audio_packet_t* pkt = (const audio_packet_t*)incomingBytes;
        
        // Monitor transmitter pairing / MAC address updates
        if (!sender_known || memcmp(sender_mac, mac, 6) != 0) {
            memcpy(sender_mac, mac, 6);
            sender_known = true;
            Serial.printf("Pairing complete. Receiving audio from Transmitter MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }

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
                    stats_frames_received++;
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
#if ENABLE_DEBUG_LOGS
                Serial.println("Buffering complete. Playback started.");
#endif
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
#if ENABLE_DEBUG_LOGS
                Serial.println("Buffer underflow. Buffering...");
#endif
                stats_underflows++;
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
    
    // Wait up to 3 seconds for Serial Monitor to connect (needed for native USB CDC)
    uint32_t start_time = millis();
    while (!Serial && (millis() - start_time < 3000)) {
        delay(10);
    }
    
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

    // Start Wi-Fi in APSTA Mode on Channel 1 to lock the radio frequency and fully start both interfaces
    WiFi.mode(WIFI_AP_STA);
    WiFi.disconnect(false, true); // Clear saved credentials
    WiFi.persistent(false); // Disable persistent settings in NVS
    WiFi.softAP("Lossless_RX", NULL, 1); // Channel 1, no password (locks radio to Channel 1)
    esp_wifi_set_ps(WIFI_PS_NONE); // Disable Wi-Fi sleep for low latency
    esp_wifi_set_max_tx_power(84); // 21 dBm max TX power for range

    uint8_t rx_mac[6];
    esp_read_mac(rx_mac, ESP_MAC_WIFI_STA); // Use STA MAC address for ESP-NOW
    Serial.printf("Receiver MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  rx_mac[0], rx_mac[1], rx_mac[2], rx_mac[3], rx_mac[4], rx_mac[5]);
    
    uint8_t primary_chan = 0;
    wifi_second_chan_t second_chan;
    esp_wifi_get_channel(&primary_chan, &second_chan);
    Serial.printf("Receiver Wi-Fi Channel initialized to: %d\n", primary_chan);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_err_t cb_err;
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    cb_err = esp_now_register_recv_cb(OnDataRecv);
#else
    cb_err = esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
#endif
    if (cb_err != ESP_OK) {
        Serial.printf("Error registering ESP-NOW receive callback: %s\n", esp_err_to_name(cb_err));
    } else {
        Serial.println("ESP-NOW receive callback registered successfully.");
    }

    // Add Broadcast Peer for pairing beacons
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcast_mac, 6);
    peerInfo.channel = 1;
    peerInfo.ifidx = WIFI_IF_STA;
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
    static uint32_t last_stats_time = 0;
    uint32_t now = millis();

    // 1. Send broadcast pairing beacon every 1 second
    if (now - last_beacon_time >= 1000) {
        last_beacon_time = now;
        
        // Blink LED briefly to show beacon transmission (only if not streaming)
        bool is_streaming = (now - last_packet_time < 200);
        if (!is_streaming) {
            digitalWrite(STATUS_LED_PIN, LOW); // LED ON
            delay(50);
            digitalWrite(STATUS_LED_PIN, HIGH); // LED OFF
        }
        
        beacon_packet_t beacon;
        memcpy(beacon.magic, "LR_BEACN", 8);
        esp_read_mac(beacon.mac, ESP_MAC_WIFI_STA);
        
        uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        esp_err_t send_err = esp_now_send(broadcast_mac, (uint8_t*)&beacon, sizeof(beacon_packet_t));
        if (send_err != ESP_OK) {
#if ENABLE_DEBUG_LOGS
            Serial.printf("Failed to send pairing beacon: %s\n", esp_err_to_name(send_err));
#endif
        }
    }

    // 2. Control status LED & connection timeout
    bool is_streaming = (now - last_packet_time < 200);
    if (is_streaming) {
        digitalWrite(STATUS_LED_PIN, LOW); // LED ON
    } else {
        digitalWrite(STATUS_LED_PIN, HIGH); // LED OFF
        // Reset sender status if stream went dead for more than 5 seconds
        if (sender_known && (now - last_packet_time > 5000)) {
            sender_known = false;
            memset(sender_mac, 0, 6);
            Serial.println("Transmitter connection lost (timeout).");
        }
    }

    // 3. Periodically print stats every 1 second
    if (now - last_stats_time >= 1000) {
        uint32_t elapsed = now - last_stats_time;
        last_stats_time = now;

        uint8_t primary_chan = 0;
        wifi_second_chan_t second_chan;
        esp_wifi_get_channel(&primary_chan, &second_chan);

        if (is_streaming) {
            uint32_t frames = stats_frames_received;
            uint32_t packets = stats_sub_packets_received;
            uint32_t underflows = stats_underflows;

            // Reset stats counters
            stats_frames_received = 0;
            stats_sub_packets_received = 0;
            stats_underflows = 0;

            size_t buffered_bytes = ring_buffer.available();
            float buffer_ms = (float)buffered_bytes / 192.0f; // 48000 Hz * 2 channels * 2 bytes/sample = 192 bytes/ms

            Serial.printf("[LINK STATUS] Chan: %d | Frames: %lu/s (Expected: ~100) | Sub-packets: %lu/s (Expected: ~800) | Buffer: %zu bytes (%.1f ms) | Underflows: %lu/s\n",
                          primary_chan, frames, packets, buffered_bytes, buffer_ms, underflows);
        } else {
            Serial.printf("[IDLE] Waiting for Transmitter... Current Wi-Fi Chan: %d\n", primary_chan);
        }
    }

    delay(20);
}