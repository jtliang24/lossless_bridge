#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <Preferences.h>
#include "driver/i2s_std.h"
#include "esp_mac.h"
#define ENABLE_DEBUG_LOGS 0 // Set to 1 to enable verbose debugging

#define STATUS_LED_PIN GPIO_NUM_21 // active-low LED on Seeed Studio XIAO ESP32S3

// I2S Pin definitions for Seeed Studio XIAO ESP32S3
#define I2S_BCLK_PIN GPIO_NUM_4 // D3
#define I2S_WS_PIN   GPIO_NUM_5 // D4
#define I2S_DOUT_PIN GPIO_NUM_6 // D5

// Hardware Volume Buttons (Active-Low with internal pull-up)
#define VOLUME_UP_PIN   GPIO_NUM_1 // D0 pin on Seeed Studio XIAO ESP32S3
#define VOLUME_DOWN_PIN GPIO_NUM_2 // D1 pin on Seeed Studio XIAO ESP32S3

#define AUDIO_PAYLOAD_SIZE    240
#define SUB_PACKETS_PER_FRAME 8
#define FRAME_SIZE            (AUDIO_PAYLOAD_SIZE * SUB_PACKETS_PER_FRAME) // 1920 bytes (480 stereo samples)

// Mode configuration (High Quality vs Low Latency)
// Default: 0 = High Quality (Pure Bit-Exact 48kHz PCM), 1 = Low Latency (Zero-Crossing Resampling)
#define ENABLE_DRIFT_COMPENSATION_DEFAULT 0 
static volatile bool config_low_latency_mode = (ENABLE_DRIFT_COMPENSATION_DEFAULT != 0);

// Hardware Mode Switch Pin (GPIO 7 / D8 on Seeed Studio XIAO ESP32S3)
// 3.3V / HIGH = Low-Latency Mode (~35ms target)
// GND / LOW  = High-Quality Mode (Pure Bit-Exact 48kHz PCM)
#define MODE_SWITCH_PIN GPIO_NUM_7 // D8 pin on Seeed Studio XIAO ESP32S3

// 21-step Logarithmic (Perceptual dB) Volume Table in Q15 format (0 to 32768)
// Step 0 = 0% (Mute), Step 20 = 100% (0 dB, bit-exact throughput)
static const uint16_t VOLUME_Q15_TABLE[21] = {
    0,     // Level 0  (0%   / Mute)
    328,   // Level 1  (5%   / -40 dB)
    519,   // Level 2  (10%  / -36 dB)
    822,   // Level 3  (15%  / -32 dB)
    1302,  // Level 4  (20%  / -28 dB)
    2064,  // Level 5  (25%  / -24 dB)
    3271,  // Level 6  (30%  / -20 dB)
    4621,  // Level 7  (35%  / -17 dB)
    6529,  // Level 8  (40%  / -14 dB)
    9222,  // Level 9  (45%  / -11 dB)
    13028, // Level 10 (50%  / -8 dB)
    15509, // Level 11 (55%  / -6.5 dB)
    18461, // Level 12 (60%  / -5 dB)
    20701, // Level 13 (65%  / -4 dB)
    23214, // Level 14 (70%  / -3 dB)
    26034, // Level 15 (75%  / -2 dB)
    29195, // Level 16 (80%  / -1 dB)
    30922, // Level 17 (85%  / -0.5 dB)
    31823, // Level 18 (90%  / -0.25 dB)
    32287, // Level 19 (95%  / -0.1 dB)
    32768  // Level 20 (100% / 0 dB)
};

static volatile uint8_t volume_level = 20; // Default 100%
static Preferences preferences;
static bool volume_dirty = false;
static uint32_t volume_last_change_time = 0;

static inline void apply_volume_scaling(int16_t* samples, size_t num_samples) {
    uint8_t lvl = volume_level;
    if (lvl >= 20) return; // 100% volume, bit-exact throughput
    uint32_t gain = VOLUME_Q15_TABLE[lvl];
    if (gain == 0) {
        memset(samples, 0, num_samples * sizeof(int16_t));
        return;
    }
    for (size_t i = 0; i < num_samples; i++) {
        samples[i] = (int16_t)(((int32_t)samples[i] * gain) >> 15);
    }
}

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

// Statistics and connection tracking
static uint32_t stats_frames_received = 0;
static uint32_t stats_sub_packets_received = 0;
static uint32_t stats_underflows = 0;
static uint32_t stats_buffer_overflows = 0;
static uint32_t stats_silence_frames = 0;
static uint32_t stats_samples_dropped = 0;
static uint32_t stats_samples_duplicated = 0;
static uint8_t sender_mac[6] = {0};
static bool sender_known = false;

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
            stats_buffer_overflows++;
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

// Allocate a 38400 byte ring buffer (20 frames / 200ms of stereo 48kHz 16-bit)
AudioRingBuffer ring_buffer(38400);

// ESP-NOW Frame Assembly Staging
static uint8_t staging_buffer[FRAME_SIZE];
static uint32_t active_seq = 0xFFFFFFFF;
static uint8_t received_mask = 0;
static uint8_t parity_buffer[AUDIO_PAYLOAD_SIZE];
static bool parity_received = false;
static uint32_t stats_fec_recoveries = 0;

static volatile uint32_t last_packet_time = 0;
static i2s_chan_handle_t tx_chan = NULL;

static bool try_fec_recovery() {
    if (!parity_received) return false;
    uint8_t missing = ~received_mask & 0xFF;
    if (__builtin_popcount(missing) != 1) return false;

    int missing_idx = __builtin_ctz(missing);
    uint8_t *target = staging_buffer + (missing_idx * AUDIO_PAYLOAD_SIZE);
    memcpy(target, parity_buffer, AUDIO_PAYLOAD_SIZE);
    for (int i = 0; i < SUB_PACKETS_PER_FRAME; i++) {
        if (i != missing_idx) {
            const uint8_t *src = staging_buffer + (i * AUDIO_PAYLOAD_SIZE);
            for (size_t b = 0; b < AUDIO_PAYLOAD_SIZE; b++) {
                target[b] ^= src[b];
            }
        }
    }
    received_mask |= (1 << missing_idx);
    stats_fec_recoveries++;
    return true;
}

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
                peer.channel = 8;
                peer.ifidx = WIFI_IF_STA;
                peer.encrypt = false;
                
                if (esp_now_is_peer_exist(conf->transmitter_mac)) {
                    esp_now_del_peer(conf->transmitter_mac);
                }
                
                esp_err_t err = esp_now_add_peer(&peer);
                if (err == ESP_OK) {
                    // Set PHY rate for audio throughput with good range
                    // 802.11g 24 Mbps (OFDM): reduced airtime, ~2400 unicast pkts/s capacity
                    esp_now_rate_config_t rate_cfg = {
                        .phymode = WIFI_PHY_MODE_11G,
                        .rate = WIFI_PHY_RATE_24M,
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
            if (active_seq != 0xFFFFFFFF) {
                // Flush incomplete frame if we received a partial frame
                if (received_mask > 0 && received_mask < 0xFF) {
                    try_fec_recovery();
                    if (received_mask < 0xFF) {
                        for (int i = 0; i < SUB_PACKETS_PER_FRAME; i++) {
                            if (!(received_mask & (1 << i))) {
                                memset(staging_buffer + (i * AUDIO_PAYLOAD_SIZE), 0, AUDIO_PAYLOAD_SIZE);
                            }
                        }
                    }
                    ring_buffer.write(staging_buffer, FRAME_SIZE);
                    stats_frames_received++;
                }

                // If sequence number skipped frames, fill with silence to keep the buffer level up
                uint32_t skipped = pkt->seq_num - active_seq - 1;
                if (skipped > 0 && skipped <= 10) { // Limit to 10 frames (100ms) to prevent overflow
                    static uint8_t silence_frame[FRAME_SIZE] = {0};
                    for (uint32_t s = 0; s < skipped; s++) {
                        ring_buffer.write(silence_frame, FRAME_SIZE);
                        stats_frames_received++;
                        stats_silence_frames++;
                    }
                }
            }
            active_seq = pkt->seq_num;
            received_mask = 0;
            parity_received = false;
        }
        
        // 2. Put sub-packet in correct slot
        if (pkt->seq_num == active_seq) {
            uint8_t idx = pkt->sub_packet_idx;
            if (idx == SUB_PACKETS_PER_FRAME) {
                // FEC Parity Sub-packet (index 8)
                memcpy(parity_buffer, pkt->audio_data, AUDIO_PAYLOAD_SIZE);
                parity_received = true;
                if (received_mask != 0xFF && __builtin_popcount(received_mask & 0xFF) == 7) {
                    try_fec_recovery();
                }
            } else if (idx < SUB_PACKETS_PER_FRAME && !(received_mask & (1 << idx))) {
                // Audio Sub-packet (indices 0..7)
                memcpy(staging_buffer + (idx * AUDIO_PAYLOAD_SIZE), pkt->audio_data, AUDIO_PAYLOAD_SIZE);
                received_mask |= (1 << idx);
                
                if (received_mask != 0xFF && __builtin_popcount(received_mask & 0xFF) == 7) {
                    try_fec_recovery();
                }
            }

            // 3. If all 8 pieces received or recovered via FEC, push to playback ring buffer
            if (received_mask == 0xFF) {
                ring_buffer.write(staging_buffer, FRAME_SIZE);
                stats_frames_received++;
                active_seq = 0xFFFFFFFF; // Reset for next frame
                received_mask = 0;
                parity_received = false;
            }
        }
    }
}

// Initialize standard I2S Master
static esp_err_t init_i2s(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 8; // 8 descriptors * 5ms = 40ms DMA buffering capacity
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

// Helper function to find zero-crossing sample index in 16-bit stereo PCM buffer
static inline int find_zero_crossing(const int16_t* samples, int num_samples) {
    int best_idx = 0;
    int32_t min_amp = 0x7FFFFFFF;
    for (int i = 0; i < num_samples; i++) {
        int32_t amp = abs((int32_t)samples[i * 2]) + abs((int32_t)samples[i * 2 + 1]);
        if (amp < min_amp) {
            min_amp = amp;
            best_idx = i;
            if (amp == 0) break; // Perfect zero crossing found
        }
    }
    return best_idx;
}

// Dedicated FreeRTOS I2S playback task pinned to Core 1
void i2s_playback_task(void *pvParameters) {
    // All buffers allocated statically in BSS memory to prevent task stack overflow
    static uint8_t raw_pcm_buf[FRAME_SIZE + 32];
    static uint8_t conceal_buf[FRAME_SIZE];
    static uint8_t silence_buf[FRAME_SIZE] = {0};
    static bool has_last_good = false;
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
                // Conceal during initial prebuffering / re-buffering
                size_t written = 0;
                if (has_last_good) {
                    // Attenuate last good frame by 50% to softly fade
                    int16_t *samples = (int16_t *)conceal_buf;
                    for (size_t i = 0; i < FRAME_SIZE / 2; i++) {
                        samples[i] >>= 1;
                    }
                    i2s_channel_write(tx_chan, conceal_buf, FRAME_SIZE, &written, portMAX_DELAY);
                } else {
                    i2s_channel_write(tx_chan, silence_buf, FRAME_SIZE, &written, portMAX_DELAY);
                }
                vTaskDelay(pdMS_TO_TICKS(2)); // Poll buffer status frequently during rebuffering
            }
        } else {
            if (ring_buffer.available() < FRAME_SIZE) {
                // Buffer underflow - trigger prebuffering but DO NOT clear the buffer
                prebuffering = true;
#if ENABLE_DEBUG_LOGS
                Serial.println("Buffer underflow. Buffering...");
#endif
                stats_underflows++;
                size_t written = 0;
                if (has_last_good) {
                    int16_t *samples = (int16_t *)conceal_buf;
                    for (size_t i = 0; i < FRAME_SIZE / 2; i++) {
                        samples[i] >>= 1;
                    }
                    i2s_channel_write(tx_chan, conceal_buf, FRAME_SIZE, &written, portMAX_DELAY);
                } else {
                    i2s_channel_write(tx_chan, silence_buf, FRAME_SIZE, &written, portMAX_DELAY);
                }
                vTaskDelay(pdMS_TO_TICKS(2));
            } else {
                size_t current_buffer = ring_buffer.available();
                size_t written = 0;

                // Instant flush excess buffer down to 35ms target (6720 B) when in Low Latency mode and buffer > 100ms
                if (config_low_latency_mode && current_buffer > 19200) {
                    while (ring_buffer.available() > 6720) {
                        ring_buffer.read(raw_pcm_buf, FRAME_SIZE);
                    }
                    current_buffer = ring_buffer.available();
                }

                // Multi-tier drift scaling (Target: ~35ms / 6720 B) - Only active when config_low_latency_mode is true
                if (config_low_latency_mode && current_buffer > 9600) {
                    int drop_count = 1;
                    if (current_buffer > 17280) {
                        drop_count = 3; // Max 3 samples/frame
                    } else if (current_buffer > 13440) {
                        drop_count = 2;
                    }

                    size_t read_bytes = FRAME_SIZE + (drop_count * 4); // Read extra samples from ring buffer
                    size_t bytes_read = ring_buffer.read(raw_pcm_buf, read_bytes);
                    if (bytes_read == read_bytes) {
                        int16_t *pcm = (int16_t *)raw_pcm_buf;
                        int samples_count = bytes_read / 4;
                        for (int d = 0; d < drop_count; d++) {
                            int drop_idx = find_zero_crossing(pcm, samples_count);
                            memmove(&pcm[drop_idx * 2], &pcm[(drop_idx + 1) * 2], (samples_count - 1 - drop_idx) * 4);
                            samples_count--;
                        }
                        // Output is ALWAYS exactly FRAME_SIZE (1,920 bytes) to match I2S DMA descriptors!
                        apply_volume_scaling((int16_t*)raw_pcm_buf, FRAME_SIZE / 2);
                        memcpy(conceal_buf, raw_pcm_buf, FRAME_SIZE);
                        has_last_good = true;
                        i2s_channel_write(tx_chan, raw_pcm_buf, FRAME_SIZE, &written, portMAX_DELAY);
                        stats_samples_dropped += drop_count;
                    } else {
                        apply_volume_scaling((int16_t*)raw_pcm_buf, bytes_read / 2);
                        memcpy(conceal_buf, raw_pcm_buf, bytes_read);
                        has_last_good = true;
                        i2s_channel_write(tx_chan, raw_pcm_buf, bytes_read, &written, portMAX_DELAY);
                    }
                } else if (config_low_latency_mode && current_buffer < 4800 && current_buffer >= (FRAME_SIZE - 4)) {
                    // Duplicate 1 sample (read 479 samples = 1916 bytes, output 480 samples = 1920 bytes)
                    size_t read_bytes = FRAME_SIZE - 4;
                    size_t bytes_read = ring_buffer.read(raw_pcm_buf, read_bytes);
                    if (bytes_read == read_bytes) {
                        int16_t *pcm = (int16_t *)raw_pcm_buf;
                        int dup_idx = find_zero_crossing(pcm, 479);
                        // Shift right by 1 sample past dup_idx
                        memmove(&pcm[(dup_idx + 2) * 2], &pcm[(dup_idx + 1) * 2], (479 - 1 - dup_idx) * 4);
                        // Duplicate sample
                        pcm[(dup_idx + 1) * 2]     = pcm[dup_idx * 2];
                        pcm[(dup_idx + 1) * 2 + 1] = pcm[dup_idx * 2 + 1];
                        
                        apply_volume_scaling((int16_t*)raw_pcm_buf, FRAME_SIZE / 2);
                        memcpy(conceal_buf, raw_pcm_buf, FRAME_SIZE);
                        has_last_good = true;
                        i2s_channel_write(tx_chan, raw_pcm_buf, FRAME_SIZE, &written, portMAX_DELAY);
                        stats_samples_duplicated++;
                    } else {
                        apply_volume_scaling((int16_t*)raw_pcm_buf, bytes_read / 2);
                        memcpy(conceal_buf, raw_pcm_buf, bytes_read);
                        has_last_good = true;
                        i2s_channel_write(tx_chan, raw_pcm_buf, bytes_read, &written, portMAX_DELAY);
                    }
                } else {
                    size_t bytes_read = ring_buffer.read(raw_pcm_buf, FRAME_SIZE);
                    apply_volume_scaling((int16_t*)raw_pcm_buf, bytes_read / 2);
                    memcpy(conceal_buf, raw_pcm_buf, FRAME_SIZE);
                    has_last_good = true;
                    i2s_channel_write(tx_chan, raw_pcm_buf, FRAME_SIZE, &written, portMAX_DELAY);
                }
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

    // Start Wi-Fi in STA Mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true); // Clear saved credentials
    WiFi.persistent(false); // Disable persistent settings in NVS
    
    // Lock the radio to Channel 8 using the promiscuous mode workaround in STA mode
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(8, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

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
    peerInfo.channel = 8;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add broadcast peer!");
    }

#if defined(MODE_SWITCH_PIN) && MODE_SWITCH_PIN >= 0
    pinMode(MODE_SWITCH_PIN, INPUT_PULLDOWN);
    config_low_latency_mode = (digitalRead(MODE_SWITCH_PIN) == HIGH);
    Serial.printf("Hardware Mode Switch initialized on GPIO %d. Initial Mode: %s\n",
                  MODE_SWITCH_PIN, config_low_latency_mode ? "Low Latency" : "High Quality");
#endif

    // Volume buttons initialization
    pinMode(VOLUME_UP_PIN, INPUT_PULLUP);
    pinMode(VOLUME_DOWN_PIN, INPUT_PULLUP);

    preferences.begin("audio_rx", false);
    volume_level = preferences.getUChar("volume", 20);
    if (volume_level > 20) volume_level = 20;
    Serial.printf("Loaded Volume Level from NVS: %d/20 (%d%%)\n", volume_level, volume_level * 5);

    // Launch Playback Task on Core 1 (increased stack size to 8192 bytes for stability)
    xTaskCreatePinnedToCore(
        i2s_playback_task,
        "i2s_playback_task",
        8192,
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
    static uint32_t last_switch_check = 0;
    uint32_t now = millis();

#if defined(MODE_SWITCH_PIN) && MODE_SWITCH_PIN >= 0
    // Poll physical mode switch pin every 100ms
    if (now - last_switch_check >= 100) {
        last_switch_check = now;
        bool hardware_high = (digitalRead(MODE_SWITCH_PIN) == HIGH);
        if (hardware_high != config_low_latency_mode) {
            config_low_latency_mode = hardware_high;
            if (config_low_latency_mode) {
                // Instant flush excess buffer down to 35ms target (6720 B)
                static uint8_t flush_tmp[FRAME_SIZE];
                while (ring_buffer.available() > 6720) {
                    ring_buffer.read(flush_tmp, FRAME_SIZE);
                }
            }
            Serial.printf("[MODE SWITCH] Hardware switch toggled on GPIO %d -> Active Mode: %s (Buffer flushed to 35ms)\n",
                          MODE_SWITCH_PIN, config_low_latency_mode ? "Low Latency (Drift Comp ON)" : "High Quality (Bit-Exact PCM)");
        }
    }
#endif

    // Flush incomplete frame if no sub-packet has arrived for > 15ms
    if (active_seq != 0xFFFFFFFF && received_mask > 0 && received_mask < 0xFF) {
        if (now - last_packet_time > 15) {
            try_fec_recovery();
            if (received_mask < 0xFF) {
                for (int i = 0; i < SUB_PACKETS_PER_FRAME; i++) {
                    if (!(received_mask & (1 << i))) {
                        memset(staging_buffer + (i * AUDIO_PAYLOAD_SIZE), 0, AUDIO_PAYLOAD_SIZE);
                    }
                }
            }
            ring_buffer.write(staging_buffer, FRAME_SIZE);
            stats_frames_received++;
            active_seq = 0xFFFFFFFF;
            received_mask = 0;
            parity_received = false;
        }
    }

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
            uint32_t overflows = stats_buffer_overflows;
            uint32_t fec_rec = stats_fec_recoveries;
            uint32_t silence = stats_silence_frames;
            uint32_t drops = stats_samples_dropped;
            uint32_t dups = stats_samples_duplicated;
            uint32_t real_frames = (frames > fec_rec + silence) ? (frames - fec_rec - silence) : 0;

            // Reset stats counters
            stats_frames_received = 0;
            stats_sub_packets_received = 0;
            stats_underflows = 0;
            stats_buffer_overflows = 0;
            stats_fec_recoveries = 0;
            stats_silence_frames = 0;
            stats_samples_dropped = 0;
            stats_samples_duplicated = 0;

            size_t buffered_bytes = ring_buffer.available();
            float buffer_ms = (float)buffered_bytes / 192.0f; // 48000 Hz * 2 channels * 2 bytes/sample = 192 bytes/ms

            Serial.printf("[LINK STATUS] Mode: %s | Vol: %d%% (%d/20) | Chan: %d | Frames: %lu/s (%lu real, %lu FEC, %lu silence) | Sub-pkts: %lu/s (Exp: ~900) | Buffer: %zu B (%.1f ms) | Drift: %lu drop/s, %lu dup/s | Underflows: %lu/s | Overflows: %lu/s\n",
                          config_low_latency_mode ? "Low-Latency" : "High-Quality",
                          volume_level * 5, volume_level,
                          primary_chan, frames, real_frames, fec_rec, silence, packets, buffered_bytes, buffer_ms, drops, dups, underflows, overflows);
        } else {
            Serial.printf("[IDLE] Waiting for Transmitter... Vol: %d%% (%d/20) | Current Wi-Fi Chan: %d\n", volume_level * 5, volume_level, primary_chan);
        }
    }

    // Hardware Volume Buttons handling (Polling with 30ms debounce, hold auto-repeat, and NVS save)
    static uint32_t up_last_press_time = 0;
    static uint32_t down_last_press_time = 0;
    static bool up_was_pressed = false;
    static bool down_was_pressed = false;
    static uint32_t up_repeat_time = 0;
    static uint32_t down_repeat_time = 0;

    bool up_pressed = (digitalRead(VOLUME_UP_PIN) == LOW);
    bool down_pressed = (digitalRead(VOLUME_DOWN_PIN) == LOW);
    bool volume_changed = false;

    if (up_pressed) {
        if (!up_was_pressed) {
            if (now - up_last_press_time >= 30) {
                up_was_pressed = true;
                up_last_press_time = now;
                up_repeat_time = now + 400; // Hold delay 400ms
                if (volume_level < 20) {
                    volume_level = volume_level + 1;
                    volume_changed = true;
                }
            }
        } else {
            if (now >= up_repeat_time) {
                up_repeat_time = now + 150; // Repeat interval 150ms
                if (volume_level < 20) {
                    volume_level = volume_level + 1;
                    volume_changed = true;
                }
            }
        }
    } else {
        up_was_pressed = false;
    }

    if (down_pressed) {
        if (!down_was_pressed) {
            if (now - down_last_press_time >= 30) {
                down_was_pressed = true;
                down_last_press_time = now;
                down_repeat_time = now + 400; // Hold delay 400ms
                if (volume_level > 0) {
                    volume_level = volume_level - 1;
                    volume_changed = true;
                }
            }
        } else {
            if (now >= down_repeat_time) {
                down_repeat_time = now + 150; // Repeat interval 150ms
                if (volume_level > 0) {
                    volume_level = volume_level - 1;
                    volume_changed = true;
                }
            }
        }
    } else {
        down_was_pressed = false;
    }

    if (volume_changed) {
        volume_dirty = true;
        volume_last_change_time = now;
        Serial.printf("[VOLUME] %s -> Level %d/20 (%d%%) | Gain Q15: %u\n",
                      (up_pressed ? "Up" : "Down"),
                      volume_level,
                      volume_level * 5,
                      VOLUME_Q15_TABLE[volume_level]);
    }

    if (volume_dirty && (now - volume_last_change_time >= 1000)) {
        volume_dirty = false;
        preferences.putUChar("volume", volume_level);
        Serial.printf("[VOLUME] Saved Level %d/20 (%d%%) to NVS.\n", volume_level, volume_level * 5);
    }

    delay(20);
}