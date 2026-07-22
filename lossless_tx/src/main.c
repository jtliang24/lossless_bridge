#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "usb_device_uac.h"
#include "esp_rom_sys.h"
#include "class/hid/hid_device.h"

static const char *TAG = "tx_main";

#define LED_GPIO GPIO_NUM_21
#define WATCHDOG_TIMEOUT_US (500 * 1000) // 500 milliseconds

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

typedef struct __attribute__((packed)) {
    char magic[8];             // "MEDIA_CTL"
    uint8_t command;           // 1 = Play/Pause, 2 = Next Track, 3 = Prev Track
} media_control_packet_t;

static void send_hid_media_key(uint16_t key_code) {
    if (tud_hid_ready()) {
        tud_hid_report(1, &key_code, sizeof(key_code));
        vTaskDelay(pdMS_TO_TICKS(20));
        uint16_t zero = 0;
        tud_hid_report(1, &zero, sizeof(zero));
    }
}


static esp_timer_handle_t watchdog_timer = NULL;
static bool stream_active = false;

static uint8_t receiver_mac[6];
static bool peer_paired = false;

// Blink LED when searching/paired
static void feed_watchdog(void) {
    if (!stream_active) {
        stream_active = true;
        ESP_LOGI(TAG, "Audio stream started.");
    }
    if (watchdog_timer) {
        esp_timer_stop(watchdog_timer);
        esp_timer_start_once(watchdog_timer, WATCHDOG_TIMEOUT_US);
    }
}

static void stream_watchdog_cb(void *arg) {
    stream_active = false;
    ESP_LOGI(TAG, "Audio stream ended/paused.");
}

static volatile uint32_t tx_fail_count = 0;
static volatile uint32_t tx_success_count = 0;

static void esp_now_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_FAIL) {
        tx_fail_count++;
    } else {
        tx_success_count++;
    }
}

static void status_led_task(void *pvParameters) {
    while (1) {
        if (!peer_paired) {
            // Blinking slowly (500ms ON, 500ms OFF) - Searching for receiver
            gpio_set_level(LED_GPIO, 0); // ON (active-low)
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_GPIO, 1); // OFF
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (stream_active) {
            // Check if we are experiencing a high rate of ESP-NOW transmit failures
            if (tx_fail_count > 50) {
                // High failure rate - blink twice then pause (visual alert)
                gpio_set_level(LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(500));
                tx_fail_count = 0; // Reset counter
            } else {
                // Blinking rapidly (100ms ON, 100ms OFF) - Streaming active
                gpio_set_level(LED_GPIO, 0); // ON
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(LED_GPIO, 1); // OFF
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        } else {
            // Solid ON - Paired but idle (active-low ON is 0)
            gpio_set_level(LED_GPIO, 0); // ON
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
}

static void periodic_test_task(void *pvParameters) {
    uint32_t seq = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (peer_paired) {
            tx_confirm_packet_t test_pkt;
            memcpy(test_pkt.magic, "TEST_UNI", 8);
            esp_read_mac(test_pkt.transmitter_mac, ESP_MAC_WIFI_STA);
            memcpy(test_pkt.receiver_mac, receiver_mac, 6);
            test_pkt.seq_num = seq++;
            
            esp_err_t err = esp_now_send(receiver_mac, (uint8_t *)&test_pkt, sizeof(tx_confirm_packet_t));
            if (err != ESP_OK) {
                ESP_LOGE("test_tx", "Periodic unicast send failed: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI("test_tx", "Periodic unicast send success to %02X:%02X:%02X:%02X:%02X:%02X",
                         receiver_mac[0], receiver_mac[1], receiver_mac[2],
                         receiver_mac[3], receiver_mac[4], receiver_mac[5]);
            }
        }
    }
}

// Receive callback for pairing beacons and media control commands
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(media_control_packet_t)) {
        const media_control_packet_t *media_pkt = (const media_control_packet_t *)data;
        if (memcmp(media_pkt->magic, "MEDIA_CTL", 8) == 0) {
            uint16_t key_code = 0;
            const char *cmd_str = "UNKNOWN";
            if (media_pkt->command == 1) {
                key_code = 0x00CD; // Play / Pause
                cmd_str = "Play/Pause";
            } else if (media_pkt->command == 2) {
                key_code = 0x00B5; // Next Track (Skip)
                cmd_str = "Next Track";
            } else if (media_pkt->command == 3) {
                key_code = 0x00B6; // Prev Track (Back)
                cmd_str = "Prev Track";
            }
            if (key_code != 0) {
                ESP_LOGI(TAG, "[MEDIA] Received command: %s (0x%04X)", cmd_str, key_code);
                send_hid_media_key(key_code);
            }
            return;
        }
    }

    if (len == sizeof(beacon_packet_t)) {
        const beacon_packet_t *beacon = (const beacon_packet_t *)data;
        if (memcmp(beacon->magic, "LR_BEACN", 8) == 0) {
            if (!peer_paired || memcmp(receiver_mac, recv_info->src_addr, 6) != 0) {
                memcpy(receiver_mac, recv_info->src_addr, 6);
                
                // Register peer
                esp_now_peer_info_t peer;
                memset(&peer, 0, sizeof(peer));
                memcpy(peer.peer_addr, receiver_mac, 6);
                peer.channel = 8;
                peer.ifidx = WIFI_IF_STA;
                peer.encrypt = false;
                
                if (esp_now_is_peer_exist(receiver_mac)) {
                    esp_now_del_peer(receiver_mac);
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
                    esp_now_set_peer_rate_config(receiver_mac, &rate_cfg);

                    peer_paired = true;
                    ESP_LOGI(TAG, "Successfully paired with Receiver: %02x:%02x:%02x:%02x:%02x:%02x",
                             receiver_mac[0], receiver_mac[1], receiver_mac[2],
                             receiver_mac[3], receiver_mac[4], receiver_mac[5]);
                } else {
                    ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(err));
                }
            }
            
            // Send broadcast confirmation packet so the receiver can register us as a peer!
            tx_confirm_packet_t conf;
            memcpy(conf.magic, "TX_CONF", 8);
            esp_read_mac(conf.transmitter_mac, ESP_MAC_WIFI_STA);
            memcpy(conf.receiver_mac, receiver_mac, 6);
            conf.seq_num = 0;
            
            uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            esp_now_send(broadcast_mac, (uint8_t*)&conf, sizeof(tx_confirm_packet_t));
        }
    }
}


// Callback for UAC audio data from PC host
// NOTE: The UAC library calls this with ~192 bytes per invocation (1ms of audio
// at 48kHz/stereo/16-bit). We send each 240-byte sub-packet as soon as it's
// accumulated, spreading ESP-NOW sends to ~1 packet per 1.25ms instead of
// bursting 8 packets every 10ms (which overflows ESP-NOW's internal queue).
static uint8_t sub_pkt_buf[AUDIO_PAYLOAD_SIZE];
static uint8_t parity_buf[AUDIO_PAYLOAD_SIZE];
static size_t  sub_pkt_offset = 0;
static uint32_t frame_seq = 0;
static uint8_t  sub_pkt_idx = 0;

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *cb_ctx) {
    feed_watchdog();

    if (!peer_paired) {
        return ESP_OK; // Discard audio if not paired yet
    }

    size_t remaining = len;
    const uint8_t *src = buf;

    while (remaining > 0) {
        size_t space = AUDIO_PAYLOAD_SIZE - sub_pkt_offset;
        size_t to_copy = (remaining < space) ? remaining : space;
        memcpy(sub_pkt_buf + sub_pkt_offset, src, to_copy);
        sub_pkt_offset += to_copy;
        src += to_copy;
        remaining -= to_copy;

        // Send immediately when we have one full sub-packet (240 bytes)
        if (sub_pkt_offset >= AUDIO_PAYLOAD_SIZE) {
            static audio_packet_t pkt;
            pkt.seq_num = frame_seq;
            pkt.sub_packet_idx = sub_pkt_idx;
            pkt.total_sub_packets = SUB_PACKETS_PER_FRAME + 1; // 9 sub-packets (8 audio + 1 FEC)
            pkt.payload_len = AUDIO_PAYLOAD_SIZE;
            memcpy(pkt.audio_data, sub_pkt_buf, AUDIO_PAYLOAD_SIZE);

            // Accumulate parity
            if (sub_pkt_idx == 0) {
                memcpy(parity_buf, sub_pkt_buf, AUDIO_PAYLOAD_SIZE);
            } else {
                for (size_t i = 0; i < AUDIO_PAYLOAD_SIZE; i++) {
                    parity_buf[i] ^= sub_pkt_buf[i];
                }
            }

            esp_err_t err;
            int retries = 5;
            do {
                err = esp_now_send(receiver_mac, (uint8_t *)&pkt, sizeof(audio_packet_t));
                if (err == ESP_ERR_ESPNOW_NO_MEM) {
                    esp_rom_delay_us(100);
                } else {
                    break;
                }
            } while (--retries > 0);

            if (err != ESP_OK) {
                tx_fail_count++;
            }

            sub_pkt_offset = 0;
            sub_pkt_idx++;

            // After sending 8 audio sub-packets, transmit 9th FEC parity sub-packet (index 8)
            if (sub_pkt_idx >= SUB_PACKETS_PER_FRAME) {
                static audio_packet_t fec_pkt;
                fec_pkt.seq_num = frame_seq;
                fec_pkt.sub_packet_idx = SUB_PACKETS_PER_FRAME; // index 8
                fec_pkt.total_sub_packets = SUB_PACKETS_PER_FRAME + 1;
                fec_pkt.payload_len = AUDIO_PAYLOAD_SIZE;
                memcpy(fec_pkt.audio_data, parity_buf, AUDIO_PAYLOAD_SIZE);

                retries = 5;
                do {
                    err = esp_now_send(receiver_mac, (uint8_t *)&fec_pkt, sizeof(audio_packet_t));
                    if (err == ESP_ERR_ESPNOW_NO_MEM) {
                        esp_rom_delay_us(100);
                    } else {
                        break;
                    }
                } while (--retries > 0);

                if (err != ESP_OK) {
                    tx_fail_count++;
                }

                sub_pkt_idx = 0;
                frame_seq++;
            }
        }
    }

    return ESP_OK;
}

static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *cb_ctx) {
    memset(buf, 0, len);
    *bytes_read = len;
    return ESP_OK;
}

static void uac_device_set_mute_cb(uint32_t mute, void *cb_ctx) {
    ESP_LOGI(TAG, "Mute: %s", mute ? "MUTED" : "UNMUTED");
}

static void uac_device_set_volume_cb(uint32_t volume, void *cb_ctx) {
    ESP_LOGI(TAG, "Volume: %lu", volume);
}

void app_main() {
    // 1. Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize GPIO & Status LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1); // LED OFF (active-low)

    // 3. Setup Watchdog Timer
    const esp_timer_create_args_t timer_args = {
        .callback = &stream_watchdog_cb,
        .name = "stream_watchdog"
    };
    esp_timer_create(&timer_args, &watchdog_timer);

    // 4. Initialize Wi-Fi in APSTA Mode on Channel 1
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Lock the radio to Channel 8 using the promiscuous mode workaround in STA mode
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(8, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(false));
    
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Disable power saving for low latency
    esp_wifi_set_max_tx_power(84); // 21 dBm max TX power for range


    // Create the Status LED task
    xTaskCreate(status_led_task, "status_led_task", 2048, NULL, 2, NULL);
    xTaskCreate(periodic_test_task, "periodic_test_task", 3072, NULL, 3, NULL);

    // 5. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));

    // Add Broadcast Peer on Transmitter (WIFI_IF_STA)
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t broadcast_peer;
    memset(&broadcast_peer, 0, sizeof(broadcast_peer));
    memcpy(broadcast_peer.peer_addr, broadcast_mac, 6);
    broadcast_peer.channel = 8;
    broadcast_peer.ifidx = WIFI_IF_STA;
    broadcast_peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&broadcast_peer));

    // 6. Initialize TinyUSB UAC Device
    uac_device_config_t config = {
        .output_cb = uac_device_output_cb,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
        .cb_ctx = NULL,
#if CONFIG_USB_DEVICE_UAC_AS_PART
        .spk_itf_num = 1,
        .mic_itf_num = 2,
#endif
    };
    ESP_ERROR_CHECK(uac_device_init(&config));
    
    ESP_LOGI(TAG, "Transmitter initialized. Waiting for Receiver beacon...");
}
