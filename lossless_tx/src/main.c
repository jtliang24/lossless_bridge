#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "usb_device_uac.h"

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

static esp_timer_handle_t watchdog_timer = NULL;
static bool stream_active = false;

static uint8_t receiver_mac[6];
static bool peer_paired = false;

// Blink LED when searching/paired
static void feed_watchdog(void) {
    if (!stream_active) {
        stream_active = true;
        gpio_set_level(LED_GPIO, 0); // Turn LED ON (active-low)
        ESP_LOGI(TAG, "Audio stream started. LED ON.");
    }
    if (watchdog_timer) {
        esp_timer_stop(watchdog_timer);
        esp_timer_start_once(watchdog_timer, WATCHDOG_TIMEOUT_US);
    }
}

static void stream_watchdog_cb(void *arg) {
    stream_active = false;
    gpio_set_level(LED_GPIO, 1); // Turn LED OFF (active-low)
    ESP_LOGI(TAG, "Audio stream ended/paused. LED OFF.");
}

// Receive callback for pairing beacons
static void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == sizeof(beacon_packet_t)) {
        const beacon_packet_t *beacon = (const beacon_packet_t *)data;
        if (memcmp(beacon->magic, "LR_BEACN", 8) == 0) {
            if (!peer_paired || memcmp(receiver_mac, recv_info->src_addr, 6) != 0) {
                memcpy(receiver_mac, recv_info->src_addr, 6);
                
                // Register peer
                esp_now_peer_info_t peer;
                memset(&peer, 0, sizeof(peer));
                memcpy(peer.peer_addr, receiver_mac, 6);
                peer.channel = 1;
                peer.encrypt = false;
                
                if (esp_now_is_peer_exist(receiver_mac)) {
                    esp_now_del_peer(receiver_mac);
                }
                
                esp_err_t err = esp_now_add_peer(&peer);
                if (err == ESP_OK) {
                    peer_paired = true;
                    ESP_LOGI(TAG, "Successfully paired with Receiver: %02x:%02x:%02x:%02x:%02x:%02x",
                             receiver_mac[0], receiver_mac[1], receiver_mac[2],
                             receiver_mac[3], receiver_mac[4], receiver_mac[5]);
                } else {
                    ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(err));
                }
            }
        }
    }
}

// Callback for UAC audio data from PC host
static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *cb_ctx) {
    feed_watchdog();

    if (len != FRAME_SIZE) {
        return ESP_OK; // Ignore incorrect frame sizes
    }

    if (!peer_paired) {
        return ESP_OK; // Discard audio if not paired yet
    }

    static uint32_t seq = 0;
    static audio_packet_t pkt;
    pkt.seq_num = seq++;
    pkt.total_sub_packets = SUB_PACKETS_PER_FRAME;
    pkt.payload_len = AUDIO_PAYLOAD_SIZE;

    // Send the 8 sub-packets back-to-back
    for (uint8_t i = 0; i < SUB_PACKETS_PER_FRAME; i++) {
        pkt.sub_packet_idx = i;
        memcpy(pkt.audio_data, buf + (i * AUDIO_PAYLOAD_SIZE), AUDIO_PAYLOAD_SIZE);
        
        esp_err_t err = esp_now_send(receiver_mac, (uint8_t *)&pkt, sizeof(audio_packet_t));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ESP-NOW send failed: %s", esp_err_to_name(err));
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

    // 4. Initialize Wi-Fi in Station Mode on Channel 1
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Disable power saving for low latency

    // 5. Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));

    // 6. Initialize TinyUSB UAC Device
    uac_device_config_t config = {
        .output_cb = uac_device_output_cb,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
        .cb_ctx = NULL,
    };
    ESP_ERROR_CHECK(uac_device_init(&config));
    
    ESP_LOGI(TAG, "Transmitter initialized. Waiting for Receiver beacon...");
}
