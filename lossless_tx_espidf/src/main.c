#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "usb_device_uac.h"

static const char *TAG = "main";

#define LED_GPIO GPIO_NUM_21
#define WATCHDOG_TIMEOUT_US (500 * 1000) // 500 milliseconds

// I2S Pin definitions for Seeed Studio XIAO ESP32S3
#define I2S_BCLK_PIN GPIO_NUM_4 // D3
#define I2S_WS_PIN   GPIO_NUM_5 // D4
#define I2S_DOUT_PIN GPIO_NUM_6 // D5

static esp_timer_handle_t watchdog_timer = NULL;
static bool stream_active = false;
static i2s_chan_handle_t tx_chan = NULL;

// Callback for watchdog timer to turn the LED off when stream is inactive
static void stream_watchdog_cb(void *arg) {
    stream_active = false;
    gpio_set_level(LED_GPIO, 1); // Turn LED OFF (active-low)
    ESP_LOGI(TAG, "Audio stream ended/paused. LED turned OFF.");
}

// Feeds the watchdog and turns the LED on
static void feed_watchdog(void) {
    if (!stream_active) {
        stream_active = true;
        gpio_set_level(LED_GPIO, 0); // Turn LED ON (active-low)
        ESP_LOGI(TAG, "Audio stream started/resumed. LED turned ON.");
    }
    if (watchdog_timer) {
        esp_timer_stop(watchdog_timer);
        esp_timer_start_once(watchdog_timer, WATCHDOG_TIMEOUT_US);
    }
}

// Initialize standard I2S master transmitter
static esp_err_t init_i2s(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;

    ret = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2S standard mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2S initialized successfully (BCLK: GPIO%d, WS: GPIO%d, DOUT: GPIO%d).",
             I2S_BCLK_PIN, I2S_WS_PIN, I2S_DOUT_PIN);
    return ESP_OK;
}

// Callback function for speaker output (data received from USB host)
static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *cb_ctx) {
    feed_watchdog();
    
    // Convert mono 16-bit to stereo 16-bit
    size_t mono_samples_count = len / 2;
    static int16_t stereo_buf[1920]; // 10ms at 48kHz mono is 480 samples. Stereo needs 960 samples (1920 bytes).
    
    size_t bytes_written = 0;
    if (mono_samples_count * 2 <= sizeof(stereo_buf) / sizeof(stereo_buf[0])) {
        int16_t *mono_samples = (int16_t *)buf;
        for (size_t i = 0; i < mono_samples_count; i++) {
            stereo_buf[2 * i] = mono_samples[i];     // Left channel
            stereo_buf[2 * i + 1] = mono_samples[i]; // Right channel
        }
        
        if (tx_chan) {
            i2s_channel_write(tx_chan, stereo_buf, mono_samples_count * 4, &bytes_written, portMAX_DELAY);
        }
    }
    
    // For demonstration, we just log a message periodically
    static int output_cnt = 0;
    if (output_cnt++ % 100 == 0) {
        ESP_LOGI(TAG, "Speaker output: received %d bytes mono, wrote %d bytes stereo to I2S", len, bytes_written);
    }
    return ESP_OK;
}

// Callback function for microphone input (data sent to USB host)
static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *cb_ctx) {
    feed_watchdog();
    
    // For a simple test, we generate silence (zeros)
    memset(buf, 0, len);
    *bytes_read = len;
    
    static int input_cnt = 0;
    if (input_cnt++ % 100 == 0) {
        ESP_LOGI(TAG, "Microphone input: requested %d bytes, writing silence", len);
    }
    return ESP_OK;
}

// Callback function for mute status change
static void uac_device_set_mute_cb(uint32_t mute, void *cb_ctx) {
    ESP_LOGI(TAG, "Mute status changed: %s", mute ? "MUTED" : "UNMUTED");
}

// Callback function for volume change
static void uac_device_set_volume_cb(uint32_t volume, void *cb_ctx) {
    ESP_LOGI(TAG, "Volume level changed: %u", volume);
}

void app_main() {
    ESP_LOGI(TAG, "Initializing LED GPIO...");
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1); // Turn LED OFF initially (active-low)

    ESP_LOGI(TAG, "Initializing stream watchdog timer...");
    const esp_timer_create_args_t timer_args = {
        .callback = &stream_watchdog_cb,
        .name = "stream_watchdog"
    };
    esp_timer_create(&timer_args, &watchdog_timer);

    ESP_LOGI(TAG, "Initializing I2S for PCM5102 DAC...");
    if (init_i2s() != ESP_OK) {
        ESP_LOGE(TAG, "I2S Initialization failed. Audio output will be disabled.");
    }

    ESP_LOGI(TAG, "Initializing USB Audio Class (UAC) device...");
    
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
    
    esp_err_t ret = uac_device_init(&config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "UAC Device initialized successfully.");
    } else {
        ESP_LOGE(TAG, "Failed to initialize UAC Device: %s", esp_err_to_name(ret));
    }
}