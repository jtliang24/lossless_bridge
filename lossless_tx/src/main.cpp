#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "USB.h"
#include "esp32-hal-tinyusb.h"
#include "class/audio/audio_device.h"

// Hardware Mailbox: Your validated Receiver MAC Address
uint8_t receiverAddress[] = {0xE0, 0x72, 0xA1, 0xFA, 0x58, 0x84};

// Pin the radio transmission to Wi-Fi Channel 1
const uint8_t WIFI_CHANNEL = 1;

// Structural container for sending packet slices over the air
struct __attribute__((packed)) AudioPacket {
    uint32_t packetId;
    uint8_t sampleData[240]; // 240 bytes matches standard PCM framing
};

AudioPacket txPacket;
uint32_t globalPacketCounter = 0;

#if CORE_DEBUG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
static const char* TAG = "UAC_TX";
#endif

// Ring Buffer for audio streaming
const int RING_BUFFER_SIZE = 8192; // 8KB ring buffer
uint8_t ringBuffer[RING_BUFFER_SIZE];
volatile int ringBufHead = 0;
volatile int ringBufTail = 0;

void writeToRingBuffer(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        int nextHead = (ringBufHead + 1) % RING_BUFFER_SIZE;
        if (nextHead != ringBufTail) {
            ringBuffer[ringBufHead] = data[i];
            ringBufHead = nextHead;
        } else {
            // Buffer overflow - drop byte to keep things real-time
        }
    }
}

int getRingBufferAvailable() {
    return (ringBufHead - ringBufTail + RING_BUFFER_SIZE) % RING_BUFFER_SIZE;
}

bool readFromRingBuffer(uint8_t* outData, int len) {
    if (getRingBufferAvailable() < len) {
        return false;
    }
    for (int i = 0; i < len; i++) {
        outData[i] = ringBuffer[ringBufTail];
        ringBufTail = (ringBufTail + 1) % RING_BUFFER_SIZE;
    }
    return true;
}

// UAC2 Endpoint definition
#define EPNUM_AUDIO_STREAMING_OUT 0x01

// 4-byte range struct with 1 subrange for sample rate reporting
typedef struct TU_ATTR_PACKED {
    uint16_t wNumSubRanges;
    struct TU_ATTR_PACKED {
        int32_t bMin;
        int32_t bMax;
        uint32_t bRes;
    } subrange[1];
} audio_control_range_4_1_t;

// Custom descriptor generator for USB Audio Class 2.0 (UAC2) Stereo Speaker
uint16_t load_custom_descriptor(uint8_t *dst, uint8_t *itf) {
    uint8_t control_itf = *itf;
    uint8_t streaming_itf = *itf + 1;
    *itf += 2;
    
    uint8_t desc[] = {
        // 1. IAD Descriptor (Interface Association Descriptor)
        TUD_AUDIO_DESC_IAD(control_itf, 2, 0),
        
        // 2. Audio Control Standard Interface
        TUD_AUDIO_DESC_STD_AC(control_itf, 0, 0),
        
        // 3. Audio Control Class-Specific Header
        TUD_AUDIO_DESC_CS_AC(0x0200, AUDIO_FUNC_DESKTOP_SPEAKER, 
                             TUD_AUDIO_DESC_CLK_SRC_LEN + TUD_AUDIO_DESC_INPUT_TERM_LEN + 
                             TUD_AUDIO_DESC_OUTPUT_TERM_LEN + TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL_LEN, 
                             AUDIO_CS_AS_INTERFACE_CTRL_LATENCY_POS),
                             
        // 4. Clock Source Descriptor
        TUD_AUDIO_DESC_CLK_SRC(0x04, AUDIO_CLOCK_SOURCE_ATT_INT_FIX_CLK, 
                               (AUDIO_CTRL_R << AUDIO_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), 
                               0x01, 0x00),
                               
        // 5. Input Terminal Descriptor (USB Streaming Input)
        TUD_AUDIO_DESC_INPUT_TERM(0x01, AUDIO_TERM_TYPE_USB_STREAMING, 0x00, 0x04, 
                                  0x02, // 2 channels
                                  0x00000003, // Front Left + Front Right
                                  0x00, 
                                  0x0000, 0x00),
                                  
        // 6. Output Terminal Descriptor (Speaker output)
        TUD_AUDIO_DESC_OUTPUT_TERM(0x03, AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER, 0x00, 0x02, 0x04, 0x0000, 0x00),
        
        // 7. Feature Unit Descriptor (Mute & Volume disabled to let OS handle volume digitally)
        TUD_AUDIO_DESC_FEATURE_UNIT_TWO_CHANNEL(0x02, 0x01, 0, 0, 0, 0),
                                                
        // 8. Standard AS Interface (Alt 0 - zero bandwidth interface)
        TUD_AUDIO_DESC_STD_AS_INT(streaming_itf, 0x00, 0x00, 0x00),
        
        // 9. Standard AS Interface (Alt 1 - streaming interface)
        TUD_AUDIO_DESC_STD_AS_INT(streaming_itf, 0x01, 0x01, 0x00),
        
        // 10. Class-Specific AS Interface
        TUD_AUDIO_DESC_CS_AS_INT(0x01, AUDIO_CTRL_NONE, AUDIO_FORMAT_TYPE_I, AUDIO_DATA_FORMAT_TYPE_I_PCM, 
                                 0x02, // 2 channels
                                 0x00000003, // Front Left + Front Right
                                 0x00),
                                 
        // 11. Format Type (Type I format)
        TUD_AUDIO_DESC_TYPE_I_FORMAT(2, 16), // 2 bytes/sample, 16 bits
        
        // 12. Standard AS Isochronous Endpoint Descriptor (OUT)
        TUD_AUDIO_DESC_STD_AS_ISO_EP(EPNUM_AUDIO_STREAMING_OUT, 
                                     TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA, 
                                     512, 1),
                                     
        // 13. Class-Specific AS Isochronous Endpoint Descriptor
        TUD_AUDIO_DESC_CS_AS_ISO_EP(AUDIO_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, 
                                    AUDIO_CTRL_NONE, 
                                    AUDIO_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 
                                    0x0000)
    };
    
    memcpy(dst, desc, sizeof(desc));
    return sizeof(desc);
}

// UAC2 Entity Control Request Callbacks
extern "C" bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;
    
    if (request->bEntityID == 0x04) { // Clock Source ID
        if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
            if (request->bRequest == AUDIO_CS_REQ_CUR) {
                uint32_t rate = 44100;
                return tud_control_xfer(rhport, p_request, &rate, sizeof(rate));
            } else if (request->bRequest == AUDIO_CS_REQ_RANGE) {
                audio_control_range_4_1_t range = {
                    .wNumSubRanges = 1,
                    .subrange = {
                        {
                            .bMin = 44100,
                            .bMax = 44100,
                            .bRes = 0
                        }
                    }
                };
                return tud_control_xfer(rhport, p_request, &range, sizeof(range));
            }
        } else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID) {
            if (request->bRequest == AUDIO_CS_REQ_CUR) {
                uint8_t valid = 1;
                return tud_control_xfer(rhport, p_request, &valid, sizeof(valid));
            }
        }
    }
    return false;
}

extern "C" bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff) {
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;
    (void)pBuff;
    
    if (request->bEntityID == 0x04) { // Clock Source ID
        if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
            if (request->bRequest == AUDIO_CS_REQ_CUR) {
                return true;
            }
        }
    }
    return false;
}

extern "C" bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
    (void)rhport;
    (void)p_request;
    return true;
}

extern "C" bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
    (void)rhport;
    (void)p_request;
    return true;
}

// Data streaming callback: Invoked when a packet is received on the OUT endpoint
extern "C" bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
    (void)rhport;
    (void)func_id;
    (void)ep_out;
    (void)cur_alt_setting;
    
    uint8_t tmp_buf[512];
    if (n_bytes_received > sizeof(tmp_buf)) {
        n_bytes_received = sizeof(tmp_buf);
    }
    
    uint16_t read_bytes = tud_audio_read(tmp_buf, n_bytes_received);
    if (read_bytes > 0) {
        writeToRingBuffer(tmp_buf, read_bytes);
    }
    return true;
}

void setup() {
    Serial.begin(921600); 
    delay(2000);
    
    // 1. Initialize Wi-Fi in Station Mode and lock the radio channel
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    
    // 2. Initialize the ESP-NOW networking engine
    if (esp_now_init() != ESP_OK) {
        return;
    }
    
    // 3. Register your hardware receiver peer configuration
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, receiverAddress, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        return;
    }

    // Configure the custom UAC2 Speaker interface
    USB.VID(0x303A); // Espressif VID
    USB.PID(0x0002); // Standard composite PID
    USB.productName("Seeed Xiao ESP32S3 Headphone");
    USB.manufacturerName("Lossless Bridge");

    // Enable custom interface to load our configuration descriptor
    tinyusb_enable_interface(USB_INTERFACE_CUSTOM, 136, load_custom_descriptor);
    USB.begin();
}

void loop() {
    // Check if we have at least 240 bytes of audio data in the ring buffer
    if (getRingBufferAvailable() >= 240) {
        if (readFromRingBuffer(txPacket.sampleData, 240)) {
            // Stamp it with a sequential identifier
            txPacket.packetId = globalPacketCounter++;
            
            // Blast the raw uncompressed PCM block instantly across the link
            esp_now_send(receiverAddress, (uint8_t *) &txPacket, sizeof(txPacket));
        }
    }
    
    // Tiny delay to keep task yielding friendly
    delay(1);
}