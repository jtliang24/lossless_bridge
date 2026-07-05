#pragma once
#include <Arduino.h>
#include "device/usbd_pvt.h"
#include "tusb.h"

// Define the missing tu_desc_subtype macro
#ifndef tu_desc_subtype
#define tu_desc_subtype(desc) (((const uint8_t*)(desc))[2])
#endif

// Define the missing SOF_CONSUMER_AUDIO
#ifndef SOF_CONSUMER_AUDIO
#define SOF_CONSUMER_AUDIO 0
#endif

// Redirect usbd_sof_enable calls to do nothing, bypassing linker issues
#define usbd_sof_enable(rhport, consumer, en) ((void)0)

// Redirect usbd_app_driver_get_cb to avoid conflicting return type definition signature
#define usbd_app_driver_get_cb usbd_app_driver_get_cb_unused

// Compatibility struct to allow USBAudioDeviceBase.h to compile with name/deinit
struct usbd_class_driver_compat_t {
  const char* name;
  void (*init)(void);
  void (*reset)(uint8_t);
  uint16_t (*open)(uint8_t, tusb_desc_interface_t const* itf_desc, uint16_t max_len);
  bool (*control_xfer_cb)(uint8_t, uint8_t, tusb_control_request_t const* request);
  bool (*xfer_cb)(uint8_t, uint8_t, xfer_result_t result, uint32_t xferred_bytes);
  void (*sof)(uint8_t, uint32_t);
  bool (*deinit)(void); 
};
#define usbd_class_driver_t usbd_class_driver_compat_t
