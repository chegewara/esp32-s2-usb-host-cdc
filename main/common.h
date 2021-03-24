
#pragma once
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_pins.h"
#include "soc/gpio_sig_map.h"
#include "hal/usbh_ll.h"
#include "hcd.h"
#include "esp_log.h"

#define USB_WEAK        __attribute__((weak))




USB_WEAK void class_specific_data_cb(usb_irp_t* irp);
USB_WEAK void ep_data_cb(usb_irp_t* irp);




