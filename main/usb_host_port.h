#pragma once
#include "hcd.h"

#define DEVICE_ADDR             1

typedef struct {
    hcd_port_handle_t port_hdl;
    hcd_port_event_t port_event;
} port_event_msg_t;

typedef void (*port_evt_cb_t)(port_event_msg_t msg);

hcd_port_handle_t port_hdl;

bool setup_usb_host();
void register_port_callback(port_evt_cb_t cb);
