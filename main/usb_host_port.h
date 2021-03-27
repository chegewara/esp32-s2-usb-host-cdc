#pragma once
#include "hcd.h"

#define USBH_WEAK_CB __attribute__((weak))

typedef struct {
    hcd_port_handle_t port_hdl;
    hcd_port_event_t port_event;
} port_event_msg_t;

typedef void (*port_evt_cb_t)(port_event_msg_t msg);

hcd_port_handle_t port_hdl;

bool setup_usb_host();
void register_port_callback(port_evt_cb_t cb);

USBH_WEAK_CB void usbh_port_connection_cb(port_event_msg_t);
USBH_WEAK_CB void usbh_port_disconnection_cb(port_event_msg_t);
USBH_WEAK_CB void usbh_port_error_cb(port_event_msg_t);
USBH_WEAK_CB void usbh_port_overcurrent_cb(port_event_msg_t);
USBH_WEAK_CB void usbh_port_sudden_disconn_cb(port_event_msg_t);
