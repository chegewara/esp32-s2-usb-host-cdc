
#pragma once
#include "hcd.h"
#include "usb.h"

#define USBH_WEAK_CB __attribute__((weak))

#define EVENT_QUEUE_LEN         10
#define NUM_XFER_REQS           4
#define TRANSFER_DATA_MAX_BYTES       256     //Just assume that will only IN/OUT 64 bytes for now
#define PORT_NUM                1

#define USB_CTRL_REQ_INIT_GET_STRING(ctrl_req_ptr, lang, desc_index, len) ({ \
    (ctrl_req_ptr)->bRequestType = USB_B_REQUEST_TYPE_DIR_IN | USB_B_REQUEST_TYPE_TYPE_STANDARD | USB_B_REQUEST_TYPE_RECIP_DEVICE;   \
    (ctrl_req_ptr)->bRequest = USB_B_REQUEST_GET_DESCRIPTOR;   \
    (ctrl_req_ptr)->wValue = (USB_W_VALUE_DT_STRING << 8) | ((desc_index) & 0xFF); \
    (ctrl_req_ptr)->wIndex = (lang);    \
    (ctrl_req_ptr)->wLength = (len);  \
})

typedef struct {
    hcd_pipe_handle_t pipe_hdl;
    hcd_pipe_event_t pipe_event;
} pipe_event_msg_t;

extern hcd_pipe_handle_t ctrl_pipe_hdl;
extern uint8_t *ctrl_data_buffers[NUM_XFER_REQS];
extern usb_irp_t *ctrl_irps[NUM_XFER_REQS];
typedef void (*ctrl_pipe_cb_t)(pipe_event_msg_t msg, usb_irp_t *irp, void *context);
void register_ctrl_pipe_callback(ctrl_pipe_cb_t);

void ctrl_pipe_event_task(void* p);
void alloc_pipe_and_irp_list(hcd_port_handle_t port_hdl, hcd_pipe_handle_t* handle);
void free_pipe_and_irp_list(hcd_pipe_handle_t pipe_hdl);

USBH_WEAK_CB void usbh_get_device_desc_cb(uint8_t*, size_t, void*);
USBH_WEAK_CB void usbh_get_config_desc_cb(uint8_t*, size_t, void*);
USBH_WEAK_CB void usbh_get_string_cb(uint8_t*, size_t, void*);
USBH_WEAK_CB void usbh_get_interface_desc_cb(uint8_t*, size_t, void*);
USBH_WEAK_CB void usbh_get_endpoint_desc_cb(uint8_t*, size_t, void*);
USBH_WEAK_CB void usbh_get_power_desc_cb(uint8_t*, size_t, void*);

USBH_WEAK_CB void usbh_set_address_cb(uint16_t, void*);
USBH_WEAK_CB void usbh_set_config_desc_cb(uint16_t, void*);
USBH_WEAK_CB void usbh_get_configuration_cb(uint8_t, void*);

USBH_WEAK_CB void usbh_ctrl_pipe_error_cb(usb_ctrl_req_t* ctrl);
USBH_WEAK_CB void usbh_ctrl_pipe_stalled_cb(usb_ctrl_req_t* ctrl);
USBH_WEAK_CB void usbh_ctrl_pipe_class_specific_cb(pipe_event_msg_t msg, usb_irp_t *irp);

void xfer_get_device_desc(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle);
void xfer_set_address(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle, uint8_t addr);
void xfer_get_current_config(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle);
void xfer_set_configuration(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle, uint8_t);
void xfer_get_desc(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle);
void xfer_get_string(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle, uint8_t);
void allocate_ctrl_pipe(hcd_port_handle_t port_hdl, hcd_pipe_handle_t* handle);
usb_irp_t* allocate_irp(hcd_port_handle_t port_hdl, size_t size);
