#pragma once
#include "hcd.h"
#include "ctrl_pipe.h"
#include "usb_host_port.h"

// #define XFER_DATA_MAX_LEN       1023     //Just assume that will only IN/OUT 1023 bytes for now, which is max length for full speed
#define DEVICE_ADDR             1

#define USB_DESC_EP_GET_ADDRESS(desc_ptr) ((desc_ptr)->bEndpointAddress & 0x7F)

#define SET_VALUE       0x21
#define GET_VALUE       0xA1

// DTR/RTS control in SET_CONTROL_LINE_STATE
#define ENABLE_DTR(val)      (val<<0)
#define ENABLE_RTS(val)      (val<<1)

#define SET_LINE_CODING 0x20
#define GET_LINE_CODING 0x21
#define SET_CONTROL_LINE_STATE 0x22
#define SERIAL_STATE    0x20

typedef struct{
    uint32_t dwDTERate;
    uint8_t bCharFormat;
    uint8_t bParityType;
    uint8_t bDataBits;
}line_coding_t;

typedef union {
    struct {
        uint8_t bRequestType;
        uint8_t bRequest;
        uint16_t wValue;
        uint16_t wIndex;
        uint16_t wLength;
        line_coding_t data;
    } USB_CTRL_REQ_ATTR;
    uint8_t val[USB_CTRL_REQ_SIZE + 7];
} cdc_ctrl_line_t;

/**
 * @brief Initializer for a SET_ADDRESS request
 *
 * Sets the address of a connected device
 */
#define USB_CTRL_REQ_CDC_SET_LINE_CODING(ctrl_req_ptr, index, bitrate, cf, parity, bits) ({  \
    (ctrl_req_ptr)->bRequestType = SET_VALUE;   \
    (ctrl_req_ptr)->bRequest = SET_LINE_CODING;  \
    (ctrl_req_ptr)->wValue = 0;   \
    (ctrl_req_ptr)->wIndex = (index);    \
    (ctrl_req_ptr)->wLength = (7);   \
    (ctrl_req_ptr)->data.dwDTERate = (bitrate);   \
    (ctrl_req_ptr)->data.bCharFormat = (cf);   \
    (ctrl_req_ptr)->data.bParityType = (parity);   \
    (ctrl_req_ptr)->data.bDataBits = (bits);   \
})

#define USB_CTRL_REQ_CDC_GET_LINE_CODING(ctrl_req_ptr, index) ({  \
    (ctrl_req_ptr)->bRequestType = GET_VALUE;   \
    (ctrl_req_ptr)->bRequest = GET_LINE_CODING;  \
    (ctrl_req_ptr)->wValue = 0;   \
    (ctrl_req_ptr)->wIndex = (index);    \
    (ctrl_req_ptr)->wLength = (7);   \
})

#define USB_CTRL_REQ_CDC_SET_CONTROL_LINE_STATE(ctrl_req_ptr, index, dtr, rts) ({  \
    (ctrl_req_ptr)->bRequestType = SET_VALUE;   \
    (ctrl_req_ptr)->bRequest = SET_CONTROL_LINE_STATE;  \
    (ctrl_req_ptr)->wValue = ENABLE_DTR(dtr) | ENABLE_RTS(rts);   \
    (ctrl_req_ptr)->wIndex = (index);    \
    (ctrl_req_ptr)->wLength = (0);   \
})

void xfer_set_line_coding(uint32_t bitrate, uint8_t cf, uint8_t parity, uint8_t bits);
void xfer_set_control_line(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle, bool dtr, bool rts);
void xfer_get_line_coding();
void xfer_intr_data();
void xfer_in_data();
void xfer_out_data(uint8_t* data, size_t len);
void delete_pipes();
void cdc_create_pipe(usb_desc_ep_t* ep);
void register_cdc_pipe_callback(ctrl_pipe_cb_t cb);
void cdc_class_specific_ctrl_cb(usb_irp_t* irp);
