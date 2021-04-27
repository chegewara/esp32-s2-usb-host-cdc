
#include <stdio.h>
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
#include "hcd.h"
#include "ctrl_pipe.h"

hcd_pipe_handle_t ctrl_pipe_hdl;
uint8_t *ctrl_data_buffers[NUM_XFER_REQS];
usb_irp_t *ctrl_irps[NUM_XFER_REQS];

static QueueHandle_t ctrl_pipe_evt_queue;
static ctrl_pipe_cb_t ctrl_pipe_cb;
uint8_t bMaxPacketSize0;

void register_ctrl_pipe_callback(ctrl_pipe_cb_t cb)
{
    ctrl_pipe_cb = cb;
}

static bool ctrl_pipe_callback(hcd_pipe_handle_t pipe_hdl, hcd_pipe_event_t pipe_event, void *user_arg, bool in_isr)
{
    QueueHandle_t pipe_evt_queue = (QueueHandle_t)user_arg;
    pipe_event_msg_t msg = {
        .pipe_hdl = pipe_hdl,
        .pipe_event = pipe_event,
    };
    if (in_isr) {
        BaseType_t xTaskWoken = pdFALSE;
        xQueueSendFromISR(pipe_evt_queue, &msg, &xTaskWoken);
        return (xTaskWoken == pdTRUE);
    } else {
        xQueueSend(pipe_evt_queue, &msg, portMAX_DELAY);
        return false;
    }
}

void free_pipe_and_irp_list(hcd_pipe_handle_t pipe_hdl)
{
    printf("Freeing IRPs and IRP list\n");
    //Dequeue transfer requests
    do{
        usb_irp_t *irp = hcd_irp_dequeue(pipe_hdl);
        if(irp == NULL) break;
    }while(1);

    ESP_LOGD("", "Freeing transfer requets\n");
    //Free transfer requests (and their associated objects such as IRPs and data buffers)
    for (int i = 0; i < NUM_XFER_REQS; i++) {
        heap_caps_free(ctrl_irps[i]);
        heap_caps_free(ctrl_data_buffers[i]);
    }
    ESP_LOGD("", "Freeing pipe\n");
    //Delete the pipe
    if(ESP_OK != hcd_pipe_free(pipe_hdl)) {
        ESP_LOGE("", "err to free pipes");
    }
}

void alloc_pipe_and_irp_list(hcd_port_handle_t port_hdl, hcd_pipe_handle_t* handle)
{
    //We don't support hubs yet. Just get the speed of the port to determine the speed of the device
    usb_speed_t port_speed;
    if(ESP_OK == hcd_port_get_speed(port_hdl, &port_speed)){}

    //Create default pipe
    // printf("Creating default pipe\n");
    hcd_pipe_config_t config = {
        .callback = ctrl_pipe_callback,
        .callback_arg = (void *)ctrl_pipe_evt_queue,
        .context = NULL,
        .ep_desc = NULL,    //NULL EP descriptor to create a default pipe
        .dev_addr = 0,
        .dev_speed = port_speed,
    };
    if(ESP_OK != hcd_pipe_alloc(port_hdl, &config, &ctrl_pipe_hdl)) ESP_LOGE("", "cant alloc pipe");
    if(NULL == ctrl_pipe_hdl) {
        ESP_LOGE("", "NULL == pipe_hdl");
        return;
    }
    //Create IRPs and their data buffers
    printf("Creating IRPs and IRP list\n");
    for (int i = 0; i < NUM_XFER_REQS; i++) {
        ctrl_irps[i] = heap_caps_calloc(1, sizeof(usb_irp_t), MALLOC_CAP_DEFAULT);
        if(NULL == ctrl_irps[i]) ESP_LOGE("", "err 6");
        //Allocate data buffer
        ctrl_data_buffers[i] = heap_caps_calloc(1, sizeof(usb_ctrl_req_t) + TRANSFER_DATA_MAX_BYTES, MALLOC_CAP_DMA);
        if(NULL == ctrl_data_buffers[i]) ESP_LOGE("", "err 5");
        //Initialize IRP and IRP list
        ctrl_irps[i]->data_buffer = ctrl_data_buffers[i];
        ctrl_irps[i]->num_iso_packets = 0;
    }
    *handle = ctrl_pipe_hdl;
}

void ctrl_pipe_event_task(void* p)
{
    printf("start pipe event task\n");
    pipe_event_msg_t msg;
    ctrl_pipe_evt_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(pipe_event_msg_t));

    while(1){
        xQueueReceive(ctrl_pipe_evt_queue, &msg, portMAX_DELAY);
        usb_irp_t *irp = hcd_irp_dequeue(msg.pipe_hdl);

        if(irp == NULL) continue;
        void *context = irp->context;

        usb_ctrl_req_t* ctrl = (usb_ctrl_req_t*)irp->data_buffer;

        switch (msg.pipe_event)
        {
            case HCD_PIPE_EVENT_NONE:
                break;

            case HCD_PIPE_EVENT_IRP_DONE:
                switch (ctrl->bRequest)
                {
                    case USB_B_REQUEST_GET_STATUS:
                        break;

                    case USB_B_REQUEST_CLEAR_FEATURE:
                        break;

                    case USB_B_REQUEST_SET_FEATURE:
                        break;

                    case USB_B_REQUEST_SET_ADDRESS:
                        usbh_set_address_cb(ctrl->wValue, context);
                        break;

                    case USB_B_REQUEST_GET_DESCRIPTOR:
                        switch ((ctrl->wValue >> 8) & 0xff)
                        {
                            case USB_W_VALUE_DT_DEVICE:
                                usbh_get_device_desc_cb(irp->data_buffer + sizeof(usb_ctrl_req_t), irp->actual_num_bytes, context);
                                break;
                            
                            case USB_W_VALUE_DT_CONFIG:
                                usbh_get_config_desc_cb(irp->data_buffer + sizeof(usb_ctrl_req_t), irp->actual_num_bytes, context);
                                break;
                            
                            case USB_W_VALUE_DT_STRING:
                                usbh_get_string_cb((irp->data_buffer + sizeof(usb_ctrl_req_t)), irp->actual_num_bytes, context);
                                break;

                            case USB_W_VALUE_DT_INTERFACE:
                                usbh_get_interface_desc_cb(irp->data_buffer + sizeof(usb_ctrl_req_t), irp->actual_num_bytes, context);
                                break;
                            
                            case USB_W_VALUE_DT_ENDPOINT:
                                usbh_get_endpoint_desc_cb(irp->data_buffer + sizeof(usb_ctrl_req_t), irp->actual_num_bytes, context);
                                break;
                            
                            case USB_W_VALUE_DT_DEVICE_QUALIFIER:
                                break;
                            
                            case USB_W_VALUE_DT_OTHER_SPEED_CONFIG:
                                break;
                            
                            case USB_W_VALUE_DT_INTERFACE_POWER:
                                usbh_get_power_desc_cb(irp->data_buffer + sizeof(usb_ctrl_req_t), irp->actual_num_bytes, context);
                                break;
                            
                            default:
                                break;
                        }
                        break;

                    case USB_B_REQUEST_GET_CONFIGURATION:
                        usbh_get_configuration_cb(*(uint8_t*)(irp->data_buffer + sizeof(usb_ctrl_req_t)), context);
                        break;

                    case USB_B_REQUEST_SET_CONFIGURATION:
                        usbh_set_config_desc_cb(ctrl->wValue, context);
                        break;

                    case USB_B_REQUEST_SET_INTERFACE:
                        break;

                    case USB_B_REQUEST_SYNCH_FRAME:
                    ESP_LOGI("", "SYNC");
                        break;

                    default:
                        usbh_ctrl_pipe_class_specific_cb(msg, irp);
                        break;
                }
            break;

        case HCD_PIPE_EVENT_ERROR_XFER:
            usbh_ctrl_pipe_error_cb(ctrl);
            ESP_LOGW("", "XFER error: %d", irp->status);
            hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
            break;
        
        case HCD_PIPE_EVENT_ERROR_STALL:
            usbh_ctrl_pipe_stalled_cb(ctrl);
            ESP_LOGW("", "Device stalled: %s pipe, state: %d", msg.pipe_hdl == ctrl_pipe_hdl?"CTRL":"BULK", hcd_pipe_get_state(msg.pipe_hdl));
            ESP_LOG_BUFFER_HEX_LEVEL("Ctrl data", ctrl, sizeof(usb_ctrl_req_t), ESP_LOG_INFO);
            hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
            break;

            default:
                break;
        }
        if (ctrl_pipe_cb != NULL)
        {
            ctrl_pipe_cb(msg, irp, context);
        }
    }
}

void xfer_get_device_desc()
{
    USB_CTRL_REQ_INIT_GET_DEVC_DESC((usb_ctrl_req_t *) ctrl_irps[0]->data_buffer);

    ctrl_irps[0]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    ctrl_irps[0]->data_buffer = ctrl_data_buffers[0];
    ctrl_irps[0]->num_iso_packets = 0;


    //Enqueue those transfer requests
    esp_err_t err;
    if(ESP_OK != (err = hcd_irp_enqueue(ctrl_pipe_hdl, ctrl_irps[0]))) {
        ESP_LOGE("", "Get device desc: %d", err);
    }
}

void xfer_set_address(uint8_t addr)
{
    USB_CTRL_REQ_INIT_SET_ADDR((usb_ctrl_req_t *) ctrl_irps[0]->data_buffer, addr);

    ctrl_irps[0]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    ctrl_irps[0]->data_buffer = ctrl_data_buffers[0];
    ctrl_irps[0]->num_iso_packets = 0;
    ctrl_irps[0]->num_bytes = 0;


    //Enqueue those transfer requests
    esp_err_t err;
    if(ESP_OK != (err = hcd_irp_enqueue(ctrl_pipe_hdl, ctrl_irps[0]))) {
        ESP_LOGE("", "Set address: %d", err);
    }
}

void xfer_get_current_config()
{
    USB_CTRL_REQ_INIT_GET_CONFIG((usb_ctrl_req_t *) ctrl_irps[0]->data_buffer);
    ctrl_irps[0]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    ctrl_irps[0]->data_buffer = ctrl_data_buffers[0];
    ctrl_irps[0]->num_iso_packets = 0;

    //Enqueue those transfer requests
    esp_err_t err;
    if(ESP_OK != (err = hcd_irp_enqueue(ctrl_pipe_hdl, ctrl_irps[0]))) {
        ESP_LOGE("", "Get current config: %d", err);
    }
}

void xfer_set_configuration(uint8_t num)
{
    USB_CTRL_REQ_INIT_SET_CONFIG((usb_ctrl_req_t *) ctrl_irps[2]->data_buffer, num);
    ctrl_irps[2]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    ctrl_irps[2]->data_buffer = ctrl_data_buffers[2];
    ctrl_irps[2]->num_iso_packets = 0;
    ctrl_irps[2]->num_bytes = 0;

    //Enqueue those transfer requests
    esp_err_t err;
    if(ESP_OK != (err = hcd_irp_enqueue(ctrl_pipe_hdl, ctrl_irps[2]))) {
        ESP_LOGE("", "Set current config: %d", err);
    }
}

void xfer_get_desc()
{
    USB_CTRL_REQ_INIT_GET_CFG_DESC((usb_ctrl_req_t *) ctrl_irps[0]->data_buffer, 1, TRANSFER_DATA_MAX_BYTES);
    //important!! if is shorter than buffer and descriptor is longer than num_bytes, then it will stuck here
    // so its best if both values are equal
    ctrl_irps[0]->num_bytes = TRANSFER_DATA_MAX_BYTES;
    ctrl_irps[0]->data_buffer = ctrl_data_buffers[0];
    ctrl_irps[0]->num_iso_packets = 0;

    //Enqueue those transfer requests
    esp_err_t err;
    if(ESP_OK != (err = hcd_irp_enqueue(ctrl_pipe_hdl, ctrl_irps[0]))) {
        ESP_LOGE("", "Get config desc: %d", err);
    }
}

void xfer_get_string(uint8_t num)
{
    USB_CTRL_REQ_INIT_GET_STRING((usb_ctrl_req_t *) ctrl_irps[num]->data_buffer, 0, num, TRANSFER_DATA_MAX_BYTES);
    ctrl_irps[num]->num_bytes = TRANSFER_DATA_MAX_BYTES;
    ctrl_irps[num]->data_buffer = ctrl_data_buffers[num];
    ctrl_irps[num]->num_iso_packets = 0;

    //Enqueue those transfer requests
    esp_err_t err;
    if(ESP_OK != (err = hcd_irp_enqueue(ctrl_pipe_hdl, ctrl_irps[num]))) {
        ESP_LOGE("", "Get string: %d, err: %d", num, err);
    }
}
