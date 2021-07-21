
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
    if (in_isr)
    {
        BaseType_t xTaskWoken = pdFALSE;
        xQueueSendFromISR(pipe_evt_queue, &msg, &xTaskWoken);
        return (xTaskWoken == pdTRUE);
    }
    else
    {
        xQueueSend(pipe_evt_queue, &msg, portMAX_DELAY);
        return false;
    }
}

void free_pipe_and_irp_list(hcd_pipe_handle_t pipe_hdl)
{
    printf("Freeing IRPs and IRP list\n");
    //Dequeue transfer requests
    do
    {
        usb_irp_t *irp = hcd_irp_dequeue(pipe_hdl);
        if (irp == NULL)
            break;
        heap_caps_free(irp->data_buffer);
        heap_caps_free(irp);
    } while (1);

    ESP_LOGD("", "Freeing transfer requets\n");
    //Free transfer requests (and their associated objects such as IRPs and data buffers)
    ESP_LOGD("", "Freeing pipe\n");
    //Delete the pipe
    if (ESP_OK != hcd_pipe_free(pipe_hdl))
    {
        ESP_LOGE("", "err to free pipes");
    }
}

void allocate_ctrl_pipe(hcd_port_handle_t port_hdl, hcd_pipe_handle_t *handle)
{
    //We don't support hubs yet. Just get the speed of the port to determine the speed of the device
    usb_speed_t port_speed;
    if (ESP_OK == hcd_port_get_speed(port_hdl, &port_speed))
    {
    }

    //Create default pipe
    ESP_LOGI("", "Creating default pipe\n");
    hcd_pipe_config_t config = {
        .callback = ctrl_pipe_callback,
        .callback_arg = (void *)ctrl_pipe_evt_queue,
        .context = NULL,
        .ep_desc = NULL, //NULL EP descriptor to create a default pipe
        .dev_addr = 0,
        .dev_speed = port_speed,
    };
    if (ESP_OK != hcd_pipe_alloc(port_hdl, &config, handle))
        ESP_LOGE("", "cant alloc pipe");
    if (NULL == handle)
    {
        ESP_LOGE("", "NULL == pipe_hdl");
    }
}

usb_irp_t *allocate_irp(hcd_port_handle_t port_hdl, size_t size)
{
    //Create IRPs and their data buffers
    ESP_LOGI("", "Creating new IRP, free memory: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    usb_irp_t *_irp = heap_caps_calloc(1, sizeof(usb_irp_t), MALLOC_CAP_DEFAULT);
    if (NULL == _irp)
        ESP_LOGE("", "err to alloc IRP");
    //Allocate data buffer
    uint8_t *_data_buffer = heap_caps_calloc(1, sizeof(usb_ctrl_req_t) + size, MALLOC_CAP_DMA);
    if (NULL == _data_buffer)
        ESP_LOGE("", "err to alloc data buffer");
    //Initialize IRP and IRP list
    _irp->data_buffer = _data_buffer;
    _irp->num_iso_packets = 0;
    _irp->num_bytes = size;

    return _irp;
}

void ctrl_pipe_event_task(void *p)
{
    printf("start pipe event task\n");
    pipe_event_msg_t msg;
    ctrl_pipe_evt_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(pipe_event_msg_t));

    while (1)
    {
        xQueueReceive(ctrl_pipe_evt_queue, &msg, portMAX_DELAY);
        usb_irp_t *irp = hcd_irp_dequeue(msg.pipe_hdl);

        if (irp == NULL)
            continue;
        void *context = irp->context;

        usb_ctrl_req_t *ctrl = (usb_ctrl_req_t *)irp->data_buffer;

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
            ESP_LOGW("", "Device stalled: %s pipe, state: %d", msg.pipe_hdl == ctrl_pipe_hdl ? "CTRL" : "BULK", hcd_pipe_get_state(msg.pipe_hdl));
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

        heap_caps_free(irp->data_buffer);
        heap_caps_free(irp);
        ESP_LOGI("", "Dequeue and delete IRP, free memory: %d", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    }
}

void xfer_get_device_desc(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle)
{
    usb_irp_t *irp = allocate_irp(port_hdl, 18);
    USB_CTRL_REQ_INIT_GET_DEVC_DESC((usb_ctrl_req_t *)irp->data_buffer);

    //Enqueue those transfer requests
    esp_err_t err;
    if (ESP_OK != (err = hcd_irp_enqueue(handle, irp)))
    {
        ESP_LOGE("", "Get device desc: %d", err);
    }
}

void xfer_set_address(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle, uint8_t addr)
{
    usb_irp_t *irp = allocate_irp(port_hdl, 0);
    USB_CTRL_REQ_INIT_SET_ADDR((usb_ctrl_req_t *)irp->data_buffer, addr);

    //Enqueue those transfer requests
    esp_err_t err;
    if (ESP_OK != (err = hcd_irp_enqueue(handle, irp)))
    {
        ESP_LOGE("", "Set address: %d", err);
    }
}

void xfer_get_current_config(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle)
{
    usb_irp_t *irp = allocate_irp(port_hdl, 1);
    USB_CTRL_REQ_INIT_GET_CONFIG((usb_ctrl_req_t *)irp->data_buffer);

    //Enqueue those transfer requests
    esp_err_t err;
    if (ESP_OK != (err = hcd_irp_enqueue(handle, irp)))
    {
        ESP_LOGE("", "Get current config: %d", err);
    }
}

void xfer_set_configuration(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle, uint8_t num)
{
    usb_irp_t *irp = allocate_irp(port_hdl, 0);
    USB_CTRL_REQ_INIT_SET_CONFIG((usb_ctrl_req_t *)irp->data_buffer, num);

    //Enqueue those transfer requests
    esp_err_t err;
    if (ESP_OK != (err = hcd_irp_enqueue(handle, irp)))
    {
        ESP_LOGE("", "Set current config: %d", err);
    }
}

void xfer_get_desc(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle)
{
    usb_irp_t *irp = allocate_irp(port_hdl, TRANSFER_DATA_MAX_BYTES);
    USB_CTRL_REQ_INIT_GET_CFG_DESC((usb_ctrl_req_t *)irp->data_buffer, 0, TRANSFER_DATA_MAX_BYTES);

    //Enqueue those transfer requests
    esp_err_t err;
    if (ESP_OK != (err = hcd_irp_enqueue(handle, irp)))
    {
        ESP_LOGE("", "Get config desc: %d", err);
    }
}

void xfer_get_string(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle, uint8_t num)
{
    usb_irp_t *irp = allocate_irp(port_hdl, TRANSFER_DATA_MAX_BYTES);
    USB_CTRL_REQ_INIT_GET_STRING((usb_ctrl_req_t *)irp->data_buffer, 0, num, TRANSFER_DATA_MAX_BYTES);

    //Enqueue those transfer requests
    esp_err_t err;
    if (ESP_OK != (err = hcd_irp_enqueue(handle, irp)))
    {
        ESP_LOGE("", "Get string: %d, err: %d", num, err);
    }
}
