
/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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
#include "ctrl_pipe.h"
#include "usb_host_port.h"
#include "cdc_class.h"


uint8_t device_state = 0;
uint8_t conf_num;

hcd_pipe_handle_t ctrl_pipe_hdl;

uint8_t bMaxPacketSize0 = 64;
uint8_t conf_num = 0;
static bool ready = false;

void parse_cfg_descriptor(uint8_t* data_buffer, usb_transfer_status_t status, uint8_t len, uint8_t* conf_num);

static void cdc_pipe_cb(pipe_event_msg_t msg, hcd_xfer_req_handle_t req_hdl)
{
    hcd_pipe_handle_t pipe_hdl;
    usb_irp_t *irp;
    void *context;
    hcd_xfer_req_get_target(req_hdl, &pipe_hdl, &irp, &context);
    ESP_LOGD("", "\t-> Pipe [%d] event: %d\n", (uint8_t)context, msg.pipe_event);

    switch (msg.pipe_event)
    {
        case HCD_PIPE_EVENT_NONE:
            break;

        case HCD_PIPE_EVENT_XFER_REQ_DONE:
            ESP_LOGD("Pipe cdc: ", "XFER status: %d, num bytes: %d, actual bytes: %d", irp->status, irp->num_bytes, irp->actual_num_bytes);
            // we are getting only IN data here
            if(irp->data_buffer[0] == 0x3f)
                xfer_out_data((uint8_t*)"test\n", 5);
            else
                ESP_LOGI("", "%.*s", irp->actual_num_bytes, irp->data_buffer);
            ready = true;
            break;

        case HCD_PIPE_EVENT_ERROR_XFER:
            ESP_LOGW("", "XFER error: %d", irp->status);
            hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
            break;
        
        case HCD_PIPE_EVENT_ERROR_STALL:
            ESP_LOGW("", "Device stalled: %s pipe, state: %d", msg.pipe_hdl == ctrl_pipe_hdl?"CTRL":"BULK", hcd_pipe_get_state(msg.pipe_hdl));
            hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
            break;
        
        default:
            ESP_LOGW("", "not handled pipe event: %d", msg.pipe_event);
            break;
    }
}


static void ctrl_pipe_cb(pipe_event_msg_t msg, hcd_xfer_req_handle_t req_hdl)
{
    hcd_pipe_handle_t pipe_hdl;
    usb_irp_t *irp;
    void *context;
    hcd_xfer_req_get_target(req_hdl, &pipe_hdl, &irp, &context);
    // ESP_LOGD("", "\t-> Pipe [%d] event: %d\n", (uint8_t)context, msg.pipe_event);

    switch (msg.pipe_event)
    {
        case HCD_PIPE_EVENT_NONE:
            break;

        case HCD_PIPE_EVENT_XFER_REQ_DONE:
            ESP_LOGD("Pipe: ", "XFER status: %d, num bytes: %d, actual bytes: %d", irp->status, irp->num_bytes, irp->actual_num_bytes);
            if(msg.pipe_hdl == ctrl_pipe_hdl){
                usb_ctrl_req_t* ctrl = (usb_ctrl_req_t*)irp->data_buffer;

                ESP_LOG_BUFFER_HEX_LEVEL("Ctrl data", ctrl, sizeof(usb_ctrl_req_t), ESP_LOG_DEBUG);
                ESP_LOG_BUFFER_HEX_LEVEL("Actual data", irp->data_buffer + sizeof(usb_ctrl_req_t), irp->actual_num_bytes, ESP_LOG_DEBUG);

                if(ctrl->bRequest == USB_B_REQUEST_GET_DESCRIPTOR){
                    parse_cfg_descriptor((irp->data_buffer + sizeof(usb_ctrl_req_t)), irp->status, irp->actual_num_bytes, &conf_num);
                    if(ctrl->wValue == (USB_W_VALUE_DT_DEVICE << 8)) 
                        xfer_set_address(DEVICE_ADDR);
                    else if((ctrl->wValue >> 8) != USB_W_VALUE_DT_STRING) 
                        xfer_set_control_line(1, 1); // control line state is mandatory, at least with CDC device with my arduino library
                } else if(ctrl->bRequest == USB_B_REQUEST_GET_CONFIGURATION) {
                    ESP_LOGI("", "get current configuration: %d", irp->data_buffer[9]);
                } else if(ctrl->bRequest == USB_B_REQUEST_SET_CONFIGURATION) {
                    ESP_LOGI("", "set current configuration: %d", ctrl->wValue);
                    xfer_get_desc();
                } else if(ctrl->bRequest == USB_B_REQUEST_SET_ADDRESS) {
                    ESP_LOGI("", "address set: %d", ctrl->wValue);
                    if(ESP_OK != hcd_pipe_update(ctrl_pipe_hdl, DEVICE_ADDR, bMaxPacketSize0)) ESP_LOGE("", "failed to update ctrl pipe");
                    xfer_set_configuration(1);
                    ready = true;
                } else {
                    ESP_LOG_BUFFER_HEX_LEVEL("Ctrl data", ctrl, sizeof(usb_ctrl_req_t), ESP_LOG_DEBUG);
                    // handling app specific CTRL msg can be done here inline or passed to function just like i did
                    class_specific_data_cb(irp);
                    vTaskDelay(1);
                    xfer_get_string(1);
                    xfer_get_string(2);
                    xfer_get_string(3);
                }
            } else {
                // we are getting IN data here
                ESP_LOGE("CDC CTRL", "%.*s", irp->actual_num_bytes, irp->data_buffer);
            }
            break;

        case HCD_PIPE_EVENT_ERROR_XFER:
            ESP_LOGW("", "XFER error: %d", irp->status);
            hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
            break;
        
        case HCD_PIPE_EVENT_ERROR_STALL:
            ESP_LOGW("", "Device stalled: %s pipe, state: %d", msg.pipe_hdl == ctrl_pipe_hdl?"CTRL":"BULK", hcd_pipe_get_state(msg.pipe_hdl));
            hcd_pipe_command(msg.pipe_hdl, HCD_PIPE_CMD_RESET);
            break;
        
        default:
            ESP_LOGW("", "not handled pipe event: %d", msg.pipe_event);
            break;
    }
}

// example of port callback
static void port_cb(port_event_msg_t msg)
{
    hcd_port_state_t state;
    switch (msg.port_event)
    {
        case HCD_PORT_EVENT_CONNECTION:
            ESP_LOGI("", "HCD_PORT_EVENT_CONNECTION");
            if(HCD_PORT_STATE_DISABLED == hcd_port_get_state(msg.port_hdl)) ESP_LOGI("", "HCD_PORT_STATE_DISABLED");
            if(ESP_OK == hcd_port_command(msg.port_hdl, HCD_PORT_CMD_RESET)) ESP_LOGI("", "USB device reset");
            if(HCD_PORT_STATE_ENABLED == hcd_port_get_state(msg.port_hdl)){
                ESP_LOGI("", "HCD_PORT_STATE_ENABLED");
                // we are already physically connected and ready, now we can perform software connection steps
                alloc_pipe_and_xfer_reqs_ctrl(msg.port_hdl, &ctrl_pipe_hdl);
                // get device descriptor on EP0, this is first mandatory step
                xfer_get_device_desc();
            }
            break;

        case HCD_PORT_EVENT_DISCONNECTION:
            hcd_port_command(msg.port_hdl, HCD_PORT_CMD_POWER_OFF);
            break;

        case HCD_PORT_EVENT_ERROR:
            break;

        case HCD_PORT_EVENT_OVERCURRENT:
            break;

        case HCD_PORT_EVENT_SUDDEN_DISCONN:{
            hcd_port_command(msg.port_hdl, HCD_PORT_CMD_RESET);
            if (HCD_PIPE_STATE_INVALID == hcd_pipe_get_state(ctrl_pipe_hdl))
            {                
                ESP_LOGW("", "pipe state: %d", hcd_pipe_get_state(ctrl_pipe_hdl));
                ready = false;
                delete_pipes();
                free_pipe_and_xfer_reqs_ctrl(ctrl_pipe_hdl);
                ctrl_pipe_hdl = NULL;

                esp_err_t err;
                if(HCD_PORT_STATE_RECOVERY == (state = hcd_port_get_state(msg.port_hdl))){
                    if(ESP_OK != (err = hcd_port_recover(msg.port_hdl))) ESP_LOGE("recovery", "should be in not powered state %d => (%d)", state, err);
                } else {
                    ESP_LOGE("", "hcd_port_state_t: %d", state);
                }
                if(ESP_OK == hcd_port_command(msg.port_hdl, HCD_PORT_CMD_POWER_ON)) ESP_LOGI("", "Port powered ON");
            }
            break;
        }
        
        default:
            ESP_LOGE("", "port event: %d", msg.port_event);
            break;
    }    
}

extern void cdc_pipe_event_task(void* p);

void app_main(void)
{
    xTaskCreate(ctrl_pipe_event_task, "pipe_task", 4*1024, NULL, 10, NULL);
    xTaskCreate(cdc_pipe_event_task, "pipe_task", 4*1024, NULL, 10, NULL);

    printf("Hello world USB host!\n");
    if(setup_usb_host()){
        register_port_callback(port_cb);
        register_ctrl_pipe_callback(ctrl_pipe_cb);
        register_cdc_pipe_callback(cdc_pipe_cb);
    }

    while(1){
        vTaskDelay(10);
        if(ready){
            ready = false;
            xfer_in_data();
        }
    }
}
