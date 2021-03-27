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
#include "usb_host_port.h"

#define EVENT_QUEUE_LEN         10
#define XFER_DATA_MAX_LEN       256     //Just assume that will only IN/OUT 256 bytes for now
#define PORT_NUM                1       // we have only 1 port

static port_evt_cb_t port_evt_cb;
static QueueHandle_t port_evt_queue;

// -------------------------------------------------- PHY Control ------------------------------------------------------

static void phy_force_conn_state(bool connected, TickType_t delay_ticks)
{
    vTaskDelay(delay_ticks);
    usb_wrap_dev_t *wrap = &USB_WRAP;
    if (connected) {
        //Swap back to internal PHY that is connected to a devicee
        wrap->otg_conf.phy_sel = 0;
    } else {
        //Set externa PHY input signals to fixed voltage levels mimicing a disconnected state
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, USB_EXTPHY_VP_IDX, false);
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, USB_EXTPHY_VM_IDX, false);
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, USB_EXTPHY_RCV_IDX, false);
        //Swap to the external PHY
        wrap->otg_conf.phy_sel = 1;
    }
}

// ------------------------------------------------ Helper Functions ---------------------------------------------------

static bool port_callback(hcd_port_handle_t port_hdl, hcd_port_event_t port_event, void *user_arg, bool in_isr)
{
    QueueHandle_t port_evt_queue = (QueueHandle_t)user_arg;
    port_event_msg_t msg = {
        .port_hdl = port_hdl,
        .port_event = port_event,
    };

    BaseType_t xTaskWoken = pdFALSE;
    xQueueSendFromISR(port_evt_queue, &msg, &xTaskWoken);
    return (xTaskWoken == pdTRUE);
}

static void port_event_task(void* p)
{
    port_event_msg_t msg;
    while(1){
        xQueueReceive(port_evt_queue, &msg, portMAX_DELAY);
        ESP_LOGI("", "port event: %d", msg.port_event);
        hcd_port_handle_event(msg.port_hdl);
        switch (msg.port_event)
        {
            case HCD_PORT_EVENT_NONE:
                break;

            case HCD_PORT_EVENT_CONNECTION:
                usbh_port_connection_cb(msg);
                break;

            case HCD_PORT_EVENT_DISCONNECTION:
                hcd_port_command(msg.port_hdl, HCD_PORT_CMD_POWER_OFF);
                usbh_port_disconnection_cb(msg);
                break;

            case HCD_PORT_EVENT_ERROR:
                usbh_port_error_cb(msg);
                break;

            case HCD_PORT_EVENT_OVERCURRENT:
                usbh_port_overcurrent_cb(msg);
                break;

            case HCD_PORT_EVENT_SUDDEN_DISCONN:
                hcd_port_command(msg.port_hdl, HCD_PORT_CMD_RESET);
                usbh_port_sudden_disconn_cb(msg);
                break;
        }
        if(port_evt_cb != NULL)
        {
            port_evt_cb(msg);
        }        
    }
}

// ------------------------------------------------ Public Functions ---------------------------------------------------

/**
 * @brief Creates port and pipe event queues. Sets up the HCD, and initializes a port.
 */
bool setup_usb_host()
{
    port_evt_queue = xQueueCreate(EVENT_QUEUE_LEN, sizeof(port_event_msg_t));

    if(port_evt_queue) xTaskCreate(port_event_task, "port_task", 4*1024, NULL, 10, NULL);

    //Install HCD
    hcd_config_t config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    if(hcd_install(&config) == ESP_OK) {
        //Initialize a port
        hcd_port_config_t port_config = {
            .callback = port_callback,
            .callback_arg = (void *)port_evt_queue,
            .context = NULL,
        };
        esp_err_t err;
        if(ESP_OK == (err = hcd_port_init(PORT_NUM, &port_config, &port_hdl))){
            if(HCD_PORT_STATE_NOT_POWERED == hcd_port_get_state(port_hdl)) ESP_LOGI("", "USB host setup properly");
        
            phy_force_conn_state(false, 0);    //Force disconnected state on PHY
            if(ESP_OK != hcd_port_command(port_hdl, HCD_PORT_CMD_POWER_ON)) return false;
            ESP_LOGI("", "Port is power ON now");
            phy_force_conn_state(true, pdMS_TO_TICKS(10));     //Allow for connected state on PHY
            return true;
        } else {
            ESP_LOGE("", "Error to init port: %d!!!", err);
        }
    } else {
        ESP_LOGE("", "Error to install HCD!!!");
    }
    return false;
}

void register_port_callback(port_evt_cb_t cb)
{
    port_evt_cb = cb;
}
