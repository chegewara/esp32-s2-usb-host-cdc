#include <stdio.h>
#include <string.h>
#include "common.h"
#include "cdc_class.h"
#include "hcd.h"


#define MAX_NUM_ENDP    3
#define EP1             0
#define EP2             1
#define EP3             2


hcd_pipe_handle_t cdc_ep_pipe_hdl[MAX_NUM_ENDP];
uint8_t *cdc_data_buffers[MAX_NUM_ENDP];
usb_irp_t *cdc_ep_irps[MAX_NUM_ENDP];
usb_desc_ep_t endpoints[MAX_NUM_ENDP];
int bMaxPacketSize0;
extern hcd_pipe_handle_t ctrl_pipe_hdl;

static QueueHandle_t cdc_pipe_evt_queue;
static ctrl_pipe_cb_t cdc_pipe_cb;

static bool cdc_pipe_callback(hcd_pipe_handle_t pipe_hdl, hcd_pipe_event_t pipe_event, void *user_arg, bool in_isr)
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

void cdc_pipe_event_task(void* p)
{
    printf("start pipe event task\n");
    pipe_event_msg_t msg;
    cdc_pipe_evt_queue = xQueueCreate(10, sizeof(pipe_event_msg_t));

    while(1){
        xQueueReceive(cdc_pipe_evt_queue, &msg, portMAX_DELAY);
        usb_irp_t* irp = hcd_irp_dequeue(msg.pipe_hdl);
        hcd_pipe_handle_t pipe_hdl;
        if(irp == NULL) continue;
        void *context = irp->context;

        if (cdc_pipe_cb != NULL)
        {
            cdc_pipe_cb(msg, irp, context);
        }
    }
}

static void free_pipe_and_xfer_reqs(hcd_pipe_handle_t pipe_hdl,
                                    // hcd_xfer_req_handle_t *req_hdls,
                                    uint8_t **data_buffers,
                                    usb_irp_t **irps,
                                    int num_xfers)
{
    //Dequeue transfer requests
    do{
        usb_irp_t* irp = hcd_irp_dequeue(pipe_hdl);
        if(irp == NULL) break;
    }while(1);

    ESP_LOGD("", "Freeing transfer requets\n");
    //Free transfer requests (and their associated objects such as IRPs and data buffers)
    for (int i = 0; i < num_xfers; i++) {
        heap_caps_free(irps[i]);
        heap_caps_free(data_buffers[i]);
    }
    ESP_LOGD("", "Freeing pipe\n");
    //Delete the pipe
    if(ESP_OK != hcd_pipe_free(pipe_hdl)) {
        ESP_LOGE("", "err to free pipes");
    }
}

static void alloc_pipe_and_xfer_reqs_cdc(hcd_port_handle_t port_hdl,
                                     QueueHandle_t pipe_evt_queue,
                                     hcd_pipe_handle_t *pipe_hdl,
                                     uint8_t **data_buffers,
                                     usb_irp_t **irps,
                                     int num_xfers,
                                     usb_desc_ep_t* ep)
{
    //We don't support hubs yet. Just get the speed of the port to determine the speed of the device
    usb_speed_t port_speed;
    if(ESP_OK == hcd_port_get_speed(port_hdl, &port_speed)){}

    hcd_pipe_config_t config = {
        .callback = cdc_pipe_callback,
        .callback_arg = (void *)pipe_evt_queue,
        .context = NULL,
        .ep_desc = ep,
        .dev_addr = DEVICE_ADDR, // TODO
        .dev_speed = port_speed,
    };
    if(ESP_OK == hcd_pipe_alloc(port_hdl, &config, pipe_hdl)) {}
    if(NULL == pipe_hdl) {
        ESP_LOGE("", "NULL == pipe_hdl");
        return;
    }
    //Create transfer requests (and other required objects such as IRPs and data buffers)
    printf("Creating transfer requests\n");
    for (int i = 0; i < num_xfers; i++) {
        irps[i] = heap_caps_calloc(1, sizeof(usb_irp_t), MALLOC_CAP_DEFAULT);
        if(NULL == irps[i]) ESP_LOGE("", "err 6");
        //Allocate data buffers
        data_buffers = heap_caps_calloc(1, sizeof(usb_ctrl_req_t) + TRANSFER_DATA_MAX_BYTES, MALLOC_CAP_DMA);
        if(NULL == data_buffers) ESP_LOGE("", "err 5");
        //Initialize IRP and IRP list
        irps[i]->data_buffer = data_buffers;
        irps[i]->num_iso_packets = 0;
    }
}

void cdc_create_pipe(usb_desc_ep_t* ep)
{
    if(cdc_pipe_evt_queue == NULL)
        cdc_pipe_evt_queue = xQueueCreate(10, sizeof(pipe_event_msg_t));
    if(USB_DESC_EP_GET_XFERTYPE(ep) == USB_TRANSFER_TYPE_INTR){
        ESP_LOGI("", "create INTR endpoint");
        memcpy(&endpoints[EP1], ep, sizeof(usb_desc_ep_t));
        alloc_pipe_and_xfer_reqs_cdc(port_hdl, cdc_pipe_evt_queue, &cdc_ep_pipe_hdl[EP1], &cdc_data_buffers[EP1], &cdc_ep_irps[EP1], 1, ep);
    } else if(USB_DESC_EP_GET_XFERTYPE(ep) == USB_TRANSFER_TYPE_BULK && USB_DESC_EP_GET_EP_DIR(ep)){
        ESP_LOGI("", "create BULK1 endpoint");
        memcpy(&endpoints[EP2], ep, sizeof(usb_desc_ep_t));
        alloc_pipe_and_xfer_reqs_cdc(port_hdl, cdc_pipe_evt_queue, &cdc_ep_pipe_hdl[EP2], &cdc_data_buffers[EP2], &cdc_ep_irps[EP2], 1, ep);
    } else {
        ESP_LOGI("", "create BULK2 endpoint");
        memcpy(&endpoints[EP3], ep, sizeof(usb_desc_ep_t));
        alloc_pipe_and_xfer_reqs_cdc(port_hdl, cdc_pipe_evt_queue, &cdc_ep_pipe_hdl[EP3], &cdc_data_buffers[EP3], &cdc_ep_irps[EP3], 1, ep);
    }
}

void delete_pipes()
{
    for (size_t i = 0; i < MAX_NUM_ENDP; i++)
    {
        if(cdc_ep_pipe_hdl[i] == NULL) continue;
        if (HCD_PIPE_STATE_INVALID == hcd_pipe_get_state(cdc_ep_pipe_hdl[i]))
        {                
            ESP_LOGD("", "pipe state: %d", hcd_pipe_get_state(cdc_ep_pipe_hdl[i]));
            free_pipe_and_xfer_reqs( cdc_ep_pipe_hdl[i], &cdc_data_buffers[i], &cdc_ep_irps[i], 1);
            cdc_ep_pipe_hdl[i] = NULL;
        }
    }
}

void xfer_set_line_coding(uint32_t bitrate, uint8_t cf, uint8_t parity, uint8_t bits)
{
    usb_irp_t *irp = allocate_irp(port_hdl, 7);
    USB_CTRL_REQ_CDC_SET_LINE_CODING((cdc_ctrl_line_t *) irp->data_buffer, 0, bitrate, cf, parity, bits);

    esp_err_t err;
    if(ESP_OK == (err = hcd_irp_enqueue(ctrl_pipe_hdl, irp))) {
        ESP_LOGD("xfer", "set line codding");
    } else {
        ESP_LOGW("xfer", "set line codding: 0x%x", err);
    }
}

void xfer_get_line_coding()
{
    usb_irp_t *irp = allocate_irp(port_hdl, 7);
    USB_CTRL_REQ_CDC_GET_LINE_CODING((usb_ctrl_req_t *) irp->data_buffer, 0);

    esp_err_t err;
    if(ESP_OK == (err = hcd_irp_enqueue(ctrl_pipe_hdl, irp))) {
        ESP_LOGD("xfer", "get line codding");
    } else {
        ESP_LOGW("xfer", "get line codding: 0x%x", err);
    }
}

void xfer_set_control_line(hcd_port_handle_t port_hdl, hcd_pipe_handle_t handle, bool dtr, bool rts)
{
    usb_irp_t *irp = allocate_irp(port_hdl, 0);
    USB_CTRL_REQ_CDC_SET_CONTROL_LINE_STATE((usb_ctrl_req_t *) irp->data_buffer, 0, dtr, rts);

    esp_err_t err;
    if(ESP_OK == (err = hcd_irp_enqueue(handle, irp))) {
        ESP_LOGD("xfer", "set control line");
    } else {
        ESP_LOGW("xfer", "set control line: 0x%x", err);
    }
}


// ENDPOINTS
void xfer_intr_data()
{
    cdc_ep_irps[EP1]->num_bytes = 8;    //1 worst case MPS
    cdc_ep_irps[EP1]->data_buffer = cdc_data_buffers[EP1];
    cdc_ep_irps[EP1]->num_iso_packets = 0;
    cdc_ep_irps[EP1]->num_bytes = 8;

    esp_err_t err;
    if(ESP_OK == (err = hcd_irp_enqueue(cdc_ep_pipe_hdl[EP1], cdc_ep_irps[EP1]))) {
        ESP_LOGI("", "INT ");
    } else {
        ESP_LOGE("", "INT err: 0x%02x", err);
    }
}

void xfer_in_data()
{
    ESP_LOGD("", "EP: 0x%02x", USB_DESC_EP_GET_ADDRESS(&endpoints[EP2]));
    cdc_ep_irps[EP2]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    cdc_ep_irps[EP2]->num_iso_packets = 0;
    cdc_ep_irps[EP2]->num_bytes = 64;

    esp_err_t err;
    if(ESP_OK != (err = hcd_irp_enqueue(cdc_ep_pipe_hdl[EP2], cdc_ep_irps[EP2]))) {
        ESP_LOGW("", "BULK %s, dir: %d, err: 0x%x", "IN", USB_DESC_EP_GET_EP_DIR(&endpoints[EP2]), err);
    }
}

void xfer_out_data(uint8_t* data, size_t len)
{
    ESP_LOGD("", "EP: 0x%02x", USB_DESC_EP_GET_ADDRESS(&endpoints[EP3]));
    memcpy(cdc_ep_irps[EP3]->data_buffer, data, len);
    cdc_ep_irps[EP3]->num_bytes = bMaxPacketSize0;    //1 worst case MPS
    cdc_ep_irps[EP3]->num_iso_packets = 0;
    cdc_ep_irps[EP3]->num_bytes = len;

    esp_err_t err;
    if(ESP_OK != (err = hcd_irp_enqueue(cdc_ep_pipe_hdl[EP3], cdc_ep_irps[EP3]))) {
        ESP_LOGW("", "BULK %s, dir: %d, err: 0x%x", "OUT", USB_DESC_EP_GET_EP_DIR(&endpoints[EP3]), err);
    }
}

void cdc_class_specific_ctrl_cb(usb_irp_t* irp)
{
    if (irp->data_buffer[0] == SET_VALUE && irp->data_buffer[1] == SET_LINE_CODING) // set line coding
    {
        line_coding_t* data = (line_coding_t*)(irp->data_buffer + sizeof(usb_ctrl_req_t));
        ESP_LOGI("Set line coding", "Bitrate: %d, stop bits: %d, parity: %d, bits: %d",
                            data->dwDTERate, data->bCharFormat, data->bParityType, data->bDataBits);
    } else if (irp->data_buffer[0] == GET_VALUE && irp->data_buffer[1] == GET_LINE_CODING) // get line coding
    {
        line_coding_t* data = (line_coding_t*)(irp->data_buffer + sizeof(usb_ctrl_req_t));
        ESP_LOGI("Get line coding", "Bitrate: %d, stop bits: %d, parity: %d, bits: %d",
                            data->dwDTERate, data->bCharFormat, data->bParityType, data->bDataBits);        
    } else if (irp->data_buffer[0] == SET_VALUE && irp->data_buffer[1] == SET_CONTROL_LINE_STATE) // set line coding
    {
        line_coding_t* data = (line_coding_t*)(irp->data_buffer + sizeof(usb_ctrl_req_t));
        ESP_LOGI("Set control line state", "");
        xfer_set_line_coding(115200, 0, 0, 5);
    }
}

void register_cdc_pipe_callback(ctrl_pipe_cb_t cb)
{
    cdc_pipe_cb = cb;
}
