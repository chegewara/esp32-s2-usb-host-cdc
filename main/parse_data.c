
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

#define USB_W_VALUE_DT_HID                  0x22
#define USB_W_VALUE_DT_CS_INTERFACE         0x24

static uint8_t itf = 0;
extern void cdc_create_pipe(usb_desc_ep_t* ep);

static void create_pipe(usb_desc_ep_t* ep)
{
    switch (itf)
    {
        case 0x02:
        case 0x0A:
            cdc_create_pipe(ep);
            break;
        
        default:
            break;
    }
}

static char* class_to_str(uint8_t class)
{
    itf = class;
    switch (class)
    {
        case 0x00:
            return ">ifc";
        case 0x01:
            return "Audio";
        case 0x02:
            return "CDC";
        case 0x03:
            return "HID";
        case 0x05:
            return "Physical";
        case 0x06:
            return "Image";
        case 0x07:
            return "Printer";
        case 0x08:
            return "Mass Storage";
        case 0x09:
            return "Hub";
        case 0x0a:
            return "CDC-data";
        case 0x0b:
            return "Smart card";
        case 0x0d:
            return "Content security";
        case 0x0e:
            return "Video";
        case 0x0f:
            return "Personal heathcare";
        case 0x10:
            return "Audio/Vdeo devices";
        case 0x11:
            return "Bilboard";
        case 0x12:
            return "USB-C bridge";
        case 0xdc:
            return "Diagnostic device";
        case 0xe0:
            return "Wireless controller";
        case 0xef:
            return "Miscellaneous";
        case 0xfe:
            return "Application specific";
        case 0xff:
            return "Vendor specific";
        
        default:
            return "Wrong class type";
    }
}

static inline int bcd_to_decimal(unsigned char x) {
    return x - 6 * (x >> 4);
}

static void utf16_to_utf8(char* in, char* out, uint8_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        out[i/2] = in[i];
        i++;
    }
}

static void parse_device_descriptor(uint8_t* data_buffer, usb_transfer_status_t status, uint8_t* num)
{
    if(status == USB_TRANSFER_STATUS_COMPLETED){
        printf("\nDevice descriptor:\n");

        usb_desc_devc_t* desc = (usb_desc_devc_t*)data_buffer;
        extern uint8_t bMaxPacketSize0;
        bMaxPacketSize0 = desc->bMaxPacketSize0;

        printf("Length: %d\n", desc->bLength);
        printf("Descriptor type: %d\n", desc->bDescriptorType);
        printf("USB version: %d.%02d\n", bcd_to_decimal(desc->bcdUSB >> 8), bcd_to_decimal(desc->bcdUSB & 0xff));
        printf("Device class: 0x%02x (%s)\n", desc->bDeviceClass, class_to_str(desc->bDeviceClass));
        printf("Device subclass: 0x%02x\n", desc->bDeviceSubClass);
        printf("Device protocol: 0x%02x\n", desc->bDeviceProtocol);
        printf("EP0 max packet size: %d\n", desc->bMaxPacketSize0);
        printf("VID: 0x%04x\n", desc->idVendor);
        printf("PID: 0x%04x\n", desc->idProduct);
        printf("Revision number: %d.%02d\n", bcd_to_decimal(desc->bcdDevice >> 8), bcd_to_decimal(desc->bcdDevice & 0xff));
        printf("Manufacturer id: %d\n", desc->iManufacturer);
        printf("Product id: %d\n", desc->iProduct);
        printf("Serial id: %d\n", desc->iSerialNumber);
        printf("Configurations num: %d\n", desc->bNumConfigurations);
        *num = desc->bNumConfigurations;
    } else {
        ESP_LOGW("", "status: %d", status);
    }
}

void parse_cfg_descriptor(uint8_t* data_buffer, usb_transfer_status_t status, uint8_t len, uint8_t* num)
{
    if(!len) return;
    if(status == USB_TRANSFER_STATUS_COMPLETED){
        uint8_t offset = 0;
        uint8_t type = *(&data_buffer[0] + offset + 1);
        do{
            ESP_LOGD("", "type: %d\n", type);
            switch (type)
            {
                case USB_W_VALUE_DT_DEVICE:
                    parse_device_descriptor(data_buffer, status, num);
                    offset += len;
                    break;

                case USB_W_VALUE_DT_CONFIG:{
                    printf("\nConfig:\n");
                    usb_desc_cfg_t* data = (usb_desc_cfg_t*)(data_buffer + offset);
                    printf("Number of Interfaces: %d\n", data->bNumInterfaces);
                    // printf("type: %d\n", data->bConfigurationValue);
                    // printf("type: %d\n", data->iConfiguration);
                    printf("Attributes: 0x%02x\n", data->bmAttributes);
                    printf("Max power: %d mA\n", data->bMaxPower * 2);
                    offset += data->bLength;
                    break;
                }
                case USB_W_VALUE_DT_STRING:{
                    usb_desc_str_t* data = (usb_desc_str_t*)(data_buffer + offset);
                    uint8_t len = 0;
                    len = data->bLength;
                    offset += len;
                    char* str = (char*)calloc(1, len);
                    utf16_to_utf8((char*)&data->val[2], str, len);
                    printf("strings: %s\n", str);
                    free(str);
                    break;
                }
                case USB_W_VALUE_DT_INTERFACE:{
                    printf("\nInterface:\n");
                    usb_desc_intf_t* data = (usb_desc_intf_t*)(data_buffer + offset);
                    offset += data->bLength;
                    printf("bInterfaceNumber: %d\n", data->bInterfaceNumber);
                    printf("bAlternateSetting: %d\n", data->bAlternateSetting);
                    printf("bNumEndpoints: %d\n", data->bNumEndpoints);
                    printf("bInterfaceClass: 0x%02x (%s)\n", data->bInterfaceClass, class_to_str(data->bInterfaceClass));
                    printf("bInterfaceSubClass: 0x%02x\n", data->bInterfaceSubClass);
                    printf("bInterfaceProtocol: 0x%02x\n", data->bInterfaceProtocol);
                    break;
                }
                case USB_W_VALUE_DT_ENDPOINT:{
                    printf("\nEndpoint:\n");
                    usb_desc_ep_t* data = (usb_desc_ep_t*)(data_buffer + offset);
                    offset += data->bLength;
                    printf("bEndpointAddress: 0x%02x\n", data->bEndpointAddress);
                    printf("bmAttributes: 0x%02x\n", data->bmAttributes);
                    printf("bDescriptorType: %d\n", data->bDescriptorType);
                    printf("wMaxPacketSize: %d\n", data->wMaxPacketSize);
                    printf("bInterval: %d ms\n", data->bInterval);
                    create_pipe(data);
                    break;
                }
                case USB_W_VALUE_DT_CS_INTERFACE:{
                    printf("\nCS_Interface:\n");
                    usb_desc_intf_t* data = (usb_desc_intf_t*)(data_buffer + offset);
                    offset += data->bLength;

                    break;
                }
                default:
                    ESP_LOGI("", "unknown descriptor: %d", type);
                    ESP_LOG_BUFFER_HEX_LEVEL("Actual data", data_buffer, len, ESP_LOG_DEBUG);

                    offset += *(data_buffer + offset);
                    break;
            }
            if(offset >= len) break;
            type = *(data_buffer + offset + 1);
        }while(1);
    } else {
        ESP_LOGW("", "status: %d", (uint8_t)status);
    }
}
