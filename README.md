## Simple cdc host example 
Simple USB host CDC example which should works with any CDC-ACM device. Tested with arduino tinyusb device built using 
https://github.com/chegewara/EspTinyUSB library.

## Example logs:

### Before connecting device
```
Hello world USB host!
I (627) : USB host setup properly
I (627) : Port powered ON
Waiting for conenction
```

### After connecting device
```
I (892) : HCD_PORT_EVENT_CONNECTION
I (892) : HCD_PORT_STATE_DISABLED
I (952) : USB device reset
I (952) : HCD_PORT_STATE_ENABLED
...
I (974) : address set: 1
I (978) : set current configuration: 1
```

### Example reading device descriptors and strings
- device descriptor on EP0
```
Device descriptor:
Length: 18
Descriptor type: 18
USB version: 2.00
Device class: 0xef (Miscellaneous)
Device subclass: 0x02
Device protocol: 0x01
EP0 max packet size: 64
VID: 0x303a
PID: 0x0002
Revision number: 1.00
Manufacturer id: 1
Product id: 2
Serial id: 3
Configurations num: 1
```

- configuration descriptor
```
Config:
Number of Interfaces: 2
Attributes: 0xa0
Max power: 500 mA

Interface:
bInterfaceNumber: 0
bAlternateSetting: 0
bNumEndpoints: 1
bInterfaceClass: 0x02 (CDC)
bInterfaceSubClass: 0x02
bInterfaceProtocol: 0x00

CS_Interface:

CS_Interface:

CS_Interface:

CS_Interface:

Endpoint:
bEndpointAddress: 0x81
bmAttributes: 0x03
bDescriptorType: 5
wMaxPacketSize: 8
bInterval: 16 ms
Creating transfer requests

Interface:
bInterfaceNumber: 1
bAlternateSetting: 0
bNumEndpoints: 2
bInterfaceClass: 0x0a (CDC-data)
bInterfaceSubClass: 0x00
bInterfaceProtocol: 0x00

Endpoint:
bEndpointAddress: 0x02
bmAttributes: 0x02
bDescriptorType: 5
wMaxPacketSize: 64
bInterval: 0 ms
Creating transfer requests

Endpoint:
bEndpointAddress: 0x82
bmAttributes: 0x02
bDescriptorType: 5
wMaxPacketSize: 64
bInterval: 0 ms
Creating transfer requests
```

- manufacturer, product and serial strings
```
strings: Espressif
strings: ESP32S2 arduino device
strings: 1234-5678
```

- set DTR/RTS lines and bitrate, stop bits, parity and data bits
```
I (7306) Set control line state:
I (7311) Set line coding: Bitrate: 115200, stop bits: 0, parity: 0, bits: 5
```

Have a nice play.
