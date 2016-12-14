#ifndef DEVICE_USB_H
#define DEVICE_USB_H

#if CONFIG_ENABLED(DEVICE_USB)

#include "CodalDevice.h"
#define usb_assert(cond)                                                                           \
    if (!(cond))                                                                                   \
        device.panic(50);

#include "samd21.h"

#define DEVICE_USB_ENDPOINTS USB_EPT_NUM
#define USB_MAX_PKT_SIZE 64

class UsbEndpointIn
{
    uint8_t buf[USB_MAX_PKT_SIZE];
    uint8_t flags;

public:
    uint8_t ep;
    uint16_t wLength;
    int stall();
    int reset();
    int write(const void *buf, int length);

    UsbEndpointIn(uint8_t idx, uint8_t type, uint8_t size = USB_MAX_PKT_SIZE);
};

class UsbEndpointOut
{
    uint8_t buf[USB_MAX_PKT_SIZE];
    uint8_t flags;

public:
    uint8_t ep;
    int stall();
    int reset();
    int read(const void *buf, int maxlength); // up to packet size
    int readBlocking(const void *buf, int length);

    UsbEndpointOut(uint8_t idx, uint8_t type, uint8_t size = USB_MAX_PKT_SIZE);
};

void usb_configure(uint8_t numEndpoints);
bool usb_recieved_setup();
void usb_clear_setup();

#endif
#endif
