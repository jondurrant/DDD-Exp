/***
 * USB Driver for uROS using the USB STDIO driver.
 * Jon Durrant
 * July 2023
 */

#ifndef PICO_USB_TRANS
#define PICO_USB_TRANS

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

bool pico_usb_transport_open(struct uxrCustomTransport * transport);
bool pico_usb_transport_close(struct uxrCustomTransport * transport);
size_t pico_usb_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t pico_usb_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

int clock_gettime(clockid_t unused, struct timespec *tp);
#endif //PICO_USB_TRANS
