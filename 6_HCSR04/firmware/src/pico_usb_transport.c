/***
 * USB Driver for uROS using the USB STDIO driver.
 * Jon Durrant
 * July 2023
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico_usb_transports.h"
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include <time.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>

/***
 * Sleep via FreeRTOS sleep
 * @param us
 */
void usleep(uint64_t us){
	if (us < 1000){
		busy_wait_us(us);
		return;
	}
   TickType_t t = pdMS_TO_TICKS(us/1000);
   if (t < 1){
	   t = 1;
   }
   vTaskDelay(t);
}

/***
 * Get Time since boot
 * @param unused
 * @param tp
 * @return
 */
int clock_gettime(clockid_t unused, struct timespec *tp){
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

/***
 * Open transport
 * @param transport
 * @return true if ok
 */
bool pico_usb_transport_open(struct uxrCustomTransport * transport){
	//Checks we have USB CDC connection
	return stdio_usb_connected();
}

/***
 * Close transport
 * @param transport
 * @return true if OK
 */
bool pico_usb_transport_close(struct uxrCustomTransport * transport){
    return true;
}

/***
 * Write to the transport
 * @param transport
 * @param buf - Data to write
 * @param len - Length of data
 * @param err - error code
 * @return number of bytes written. <0 if error occurs
 */
size_t pico_usb_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode){
	stdio_usb.out_chars(buf, len);
    return len;
}

/***
 * Read buffer with timeout
 * @param transport
 * @param buf - Data buffer to write to
 * @param len - Max length of buffer
 * @param timeout - timeout in micro Seconds
 * @param err
 * @return returns number of bytes read. < 0 if error occurs
 */
size_t pico_usb_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode){
    uint64_t until_time_us = time_us_64() + timeout;

    size_t read = 0;
    while (time_us_64() < until_time_us){
    	read = stdio_usb.in_chars(buf, len);
    	if (read != 0){
    		vTaskDelay(1);
    		return read;
    	}

    	taskYIELD();

    }

    return 0;
}

#define DEBUG_LINE 30

/***
 * Print the buffer in hex and plain text for debugging
 */
void debugPrintBuffer(const char *title, const void * pBuffer, size_t bytes){
	size_t count =0;
	size_t lineEnd=0;
	const uint8_t *pBuf = (uint8_t *)pBuffer;

	printf("DEBUG: %s of size %d\n", title, bytes);

	while (count < bytes){
		lineEnd = count + DEBUG_LINE;
		if (lineEnd > bytes){
			lineEnd = bytes;
		}

		//Print HEX DUMP
		for (size_t i=count; i < lineEnd; i++){
			if (pBuf[i] <= 0x0F){
				printf("0%X ", pBuf[i]);
			} else {
				printf("%X ", pBuf[i]);
			}
		}

		//Pad for short lines
		size_t pad = (DEBUG_LINE - (lineEnd - count)) * 3;
		for (size_t i=0; i < pad; i++){
			printf(" ");
		}

		//Print Plain Text
		for (size_t i=count; i < lineEnd; i++){
			if ((pBuf[i] >= 0x20) && (pBuf[i] <= 0x7e)){
				printf("%c", pBuf[i]);
			} else {
				printf(".");
			}
		}

		printf("\n");

		count = lineEnd;

	}
}
