/***
 * FreeRTOS Allocation mapping for uROS.
 */




#ifndef _FREERTOS_ALLOCATORS
#define _FREERTOS_ALLOCATORS


#include "pico/stdlib.h"

void * __freertos_allocate(size_t size, void * state);

void __freertos_deallocate(void * pointer, void * state);

void * __freertos_reallocate(void * pointer, size_t size, void * state);

void * __freertos_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#endif
