/***
 * FreeRTOS Allocation mapping for uROS.
 * Jon Durrant
 * July 2023
 */

#include <rcutils/allocator.h>
#include "freertos_allocators.h"
#include "FreeRTOS.h"
#include <string.h>


void * __freertos_allocate(size_t size, void * state){
  return (void *)pvPortMalloc(size);
}

void __freertos_deallocate(void * pointer, void * state){
    vPortFree(pointer);
}

void * __freertos_reallocate(void * pointer, size_t size, void * state){
  if (NULL == pointer){
    return (void *)pvPortMalloc(size);
  } else {
	vPortFree(pointer);
    return (void *)pvPortMalloc(size);
  }
}

void * __freertos_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state){
  void * res = 	(void *)pvPortMalloc(number_of_elements * size_of_element);
  memset(res, 0, number_of_elements * size_of_element);
  return res;
}
