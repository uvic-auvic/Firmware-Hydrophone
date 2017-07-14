#ifndef BUFFER_H_
#define BUFFER_H_

#include "stm32f4xx.h"

#define MAX_BUFFER_SIZE (8)

typedef struct Buffer{
	const void* data_ptr[MAX_BUFFER_SIZE]; // Stores pointers to memory of location of data array to transmit
	uint8_t data_len[MAX_BUFFER_SIZE]; // Stores length of data in each memory location
	uint8_t idx_to_load; // Stores index where new element should go
	uint8_t idx_to_pop; // Stores index of the next element to remove
	uint8_t size; // Stores the number of elements in the buffer
	uint8_t overflow_cnt; // Stores the number of buffer overflows
}Buffer;

//Public functions ------------------------------

extern void Buffer_add_ptr(Buffer* b, const void* data_ptr, uint8_t data_len);
extern int Buffer_pop(Buffer* b, char* data);
extern int Buffer_size(Buffer* b);
extern int Buffer_overflow(Buffer* b);
extern void Buffer_init();

//-----------------------------------------------

#endif
