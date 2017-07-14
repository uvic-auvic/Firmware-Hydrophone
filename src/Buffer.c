/*Buffer - Circular FIFO Buffer implementation*/

#include "Buffer.h"
#include <stdlib.h>

// Adds an element to the end of the queue
//  - data_ptr -> pointer to data I want to transmit
//  - data_len -> length of data I want to transmit
// Tidbit: const void* const - first const means the thing I'm pointing at can't change
//							 - second const means the location I'm pointing to can't change
extern void Buffer_add_ptr(Buffer* b, const void* const data_ptr, uint8_t data_len){
	// Insert element location
	b->data_ptr[b->idx_to_load] = data_ptr;
	b->data_len[b->idx_to_load] = data_len;
	b->idx_to_load++;
	b->idx_to_load %= MAX_BUFFER_SIZE;
	b->size++;

	// Check if buffer is full
	if(b->size > MAX_BUFFER_SIZE)
	{
		// Remove oldest element
		b->idx_to_pop++;
		b->idx_to_pop %= MAX_BUFFER_SIZE;
		b->size--;
		b->overflow_cnt++;
	}
}

// Removes an element from the front of the queue
extern int Buffer_pop_ptr(Buffer* b, void* data_ptr) {
	uint8_t ret = 0;

	// Check if the buffer has anything to pop
	if(b->size)
	{
		// Pop oldest element and store it in data
		data_ptr = (void*)b->data_ptr[b->idx_to_pop];
		ret = b->data_len[b->idx_to_pop];
		b->idx_to_pop++;
		b->idx_to_pop %= MAX_BUFFER_SIZE;
		b->size--;
	}

	return ret;
}

// Reset all variables of the buffer
extern void Buffer_init(Buffer* b){
	b->idx_to_load = 0;
	b->idx_to_pop = 0;
	b->size = 0;
	b->overflow_cnt = 0;
}

// Get the size of the buffer
extern int Buffer_size(Buffer* b){
	return b->size;
}

// Get the number of overflows that have occurred. Resets on read.
extern int Buffer_overflow(Buffer* b){
	int ret = b->overflow_cnt;
	b->overflow_cnt = 0;
	return ret;
}
