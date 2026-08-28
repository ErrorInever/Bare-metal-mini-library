#include "bmml_ring_buffer.h"
#include <stdint.h>
#include <string.h>

#define ENTER_CRITICAL() __disable_irq()
#define EXIT_CRITICAL()  __enable_irq()

bool ring_buffer_init(RingBuffer_t *rb, void *storage, size_t size, size_t element_size) {
    if (!rb || !storage || element_size == 0 || 
        size == 0 || (size & (size - 1)) != 0) return false;
    rb->buff = (uint8_t *)storage;
    rb->size = size;
    rb->mask = size - 1;
    rb->element_size = element_size;
    rb->head = 0;
    rb->tail = 0;
    return true;
}

bool ring_buffer_push(RingBuffer_t *rb, const void *data) {
    if (!rb || !data) return false;
    else if(is_full(rb)) {
        rb->overflow = true;
        rb->dropped++;
        return false;
    }
    void *dest = &rb->buff[rb->head * rb->element_size];
    memcpy(dest, data, rb->element_size);
    rb->head = (rb->head + 1) & rb->mask;
    return true;
}

bool ring_buffer_push_overwrite(RingBuffer_t *rb, const void *data) {
    if (!rb || !data) return false;
    if(is_full(rb)) rb->tail = (rb->tail + 1) & rb->mask;
    uint8_t *dest = &rb->buff[rb->head * rb->element_size];
    memcpy(dest, data, rb->element_size);
    rb->head = (rb->head + 1) & rb->mask;
    return true;
}

bool ring_buffer_pop(RingBuffer_t *rb, void *out_data) {
    if (is_empty(rb)) return false;
    uint8_t *src = &rb->buff[rb->tail * rb->element_size];
    memcpy(out_data, src, rb->element_size);
    rb->tail = (rb->tail + 1) & rb->mask;
    return true;
}
bool ring_buffer_peek(const RingBuffer_t *rb, void *out_data) {
    if (is_empty(rb)) return false;
    void *src = &rb->buff[rb->tail * rb->element_size];
    memcpy(out_data, src, rb->element_size);
    return true;
}