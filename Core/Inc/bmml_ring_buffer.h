#ifndef BMML_RING_BUFFER_H
#define BMML_RING_BUFFER_H


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


typedef struct {
    uint8_t *buff;          // buffer memory
    size_t size;            // size of
    size_t element_size;    // size of type
    size_t mask;            // bit mask for '&' operation (buff size - 1)
    volatile size_t head;   // write index (producer)
    volatile size_t tail;   // read index (consumer)
    bool overflow;          // overflow flag
    uint32_t dropped;       // count of data lost
} RingBuffer_t;

/**
 * @brief Initialize ring buffer.
 *
 * Configures a ring buffer instance using user-provided storage.
 * Buffer size must be a power of two because index wrapping uses
 * bitwise AND instead of modulo operation.
 *
 * Note: Maximum number of stored elements is (size - 1).
 *
 * @param rb            Pointer to ring buffer object.
 * @param storage       Pointer to preallocated memory for buffer data.
 * @param size          Number of elements in buffer (must be power of two).
 * @param element_size  Size of one element in bytes.
 *
 * @return true if initialization succeeded.
 * @return false if arguments are invalid.
 */
bool ring_buffer_init(RingBuffer_t *rb, void *storage, size_t size, size_t element_size);

/**
 * @brief Insert element into ring buffer.
 *
 * Copies one element into the buffer and advances write index.
 * Operation fails if buffer is full.
 *
 * @param rb    Pointer to ring buffer object.
 * @param data  Pointer to source data.
 *
 * @return true if element was inserted.
 * @return false if buffer is full or arguments are invalid.
 */
bool ring_buffer_push(RingBuffer_t *rb, const void *data);

/**
 * @brief Insert element and overwrite oldest data if buffer is full.
 *
 * Copies one element into the buffer. If the buffer is full,
 * the oldest element is discarded by advancing the read index.
 *
 * @param rb    Pointer to ring buffer object.
 * @param data  Pointer to source data.
 *
 * @return true if operation succeeded.
 * @return false if arguments are invalid.
 */
bool ring_buffer_push_overwrite(RingBuffer_t *rb, const void *data);

/**
 * @brief Remove oldest element from ring buffer.
 *
 * Copies one element from the buffer into output storage
 * and advances read index.
 *
 * @param rb        Pointer to ring buffer object.
 * @param out_data  Pointer to destination memory.
 *
 * @return true if element was read.
 * @return false if buffer is empty.
 */
bool ring_buffer_pop(RingBuffer_t *rb, void *out_data);

/**
 * @brief Read oldest element without removing it.
 *
 * Copies one element from the buffer into output storage
 * without advancing read index.
 *
 * @param rb        Pointer to ring buffer object.
 * @param out_data  Pointer to destination memory.
 *
 * @return true if element was copied.
 * @return false if buffer is empty.
 */
bool ring_buffer_peek(const RingBuffer_t *rb, void *out_data);


/**
 * @brief Check whether buffer is full.
 *
 * @param rb Pointer to ring buffer object.
 *
 * @return true if buffer cannot accept more elements.
 * @return false otherwise.
 */
static inline bool is_full(const RingBuffer_t *rb) {
    return ((rb->head + 1) & rb->mask) == rb->tail;
}


/**
 * @brief Check whether buffer is empty.
 *
 * @param rb Pointer to ring buffer object.
 *
 * @return true if buffer contains no elements.
 * @return false otherwise.
 */
static inline bool is_empty(const RingBuffer_t *rb) {
    return rb->head == rb->tail;
}


/**
 * @brief Get current number of stored elements.
 *
 * Returns number of unread elements currently stored
 * in the ring buffer.
 *
 * @param rb Pointer to ring buffer object.
 *
 * @return Number of stored elements.
 */
static inline size_t ring_buffer_count(const RingBuffer_t *rb){
    return (rb->head - rb->tail) & rb->mask;
}

#ifdef __cplusplus
}
#endif

#endif /*BMML_RING_BUFFER_H*/