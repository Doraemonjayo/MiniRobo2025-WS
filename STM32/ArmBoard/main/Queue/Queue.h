/*
 * Queue.h
 *
 *  Created on: Apr 14, 2025
 *      Author: Doradora
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>
#include <stdbool.h>

typedef struct {
	void *buffer;
	size_t element_size;
	size_t capacity;
	size_t front;
	size_t rear;
	size_t count;
	bool overwrite;
	void (*lock)();
	void (*unlock)();
} Queue;

int Queue_isValid(Queue *queue);
int Queue_init(Queue *queue, void *buffer, size_t element_size, size_t capacity, bool overwrite, void (*lock)(), void (*unlock)());
int Queue_push(Queue *queue, const void *element);
int Queue_pop(Queue *queue, void *element);
int Queue_clear(Queue *queue);
int Queue_isEmpty(Queue *queue);
int Queue_isFull(Queue *queue);
int Queue_size(Queue *queue);
int Queue_capacity(Queue *queue);
int Queue_peek(Queue *queue, void *element);

#ifdef __cplusplus
}
#endif

#endif /* QUEUE_H_ */
