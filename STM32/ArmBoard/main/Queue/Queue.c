/*
 * Queue.c
 *
 *  Created on: Apr 14, 2025
 *      Author: Doradora
 */

#include "Queue.h"
#include <string.h>

int Queue_isValid(Queue *queue) {
	return queue != NULL && queue->buffer != NULL && queue->element_size > 0 && queue->capacity > 0;
}

int Queue_init(Queue *queue, void *buffer, size_t element_size, size_t capacity, bool overwrite, void (*lock)(), void (*unlock)()) {
	if (lock != NULL) lock();

	if (queue == NULL || buffer == NULL || element_size == 0 || capacity == 0) {
		if (unlock != NULL) unlock();
		return -1;
	}

	queue->buffer = buffer;
	queue->element_size = element_size;
	queue->capacity = capacity;
	queue->front = 0;
	queue->rear = 0;
	queue->count = 0;
	queue->overwrite = overwrite;
	queue->lock = lock;
	queue->unlock = unlock;

	if (unlock != NULL) unlock();
	return 0;
}

int Queue_push(Queue *queue, const void *element) {
	if (queue->lock != NULL) queue->lock();

	if (!Queue_isValid(queue) || element == NULL) {
		if (queue->unlock != NULL) queue->unlock();
		return -1;
	}

	if (queue->count == queue->capacity) {
		if (queue->overwrite) {
			queue->front = (queue->front + 1) % queue->capacity;
			queue->count--;
		}
		else {
			if (queue->unlock != NULL) queue->unlock();
			return -1;
		}
	}

	void *target = (char *)queue->buffer + (queue->rear * queue->element_size);
	memcpy(target, element, queue->element_size);
	queue->rear = (queue->rear + 1) % queue->capacity;
	queue->count++;
	if (queue->unlock != NULL) queue->unlock();
	return 0;
}

int Queue_pop(Queue *queue, void *element) {
	if (queue->lock != NULL) queue->lock();

	if (!Queue_isValid(queue) || element == NULL || queue->count == 0) {
		if (queue->unlock != NULL) queue->unlock();
		return -1;
	}

	void *source = (char *)queue->buffer + (queue->front * queue->element_size);
	memcpy(element, source, queue->element_size);
	queue->front = (queue->front + 1) % queue->capacity;
	queue->count--;
	if (queue->unlock != NULL) queue->unlock();
	return 0;
}

int Queue_clear(Queue *queue) {
	if (queue->lock != NULL) queue->lock();

	if (!Queue_isValid(queue)) {
		if (queue->unlock != NULL) queue->unlock();
		return -1;
	}

	queue->front = 0;
	queue->rear = 0;
	queue->count = 0;
	if (queue->unlock != NULL) queue->unlock();
	return 0;
}

int Queue_isEmpty(Queue *queue) {
	if (queue->lock != NULL) queue->lock();

	if (!Queue_isValid(queue)) {
		if (queue->unlock != NULL) queue->unlock();
		return -1;
	}

	if (queue->unlock != NULL) queue->unlock();
	return queue->count == 0;
}

int Queue_isFull(Queue *queue) {
	if (queue->lock != NULL) queue->lock();

	if (!Queue_isValid(queue)) {
		if (queue->unlock != NULL) queue->unlock();
		return -1;
	}

	if (queue->unlock != NULL) queue->unlock();
	return queue->count == queue->capacity;
}

int Queue_size(Queue *queue) {
	if (queue->lock != NULL) queue->lock();

	if (!Queue_isValid(queue)) {
		if (queue->unlock != NULL) queue->unlock();
		return -1;
	}

	if (queue->unlock != NULL) queue->unlock();
	return queue->count;
}

int Queue_capacity(Queue *queue) {
	if (queue->lock != NULL) queue->lock();

	if (!Queue_isValid(queue)) {
		if (queue->unlock != NULL) queue->unlock();
		return -1;
	}

	if (queue->unlock != NULL) queue->unlock();
	return queue->capacity;
}

int Queue_peek(Queue *queue, void *element) {
	if (queue->lock != NULL) queue->lock();

	if (!Queue_isValid(queue) || element == NULL || queue->count == 0) {
		if (queue->unlock != NULL) queue->unlock();
		return -1;
	}

	void *source = (char *)queue->buffer + (queue->front * queue->element_size);
	memcpy(element, source, queue->element_size);

	if (queue->unlock != NULL) queue->unlock();
	return 0;
}
