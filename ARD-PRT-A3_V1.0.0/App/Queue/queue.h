/*
 * queue.h
 *
 *  Created on: 2020年4月3日
 *      Author: ylj
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include "stm32f1xx_hal.h"

#define QUEUE_LENGTH 	32

struct CommData
{
	char data[100];
	uint8_t length;
};

struct CommQueue
{
	uint8_t head;
	uint8_t tail;
	uint8_t isEmpty;
	struct CommData data[QUEUE_LENGTH];
};

void QUEUE_init(struct CommQueue *queue);
uint8_t QUEUE_add(struct CommQueue *queue,struct CommData data);
uint8_t QUEUE_get(struct CommQueue *queue,struct CommData *data);

extern struct CommQueue sendQueue,recvQueue;

#endif /* QUEUE_H_ */
