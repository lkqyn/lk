/*
 * queue.c
 *
 *  Created on: 2020年4月3日
 *      Author: ylj
 */


#include "queue/queue.h"
#include "string.h"

struct CommQueue sendQueue,recvQueue;

void QUEUE_init(struct CommQueue *queue)
{
	queue->head = 0;
	queue->tail = 0;
	queue->isEmpty = 1;
}


uint8_t QUEUE_add(struct CommQueue *queue,struct CommData data)
{
	uint8_t rtl = 1;

	__disable_irq();

	if(queue->isEmpty)
	{
		memcpy(&queue->data[queue->tail],&data,sizeof(data));
		queue->tail ++;
		queue->tail %= QUEUE_LENGTH;
		queue->isEmpty = 0;
	}
	else
	{
		if(queue->tail == queue->head)
		{
			__enable_irq();
			return 0;
		}

		memcpy(&queue->data[queue->tail],&data,sizeof(data));
		queue->tail ++;
		queue->tail %= QUEUE_LENGTH;
	}

	__enable_irq();

	return rtl;
}

uint8_t QUEUE_get(struct CommQueue *queue,struct CommData *data)
{
	uint8_t rtl=1;

	__disable_irq();

	if(queue->isEmpty)
	{
		__enable_irq();
		return 0;
	}
	else
	{
		memcpy(data,&queue->data[queue->head],sizeof(struct CommData));
		queue->head ++;
		queue->head %= QUEUE_LENGTH;

		if(queue->head == queue->tail)
		{
			queue->isEmpty = 1;
		}
	}

	__enable_irq();

	return rtl;
}




