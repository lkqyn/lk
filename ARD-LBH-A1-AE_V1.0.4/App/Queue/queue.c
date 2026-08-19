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

#if 0

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

#else

// 队列满的条件：(tail + 1) % QUEUE_LENGTH == head
// 队列空的条件：head == tail

uint8_t next_tail;

uint8_t QUEUE_add(struct CommQueue *queue, struct CommData data)
{
    uint8_t rtl = 0;
//    uint8_t next_tail;

    __disable_irq();

    next_tail = (queue->tail + 1) % QUEUE_LENGTH;
    if (next_tail != queue->head) {  // 队列未满
        memcpy(&queue->data[queue->tail], &data, sizeof(struct CommData));
        queue->tail = next_tail;
        rtl = 1;
    }

    __enable_irq();

    return rtl;
}

uint8_t QUEUE_get(struct CommQueue *queue, struct CommData *data)
{
    uint8_t rtl = 0;  // 默认返回失败（队列为空）

    if (data == NULL || queue == NULL) {
        return 0;  // 入参校验，避免空指针访问
    }

    __disable_irq();  // 进入临界区，保护队列指针操作

    // 标准环形队列判断：head == tail 表示队列为空
    if (queue->head != queue->tail) {
        // 复制数据到输出缓冲区
        memcpy(data, &queue->data[queue->head], sizeof(struct CommData));

        // 更新head指针，循环取模
        queue->head = (queue->head + 1) % QUEUE_LENGTH;

        rtl = 1;  // 获取成功
    }

    __enable_irq();  // 退出临界区，恢复中断

    return rtl;
}

#endif


