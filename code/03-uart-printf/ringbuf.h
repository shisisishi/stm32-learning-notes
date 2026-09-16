/* ringbuf.h —— 串口环形缓冲区
 * 纯 C 实现，不依赖 HAL，可以单独拿到 PC 上编译测试。
 * 约定：head 只在中断（生产者）里改，tail 只在主循环（消费者）里改，
 *       因此不需要关中断保护；但两个变量都必须是 volatile。
 */
#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>

#define RINGBUF_SIZE 128u          /* 必须是 2 的幂，这样取模可以用 & 代替 % */
                                   /* 注意：这种写法下最多只能存 127 个字节，留一个空位区分"空"和"满" */

typedef struct {
    uint8_t  buf[RINGBUF_SIZE];
    volatile uint16_t head;        /* 写指针：中断里改，所以必须 volatile */
    volatile uint16_t tail;        /* 读指针：主循环里改，同样必须 volatile */
    volatile uint32_t overflow;    /* 因缓冲区满而丢弃的字节数，用于诊断 */
} ringbuf_t;

/* 清空缓冲区。必须在使能接收中断之前调用 */
static inline void ringbuf_init(ringbuf_t *rb)
{
    rb->head = 0u;
    rb->tail = 0u;
    rb->overflow = 0u;
}

/* 已存字节数：(head - tail) 在 128 的模意义下取正值 */
static inline uint16_t ringbuf_count(const ringbuf_t *rb)
{
    return (uint16_t)((rb->head - rb->tail) & (RINGBUF_SIZE - 1u));
}

/* 是否为空 */
static inline uint8_t ringbuf_is_empty(const ringbuf_t *rb)
{
    return (rb->head == rb->tail) ? 1u : 0u;
}

/* 取一个字节：成功返回 1 并把数据写进 *out；缓冲区空则返回 0
 * 只在主循环（消费者）里调用 */
static inline uint8_t ringbuf_get(ringbuf_t *rb, uint8_t *out)
{
    if (rb->head == rb->tail) {
        return 0u;                                                  /* 空，没有数据 */
    }
    *out = rb->buf[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1u) & (RINGBUF_SIZE - 1u));    /* 到末尾就绕回开头 */
    return 1u;
}

/* 塞一个字节：缓冲区满则丢弃新字节并累加溢出计数
 * 只在中断回调（生产者）里调用 */
static inline void ringbuf_put(ringbuf_t *rb, uint8_t byte)
{
    uint16_t next = (uint16_t)((rb->head + 1u) & (RINGBUF_SIZE - 1u));

    if (next == rb->tail) {
        rb->overflow++;                 /* 满了：丢新数据，但一定要记账，不能默默吞掉 */
        return;
    }
    rb->buf[rb->head] = byte;
    rb->head = next;
}

#endif /* __RINGBUF_H */
