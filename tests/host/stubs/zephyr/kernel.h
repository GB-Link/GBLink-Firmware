// Host stand-ins for the few Zephyr kernel calls the packet layer and the USB
// section use, over pthreads. Only what handshakeTest.cpp needs; not a port.
#pragma once

#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <errno.h>

typedef struct { int64_t ms; } k_timeout_t;
#define K_FOREVER ((k_timeout_t){ -1 })
#define K_NO_WAIT ((k_timeout_t){ 0 })
#define K_MSEC(n) ((k_timeout_t){ (n) })

typedef long atomic_t;

#ifdef __cplusplus
extern "C" {
#endif
unsigned int irq_lock(void);
void irq_unlock(unsigned int key);
int32_t k_sleep(k_timeout_t t);
#ifdef __cplusplus
}
#endif

struct k_sem { pthread_mutex_t m; pthread_cond_t c; unsigned count, limit; };

static inline int k_sem_init(struct k_sem* s, unsigned initial, unsigned limit)
{
    pthread_mutex_init(&s->m, nullptr);
    pthread_cond_init(&s->c, nullptr);
    s->count = initial;
    s->limit = limit;
    return 0;
}

static inline void k_sem_give(struct k_sem* s)
{
    pthread_mutex_lock(&s->m);
    if (s->count < s->limit) s->count++;
    pthread_cond_signal(&s->c);
    pthread_mutex_unlock(&s->m);
}

static inline int k_sem_take(struct k_sem* s, k_timeout_t t)
{
    pthread_mutex_lock(&s->m);
    if (t.ms < 0)
    {
        while (s->count == 0) pthread_cond_wait(&s->c, &s->m);
    }
    else
    {
        struct timespec end;
        clock_gettime(CLOCK_REALTIME, &end);
        end.tv_sec += t.ms / 1000;
        end.tv_nsec += (t.ms % 1000) * 1000000L;
        if (end.tv_nsec >= 1000000000L) { end.tv_sec++; end.tv_nsec -= 1000000000L; }
        while (s->count == 0)
        {
            if (pthread_cond_timedwait(&s->c, &s->m, &end) == ETIMEDOUT) break;
        }
    }
    int ret = s->count ? 0 : -EAGAIN;
    if (s->count) s->count--;
    pthread_mutex_unlock(&s->m);
    return ret;
}

struct k_timer { int unused; };

// A message queue of fixed-size items, enough for usbLinkCommand.cpp.
struct k_msgq { pthread_mutex_t m; size_t size, max, head, count; uint8_t* buf; };

#define K_MSGQ_DEFINE(name, msg_size, max_msgs, align) \
    static uint8_t name##_buf[(msg_size) * (max_msgs)]; \
    struct k_msgq name = { PTHREAD_MUTEX_INITIALIZER, (msg_size), (max_msgs), 0, 0, name##_buf }

static inline int k_msgq_put(struct k_msgq* q, const void* data, k_timeout_t)
{
    pthread_mutex_lock(&q->m);
    int ret = -ENOMSG;
    if (q->count < q->max)
    {
        memcpy(q->buf + ((q->head + q->count) % q->max) * q->size, data, q->size);
        q->count++;
        ret = 0;
    }
    pthread_mutex_unlock(&q->m);
    return ret;
}

static inline int k_msgq_get(struct k_msgq* q, void* data, k_timeout_t)
{
    pthread_mutex_lock(&q->m);
    int ret = -ENOMSG;
    if (q->count)
    {
        memcpy(data, q->buf + q->head * q->size, q->size);
        q->head = (q->head + 1) % q->max;
        q->count--;
        ret = 0;
    }
    pthread_mutex_unlock(&q->m);
    return ret;
}

static inline void k_msgq_purge(struct k_msgq* q)
{
    pthread_mutex_lock(&q->m);
    q->head = q->count = 0;
    pthread_mutex_unlock(&q->m);
}
