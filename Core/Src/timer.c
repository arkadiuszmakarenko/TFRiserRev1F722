/*
 * timer.c
 * 
 * Deadline-based timer callbacks.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

/* TIM4: IRQ 30. */
//void IRQ_30(void) __attribute__((alias("IRQ_timer")));
//#define TIMER_IRQ 30
//#define tim tim4

#include "stm32f7xx_hal.h"
#include "util.h"
#include "timer.h"


TIM_HandleTypeDef htim10;


/* IRQ only on counter overflow, one-time enable. */
#define TIM_CR1 (TIM_CR1_URS | TIM_CR1_OPM)
#define SYSCLK_MHZ 216
#define STK_MHZ (SYSCLK_MHZ / 8)
#define TIME_MHZ STK_MHZ
#define SYSCLK     (SYSCLK_MHZ * 1000000)
#define sysclk_ns(x) (((x) * SYSCLK_MHZ) / 1000)
#define sysclk_us(x) ((x) * SYSCLK_MHZ)
#define sysclk_ms(x) ((x) * SYSCLK_MHZ * 1000)
#define sysclk_stk(x) ((x) * (SYSCLK_MHZ / STK_MHZ))

#define TIME_MHZ STK_MHZ
#define time_us(x) stk_us(x)
#define time_ms(x) stk_ms(x)
#define time_sysclk(x) stk_sysclk(x)
#define sysclk_time(x) sysclk_stk(x)

#define stk_us(x) ((x) * STK_MHZ)
#define stk_ms(x) stk_us((x) * 1000)
#define stk_sysclk(x) ((x) / (SYSCLK_MHZ / STK_MHZ))
#define time_diff(x,y) ((int32_t)((y)-(x))) /* d = y - x */
#define time_add(x,d)  ((time_t)((x)+(d)))  /* y = x + d */
#define time_sub(x,d)  ((time_t)((x)-(d)))  /* y = x - d */
#define time_since(x)  time_diff(x, time_now())

#define stk_now() HAL_GetTick()

/* Empirically-determined offset applied to timer deadlines to counteract the
 g* latency incurred by reprogram_timer() and IRQ_timer(). */
#define SLACK_TICKS 12

#define TIMER_INACTIVE ((struct timer *)1ul)

static struct timer *head;

static volatile time_t time_stamp;
static struct timer time_stamp_timer;

time_t time_now(void)
{
    time_t s, t;
    s = time_stamp;
    t = stk_now() | (s & (0xff << 24));
    if (t > s)
        t -= 1u << 24;
    return ~t;
}


static void reprogram_timer(int32_t delta)
{
	TIM10->CR1 = TIM_CR1;
    if (delta < 0x10000) {
        /* Fine-grained deadline (sub-microsecond accurate) */
        TIM10->PSC = SYSCLK_MHZ/TIME_MHZ-1;
        TIM10->ARR = (delta <= SLACK_TICKS) ? 1 : delta-SLACK_TICKS;
    } else {
        /* Coarse-grained deadline, fires in time to set a shorter,
         * fine-grained deadline. */
        TIM10->PSC = sysclk_us(100)-1;
        TIM10->ARR = min_t(uint32_t, 0xffffu,
                         delta/time_us(100)-50); /* 5ms early */
    }
    TIM10->EGR = TIM_EGR_UG; /* update CNT, PSC, ARR */
    TIM10->SR = 0; /* dummy write, gives hardware time to process EGR.UG=1 */
    TIM10->CR1 = TIM_CR1 | TIM_CR1_CEN;
}

void timer_init(struct timer *timer, void (*cb_fn)(void *), void *cb_dat)
{
    timer->cb_fn = cb_fn;
    timer->cb_dat = cb_dat;
    timer->next = TIMER_INACTIVE;

}


static bool_t timer_is_active(struct timer *timer)
{
    return timer->next != TIMER_INACTIVE;
}

static void _timer_cancel(struct timer *timer)
{
    struct timer *t, **pprev;

    if (!timer_is_active(timer))
        return;

    for (pprev = &head; (t = *pprev) != timer; pprev = &t->next)
        continue;

    *pprev = t->next;
    t->next = TIMER_INACTIVE;

}

void timer_set(struct timer *timer, time_t deadline)
{
    struct timer *t, **pprev;
    time_t now;
    int32_t delta;
  //  uint32_t oldpri;

   // oldpri = IRQ_save(TIMER_IRQ_PRI);

    _timer_cancel(timer);

    timer->deadline = deadline;

    now = time_now();
    delta = time_diff(now, deadline);
    for (pprev = &head; (t = *pprev) != NULL; pprev = &t->next)
        if (delta <= time_diff(now, t->deadline))
            break;
    timer->next = *pprev;
    *pprev = timer;

    if (head == timer)
        reprogram_timer(delta);

  //  IRQ_restore(oldpri);

}

void timer_cancel(struct timer *timer)
{
	_timer_cancel(timer);
}

void timers_init()
{
	htim10.Instance->CR2 = 0;
	htim10.Instance->DIER = TIM_DIER_UIE;


}

static void IRQ_timer()
{
    struct timer *t;
    int32_t delta;

    TIM10->SR = 0;

    while ((t = head) != NULL) {
        if ((delta = time_diff(time_now(), t->deadline)) > SLACK_TICKS) {
            reprogram_timer(delta);
            break;
        }
        head = t->next;
        t->next = TIMER_INACTIVE;
        (*t->cb_fn)(t->cb_dat);
    }

}

void irq_call()
{
	IRQ_timer();
}


