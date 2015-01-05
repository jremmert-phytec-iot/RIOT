/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_kinetis_common_timer
 *
 * @{
 *
 * @file
 * @brief       Low-level timer driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Johann Fischer <j.fischer@phytec.de>
 *
 * @}
 */

#include <stdlib.h>

#include "cpu.h"
#include "board.h"
#include "sched.h"
#include "thread.h"
#include "periph_conf.h"
#include "periph/timer.h"

#if TIMER_0_EN

/** Type for timer state */
typedef struct {
    void (*cb)(int);
    uint32_t value;
} timer_conf_t;

/** Timer state memory */
static timer_conf_t config[TIMER_NUMOF];


int timer_init(tim_t dev, unsigned int ticks_per_us, void (*callback)(int))
{
    PIT_Type *timer;

    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            /* enable timer peripheral clock */
            TIMER_0_CLKEN();
            /* set timer's IRQ priority */
            NVIC_SetPriority(TIMER_0_IRQ_CHAN, TIMER_IRQ_PRIO);
            /* select timer, use channel 0 as prescaler */
            timer = TIMER_0_DEV;
            timer->MCR = PIT_MCR_FRZ_MASK;
            timer->CHANNEL[0].TCTRL = 0x0;
            timer->CHANNEL[1].TCTRL = 0x0;
            timer->CHANNEL[0].LDVAL = (TIMER_0_CLOCK / 1e6) / ticks_per_us;
            timer->CHANNEL[0].TCTRL = (PIT_TCTRL_TEN_MASK);

            timer->CHANNEL[1].LDVAL = TIMER_0_MAX_VALUE;
            timer->CHANNEL[1].TCTRL = (PIT_TCTRL_TIE_MASK | PIT_TCTRL_CHN_MASK);
            timer->CHANNEL[1].TCTRL |= (PIT_TCTRL_TEN_MASK);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            /* enable timer peripheral clock */
            TIMER_1_CLKEN();
            /* set timer's IRQ priority */
            NVIC_SetPriority(TIMER_1_IRQ_CHAN, TIMER_IRQ_PRIO);
            /* select timer, use channel 0 as prescaler */
            timer = TIMER_1_DEV;
            timer->MCR = 0x0;
            timer->CHANNEL[2].TCTRL = 0x0;
            timer->CHANNEL[3].TCTRL = 0x0;
            timer->CHANNEL[2].LDVAL = TIMER_0_CLOCK / ticks_per_us;
            timer->CHANNEL[2].TCTRL = (PIT_TCTRL_TEN_MASK);

            timer->CHANNEL[3].TCTRL = (PIT_TCTRL_TIE_MASK | PIT_TCTRL_CHN_MASK);
            timer->CHANNEL[3].TCTRL |= (PIT_TCTRL_TEN_MASK);
            break;
#endif

        case TIMER_UNDEFINED:
        default:
            return -1;
    }

    /* set callback function */
    config[dev].cb = callback;
    config[dev].value = 0;

    /* enable the timer's interrupt */
    timer_irq_enable(dev);

    /* start the timer */
    timer_start(dev);

    return 0;
}

int timer_set(tim_t dev, int channel, unsigned int timeout)
{
    int now = timer_read(dev);
    return timer_set_absolute(dev, channel, now + timeout - 1);
    //return timer_set_absolute(dev, channel, timeout);
}

int timer_set_absolute(tim_t dev, int channel, unsigned int value)
{
    config[dev].value = value;

    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            TIMER_0_DEV->CHANNEL[1].TCTRL &= ~PIT_TCTRL_TEN_MASK;
            TIMER_0_DEV->CHANNEL[1].LDVAL = value;
            TIMER_0_DEV->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
            TIMER_0_DEV->CHANNEL[1].TCTRL |= (PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TIMER_1_DEV->CHANNEL[3].TCTRL &= ~PIT_TCTRL_TEN_MASK;
            TIMER_1_DEV->CHANNEL[3].LDVAL = value;
            TIMER_1_DEV->CHANNEL[3].TFLG = PIT_TFLG_TIF_MASK;
            TIMER_1_DEV->CHANNEL[3].TCTRL |= (PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK);
            break;
#endif

        case TIMER_UNDEFINED:
        default:
            return -1;
    }

    return 0;
}

int timer_clear(tim_t dev, int channel)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            config[dev].value = 0;
            TIMER_0_DEV->CHANNEL[1].TCTRL |= (1 << PIT_TCTRL_TIE_SHIFT);
            TIMER_0_DEV->CHANNEL[1].TFLG = (1 << PIT_TFLG_TIF_SHIFT);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TIMER_1_DEV->CHANNEL[3].TFLG = (1 << PIT_TFLG_TIF_SHIFT);
            TIMER_1_DEV->CHANNEL[3].TCTRL |= (1 << PIT_TCTRL_TIE_SHIFT);
            break;
#endif

        case TIMER_UNDEFINED:
        default:
            return -1;
    }

    return 0;
}

unsigned int timer_read(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            return config[dev].value - TIMER_0_DEV->CHANNEL[1].CVAL;
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            return TIMER_1_MAX_VALUE - TIMER_1_DEV->CHANNEL[3].CVAL;
            break;
#endif

        case TIMER_UNDEFINED:
        default:
            return 0;
    }
}

void timer_start(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            TIMER_0_DEV->CHANNEL[1].TCTRL |= (1 << PIT_TCTRL_TEN_SHIFT);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TIMER_0_DEV->CHANNEL[3].TCTRL |= (1 << PIT_TCTRL_TEN_SHIFT);
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }
}

void timer_stop(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            TIMER_0_DEV->CHANNEL[1].TCTRL &= ~(1 << PIT_TCTRL_TEN_SHIFT);
            config[dev].value = 0;
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TIMER_0_DEV->CHANNEL[3].TCTRL &= ~(1 << PIT_TCTRL_TEN_SHIFT);
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }
}

void timer_irq_enable(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            NVIC_EnableIRQ(TIMER_0_IRQ_CHAN);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            NVIC_EnableIRQ(TIMER_1_IRQ_CHAN);
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }
}

void timer_irq_disable(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            NVIC_DisableIRQ(TIMER_0_IRQ_CHAN);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            NVIC_DisableIRQ(TIMER_1_IRQ_CHAN);
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }
}

void timer_reset(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            config[dev].value = 0;
            TIMER_0_DEV->CHANNEL[1].LDVAL = TIMER_0_MAX_VALUE;
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TIMER_0_DEV->CHANNEL[3].LDVAL = TIMER_0_MAX_VALUE;
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }
}

void TIMER_0_ISR(void)
{
    TIMER_0_DEV->CHANNEL[1].TFLG = (1 << PIT_TFLG_TIF_SHIFT);
    config[0].cb(0);

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void TIMER_1_ISR(void)
{
    TIMER_1_DEV->CHANNEL[3].TFLG = (1 << PIT_TFLG_TIF_SHIFT);

    //config[TIMER_1].cb(0);
    if (sched_context_switch_request) {
        thread_yield();
    }
}
#endif
