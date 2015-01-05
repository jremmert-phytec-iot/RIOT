/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_kinetis_common
 * @{
 *
 * @file
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Hauke Petersen <mail@haukepetersen.de>
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 *
 * @}
 */

#include "cpu.h"
#include "sched.h"
#include "thread.h"
#include "periph/gpio.h"
#include "periph_conf.h"

#if GPIO_NUMOF

typedef struct {
    gpio_cb_t cb;       /**< callback called from GPIO interrupt */
    void *arg;          /**< argument passed to the callback */
} gpio_state_t;

/**
 * @brief Unified IRQ handler shared by all interrupt routines
 *
 * @param[in] dev   the device that triggered the interrupt
 */
static inline void irq_handler(gpio_t dev);

/**
 * @brief Hold one callback function pointer for each gpio device
 */
static gpio_state_t config[GPIO_NUMOF];

int gpio_init_out(gpio_t dev, gpio_pp_t pushpull)
{
    PORT_Type *port;
    GPIO_Type *gpio;
    uint32_t pin = 0;

    switch (dev) {
#if GPIO_0_EN

        case GPIO_0:
            GPIO_0_CLKEN();
            port = GPIO_0_PORT;
            gpio = GPIO_0_DEV;
            pin = GPIO_0_PIN;
            break;
#endif
#if GPIO_1_EN

        case GPIO_1:
            GPIO_1_CLKEN();
            port = GPIO_1_PORT;
            gpio = GPIO_1_DEV;
            pin = GPIO_1_PIN;
            break;
#endif
#if GPIO_2_EN

        case GPIO_2:
            GPIO_2_CLKEN();
            port = GPIO_2_PORT;
            gpio = GPIO_2_DEV;
            pin = GPIO_2_PIN;
            break;
#endif
#if GPIO_3_EN

        case GPIO_3:
            GPIO_3_CLKEN();
            port = GPIO_3_PORT;
            gpio = GPIO_3_DEV;
            pin = GPIO_3_PIN;
            break;
#endif
#if GPIO_4_EN

        case GPIO_4:
            GPIO_4_CLKEN();
            port = GPIO_4_PORT;
            gpio = GPIO_4_DEV;
            pin = GPIO_4_PIN;
            break;
#endif
#if GPIO_5_EN

        case GPIO_5:
            GPIO_5_CLKEN();
            port = GPIO_5_PORT;
            gpio = GPIO_5_DEV;
            pin = GPIO_5_PIN;
            break;
#endif
#if GPIO_6_EN

        case GPIO_6:
            GPIO_6_CLKEN();
            port = GPIO_6_PORT;
            gpio = GPIO_6_DEV;
            pin = GPIO_6_PIN;
            break;
#endif
#if GPIO_7_EN

        case GPIO_7:
            GPIO_7_CLKEN();
            port = GPIO_7_PORT;
            gpio = GPIO_7_DEV;
            pin = GPIO_7_PIN;
            break;
#endif
#if GPIO_8_EN

        case GPIO_8:
            GPIO_8_CLKEN();
            port = GPIO_8_PORT;
            gpio = GPIO_8_DEV;
            pin = GPIO_8_PIN;
            break;
#endif
#if GPIO_9_EN

        case GPIO_9:
            GPIO_9_CLKEN();
            port = GPIO_9_PORT;
            gpio = GPIO_9_DEV;
            pin = GPIO_9_PIN;
            break;
#endif
#if GPIO_10_EN

        case GPIO_10:
            GPIO_10_CLKEN();
            port = GPIO_10_PORT;
            gpio = GPIO_10_DEV;
            pin = GPIO_10_PIN;
            break;
#endif
#if GPIO_11_EN

        case GPIO_11:
            GPIO_11_CLKEN();
            port = GPIO_11_PORT;
            gpio = GPIO_11_DEV;
            pin = GPIO_11_PIN;
            break;
#endif
#if GPIO_12_EN

        case GPIO_12:
            GPIO_12_CLKEN();
            port = GPIO_12_PORT;
            gpio = GPIO_12_DEV;
            pin = GPIO_12_PIN;
            break;
#endif
#if GPIO_13_EN

        case GPIO_13:
            GPIO_13_CLKEN();
            port = GPIO_13_PORT;
            gpio = GPIO_13_DEV;
            pin = GPIO_13_PIN;
            break;
#endif
#if GPIO_14_EN

        case GPIO_14:
            GPIO_14_CLKEN();
            port = GPIO_14_PORT;
            gpio = GPIO_14_DEV;
            pin = GPIO_14_PIN;
            break;
#endif
#if GPIO_15_EN

        case GPIO_15:
            GPIO_15_CLKEN();
            port = GPIO_15_PORT;
            gpio = GPIO_15_DEV;
            pin = GPIO_15_PIN;
            break;
#endif
#if GPIO_16_EN

        case GPIO_16:
            GPIO_16_CLKEN();
            port = GPIO_16_PORT;
            gpio = GPIO_16_DEV;
            pin = GPIO_16_PIN;
            break;
#endif
#if GPIO_17_EN

        case GPIO_17:
            GPIO_17_CLKEN();
            port = GPIO_17_PORT;
            gpio = GPIO_17_DEV;
            pin = GPIO_17_PIN;
            break;
#endif
#if GPIO_18_EN

        case GPIO_18:
            GPIO_18_CLKEN();
            port = GPIO_18_PORT;
            gpio = GPIO_18_DEV;
            pin = GPIO_18_PIN;
            break;
#endif
#if GPIO_19_EN

        case GPIO_19:
            GPIO_19_CLKEN();
            port = GPIO_19_PORT;
            gpio = GPIO_19_DEV;
            pin = GPIO_19_PIN;
            break;
#endif
#if GPIO_20_EN

        case GPIO_20:
            GPIO_20_CLKEN();
            port = GPIO_20_PORT;
            gpio = GPIO_20_DEV;
            pin = GPIO_20_PIN;
            break;
#endif
#if GPIO_21_EN

        case GPIO_21:
            GPIO_21_CLKEN();
            port = GPIO_21_PORT;
            gpio = GPIO_21_DEV;
            pin = GPIO_21_PIN;
            break;
#endif
#if GPIO_22_EN

        case GPIO_22:
            GPIO_22_CLKEN();
            port = GPIO_22_PORT;
            gpio = GPIO_22_DEV;
            pin = GPIO_22_PIN;
            break;
#endif
#if GPIO_23_EN

        case GPIO_23:
            GPIO_23_CLKEN();
            port = GPIO_23_PORT;
            gpio = GPIO_23_DEV;
            pin = GPIO_23_PIN;
            break;
#endif
#if GPIO_24_EN

        case GPIO_24:
            GPIO_24_CLKEN();
            port = GPIO_24_PORT;
            gpio = GPIO_24_DEV;
            pin = GPIO_24_PIN;
            break;
#endif
#if GPIO_25_EN

        case GPIO_25:
            GPIO_25_CLKEN();
            port = GPIO_25_PORT;
            gpio = GPIO_25_DEV;
            pin = GPIO_25_PIN;
            break;
#endif
#if GPIO_26_EN

        case GPIO_26:
            GPIO_26_CLKEN();
            port = GPIO_26_PORT;
            gpio = GPIO_26_DEV;
            pin = GPIO_26_PIN;
            break;
#endif
#if GPIO_27_EN

        case GPIO_27:
            GPIO_27_CLKEN();
            port = GPIO_27_PORT;
            gpio = GPIO_27_DEV;
            pin = GPIO_27_PIN;
            break;
#endif
#if GPIO_28_EN

        case GPIO_28:
            GPIO_28_CLKEN();
            port = GPIO_28_PORT;
            gpio = GPIO_28_DEV;
            pin = GPIO_28_PIN;
            break;
#endif
#if GPIO_29_EN

        case GPIO_29:
            GPIO_29_CLKEN();
            port = GPIO_29_PORT;
            gpio = GPIO_29_DEV;
            pin = GPIO_29_PIN;
            break;
#endif
#if GPIO_30_EN

        case GPIO_30:
            GPIO_30_CLKEN();
            port = GPIO_30_PORT;
            gpio = GPIO_30_DEV;
            pin = GPIO_30_PIN;
            break;
#endif
#if GPIO_31_EN

        case GPIO_31:
            GPIO_31_CLKEN();
            port = GPIO_31_PORT;
            gpio = GPIO_31_DEV;
            pin = GPIO_31_PIN;
            break;
#endif

        default:
            return -1;
    }

    port->PCR[pin] = PORT_PCR_MUX(1);

    /* set to push-pull configuration */
    switch (pushpull) {
        case GPIO_PULLUP:
            port->PCR[pin] |= (1 << PORT_PCR_PE_SHIFT | 1 << PORT_PCR_PS_SHIFT);
            break;

        case GPIO_PULLDOWN:
            port->PCR[pin] |= (1 << PORT_PCR_PE_SHIFT);
            break;

        default:
            break;
    }

    gpio->PDDR |= (1 << pin);
    gpio->PCOR |= (1 << pin);

    return 0;
}

int gpio_init_in(gpio_t dev, gpio_pp_t pushpull)
{
    PORT_Type *port;
    GPIO_Type *gpio;
    uint32_t pin = 0;

    switch (dev) {
#if GPIO_0_EN

        case GPIO_0:
            GPIO_0_CLKEN();
            port = GPIO_0_PORT;
            gpio = GPIO_0_DEV;
            pin = GPIO_0_PIN;
            break;
#endif
#if GPIO_1_EN

        case GPIO_1:
            GPIO_1_CLKEN();
            port = GPIO_1_PORT;
            gpio = GPIO_1_DEV;
            pin = GPIO_1_PIN;
            break;
#endif
#if GPIO_2_EN

        case GPIO_2:
            GPIO_2_CLKEN();
            port = GPIO_2_PORT;
            gpio = GPIO_2_DEV;
            pin = GPIO_2_PIN;
            break;
#endif
#if GPIO_3_EN

        case GPIO_3:
            GPIO_3_CLKEN();
            port = GPIO_3_PORT;
            gpio = GPIO_3_DEV;
            pin = GPIO_3_PIN;
            break;
#endif
#if GPIO_4_EN

        case GPIO_4:
            GPIO_4_CLKEN();
            port = GPIO_4_PORT;
            gpio = GPIO_4_DEV;
            pin = GPIO_4_PIN;
            break;
#endif
#if GPIO_5_EN

        case GPIO_5:
            GPIO_5_CLKEN();
            port = GPIO_5_PORT;
            gpio = GPIO_5_DEV;
            pin = GPIO_5_PIN;
            break;
#endif
#if GPIO_6_EN

        case GPIO_6:
            GPIO_6_CLKEN();
            port = GPIO_6_PORT;
            gpio = GPIO_6_DEV;
            pin = GPIO_6_PIN;
            break;
#endif
#if GPIO_7_EN

        case GPIO_7:
            GPIO_7_CLKEN();
            port = GPIO_7_PORT;
            gpio = GPIO_7_DEV;
            pin = GPIO_7_PIN;
            break;
#endif
#if GPIO_8_EN

        case GPIO_8:
            GPIO_8_CLKEN();
            port = GPIO_8_PORT;
            gpio = GPIO_8_DEV;
            pin = GPIO_8_PIN;
            break;
#endif
#if GPIO_9_EN

        case GPIO_9:
            GPIO_9_CLKEN();
            port = GPIO_9_PORT;
            gpio = GPIO_9_DEV;
            pin = GPIO_9_PIN;
            break;
#endif
#if GPIO_10_EN

        case GPIO_10:
            GPIO_10_CLKEN();
            port = GPIO_10_PORT;
            gpio = GPIO_10_DEV;
            pin = GPIO_10_PIN;
            break;
#endif
#if GPIO_11_EN

        case GPIO_11:
            GPIO_11_CLKEN();
            port = GPIO_11_PORT;
            gpio = GPIO_11_DEV;
            pin = GPIO_11_PIN;
            break;
#endif
#if GPIO_12_EN

        case GPIO_12:
            GPIO_12_CLKEN();
            port = GPIO_12_PORT;
            gpio = GPIO_12_DEV;
            pin = GPIO_12_PIN;
            break;
#endif
#if GPIO_13_EN

        case GPIO_13:
            GPIO_13_CLKEN();
            port = GPIO_13_PORT;
            gpio = GPIO_13_DEV;
            pin = GPIO_13_PIN;
            break;
#endif
#if GPIO_14_EN

        case GPIO_14:
            GPIO_14_CLKEN();
            port = GPIO_14_PORT;
            gpio = GPIO_14_DEV;
            pin = GPIO_14_PIN;
            break;
#endif
#if GPIO_15_EN

        case GPIO_15:
            GPIO_15_CLKEN();
            port = GPIO_15_PORT;
            gpio = GPIO_15_DEV;
            pin = GPIO_15_PIN;
            break;
#endif
#if GPIO_16_EN

        case GPIO_16:
            GPIO_16_CLKEN();
            port = GPIO_16_PORT;
            gpio = GPIO_16_DEV;
            pin = GPIO_16_PIN;
            break;
#endif
#if GPIO_17_EN

        case GPIO_17:
            GPIO_17_CLKEN();
            port = GPIO_17_PORT;
            gpio = GPIO_17_DEV;
            pin = GPIO_17_PIN;
            break;
#endif
#if GPIO_18_EN

        case GPIO_18:
            GPIO_18_CLKEN();
            port = GPIO_18_PORT;
            gpio = GPIO_18_DEV;
            pin = GPIO_18_PIN;
            break;
#endif
#if GPIO_19_EN

        case GPIO_19:
            GPIO_19_CLKEN();
            port = GPIO_19_PORT;
            gpio = GPIO_19_DEV;
            pin = GPIO_19_PIN;
            break;
#endif
#if GPIO_20_EN

        case GPIO_20:
            GPIO_20_CLKEN();
            port = GPIO_20_PORT;
            gpio = GPIO_20_DEV;
            pin = GPIO_20_PIN;
            break;
#endif
#if GPIO_21_EN

        case GPIO_21:
            GPIO_21_CLKEN();
            port = GPIO_21_PORT;
            gpio = GPIO_21_DEV;
            pin = GPIO_21_PIN;
            break;
#endif
#if GPIO_22_EN

        case GPIO_22:
            GPIO_22_CLKEN();
            port = GPIO_22_PORT;
            gpio = GPIO_22_DEV;
            pin = GPIO_22_PIN;
            break;
#endif
#if GPIO_23_EN

        case GPIO_23:
            GPIO_23_CLKEN();
            port = GPIO_23_PORT;
            gpio = GPIO_23_DEV;
            pin = GPIO_23_PIN;
            break;
#endif
#if GPIO_24_EN

        case GPIO_24:
            GPIO_24_CLKEN();
            port = GPIO_24_PORT;
            gpio = GPIO_24_DEV;
            pin = GPIO_24_PIN;
            break;
#endif
#if GPIO_25_EN

        case GPIO_25:
            GPIO_25_CLKEN();
            port = GPIO_25_PORT;
            gpio = GPIO_25_DEV;
            pin = GPIO_25_PIN;
            break;
#endif
#if GPIO_26_EN

        case GPIO_26:
            GPIO_26_CLKEN();
            port = GPIO_26_PORT;
            gpio = GPIO_26_DEV;
            pin = GPIO_26_PIN;
            break;
#endif
#if GPIO_27_EN

        case GPIO_27:
            GPIO_27_CLKEN();
            port = GPIO_27_PORT;
            gpio = GPIO_27_DEV;
            pin = GPIO_27_PIN;
            break;
#endif
#if GPIO_28_EN

        case GPIO_28:
            GPIO_28_CLKEN();
            port = GPIO_28_PORT;
            gpio = GPIO_28_DEV;
            pin = GPIO_28_PIN;
            break;
#endif
#if GPIO_29_EN

        case GPIO_29:
            GPIO_29_CLKEN();
            port = GPIO_29_PORT;
            gpio = GPIO_29_DEV;
            pin = GPIO_29_PIN;
            break;
#endif
#if GPIO_30_EN

        case GPIO_30:
            GPIO_30_CLKEN();
            port = GPIO_30_PORT;
            gpio = GPIO_30_DEV;
            pin = GPIO_30_PIN;
            break;
#endif
#if GPIO_31_EN

        case GPIO_31:
            GPIO_31_CLKEN();
            port = GPIO_31_PORT;
            gpio = GPIO_31_DEV;
            pin = GPIO_31_PIN;
            break;
#endif

        default:
            return -1;
    }

    port->PCR[pin] = PORT_PCR_MUX(1);

    /* set to push-pull configuration */
    switch (pushpull) {
        case GPIO_PULLUP:
            port->PCR[pin] |= (1 << PORT_PCR_PE_SHIFT | 1 << PORT_PCR_PS_SHIFT);
            break;

        case GPIO_PULLDOWN:
            port->PCR[pin] |= (1 << PORT_PCR_PE_SHIFT);
            break;

        default:
            break;
    }

    gpio->PDDR &= ~(1 << pin);

    return 0;
}

int gpio_init_int(gpio_t dev, gpio_pp_t pushpull, gpio_flank_t flank, gpio_cb_t cb, void *arg)
{
    PORT_Type *port;
    int res;
    uint32_t pin = 0;

    res = gpio_init_in(dev, pushpull);

    if (res < 0) {
        return res;
    }

    switch (dev) {
#if GPIO_0_EN

        case GPIO_0:
            port = GPIO_0_PORT;
            pin = GPIO_0_PIN;
            NVIC_SetPriority(GPIO_0_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_0_IRQ);
            break;
#endif
#if GPIO_1_EN

        case GPIO_1:
            port = GPIO_1_PORT;
            pin = GPIO_1_PIN;
            NVIC_SetPriority(GPIO_1_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_1_IRQ);
            break;
#endif
#if GPIO_2_EN

        case GPIO_2:
            port = GPIO_2_PORT;
            pin = GPIO_2_PIN;
            NVIC_SetPriority(GPIO_2_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_2_IRQ);
            break;
#endif
#if GPIO_3_EN

        case GPIO_3:
            port = GPIO_3_PORT;
            pin = GPIO_3_PIN;
            NVIC_SetPriority(GPIO_3_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_3_IRQ);
            break;
#endif
#if GPIO_4_EN

        case GPIO_4:
            port = GPIO_4_PORT;
            pin = GPIO_4_PIN;
            NVIC_SetPriority(GPIO_4_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_4_IRQ);
            break;
#endif
#if GPIO_5_EN

        case GPIO_5:
            port = GPIO_5_PORT;
            pin = GPIO_5_PIN;
            NVIC_SetPriority(GPIO_5_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_5_IRQ);
            break;
#endif
#if GPIO_6_EN

        case GPIO_6:
            port = GPIO_6_PORT;
            pin = GPIO_6_PIN;
            NVIC_SetPriority(GPIO_6_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_6_IRQ);
            break;
#endif
#if GPIO_7_EN

        case GPIO_7:
            port = GPIO_7_PORT;
            pin = GPIO_7_PIN;
            NVIC_SetPriority(GPIO_7_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_7_IRQ);
            break;
#endif
#if GPIO_8_EN

        case GPIO_8:
            port = GPIO_8_PORT;
            pin = GPIO_8_PIN;
            NVIC_SetPriority(GPIO_8_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_8_IRQ);
            break;
#endif
#if GPIO_9_EN

        case GPIO_9:
            port = GPIO_9_PORT;
            pin = GPIO_9_PIN;
            NVIC_SetPriority(GPIO_9_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_9_IRQ);
            break;
#endif
#if GPIO_10_EN

        case GPIO_10:
            port = GPIO_10_PORT;
            pin = GPIO_10_PIN;
            NVIC_SetPriority(GPIO_10_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_10_IRQ);
            break;
#endif
#if GPIO_11_EN

        case GPIO_11:
            port = GPIO_11_PORT;
            pin = GPIO_11_PIN;
            NVIC_SetPriority(GPIO_11_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_11_IRQ);
            break;
#endif
#if GPIO_12_EN

        case GPIO_12:
            port = GPIO_12_PORT;
            pin = GPIO_12_PIN;
            NVIC_SetPriority(GPIO_12_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_12_IRQ);
            break;
#endif
#if GPIO_13_EN

        case GPIO_13:
            port = GPIO_13_PORT;
            pin = GPIO_13_PIN;
            NVIC_SetPriority(GPIO_13_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_13_IRQ);
            break;
#endif
#if GPIO_14_EN

        case GPIO_14:
            port = GPIO_14_PORT;
            pin = GPIO_14_PIN;
            NVIC_SetPriority(GPIO_14_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_14_IRQ);
            break;
#endif
#if GPIO_15_EN

        case GPIO_15:
            port = GPIO_15_PORT;
            pin = GPIO_15_PIN;
            NVIC_SetPriority(GPIO_15_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_15_IRQ);
            break;
#endif
#if GPIO_16_EN

        case GPIO_16:
            port = GPIO_16_PORT;
            pin = GPIO_16_PIN;
            NVIC_SetPriority(GPIO_16_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_16_IRQ);
            break;
#endif
#if GPIO_17_EN

        case GPIO_17:
            port = GPIO_17_PORT;
            pin = GPIO_17_PIN;
            NVIC_SetPriority(GPIO_17_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_17_IRQ);
            break;
#endif
#if GPIO_18_EN

        case GPIO_18:
            port = GPIO_18_PORT;
            pin = GPIO_18_PIN;
            NVIC_SetPriority(GPIO_18_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_18_IRQ);
            break;
#endif
#if GPIO_19_EN

        case GPIO_19:
            port = GPIO_19_PORT;
            pin = GPIO_19_PIN;
            NVIC_SetPriority(GPIO_19_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_19_IRQ);
            break;
#endif
#if GPIO_20_EN

        case GPIO_20:
            port = GPIO_20_PORT;
            pin = GPIO_20_PIN;
            NVIC_SetPriority(GPIO_20_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_20_IRQ);
            break;
#endif
#if GPIO_21_EN

        case GPIO_21:
            port = GPIO_21_PORT;
            pin = GPIO_21_PIN;
            NVIC_SetPriority(GPIO_21_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_21_IRQ);
            break;
#endif
#if GPIO_22_EN

        case GPIO_22:
            port = GPIO_22_PORT;
            pin = GPIO_22_PIN;
            NVIC_SetPriority(GPIO_22_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_22_IRQ);
            break;
#endif
#if GPIO_23_EN

        case GPIO_23:
            port = GPIO_23_PORT;
            pin = GPIO_23_PIN;
            NVIC_SetPriority(GPIO_23_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_23_IRQ);
            break;
#endif
#if GPIO_24_EN

        case GPIO_24:
            port = GPIO_24_PORT;
            pin = GPIO_24_PIN;
            NVIC_SetPriority(GPIO_24_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_24_IRQ);
            break;
#endif
#if GPIO_25_EN

        case GPIO_25:
            port = GPIO_25_PORT;
            pin = GPIO_25_PIN;
            NVIC_SetPriority(GPIO_25_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_25_IRQ);
            break;
#endif
#if GPIO_26_EN

        case GPIO_26:
            port = GPIO_26_PORT;
            pin = GPIO_26_PIN;
            NVIC_SetPriority(GPIO_26_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_26_IRQ);
            break;
#endif
#if GPIO_27_EN

        case GPIO_27:
            port = GPIO_27_PORT;
            pin = GPIO_27_PIN;
            NVIC_SetPriority(GPIO_27_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_27_IRQ);
            break;
#endif
#if GPIO_28_EN

        case GPIO_28:
            port = GPIO_28_PORT;
            pin = GPIO_28_PIN;
            NVIC_SetPriority(GPIO_28_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_28_IRQ);
            break;
#endif
#if GPIO_29_EN

        case GPIO_29:
            port = GPIO_29_PORT;
            pin = GPIO_29_PIN;
            NVIC_SetPriority(GPIO_29_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_29_IRQ);
            break;
#endif
#if GPIO_30_EN

        case GPIO_30:
            port = GPIO_30_PORT;
            pin = GPIO_30_PIN;
            NVIC_SetPriority(GPIO_30_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_30_IRQ);
            break;
#endif
#if GPIO_31_EN

        case GPIO_31:
            port = GPIO_31_PORT;
            pin = GPIO_31_PIN;
            NVIC_SetPriority(GPIO_31_IRQ, GPIO_IRQ_PRIO);
            NVIC_EnableIRQ(GPIO_31_IRQ);
            break;
#endif

        default:
            return -1;
    }

    /* set callback */
    config[dev].cb = cb;
    config[dev].arg = arg;

    /* configure the active edges */
    switch (flank) {
        case GPIO_RISING:
            port->PCR[pin] |= PORT_PCR_IRQC(9);
            break;

        case GPIO_FALLING:
            port->PCR[pin] |= PORT_PCR_IRQC(10);
            break;

        case GPIO_BOTH:
            port->PCR[pin] |= PORT_PCR_IRQC(11);
            break;
    }

    return 0;
}

int gpio_read(gpio_t dev)
{
    GPIO_Type *gpio;
    uint32_t pin = 0;

    switch (dev) {
#if GPIO_0_EN

        case GPIO_0:
            gpio = GPIO_0_DEV;
            pin = GPIO_0_PIN;
            break;
#endif
#if GPIO_1_EN

        case GPIO_1:
            gpio = GPIO_1_DEV;
            pin = GPIO_1_PIN;
            break;
#endif
#if GPIO_2_EN

        case GPIO_2:
            gpio = GPIO_2_DEV;
            pin = GPIO_2_PIN;
            break;
#endif
#if GPIO_3_EN

        case GPIO_3:
            gpio = GPIO_3_DEV;
            pin = GPIO_3_PIN;
            break;
#endif
#if GPIO_4_EN

        case GPIO_4:
            gpio = GPIO_4_DEV;
            pin = GPIO_4_PIN;
            break;
#endif
#if GPIO_5_EN

        case GPIO_5:
            gpio = GPIO_5_DEV;
            pin = GPIO_5_PIN;
            break;
#endif
#if GPIO_6_EN

        case GPIO_6:
            gpio = GPIO_6_DEV;
            pin = GPIO_6_PIN;
            break;
#endif
#if GPIO_7_EN

        case GPIO_7:
            gpio = GPIO_7_DEV;
            pin = GPIO_7_PIN;
            break;
#endif
#if GPIO_8_EN

        case GPIO_8:
            gpio = GPIO_8_DEV;
            pin = GPIO_8_PIN;
            break;
#endif
#if GPIO_9_EN

        case GPIO_9:
            gpio = GPIO_9_DEV;
            pin = GPIO_9_PIN;
            break;
#endif
#if GPIO_10_EN

        case GPIO_10:
            gpio = GPIO_10_DEV;
            pin = GPIO_10_PIN;
            break;
#endif
#if GPIO_11_EN

        case GPIO_11:
            gpio = GPIO_11_DEV;
            pin = GPIO_11_PIN;
            break;
#endif
#if GPIO_12_EN

        case GPIO_12:
            gpio = GPIO_12_DEV;
            pin = GPIO_12_PIN;
            break;
#endif
#if GPIO_13_EN

        case GPIO_13:
            gpio = GPIO_13_DEV;
            pin = GPIO_13_PIN;
            break;
#endif
#if GPIO_14_EN

        case GPIO_14:
            gpio = GPIO_14_DEV;
            pin = GPIO_14_PIN;
            break;
#endif
#if GPIO_15_EN

        case GPIO_15:
            gpio = GPIO_15_DEV;
            pin = GPIO_15_PIN;
            break;
#endif
#if GPIO_16_EN

        case GPIO_16:
            gpio = GPIO_16_DEV;
            pin = GPIO_16_PIN;
            break;
#endif
#if GPIO_17_EN

        case GPIO_17:
            gpio = GPIO_17_DEV;
            pin = GPIO_17_PIN;
            break;
#endif
#if GPIO_18_EN

        case GPIO_18:
            gpio = GPIO_18_DEV;
            pin = GPIO_18_PIN;
            break;
#endif
#if GPIO_19_EN

        case GPIO_19:
            gpio = GPIO_19_DEV;
            pin = GPIO_19_PIN;
            break;
#endif
#if GPIO_20_EN

        case GPIO_20:
            gpio = GPIO_20_DEV;
            pin = GPIO_20_PIN;
            break;
#endif
#if GPIO_21_EN

        case GPIO_21:
            gpio = GPIO_21_DEV;
            pin = GPIO_21_PIN;
            break;
#endif
#if GPIO_22_EN

        case GPIO_22:
            gpio = GPIO_22_DEV;
            pin = GPIO_22_PIN;
            break;
#endif
#if GPIO_23_EN

        case GPIO_23:
            gpio = GPIO_23_DEV;
            pin = GPIO_23_PIN;
            break;
#endif
#if GPIO_24_EN

        case GPIO_24:
            gpio = GPIO_24_DEV;
            pin = GPIO_24_PIN;
            break;
#endif
#if GPIO_25_EN

        case GPIO_25:
            gpio = GPIO_25_DEV;
            pin = GPIO_25_PIN;
            break;
#endif
#if GPIO_26_EN

        case GPIO_26:
            gpio = GPIO_26_DEV;
            pin = GPIO_26_PIN;
            break;
#endif
#if GPIO_27_EN

        case GPIO_27:
            gpio = GPIO_27_DEV;
            pin = GPIO_27_PIN;
            break;
#endif
#if GPIO_28_EN

        case GPIO_28:
            gpio = GPIO_28_DEV;
            pin = GPIO_28_PIN;
            break;
#endif
#if GPIO_29_EN

        case GPIO_29:
            gpio = GPIO_29_DEV;
            pin = GPIO_29_PIN;
            break;
#endif
#if GPIO_30_EN

        case GPIO_30:
            gpio = GPIO_30_DEV;
            pin = GPIO_30_PIN;
            break;
#endif
#if GPIO_31_EN

        case GPIO_31:
            gpio = GPIO_31_DEV;
            pin = GPIO_31_PIN;
            break;
#endif

        default:
            return -1;
    }

    if (gpio->PDDR & (1 << pin)) {
        return gpio->PDOR & (1 << pin);          /* read output data register */
    }
    else {
        return gpio->PDIR & (1 << pin);          /* else read input data register */
    }
}

void gpio_set(gpio_t dev)
{
    switch (dev) {
#if GPIO_0_EN

        case GPIO_0:
            GPIO_0_DEV->PSOR |= (1 << GPIO_0_PIN);
            break;
#endif
#if GPIO_1_EN

        case GPIO_1:
            GPIO_1_DEV->PSOR |= (1 << GPIO_1_PIN);
            break;
#endif
#if GPIO_2_EN

        case GPIO_2:
            GPIO_2_DEV->PSOR |= (1 << GPIO_2_PIN);
            break;
#endif
#if GPIO_3_EN

        case GPIO_3:
            GPIO_3_DEV->PSOR |= (1 << GPIO_3_PIN);
            break;
#endif
#if GPIO_4_EN

        case GPIO_4:
            GPIO_4_DEV->PSOR |= (1 << GPIO_4_PIN);
            break;
#endif
#if GPIO_5_EN

        case GPIO_5:
            GPIO_5_DEV->PSOR |= (1 << GPIO_5_PIN);
            break;
#endif
#if GPIO_6_EN

        case GPIO_6:
            GPIO_6_DEV->PSOR |= (1 << GPIO_6_PIN);
            break;
#endif
#if GPIO_7_EN

        case GPIO_7:
            GPIO_7_DEV->PSOR |= (1 << GPIO_7_PIN);
            break;
#endif
#if GPIO_8_EN

        case GPIO_8:
            GPIO_8_DEV->PSOR |= (1 << GPIO_8_PIN);
            break;
#endif
#if GPIO_9_EN

        case GPIO_9:
            GPIO_9_DEV->PSOR |= (1 << GPIO_9_PIN);
            break;
#endif
#if GPIO_10_EN

        case GPIO_10:
            GPIO_10_DEV->PSOR |= (1 << GPIO_10_PIN);
            break;
#endif
#if GPIO_11_EN

        case GPIO_11:
            GPIO_11_DEV->PSOR |= (1 << GPIO_11_PIN);
            break;
#endif
#if GPIO_12_EN

        case GPIO_12:
            GPIO_12_DEV->PSOR |= (1 << GPIO_12_PIN);
            break;
#endif
#if GPIO_13_EN

        case GPIO_13:
            GPIO_13_DEV->PSOR |= (1 << GPIO_13_PIN);
            break;
#endif
#if GPIO_14_EN

        case GPIO_14:
            GPIO_14_DEV->PSOR |= (1 << GPIO_14_PIN);
            break;
#endif
#if GPIO_15_EN

        case GPIO_15:
            GPIO_15_DEV->PSOR |= (1 << GPIO_15_PIN);
            break;
#endif
#if GPIO_16_EN

        case GPIO_16:
            GPIO_16_DEV->PSOR |= (1 << GPIO_16_PIN);
            break;
#endif
#if GPIO_17_EN

        case GPIO_17:
            GPIO_17_DEV->PSOR |= (1 << GPIO_17_PIN);
            break;
#endif
#if GPIO_18_EN

        case GPIO_18:
            GPIO_18_DEV->PSOR |= (1 << GPIO_18_PIN);
            break;
#endif
#if GPIO_19_EN

        case GPIO_19:
            GPIO_19_DEV->PSOR |= (1 << GPIO_19_PIN);
            break;
#endif
#if GPIO_20_EN

        case GPIO_20:
            GPIO_20_DEV->PSOR |= (1 << GPIO_20_PIN);
            break;
#endif
#if GPIO_21_EN

        case GPIO_21:
            GPIO_21_DEV->PSOR |= (1 << GPIO_21_PIN);
            break;
#endif
#if GPIO_22_EN

        case GPIO_22:
            GPIO_22_DEV->PSOR |= (1 << GPIO_22_PIN);
            break;
#endif
#if GPIO_23_EN

        case GPIO_23:
            GPIO_23_DEV->PSOR |= (1 << GPIO_23_PIN);
            break;
#endif
#if GPIO_24_EN

        case GPIO_24:
            GPIO_24_DEV->PSOR |= (1 << GPIO_24_PIN);
            break;
#endif
#if GPIO_25_EN

        case GPIO_25:
            GPIO_25_DEV->PSOR |= (1 << GPIO_25_PIN);
            break;
#endif
#if GPIO_26_EN

        case GPIO_26:
            GPIO_26_DEV->PSOR |= (1 << GPIO_26_PIN);
            break;
#endif
#if GPIO_27_EN

        case GPIO_27:
            GPIO_27_DEV->PSOR |= (1 << GPIO_27_PIN);
            break;
#endif
#if GPIO_28_EN

        case GPIO_28:
            GPIO_28_DEV->PSOR |= (1 << GPIO_28_PIN);
            break;
#endif
#if GPIO_29_EN

        case GPIO_29:
            GPIO_29_DEV->PSOR |= (1 << GPIO_29_PIN);
            break;
#endif
#if GPIO_30_EN

        case GPIO_30:
            GPIO_30_DEV->PSOR |= (1 << GPIO_30_PIN);
            break;
#endif
#if GPIO_31_EN

        case GPIO_31:
            GPIO_31_DEV->PSOR |= (1 << GPIO_31_PIN);
            break;
#endif
    }
}

void gpio_clear(gpio_t dev)
{
    switch (dev) {
#if GPIO_0_EN

        case GPIO_0:
            GPIO_0_DEV->PCOR |= (1 << GPIO_0_PIN);
            break;
#endif
#if GPIO_1_EN

        case GPIO_1:
            GPIO_1_DEV->PCOR |= (1 << GPIO_1_PIN);
            break;
#endif
#if GPIO_2_EN

        case GPIO_2:
            GPIO_2_DEV->PCOR |= (1 << GPIO_2_PIN);
            break;
#endif
#if GPIO_3_EN

        case GPIO_3:
            GPIO_3_DEV->PCOR |= (1 << GPIO_3_PIN);
            break;
#endif
#if GPIO_4_EN

        case GPIO_4:
            GPIO_4_DEV->PCOR |= (1 << GPIO_4_PIN);
            break;
#endif
#if GPIO_5_EN

        case GPIO_5:
            GPIO_5_DEV->PCOR |= (1 << GPIO_5_PIN);
            break;
#endif
#if GPIO_6_EN

        case GPIO_6:
            GPIO_6_DEV->PCOR |= (1 << GPIO_6_PIN);
            break;
#endif
#if GPIO_7_EN

        case GPIO_7:
            GPIO_7_DEV->PCOR |= (1 << GPIO_7_PIN);
            break;
#endif
#if GPIO_8_EN

        case GPIO_8:
            GPIO_8_DEV->PCOR |= (1 << GPIO_8_PIN);
            break;
#endif
#if GPIO_9_EN

        case GPIO_9:
            GPIO_9_DEV->PCOR |= (1 << GPIO_9_PIN);
            break;
#endif
#if GPIO_10_EN

        case GPIO_10:
            GPIO_10_DEV->PCOR |= (1 << GPIO_10_PIN);
            break;
#endif
#if GPIO_11_EN

        case GPIO_11:
            GPIO_11_DEV->PCOR |= (1 << GPIO_11_PIN);
            break;
#endif
#if GPIO_12_EN

        case GPIO_12:
            GPIO_12_DEV->PCOR |= (1 << GPIO_12_PIN);
            break;
#endif
#if GPIO_13_EN

        case GPIO_13:
            GPIO_13_DEV->PCOR |= (1 << GPIO_13_PIN);
            break;
#endif
#if GPIO_14_EN

        case GPIO_14:
            GPIO_14_DEV->PCOR |= (1 << GPIO_14_PIN);
            break;
#endif
#if GPIO_15_EN

        case GPIO_15:
            GPIO_15_DEV->PCOR |= (1 << GPIO_15_PIN);
            break;
#endif
#if GPIO_16_EN

        case GPIO_16:
            GPIO_16_DEV->PCOR |= (1 << GPIO_16_PIN);
            break;
#endif
#if GPIO_17_EN

        case GPIO_17:
            GPIO_17_DEV->PCOR |= (1 << GPIO_17_PIN);
            break;
#endif
#if GPIO_18_EN

        case GPIO_18:
            GPIO_18_DEV->PCOR |= (1 << GPIO_18_PIN);
            break;
#endif
#if GPIO_19_EN

        case GPIO_19:
            GPIO_19_DEV->PCOR |= (1 << GPIO_19_PIN);
            break;
#endif
#if GPIO_20_EN

        case GPIO_20:
            GPIO_20_DEV->PCOR |= (1 << GPIO_20_PIN);
            break;
#endif
#if GPIO_21_EN

        case GPIO_21:
            GPIO_21_DEV->PCOR |= (1 << GPIO_21_PIN);
            break;
#endif
#if GPIO_22_EN

        case GPIO_22:
            GPIO_22_DEV->PCOR |= (1 << GPIO_22_PIN);
            break;
#endif
#if GPIO_23_EN

        case GPIO_23:
            GPIO_23_DEV->PCOR |= (1 << GPIO_23_PIN);
            break;
#endif
#if GPIO_24_EN

        case GPIO_24:
            GPIO_24_DEV->PCOR |= (1 << GPIO_24_PIN);
            break;
#endif
#if GPIO_25_EN

        case GPIO_25:
            GPIO_25_DEV->PCOR |= (1 << GPIO_25_PIN);
            break;
#endif
#if GPIO_26_EN

        case GPIO_26:
            GPIO_26_DEV->PCOR |= (1 << GPIO_26_PIN);
            break;
#endif
#if GPIO_27_EN

        case GPIO_27:
            GPIO_27_DEV->PCOR |= (1 << GPIO_27_PIN);
            break;
#endif
#if GPIO_28_EN

        case GPIO_28:
            GPIO_28_DEV->PCOR |= (1 << GPIO_28_PIN);
            break;
#endif
#if GPIO_29_EN

        case GPIO_29:
            GPIO_29_DEV->PCOR |= (1 << GPIO_29_PIN);
            break;
#endif
#if GPIO_30_EN

        case GPIO_30:
            GPIO_30_DEV->PCOR |= (1 << GPIO_30_PIN);
            break;
#endif
#if GPIO_31_EN

        case GPIO_31:
            GPIO_31_DEV->PCOR |= (1 << GPIO_31_PIN);
            break;
#endif
    }
}

void gpio_toggle(gpio_t dev)
{
    switch (dev) {
#if GPIO_0_EN

        case GPIO_0:
            GPIO_0_DEV->PTOR |= (1 << GPIO_0_PIN);
            break;
#endif
#if GPIO_1_EN

        case GPIO_1:
            GPIO_1_DEV->PTOR |= (1 << GPIO_1_PIN);
            break;
#endif
#if GPIO_2_EN

        case GPIO_2:
            GPIO_2_DEV->PTOR |= (1 << GPIO_2_PIN);
            break;
#endif
#if GPIO_3_EN

        case GPIO_3:
            GPIO_3_DEV->PTOR |= (1 << GPIO_3_PIN);
            break;
#endif
#if GPIO_4_EN

        case GPIO_4:
            GPIO_4_DEV->PTOR |= (1 << GPIO_4_PIN);
            break;
#endif
#if GPIO_5_EN

        case GPIO_5:
            GPIO_5_DEV->PTOR |= (1 << GPIO_5_PIN);
            break;
#endif
#if GPIO_6_EN

        case GPIO_6:
            GPIO_6_DEV->PTOR |= (1 << GPIO_6_PIN);
            break;
#endif
#if GPIO_7_EN

        case GPIO_7:
            GPIO_7_DEV->PTOR |= (1 << GPIO_7_PIN);
            break;
#endif
#if GPIO_8_EN

        case GPIO_8:
            GPIO_8_DEV->PTOR |= (1 << GPIO_8_PIN);
            break;
#endif
#if GPIO_9_EN

        case GPIO_9:
            GPIO_9_DEV->PTOR |= (1 << GPIO_9_PIN);
            break;
#endif
#if GPIO_10_EN

        case GPIO_10:
            GPIO_10_DEV->PTOR |= (1 << GPIO_10_PIN);
            break;
#endif
#if GPIO_11_EN

        case GPIO_11:
            GPIO_11_DEV->PTOR |= (1 << GPIO_11_PIN);
            break;
#endif
#if GPIO_12_EN

        case GPIO_12:
            GPIO_12_DEV->PTOR |= (1 << GPIO_12_PIN);
            break;
#endif
#if GPIO_13_EN

        case GPIO_13:
            GPIO_13_DEV->PTOR |= (1 << GPIO_13_PIN);
            break;
#endif
#if GPIO_14_EN

        case GPIO_14:
            GPIO_14_DEV->PTOR |= (1 << GPIO_14_PIN);
            break;
#endif
#if GPIO_15_EN

        case GPIO_15:
            GPIO_15_DEV->PTOR |= (1 << GPIO_15_PIN);
            break;
#endif
#if GPIO_16_EN

        case GPIO_16:
            GPIO_16_DEV->PTOR |= (1 << GPIO_16_PIN);
            break;
#endif
#if GPIO_17_EN

        case GPIO_17:
            GPIO_17_DEV->PTOR |= (1 << GPIO_17_PIN);
            break;
#endif
#if GPIO_18_EN

        case GPIO_18:
            GPIO_18_DEV->PTOR |= (1 << GPIO_18_PIN);
            break;
#endif
#if GPIO_19_EN

        case GPIO_19:
            GPIO_19_DEV->PTOR |= (1 << GPIO_19_PIN);
            break;
#endif
#if GPIO_20_EN

        case GPIO_20:
            GPIO_20_DEV->PTOR |= (1 << GPIO_20_PIN);
            break;
#endif
#if GPIO_21_EN

        case GPIO_21:
            GPIO_21_DEV->PTOR |= (1 << GPIO_21_PIN);
            break;
#endif
#if GPIO_22_EN

        case GPIO_22:
            GPIO_22_DEV->PTOR |= (1 << GPIO_22_PIN);
            break;
#endif
#if GPIO_23_EN

        case GPIO_23:
            GPIO_23_DEV->PTOR |= (1 << GPIO_23_PIN);
            break;
#endif
#if GPIO_24_EN

        case GPIO_24:
            GPIO_24_DEV->PTOR |= (1 << GPIO_24_PIN);
            break;
#endif
#if GPIO_25_EN

        case GPIO_25:
            GPIO_25_DEV->PTOR |= (1 << GPIO_25_PIN);
            break;
#endif
#if GPIO_26_EN

        case GPIO_26:
            GPIO_26_DEV->PTOR |= (1 << GPIO_26_PIN);
            break;
#endif
#if GPIO_27_EN

        case GPIO_27:
            GPIO_27_DEV->PTOR |= (1 << GPIO_27_PIN);
            break;
#endif
#if GPIO_28_EN

        case GPIO_28:
            GPIO_28_DEV->PTOR |= (1 << GPIO_28_PIN);
            break;
#endif
#if GPIO_29_EN

        case GPIO_29:
            GPIO_29_DEV->PTOR |= (1 << GPIO_29_PIN);
            break;
#endif
#if GPIO_30_EN

        case GPIO_30:
            GPIO_30_DEV->PTOR |= (1 << GPIO_30_PIN);
            break;
#endif
#if GPIO_31_EN

        case GPIO_31:
            GPIO_31_DEV->PTOR |= (1 << GPIO_31_PIN);
            break;
#endif
    }
}

void gpio_write(gpio_t dev, int value)
{
    if (value) {
        gpio_set(dev);
    }
    else {
        gpio_clear(dev);
    }
}

static inline void irq_handler(gpio_t dev)
{
    config[dev].cb(config[dev].arg);

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void isr_portb(void)
{
    if (PORTB->ISFR & (1 << GPIO_1_PIN)) {
        PORTB->ISFR |= (1 << GPIO_1_PIN);
        irq_handler(GPIO_1);
    }
}

void GPIO_0_ISR(void)
{
    if (PORTD->ISFR & (1 << GPIO_0_PIN)) {
        PORTD->ISFR |= (1 << GPIO_0_PIN);
        irq_handler(GPIO_0);
    }
}
#endif
