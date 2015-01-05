/*
 * Copyright (C) 2014 Hamburg University of Applied Sciences
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
 * @brief       Low-level SPI driver implementation
 *
 * @author      Peter Kietzmann <peter.kietzmann@haw-hamburg.de>
 * @author      Johann Fischer <j.fischer@phytec.de>
 *
 * @}
 */
#include <stdio.h>

#include "board.h"
#include "cpu.h"
#include "periph/spi.h"
#include "periph_conf.h"
#include "thread.h"
#include "sched.h"
#include "vtimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/* guard this file in case no SPI device is defined */
#if SPI_NUMOF

typedef struct {
    char(*cb)(char data);
} spi_state_t;

static inline void irq_handler_transfer(SPI_Type *spi, spi_t dev);

static spi_state_t spi_config[SPI_NUMOF];

int spi_init_master(spi_t dev, spi_conf_t conf, spi_speed_t speed)
{

    SPI_Type *spi_dev;

    switch (speed) {
        case SPI_SPEED_100KHZ:
            return -2;          /* not possible */
            break;

        case SPI_SPEED_400KHZ:
            /* TODO */
            break;

        case SPI_SPEED_1MHZ:
            /* TODO */
            break;

        case SPI_SPEED_5MHZ:
            /* TODO */
            break;

        case SPI_SPEED_10MHZ:
            /* TODO */
            break;

        default:
            return -2;
    }

    switch (dev) {
#if SPI_0_EN

        case SPI_0:
            spi_dev = SPI_0_DEV;
            /* enable clocks */
            SPI_0_CLKEN();
            SPI_0_PORT_CLKEN();
            /* Set PORT to AF mode */
            SPI_0_PORT->PCR[SPI_0_PCS0_PIN] = PORT_PCR_MUX(SPI_0_PIN_AF);
            SPI_0_PORT->PCR[SPI_0_SCK_PIN] = PORT_PCR_MUX(SPI_0_PIN_AF);
            SPI_0_PORT->PCR[SPI_0_SOUT_PIN] = PORT_PCR_MUX(SPI_0_PIN_AF);
            SPI_0_PORT->PCR[SPI_0_SIN_PIN] = PORT_PCR_MUX(SPI_0_PIN_AF);
            break;
#endif /* SPI_0_EN */

        default:
            return -1;
    }

    /* set speed for 8-bit access */
    spi_dev->CTAR[0] = SPI_CTAR_FMSZ(7)
                       | SPI_CTAR_PBR(0)
                       | SPI_CTAR_BR(1);
    /* set speed for 16-bit access */
    spi_dev->CTAR[1] = SPI_CTAR_FMSZ(15)
                       | SPI_CTAR_PBR(0)
                       | SPI_CTAR_BR(1);

    /* enable SPI */
    spi_dev->MCR = SPI_MCR_MSTR_MASK
                   | SPI_MCR_PCSIS(1)
                   | SPI_MCR_DOZE_MASK
                   | SPI_MCR_CLR_TXF_MASK
                   | SPI_MCR_CLR_RXF_MASK;

    spi_dev->RSER = (uint32_t)0;

    return 0;
}

int spi_init_slave(spi_t dev, spi_conf_t conf, char(*cb)(char data))
{
    SPI_Type *spi_dev;

    switch (dev) {
#if SPI_0_EN

        case SPI_0:
            spi_dev = SPI_0_DEV;
            /* enable clocks */
            SPI_0_CLKEN();
            SPI_0_PORT_CLKEN();
            /* Set PORT to AF mode */
            SPI_0_PORT->PCR[SPI_0_PCS0_PIN] = PORT_PCR_MUX(SPI_0_PIN_AF);
            SPI_0_PORT->PCR[SPI_0_SCK_PIN] = PORT_PCR_MUX(SPI_0_PIN_AF);
            SPI_0_PORT->PCR[SPI_0_SOUT_PIN] = PORT_PCR_MUX(SPI_0_PIN_AF);
            SPI_0_PORT->PCR[SPI_0_SIN_PIN] = PORT_PCR_MUX(SPI_0_PIN_AF);
            break;
#endif /* SPI_0_EN */

        default:
            return -1;
    }

    /* set speed */
    spi_dev->CTAR[0] = SPI_CTAR_SLAVE_FMSZ(7);

    /* enable SPI */
    spi_dev->MCR = SPI_MCR_DOZE_MASK
                   | SPI_MCR_CLR_TXF_MASK
                   | SPI_MCR_CLR_RXF_MASK;

    spi_dev->RSER = (uint32_t)0;

    return 0;
}

int spi_transfer_byte(spi_t dev, char out, char *in)
{
    SPI_Type *spi_dev;

    switch (dev) {
#if SPI_0_EN

        case SPI_0:
            spi_dev = SPI_0_DEV;
            break;
#endif

        default:
            return -1;
    }

    while (!(spi_dev->SR & SPI_SR_TFFF_MASK));

    spi_dev->PUSHR = SPI_PUSHR_CTAS(0)
                     | SPI_PUSHR_EOQ_MASK
                     | SPI_PUSHR_PCS(1)
                     | SPI_PUSHR_TXDATA(out);

    while (!(spi_dev->SR & SPI_SR_RXCTR_MASK));

    spi_dev->SR = SPI_SR_EOQF_MASK;

    if (in != NULL) {
        *in = (char)spi_dev->POPR;
    }
    else {
        spi_dev->POPR;
    }

    return 1;
}

int spi_transfer_bytes(spi_t dev, char *out, char *in, unsigned int length)
{
    SPI_Type *spi_dev;

    switch (dev) {
#if SPI_0_EN

        case SPI_0:
            spi_dev = SPI_0_DEV;
            break;
#endif

        default:
            return -1;
    }

    if (out == NULL || in == NULL) {
        return -1;
    }

    int i, trans_bytes = 0;

    for (i = 0; i < (length - 1); i++) {
        while (!(spi_dev->SR & SPI_SR_TFFF_MASK));

        spi_dev->PUSHR = SPI_PUSHR_CTAS(0)
                         | SPI_PUSHR_CONT_MASK
                         | SPI_PUSHR_PCS(1)
                         | SPI_PUSHR_TXDATA(out[i]);

        while (!(spi_dev->SR & SPI_SR_RXCTR_MASK));

        in[i] = (char)spi_dev->POPR;

        trans_bytes++;
    }

    while (!(spi_dev->SR & SPI_SR_TFFF_MASK));

    spi_dev->PUSHR = SPI_PUSHR_CTAS(0)
                     | SPI_PUSHR_EOQ_MASK
                     | SPI_PUSHR_PCS(1)
                     | SPI_PUSHR_TXDATA(out[i]);

    while (!(spi_dev->SR & SPI_SR_RXCTR_MASK));

    spi_dev->SR = SPI_SR_EOQF_MASK;
    in[i] = (char)spi_dev->POPR;
    trans_bytes++;

    return trans_bytes++;
}

int spi_transfer_reg(spi_t dev, uint8_t reg, char out, char *in)
{
    SPI_Type *spi_dev;

    switch (dev) {
#if SPI_0_EN

        case SPI_0:
            spi_dev = SPI_0_DEV;
            break;
#endif

        default:
            return -1;
    }

    while (!(spi_dev->SR & SPI_SR_TFFF_MASK));

    spi_dev->PUSHR = SPI_PUSHR_CTAS(1)
                     | SPI_PUSHR_EOQ_MASK
                     | SPI_PUSHR_PCS(1)
                     | SPI_PUSHR_TXDATA((uint16_t)(reg << 8) | (uint16_t)out);

    while (!(spi_dev->SR & SPI_SR_RXCTR_MASK));

    spi_dev->SR = SPI_SR_EOQF_MASK;

    if (in != NULL) {
        *in = (char)spi_dev->POPR;
    }
    else {
        spi_dev->POPR;
    }

    return 2;
}

int spi_transfer_regs(spi_t dev, uint8_t reg, char *out, char *in, unsigned int length)
{
    SPI_Type *spi_dev;

    switch (dev) {
#if SPI_0_EN

        case SPI_0:
            spi_dev = SPI_0_DEV;
            break;
#endif

        default:
            return -1;
    }

    if (out == NULL || in == NULL) {
        return -1;
    }

    int i, trans_bytes = 0;

    while (!(spi_dev->SR & SPI_SR_TFFF_MASK));

    spi_dev->PUSHR = SPI_PUSHR_CTAS(1)
                     | SPI_PUSHR_CONT_MASK
                     | SPI_PUSHR_PCS(1)
                     | SPI_PUSHR_TXDATA((uint16_t)(reg << 8) | (uint16_t)out[0]);

    while (!(spi_dev->SR & SPI_SR_RXCTR_MASK));

    spi_dev->SR = SPI_SR_EOQF_MASK;
    in[0] = (char)spi_dev->POPR;
    trans_bytes++;


    for (i = 1; i < (length - 1); i++) {
        while (!(spi_dev->SR & SPI_SR_TFFF_MASK));

        spi_dev->PUSHR = SPI_PUSHR_CTAS(0)
                         | SPI_PUSHR_CONT_MASK
                         | SPI_PUSHR_PCS(1)
                         | SPI_PUSHR_TXDATA(out[i]);

        while (!(spi_dev->SR & SPI_SR_RXCTR_MASK));

        in[i] = (char)spi_dev->POPR;

        trans_bytes++;
    }

    while (!(spi_dev->SR & SPI_SR_TFFF_MASK));

    spi_dev->PUSHR = SPI_PUSHR_CTAS(0)
                     | SPI_PUSHR_EOQ_MASK
                     | SPI_PUSHR_PCS(1)
                     | SPI_PUSHR_TXDATA(out[i]);

    while (!(spi_dev->SR & SPI_SR_RXCTR_MASK));

    spi_dev->SR = SPI_SR_EOQF_MASK;
    in[i] = (char)spi_dev->POPR;
    trans_bytes++;

    return trans_bytes++;
}

void spi_transmission_begin(spi_t dev, char reset_val)
{

    switch (dev) {
#if SPI_0_EN

        case SPI_0:
            SPI_0_DEV->PUSHR = SPI_PUSHR_CTAS(0)
                               | SPI_PUSHR_EOQ_MASK
                               | SPI_PUSHR_PCS(0)
                               | SPI_PUSHR_TXDATA(reset_val);
            break;
#endif
    }
}

void spi_poweron(spi_t dev)
{
    switch (dev) {
#if SPI_0_EN

        case SPI_0:
            SPI_0_CLKEN();
            break;
#endif
    }
}

void spi_poweroff(spi_t dev)
{
    switch (dev) {
#if SPI_0_EN

        case SPI_0:
            while (SPI_0_DEV->SR & SPI_SR_EOQF_MASK);

            SPI_0_CLKDIS();
            break;
#endif
    }
}

static inline void irq_handler_transfer(SPI_Type *spi, spi_t dev)
{

    if (spi->SR & SPI_SR_RFDF_MASK) {
        char data;
        data = (char)spi->POPR;
        data = spi_config[dev].cb(data);
        spi->PUSHR = SPI_PUSHR_CTAS(0)
                     | SPI_PUSHR_EOQ_MASK
                     | SPI_PUSHR_PCS(0)
                     | SPI_PUSHR_TXDATA(data);
    }

    /* see if a thread with higher priority wants to run now */
    if (sched_context_switch_request) {
        thread_yield();
    }
}

#if SPI_0_EN
void SPI_0_IRQ_HANDLER(void)
{
    irq_handler_transfer(SPI_0_DEV, SPI_0);
}
#endif

#endif /* SPI_NUMOF */
