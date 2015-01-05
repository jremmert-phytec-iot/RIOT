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
 * @brief       Low-level random number generator driver implementation.
 *              Driver for Freescale's RNGA module. RNGA generates data that
 *              looks random. Reference Manual recommends to use the RNGA as entropy
 *              source.
 *
 * @author      Johann Fischer <j.fischer@phytec.de> (adaption for Freescale's RNGA)
 * @author      Hauke Petersen <mail@haukepetersen.de>
 *
 * @}
 */

#include "cpu.h"
#include "periph/random.h"
#include "periph_conf.h"

#if RANDOM_NUMOF

typedef struct RNG_MemMap {
    uint32_t CR;
    uint32_t SR;
    uint32_t ER;
    uint32_t OR;
} volatile RNGA_Type;

#define RNGA_CR_SLP_MASK           0x10u
#define RNGA_CR_CLRI_MASK          0x08u
#define RNGA_CR_INTM_MASK          0x04u
#define RNGA_CR_HA_MASK            0x02u
#define RNGA_CR_GO_MASK            0x01u

#define RNGA_SR_OREG_SIZE_MASK     0xFF0000u
#define RNGA_SR_OREG_SIZE(x)       (((uint32_t)(((uint32_t)(x))<<16))&RNGA_SR_OREG_SIZE_MASK)
#define RNGA_SR_OREG_LVL_MASK      0xFF00u
#define RNGA_SR_OREG_LVL(x)        (((uint32_t)(((uint32_t)(x))<<8))&RNGA_SR_OREG_LVL_MASK)
#define RNGA_SR_SLP_MASK           0x10u
#define RNGA_SR_ERRI_MASK          0x08u
#define RNGA_SR_ORU_MASK           0x04u
#define RNGA_SR_LRS_MASK           0x02u
#define RNGA_SR_SECV_MASK          0x01u

#define RNGA                       ((RNGA_Type *)RANDOM_RNGA_BASE)

void random_init(void)
{
    random_poweron();
}

int random_read(char *buf, unsigned int num)
{
    /* cppcheck-suppress variableScope */
    uint32_t tmp;
    int count = 0;

    /* self-seeding */
    while (!(RNGA->SR & RNGA_SR_OREG_LVL_MASK));

    RNGA->ER = RNGA->OR ^ (uint32_t)buf;

    while (count < num) {
        /* wait for random data to be ready to read */
        while (!(RNGA->SR & RNGA_SR_OREG_LVL_MASK));

        tmp = RNGA->OR;

        /* copy data into result vector */
        for (int i = 0; i < 4 && count < num; i++) {
            buf[count++] = (char)tmp;
            tmp = tmp >> 8;
        }
    }

    return count;
}

void random_poweron(void)
{
    RANDOM_CLKEN();
    RNGA->CR = RNGA_CR_INTM_MASK | RNGA_CR_HA_MASK | RNGA_CR_GO_MASK;
}

void random_poweroff(void)
{
    RNGA->CR = 0;
    RANDOM_CLKDIS();
}

/*
void isr_rng(void)
{
}
*/

#endif /* RANDOM_NUMOF */
