/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_kw2x
 * @{
 *
 * @file
 * @brief       Implementation of the KW2xD CPU initialization
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @}
 */

#include <stdint.h>
#include "cpu-conf.h"

#define FLASH_BASE         	(0x00000000)

static void cpu_clock_init(void);

/**
 * @brief Initialize the CPU, set IRQ priorities
 */
void cpu_init(void)
{
    /* configure the vector table location to internal flash */
    SCB->VTOR = FLASH_BASE;

    /* initialize the clock system */
    cpu_clock_init();

    /* set pendSV interrupt to lowest possible priority */
    NVIC_SetPriority(PendSV_IRQn, 0xff);
}

/**
 * @brief Configure the controllers clock system
 *
 * WIP: Finally it initializes the PLL clock with 48 MHz. This is initial work.
 */
static void cpu_clock_init(void)
{
    /* setup MCG in FEI Mode, core clock = bus clock = 41.94MHz */
    /* setup system prescalers */
    SIM->CLKDIV1 = (uint32_t)SIM_CLKDIV1_OUTDIV4(1);

    /* enable and select slow internal reference clock */
    MCG->C1 = (uint8_t)(1 << MCG_C1_IREFS_SHIFT | 1 << MCG_C1_IRCLKEN_SHIFT);

    /* defaults */
    MCG->C2 = (uint8_t)0;

    /* DCO Range: 40MHz .. 50MHz, FLL Factor = 1280 */
    MCG->C4 &= ~(uint8_t)(1 << MCG_C4_DMX32_SHIFT);
    MCG->C4 |= (uint8_t)MCG_C4_DRST_DRS(1);

    /* defaults */
    MCG->C5 = (uint8_t)0;

    /* defaults */
    MCG->C6 = (uint8_t)0;

    /* source of the FLL reference clock shall be internal reference clock */
    while ((MCG->S & MCG_S_IREFST_MASK) == 0);

    /* Wait until output of the FLL is selected */
    while (MCG->S & (MCG_S_LOCK0_MASK | MCG_S_LOLS_MASK));

    SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK);
    SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK);
    /* Use the CLK_OUT of the modem as the clock source. */
    /* Path: FEI-Mode -> FBE-Mode -> PBE-Mode -> PEE-Mode */
    /* Modem RST_B is connected to PTB19 and can be used to reset the modem. */
    PORTB->PCR[19] = PORT_PCR_MUX(1);
    GPIOB->PDDR |= (1 << 19);
    GPIOB->PCOR |= (1 << 19);
    /* Modem GPIO5 is connected to PTC0 and can be used to select CLK_OUT frequency, */
    /* set PTC0 high for CLK_OUT=32.787kHz and low for CLK_OUT=4MHz. */
    PORTC->PCR[0] = PORT_PCR_MUX(1);
    GPIOC->PDDR |= (1 << 0);
    GPIOC->PCOR |= (1 << 0);
    /* Modem IRQ_B is connected to PTB3, modem interrupt request to the MCU. */
    PORTB->PCR[KW2XDRF_IRQ_PIN] = PORT_PCR_MUX(1);
    GPIOB->PDDR &= ~(1 << KW2XDRF_IRQ_PIN);

    /* release the reset */
    GPIOB->PSOR |= (1 << 19);

    /* wait for modem IRQ_B interrupt request */
    while (GPIOB->PDIR & (1 << KW2XDRF_IRQ_PIN));

    /* program PLL and switch MCU clock, 48MHz */
    /* setup system prescalers */
    SIM->CLKDIV1 = (uint32_t)SIM_CLKDIV1_OUTDIV4(1);

    /* clear OSC control register*/
    OSC->CR = (uint8_t)OSC_CR_ERCLKEN_MASK;

    /* Select OSCCLK */
    MCG->C7 = (uint8_t)0;

    /* select high frequency range and external clock mode*/
    MCG->C2 = (uint8_t)(MCG_C2_RANGE0(1));

    /* select external reference clock and divide factor 128 */
    MCG->C1 = (uint8_t)(MCG_C1_CLKS(2) | MCG_C1_FRDIV(2));

    /* wait fo OSC initialization */
    /* while ((MCG->S & MCG_S_OSCINIT0_MASK) == 0); */

    /* source of the FLL reference clock shall be internal reference clock */
    while (MCG->S & MCG_S_IREFST_MASK);

    /* Wait until external reference clock is selected */
    while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));

    /* clear DCO register */
    MCG->C4 &= (uint8_t)~(MCG_C4_FCTRIM_MASK | MCG_C4_SCFTRIM_MASK);

    /* set external reference devider to 2 (0b0001) */
    MCG->C5 = (uint8_t)(MCG_C5_PRDIV0(1));

    /* select PLL */
    MCG->C6 = (uint8_t)(MCG_C6_PLLS_MASK);

    /* Wait until the source of the PLLS clock is PLL */
    while ((MCG->S & MCG_S_PLLST_MASK) == 0);

    /* Wait until PLL locked */
    while ((MCG->S & MCG_S_LOCK0_MASK) == 0);

    /* select internal reference clock and divide factor 128 */
    MCG->C1 = (uint8_t)(MCG_C1_FRDIV(2));

    /* Wait until output of the PLL is selected */
    while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3));

    /* Wait until PLL locked */
    while ((MCG->S & MCG_S_LOCK0_MASK) == 0);
}
