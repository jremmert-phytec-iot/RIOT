/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2014 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup        cpu_kw2x KW2xD SiP
 * @ingroup         cpu
 * @brief           CPU specific implementations for the Freescale KW2xD SiP.
 *                  The SiP incorporates a low power 2.4 GHz transceiver and a
 *                  Kinetis Cortex-M4 MCU.
 * @{
 *
 * @file
 * @brief           Implementation specific CPU configuration options
 *
 * @author          Hauke Petersen <hauke.peterse@fu-berlin.de>
 * @author          Johann Fischer <j.fischer@phytec.de>
 */

#ifndef __CPU_CONF_H
#define __CPU_CONF_H

#ifdef CPU_MODEL_KW22D512
#include "MKW22D5.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @name Kernel configuration
 *
 * @{
 */
#define KERNEL_CONF_STACKSIZE_PRINTF     (1024)

#ifndef KERNEL_CONF_STACKSIZE_DEFAULT
#define KERNEL_CONF_STACKSIZE_DEFAULT    (1024)
#endif

#define KERNEL_CONF_STACKSIZE_IDLE       (256)
/** @} */

/**
 * @brief Length for reading CPU_ID in octets
 */
#define CPUID_ID_LEN                     (16)

/**
 * @brief Pointer to CPU_ID
 */
#define CPUID_ID_PTR                     ((void *)(&(SIM_UIDH)))

/**
 * @name UART0 buffer size definition for compatibility reasons.
 *
 * TODO: remove once the remodeling of the uart0 driver is done.
 * @{
 */
#ifndef UART0_BUFSIZE
#define UART0_BUFSIZE                    (128)
#endif
/** @} */

#define TRANSCEIVER_BUFFER_SIZE          (3) /**< Buffer Size for Transceiver Module */

/**
 * @name Clock settings for the lptmr timer
 */
#define LPTIMER_CLKSRC_MCGIRCLK          0    /**< internal reference clock (4MHz) */
#define LPTIMER_CLKSRC_LPO               1    /**< PMC 1kHz output */
#define LPTIMER_CLKSRC_ERCLK32K          2    /**< RTC clock 32768Hz */
#define LPTIMER_CLKSRC_OSCERCLK          3    /**< system oscillator output, clock from RF-Part */

#ifndef LPTIMER_CLKSRC
#define LPTIMER_CLKSRC                   LPTIMER_CLKSRC_LPO    /**< default clock source */
#endif

#if (LPTIMER_CLKSRC == LPTIMER_CLKSRC_MCGIRCLK)
#define LPTIMER_CLK_PRESCALE    1
#define LPTIMER_SPEED           1000000
#elif (LPTIMER_CLKSRC == LPTIMER_CLKSRC_OSCERCLK)
#define LPTIMER_CLK_PRESCALE    1
#define LPTIMER_SPEED           1000000
#elif (LPTIMER_CLKSRC == LPTIMER_CLKSRC_ERCLK32K)
#define LPTIMER_CLK_PRESCALE    0
#define LPTIMER_SPEED           32768
#else
#define LPTIMER_CLK_PRESCALE    0
#define LPTIMER_SPEED           1000
#endif

/**
 * @name KW2XD SiP internal interconnects between MCU and Modem.
 *
 * @{
 */
#define KW2XDRF_PORT            PORTB /**< MCU Port connected to Modem*/
#define KW2XDRF_GPIO            GPIOB /**< GPIO Device connected to Modem */
#define KW2XDRF_PORT_IRQn       PORTB_IRQn
#define KW2XDRF_PORT_CLKEN()    (SIM->SCGC5 |= (SIM_SCGC5_PORTB_MASK)) /**< Clock Enable for PORTB*/
#define KW2XDRF_PIN_AF          2  /**< Pin Muxing Parameter for GPIO Device*/
#define KW2XDRF_PCS0_PIN        10 /**< SPI Slave Select Pin */
#define KW2XDRF_SCK_PIN         11 /**< SPI Clock Output Pin */
#define KW2XDRF_SOUT_PIN        16 /**< SPI Master Data Output Pin */
#define KW2XDRF_SIN_PIN         17 /**< SPI Master Data Input Pin */

#define KW2XDRF_IRQ_PIN         3  /**< Modem's IRQ Output (activ low) */
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __CPU_CONF_H */
/** @} */
