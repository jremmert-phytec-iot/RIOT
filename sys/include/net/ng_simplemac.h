/*
 * Copyright (C) 2015 Freie Universität Berlin
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_simplemac basic CSMA MAC layer
 * @ingroup     net
 * @brief       Basic CSMA MAC protocol that sends in unslotted CSMA Mode
 * @{
 *
 * @file
 * @brief       Interface definition for the SIMPLEMAC MAC layer
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 */

#ifndef NG_SIMPLEMAC_H_
#define NG_SIMPLEMAC_H_

#include "kernel.h"
#include "net/ng_netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MAC sublayer constants, referring to IEEE 802.15.4-2011 Table 51, 52 */

/**
 * @brief    The maximum number of backoff intervals.
 * @details  The maximum number of backoffs the CSMA-CA algorithm 
 *           will attempt before declaring a channel access failure.
 *           The IEEE 802.15.4 Standard allows values from 0-5.
 */
#define MAX_CSMA_BACKOFFS       4

/**
 * @brief    The maximum value of the backoff exponent BE.
 * @details  The IEEE 802.15.4 Standard allows values from 3-8.
 */
#define MAX_BE                  4

/**
 * @brief    The minimum value of the backoff exponent BE.
 * @details  The IEEE 802.15.4 Standard allows values from 0-MAX_BE.
 */
#define MIN_BE                  3

/**
 * @brief    The maximum number of retries allowed after a transmission failure.
 * @details  The IEEE 802.15.4 Standard allows values from 0-7.
 */
#define MAX_FRAME_RETRIES       3

/**
 * @brief    Max symbols to wait for ack (value 30 only for testing!!! calculate it!!).
 * @details  The maximum number of symbols to
 *           wait for an acknowledgment frame to arrive following a transmitted data 
 *           frame. This value is dependent on the supported PHY, which determines both
 *           the selected channel and channel page. The calculated value is the time to 
 *           commence transmitting the ACK plus the length of the ACK frame.
 */
#define MAX_ACK_WAIT_DURATION   30

/**
 * @brief   The number of symbols forming the basic time period
 *          used by the CSMA-CA algorithm. 
 */
#define UNIT_BACKOFF_PERIOD     20

/**
 * @brief   Length of a single simbol in us. (PHY dependant)  
 * @details Is calculated as follows: 
 */
#define SYMBOL_LENGTH           16

/**
 * @brief   The physical timer that is used for the MAC-layer, (periph/timer - IF).
 */
#define MAC_TIMER               TIMER_0

/**
 * @brief   The Channel to use for timer in MAC-layer.
 */
#define MAC_TMR_CH              0

/**
 * @brief   Set the default message queue size for SIMPLEMAC layers
 */
#ifndef NG_SIMPLEMAC_MSG_QUEUE_SIZE
#define NG_SIMPLEMAC_MSG_QUEUE_SIZE         (8U)
#endif

/**
 * @brief   Initialize an instance of the SIMPLEMAC layer
 *
 * The initialization starts a new thread that connects to the given netdev
 * device and starts a link layer event loop.
 *
 * @param[in] stack         stack for the control thread
 * @param[in] stacksize     size of *stack*
 * @param[in] priority      priority for the thread housing the SIMPLEMAC instance
 * @param[in] name          name of the thread housing the SIMPLEMAC instance
 * @param[in] dev           netdev device, needs to be already initialized
 *
 * @return                  PID of SIMPLEMAC thread on success
 * @return                  -EINVAL if creation of thread fails
 * @return                  -ENODEV if *dev* is invalid
 */
kernel_pid_t ng_simplemac_init(char *stack, int stacksize, char priority,
                        const char *name, ng_netdev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLEMAC_H_ */
/** @} */
