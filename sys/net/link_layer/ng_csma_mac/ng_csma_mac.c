/*
 * Copyright (C) 2015 Freie Universität Berlin
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_csma_mac
 * @file
 * @brief       Implementation of the CSMA_MAC MAC protocol
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 * @}
 */

#include <errno.h>

#include "kernel.h"
#include "msg.h"
#include "thread.h"
#include "net/ng_netreg.h"
#include "net/ng_pkt.h"
#include "net/ng_nettype.h"
#include "net/ng_netdev.h"
#include "net/ng_netapi.h"
#include "net/ng_netif.h"
#include "periph/timer.h"
#include "periph/random.h"
#include "net/ng_csma_mac.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

typedef enum {
    CSMA_IDLE,
    CSMA_PERFORM_CCA,
    CSMA_WAIT,
    CSMA_TX_FRAME,
    CSMA_WAIT_FOR_ACK
} csma_mac_states_t;

static csma_mac_states_t csma_mac_state = CSMA_IDLE;
static uint8_t be = 0;
static uint8_t retries = 0;
//static tim_t mac_tmr = CSMA_MAC_TIMER;
static ng_netdev_t *csma_mac_dev = NULL;
//static ng_pktsnip_t *csma_mac_pkt = NULL;
static msg_t msg;  

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event         type of event
 * @param[in] data          optional parameter
 */
static void _event_cb(ng_netdev_event_t event, void *data)
{
    DEBUG("csma_mac: event triggered -> %i\n", event);

    /* CSMA_MAC only understands the RX_COMPLETE event... */
    if (event == NETDEV_EVENT_RX_COMPLETE) {
        ng_pktsnip_t *pkt;
        ng_netreg_entry_t *sendto;

        /* get pointer to the received packet */
        pkt = (ng_pktsnip_t *)data;
        /* find out, who to send the packet to */
        sendto = ng_netreg_lookup(pkt->next->type, 0);

#if ENABLE_DEBUG

        if (sendto == NULL) {
            DEBUG("csma_mac: unable to forward packet of type %i\n",
                  pkt->next->type);
        }

#endif

        /* send the packet to everyone interested in it's type */
        while (sendto != NULL) {
            DEBUG("csma_mac: sending pkt %p to PID %u\n", pkt, sendto->pid);
            ng_netapi_send(sendto->pid, pkt);
            sendto = ng_netreg_getnext(sendto);
        }
    }
}

/**
 * @brief     wait function that sets timer event. Wait time depends on backoff_exp.
 *
 * @param[in] backoff_exponent    wait-time depends on this exponent.
 *
 * @return number of bytes that were actually send out
 * @return CHANNEL_ACCESS_FAILURE if packet could not sent out
 */
static int backoff_wait(uint8_t backoff_exponent)
{
    uint16_t random = 0;
    uint16_t backoff_intervall = 0;
    char buf[2];

    if (backoff_exponent < CSMA_MAC_MAX_BE) {
        backoff_exponent = CSMA_MAC_MAX_BE;
    }

    /* backoff_intervall is defined as ((2^bf)-1)*CSMA_MAC_SYMBOL_LENGTH */
    backoff_intervall = 1;
    backoff_intervall <<= backoff_exponent;
    backoff_intervall--;
    backoff_intervall *= CSMA_MAC_SYMBOL_LENGTH;
    DEBUG("Upper limit for backoff_intervall: %i\n", backoff_intervall);

    /* Generate 16bit random number */
    if (random_read(buf, 2) != 2) {
        return 0;   /* what to return in case of random_read error? */
    }

    random = buf[0];
    random <<= 8;
    random = random | (0x00ff & buf[1]);

    backoff_intervall = random % backoff_intervall;
    DEBUG("Random number generated: %i\n", random);
    DEBUG("Wait interval: %i\n", backoff_intervall);

    //timer_set(mac_tmr, CSMA_MAC_TIMER_CH, backoff_intervall);
    DEBUG("Timer set to %i\n", random);
    return 0;
}

/**
 * @brief     The mac_send_statechart function implements an statechart for better clarity
 *
 * @param[in] dev   network device descriptor
 * @param[in] pkt   pointer to the data in the packet buffer
 *
 * @return number of bytes that were actually send out
 * @return -1 if packet could not sent out
 */
static void _mac_send_statechart(int dummy)
{
    ng_netapi_opt_t *conf = NULL;
    int res;

    while (1) {
        switch (csma_mac_state) {

            case CSMA_IDLE:
                DEBUG("csma_mac-state: CSMA_IDLE\n");
                be = 0;
                retries = 0;
                csma_mac_state = CSMA_PERFORM_CCA;
                break;

            case CSMA_PERFORM_CCA:
                DEBUG("csma_mac-state: CSMA_PERFORM_CCA\n");
                //timer_clear(mac_tmr, CSMA_MAC_TIMER_CH);

                if (be > CSMA_MAC_MAX_BACKOFFS) {
                    return;
                    csma_mac_state = CSMA_IDLE;
                }

                conf -> opt = NETCONF_OPT_IS_CHANNEL_CLR;
                conf -> data = 0;
                /* TODO: What has to be assigned to data? */
                /* In my opinion the CCA-check must be interrupt driven by the radio.
                 * Poke CCA on radio driver, driver should trigger an interrupt
                 * if CCA is ready.
                 */
                res = csma_mac_dev->driver->get(csma_mac_dev,
                                     conf->opt, conf->data, (size_t*)(&(conf->data_len)));
                csma_mac_state = CSMA_WAIT;
                return;
                break;

            case CSMA_WAIT:
                DEBUG("csma_mac-state: CSMA_WAIT\n");

                /* It is not quite clear to me, in what way the get/set CCA Option should be
                 * implemented in the radio driver.
                 */
                if (1) {      //<CCA Successful>
                    /* As the radio triggers an interrupt when the CCA
                                             * interval is expired, ask via SPI weather CCA
                                             * was successfull or not. */
                    csma_mac_state = CSMA_TX_FRAME;
                }
                else {
                    backoff_wait(be);
                    be ++;

                    if (be > CSMA_MAC_MAX_BE) { /* Signalize failure, but how? Static Variable? */
                        csma_mac_state = CSMA_IDLE;
                        return;
                    }

                    csma_mac_state = CSMA_PERFORM_CCA;
                    return;
                }

                break;

            case CSMA_TX_FRAME:
                DEBUG("csma_mac-state: CSMA_TX_FRAME\n");
                /* If ack was not successfull after this timer expires,
                 * mark as CHANNEL_ACCESS_FAILURE */
                res = csma_mac_dev->driver->send_data(csma_mac_dev,
                                                       (ng_pktsnip_t *)msg.content.ptr);

                if (res < 1) {
                    return;
                    /* Signalize failure */
                }

                //timer_set(mac_tmr, CSMA_MAC_TIMER_CH, CSMA_MAC_MAX_ACK_WAIT_DURATION);
                csma_mac_state = CSMA_WAIT_FOR_ACK;
                return;
                break;

            case CSMA_WAIT_FOR_ACK:
                DEBUG("csma_mac-state: CSMA_WAIT_FOR_ACK\n");
                /* TODO: Determine ISR source */
                //timer_clear(mac_tmr, CSMA_MAC_TIMER_CH);

                if (1) {  // <Timer Interrupt>
                    csma_mac_state = CSMA_IDLE;
                    /* Signalize success*/
                    return;
                }
                else {
                    retries ++;

                    if (retries > CSMA_MAC_MAX_FRAME_RETRIES) {
                        csma_mac_state = CSMA_PERFORM_CCA;
                    }
                    else {
                        csma_mac_state = CSMA_IDLE;
                        /* Signalize failure */
                        return;
                    }
                }

                break;
        }
    }
}

/**
 * @brief   Startup code and event loop of the CSMA_MAC layer
 *
 * @param[in] args          expects a pointer to the underlying netdev device
 *
 * @return                  never returns
 */
static void *_csma_mac_thread(void *args)
{
    //ng_netdev_t *csma_mac_dev = (ng_netdev_t *)args;
    ng_netapi_opt_t *opt;
    int res;
    msg_t reply, msg_queue[CSMA_MAC_MSG_QUEUE_SIZE];

    /* TODO: Initialize Network device with the following options:
     * Enable Auto Ack:                        NETCONF_OPT_AUTOACK
     * Enable CCA Interrupt in radio:
     * Enable Sent ready Interrupt in radio:
     * Enable Auto Ack Interrupt in radio:
     */

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, CSMA_MAC_MSG_QUEUE_SIZE);
    /* save the PID to the device descriptor and register the device */
    //csma_mac_dev->mac_pid = thread_getpid();
    //ng_netif_add(csma_mac_dev->mac_pid);
    /* register the event callback with the device driver */
    //csma_mac_dev->driver->add_event_callback(csma_mac_dev, _event_cb);

    /* TODO: Ask device for certain HW-MAC Support and store config for statechart
     *       AUTOACK supported?;
     * TODO: Add option NETCONF_OPT_CCA_TX to ng_netconf_opt_t
     *       (Auto tx after CCA success)
     */

    /* start the event loop */
    while (1) {
        DEBUG("csma_mac: waiting for incoming messages\n");
        msg_receive(&msg);

        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NG_NETDEV_MSG_TYPE_EVENT:
                DEBUG("csma_mac: NG_NETDEV_MSG_TYPE_EVENT received\n");
                csma_mac_dev->driver->isr_event(csma_mac_dev, msg.content.value);
                break;

            case NG_NETAPI_MSG_TYPE_SND:
                DEBUG("csma_mac: NG_NETAPI_MSG_TYPE_SND received\n");
                _mac_send_statechart(0);
                break;

            case NG_NETAPI_MSG_TYPE_SET:
                /* TODO: filter out MAC layer options -> for now forward
                         everything to the device driver */
                DEBUG("csma_mac: NG_NETAPI_MSG_TYPE_SET received\n");
                /* read incoming options */
                opt = (ng_netapi_opt_t *)msg.content.ptr;
                /* set option for device driver */
                res = csma_mac_dev->driver->set(csma_mac_dev, opt->opt, opt->data, opt->data_len);
                /* send reply to calling thread */
                reply.type = NG_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;

            case NG_NETAPI_MSG_TYPE_GET:
                /* TODO: filter out MAC layer options -> for now forward
                         everything to the device driver */
                DEBUG("csma_mac: NG_NETAPI_MSG_TYPE_GET received\n");
                /* read incoming options */
                opt = (ng_netapi_opt_t *)msg.content.ptr;
                /* get option from device driver */
                res = csma_mac_dev->driver->get(csma_mac_dev, opt->opt, opt->data,
                                                 (size_t *)(&(opt->data_len)));
                /* send reply to calling thread */
                reply.type = NG_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
        }
    }

    /* never reached */
    return NULL;
}

kernel_pid_t csma_mac_init(char *stack, int stacksize, char priority,
                               const char *name, ng_netdev_t *dev)
{
    //unsigned int us_per_tick = 1;
    kernel_pid_t res;

    DEBUG("Timer and RNG successfull initialized.\n");
    /* check if given netdev device is defined */
    //if (dev == NULL) {
    //    return -ENODEV;
    //}

    /* create new CSMA_MAC thread */
    res = thread_create(stack, stacksize, priority, CREATE_STACKTEST,
                        _csma_mac_thread, (void *)dev, name);

    //if (res <= 0) {
    //    return -EINVAL;
    //}

    random_init();
    /* If timer event occures, _mac_send_statechart is called in ISR. */
    //timer_init(mac_tmr, us_per_tick, _mac_send_statechart);
    DEBUG("Timer and RNG successfull initialized.\n");
    return res;
}
