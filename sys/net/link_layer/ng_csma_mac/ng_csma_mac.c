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
#include "hwtimer.h"
#include "mutex.h"

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
static ng_netdev_t *dev;
static ng_pktsnip_t index_csma_mac_pkt;
static ng_pktsnip_t *csma_mac_pkt = &index_csma_mac_pkt;
static msg_t msg;
static mutex_t mutex = MUTEX_INIT;
static ng_netapi_opt_t *opt;

static void task_block(void){
    mutex_lock(&mutex);

    /* try to lock mutex again will cause the thread to go into
     * STATUS_MUTEX_BLOCKED until task_release fires the releasemutex */
    mutex_lock(&mutex);
}

static void task_release(void){
    mutex_unlock(&mutex);
}

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event         type of event
 * @param[in] data          optional parameter
 */
static void _event_cb(ng_netdev_event_t event, void *data)
{
    DEBUG("csma_mac: event triggered -> %i\n", event);

    ng_pktsnip_t *pkt;
    ng_netreg_entry_t *sendto;

    switch (event) {

    case NETDEV_EVENT_RX_COMPLETE:

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
        break;

   case NETDEV_EVENT_TX_COMPLETE:
        task_release();
        break;

   default:
        break;
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

    //timer_set(mac_tmr, CSMA_MAC_TIMER_CH, backoff_intervall);
    DEBUG("Backoff wait %i\n", backoff_intervall);
    hwtimer_wait(backoff_intervall);
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
static int _mac_send_statechart(void)
{
    ng_netconf_state_t res;
    /* Calculate µs value to tick amount for setting up HW-timer */
    unsigned ticks = HWTIMER_TICKS(100000);
    while (1) {
        switch (csma_mac_state) {

            case CSMA_IDLE:
                DEBUG("csma_mac-state: CSMA_IDLE\n");
                be = 0;
                retries = 0;
                hwtimer_wait(ticks);
                csma_mac_state = CSMA_PERFORM_CCA;
                break;

            case CSMA_PERFORM_CCA:
                DEBUG("csma_mac-state: CSMA_PERFORM_CCA\n");

                if (be > CSMA_MAC_MAX_BACKOFFS) {
                    return -EBUSY;
                    csma_mac_state = CSMA_IDLE;
                }

                //int *index_data = 0;
                //opt->data =index_data;
                //opt->data_len = 1;
                opt->opt = NETCONF_OPT_IS_CHANNEL_CLR;
                res = dev->driver->set(dev,
                                     opt->opt, opt->data,(size_t)opt->data_len);
                DEBUG("csma_mac: get CCA returned %i\n", res);
                if (res == NETCONF_STATE_PERFORM_CCA){
                    csma_mac_state = CSMA_WAIT;
                    break;
                }
                if (res == NETCONF_STATE_AUTO_CCA){
                    csma_mac_state = CSMA_TX_FRAME;
                    break;
                }
                DEBUG("csma_mac-Error: Invalid return from get_cca function\n");
                while(1);
                break;

            /*
             * State-machine jumps to this state if radio performs cca. The task is blocked, 
             *  therefore the radio must execute an interrupt to unlock the task again.
             */
            case CSMA_WAIT:
                DEBUG("csma_mac-state: CSMA_WAIT, task is set to blocking state:\n");
                task_block();
                DEBUG("csma_mac-state: CSMA_WAIT, task unblocked");

                if (1) {      //<CCA Successful>
                    /* As the radio triggers an interrupt when the CCA
                                             * interval is expired, ask via SPI weather CCA
                                             * was successfull or not. */
                    hwtimer_wait(ticks);
                    csma_mac_state = CSMA_TX_FRAME;
                }
                else {
                    backoff_wait(be);
                    be ++;

                    if (be > CSMA_MAC_MAX_BE) {     /* return error code */
                        csma_mac_state = CSMA_IDLE;
                        return -EBUSY;              /* Resource busy */
                    }

                    hwtimer_wait(ticks);
                    be++;
                    csma_mac_state = CSMA_PERFORM_CCA;
                }

                break;

            case CSMA_TX_FRAME:
                DEBUG("csma_mac-state: CSMA_TX_FRAME\n");
                /* If ack was not successfull after this timer expires,
                 * mark as CHANNEL_ACCESS_FAILURE */
                res = dev->driver->send_data(dev,
                                                       csma_mac_pkt);

                //if (res < 1) {
                //    return res;
                    /* Signalize failure */
                //}

                //timer_set(mac_tmr, CSMA_MAC_TIMER_CH, CSMA_MAC_MAX_ACK_WAIT_DURATION);
                hwtimer_wait(ticks);
                csma_mac_state = CSMA_WAIT_FOR_ACK;
                break;

            case CSMA_WAIT_FOR_ACK:
                DEBUG("csma_mac-state: CSMA_WAIT_FOR_ACK\n");
                /* TODO: Determine ISR source */
                //timer_clear(mac_tmr, CSMA_MAC_TIMER_CH);
                hwtimer_wait(ticks);

                if (1) {  // <Timer Interrupt>
                    csma_mac_state = CSMA_IDLE;
                    /* Signalize success and return positive number of sent bytes*/
                    return res;
                }
                else {
                    retries ++;

                    if (retries > CSMA_MAC_MAX_FRAME_RETRIES) {
                        csma_mac_state = CSMA_PERFORM_CCA;
                    }
                    else {
                        csma_mac_state = CSMA_IDLE;
                        return -EBUSY;                  /* Resource busy */
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
    dev = (ng_netdev_t *)args;
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
    //dev->mac_pid = thread_getpid();
    //ng_netif_add(dev->mac_pid);
    /* register the event callback with the device driver */
    //dev->driver->add_event_callback(dev, _event_cb);

    /* TODO: Ask device for certain HW-MAC Support and store config for statechart
     *       AUTOACK supported?;
     * TODO: Add option NETCONF_OPT_CCA_TX to ng_netconf_opt_t
     *       (Auto tx after CCA success)
     */

    /* start the event loop */
    while (1) {
        DEBUG("csma_mac: waiting for incoming messages\n");
        msg_receive(&msg);
        opt = (ng_netapi_opt_t * )msg.content.ptr;
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NG_NETDEV_MSG_TYPE_EVENT:
                DEBUG("csma_mac: NG_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr_event(dev, msg.content.value);
                break;

            case NG_NETAPI_MSG_TYPE_SND:
                DEBUG("csma_mac: NG_NETAPI_MSG_TYPE_SND received, content.value = %i\n",
                            (int)msg.content.value);
                _mac_send_statechart();
                break;

            case NG_NETAPI_MSG_TYPE_SET:
                /* TODO: filter out MAC layer options -> for now forward
                         everything to the device driver */
                DEBUG("csma_mac: NG_NETAPI_MSG_TYPE_SET received\n");
                /* read incoming options */
                opt = (ng_netapi_opt_t *)msg.content.ptr;
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
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
                res = dev->driver->get(dev, opt->opt, opt->data,
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
    if (dev == NULL) {
        return -ENODEV;
    }

    /* create new CSMA_MAC thread */
    res = thread_create(stack, stacksize, priority, CREATE_STACKTEST,
                        _csma_mac_thread, (void *)dev, name);

    if (res <= 0) {
        return -EINVAL;
    }

    random_init();
    /* If timer event occures, _mac_send_statechart is called in ISR. */
    //timer_init(mac_tmr, us_per_tick, _mac_send_statechart);
    DEBUG("Timer and RNG successfull initialized.\n");
    return res;
}
