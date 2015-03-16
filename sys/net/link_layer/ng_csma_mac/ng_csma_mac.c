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
#include "net/ng_pktbuf.h"
#include "periph/timer.h"
#include "periph/random.h"
#include "net/ng_csma_mac.h"
#include "hwtimer.h"
#include "mutex.h"
#include "vtimer.h"

#define ENABLE_DEBUG        (0)

/* Enables integrated testing functions, such as LED-toggling
 * on state-change and idle-wait functions to test arbitration
 * using multiple boards.
 */
#define TESTING_FUNCTIONS   (1)
#include "debug.h"

typedef enum {
    CSMA_IDLE,
    CSMA_WAIT,
    CSMA_CCA_TX,
    CSMA_WAIT_FOR_ACK
} csma_mac_states_t;

static csma_mac_states_t csma_mac_state = CSMA_IDLE;
static tim_t mac_tmr = CSMA_MAC_TIMER;
static msg_t msg;
static mutex_t mutex = MUTEX_INIT;
static ng_netapi_opt_t *opt;
static uint8_t event_occured;
static uint8_t cca_failed;
static uint8_t dev_options;
static uint8_t ack_received;

/* needed as callback function for the timer-interface */
inline static void task_release(int tmr_dev){
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

    switch (event) {

   case NETDEV_EVENT_RX_COMPLETE:
        {
            ng_pktsnip_t *pkt;
            ng_netreg_entry_t *sendto;

            /* get pointer to the received packet */
            pkt = (ng_pktsnip_t *)data;
            /* find out, who to send the packet to */
            sendto = ng_netreg_lookup(pkt->type, NG_NETREG_DEMUX_CTX_ALL);
            /* throw away packet if no one is interested */
            if (sendto == NULL) {
                DEBUG("csma_mac: unable to forward packet of type %i\n", pkt->type);
                ng_pktbuf_release(pkt);
            }
            /* send the packet to everyone interested in it's type */
            ng_pktbuf_hold(pkt, ng_netreg_num(pkt->type, NG_NETREG_DEMUX_CTX_ALL) - 1);
            while (sendto != NULL) {
                DEBUG("csma_mac: sending pkt %p to PID %u\n", pkt, sendto->pid);
                ng_netapi_send(sendto->pid, pkt);
                sendto = ng_netreg_getnext(sendto);
            }
            /* TODO: Check if this receive event is really an ack packet. */
            ack_received = 1;
            mutex_unlock(&mutex); // unblock task in case of wait_for_ack
        }
        break;

   case NETDEV_EVENT_TX_COMPLETE:
        event_occured = 1;
        event = NETDEV_EVENT_TX_COMPLETE;
        cca_failed = 0;
        mutex_unlock(&mutex);
        break;

   case NETDEV_EVENT_CCA_CHANNEL_BUSY:
        event_occured = 1;
        event = NETDEV_EVENT_CCA_CHANNEL_BUSY;
        cca_failed = 1;
        mutex_unlock(&mutex);
        break;

   default:
        DEBUG("csma_mac-error: Unknown Event triggered\n");
        break;
        }
}

/**
 * @brief     Random wait function; max delay is determined with
 *            backoff exponent.
 *
 * @param[in] be    expects a number between
              CSMA_MAC_MIN_BE and CSMA_MAC_MAX_BE.
 */
static void backoff_wait(uint8_t be)
{
    uint16_t random = 0;
    uint16_t backoff_intervall = 0;
    char buf[2];

    /* backoff_intervall is defined as ((2^bf)-1)*CSMA_MAC_SYMBOL_LENGTH */
    backoff_intervall = 1;
    backoff_intervall <<= be;
    backoff_intervall--;
    backoff_intervall *= CSMA_MAC_UNIT_BACKOFF_PERIOD * CSMA_MAC_SYMBOL_LENGTH;

    /* Generate 16bit random number */
    if (random_read(buf, 2) != 2) {
        return;   /* what to return in case of random_read error? */
    }

    random = buf[0];
    random <<= 8;
    random = random | (0x00ff & buf[1]);

    if (backoff_intervall) {  /* avoid div/0 */
        backoff_intervall = random % backoff_intervall;
    }

    DEBUG("csma_mac-info: Backoff_wait:Random Backoff_interval = %ius\n"\
    , backoff_intervall);

    #if (TESTING_FUNCTIONS)
    LED_B_ON; /* Measurement indication for Wait duration */
    #endif
    if (backoff_intervall > 10) {   /* Spinlock barrier for Timer, skip wait because span is to short */
        timer_reset(mac_tmr);
        timer_set(mac_tmr, CSMA_MAC_TIMER_CH, backoff_intervall);
        mutex_lock(&mutex);
        mutex_lock(&mutex);
        mutex_unlock(&mutex);
        timer_clear(mac_tmr, CSMA_MAC_TIMER_CH);
    }
    #if (TESTING_FUNCTIONS)
    LED_B_OFF; /* Measurement indication for Wait duration */
    #endif
    return;
}

/**
 * @brief            Random wait function; max delay is defined by parameter.
 *
 * @param[in] max    maximum wait-time.
 */
#if (TESTING_FUNCTIONS)
static void random_wait(uint16_t max)
{
    uint16_t random = 0;
    char buf[2];

    /* Generate 16bit random number */
    if (random_read(buf, 2) != 2) {
        return;   /* what to return in case of random_read error? */
    }

    random = buf[0];
    random <<= 8;
    random = random | (0x00ff & buf[1]);

    if (max) {  /* avoid div/0 */
        max = random % max;
    }

    LED_G_ON; /* Measurement indication for Wait duration */
    vtimer_usleep(max);
    LED_G_OFF; /* Measurement indication for Wait duration */
    return;
}
#endif

/**
 * @brief     If the mac layer needs to send out a packet,
 *            this state-machine handles the send process.
 *
 * @return number of bytes that were actually send out; -1 if packet could not sent out.
 */
static int _mac_send_statemachine(ng_netdev_t *dev, ng_pktsnip_t *pkt)
{
    ng_netconf_state_t res;
    unsigned int state = 0;          /* Used for IRQ disable and restore */
    uint8_t backoff_exponent = 0;
    uint8_t retries = 0;
    csma_mac_state = CSMA_IDLE;

    #if (TESTING_FUNCTIONS)
    uint16_t random_max_val = 16000; /* wait max 4ms */
    #endif

    event_occured = 0;
    while (1) {
        switch (csma_mac_state) {

            case CSMA_IDLE:
                DEBUG("csma_mac-state: CSMA_IDLE\n");
                backoff_exponent= CSMA_MAC_MIN_BE;
                retries = 0;
                ack_received = 0;
                csma_mac_state = CSMA_CCA_TX;
                res = dev->driver->send_data(dev, pkt);
                if (res < 0) {
                    return res;
                }
                break;

            case CSMA_CCA_TX:
                DEBUG("csma_mac-state: CSMA_CCA_TX\n");

                opt->opt = NETCONF_OPT_STATE;
                int tmp = NETCONF_STATE_TX;
                opt->data = &tmp;
                res = dev->driver->set(dev,
                                     opt->opt, opt->data,(size_t)opt->data_len);
                if (res != sizeof(opt->data_len)) {
                    return res;
                }
                csma_mac_state = CSMA_WAIT;
                break;

            case CSMA_WAIT:
                DEBUG("csma_mac-state: CSMA_WAIT.\n");
                state = disableIRQ();
                if (event_occured == 1) {
                    event_occured = 0;
                    restoreIRQ(state);
                }
                else {
                    mutex_lock(&mutex);
                    restoreIRQ(state);
                    mutex_lock(&mutex);
                    mutex_unlock(&mutex);
                    event_occured = 0;
                }

                if (cca_failed) {
                    DEBUG("csma_mac-info: CCA: Channel busy,\
                     backoff_exponent = %i\n", backoff_exponent);
                    csma_mac_state = CSMA_CCA_TX;
                    backoff_wait(backoff_exponent);
                    backoff_exponent++;

                    if (backoff_exponent > CSMA_MAC_MAX_BE) {     /* return error code */
                        DEBUG("csma_mac-error: CCA Failed, max retries reached. Exiting with -EBUSY.\n");
                        return -EBUSY;                            /* Resource busy */
                    }
                }
                else {
                    csma_mac_state = CSMA_WAIT_FOR_ACK;
                }
                break;

            case CSMA_WAIT_FOR_ACK:
                DEBUG("csma_mac-state: CSMA_WAIT_FOR_ACK.\n");
                /* TODO: Set RX state but only react on ACK, maybe introduce a new state */
                opt->opt = NETCONF_OPT_STATE;
                tmp = NETCONF_STATE_RX;
                opt->data = &tmp;
                res = dev->driver->set(dev,
                                     opt->opt, opt->data,(size_t)opt->data_len);
                if (res != sizeof(opt->data_len)) {
                    return res;
                }
                state = disableIRQ();
                if (event_occured == 1) {
                    event_occured = 0;
                    restoreIRQ(state);
                }
                else {
                    timer_reset(mac_tmr);
                    timer_set(mac_tmr, CSMA_MAC_TIMER_CH, CSMA_MAC_MAX_ACK_WAIT_DURATION);
                    mutex_lock(&mutex);
                    LED_G_ON; /* Measurement indication for Wait duration */
                    restoreIRQ(state);
                    mutex_lock(&mutex);
                    mutex_unlock(&mutex);
                    LED_G_OFF; /* Measurement indication for Wait duration */
                    timer_clear(mac_tmr, CSMA_MAC_TIMER_CH);
                    event_occured = 0;
                }

                if (ack_received) {
                    csma_mac_state = CSMA_IDLE;
                    #if (TESTING_FUNCTIONS)
                    random_wait(random_max_val);
                    #endif
                    DEBUG("Packet successfull delivered\n");
                    return 0;     /* Successfull */
                }
                /* In case of no ACK, proceed in state machine */
                retries ++;

                if (retries > CSMA_MAC_MAX_FRAME_RETRIES) {
                    backoff_exponent = CSMA_MAC_MIN_BE;
                    csma_mac_state = CSMA_CCA_TX;
                    DEBUG("csma_mac-error: Ack failed, retries: %i\n", retries);
                    break;
                }
                else {
                    DEBUG("csma_mac-error: Ack failed, max retries reached. Exiting with -EBUSY.\n");
                    return -EBUSY;                  /* Resource busy */
                }
                break;

            default:
                DEBUG("csma_mac-error: Default state, error in state machine\n");
                return -ENOTSUP;
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
    ng_netdev_t *dev = (ng_netdev_t *)args;
    int res;
    msg_t reply, msg_queue[CSMA_MAC_MSG_QUEUE_SIZE];
    dev_options=0;

    /* TODO: Initialization, move to main or so */
    ng_netapi_opt_t index_init;
    index_init.data_len=1;
    uint8_t index_data;
    index_init.data = &index_data;
    ng_netapi_opt_t *init = &index_init;

    /* Check for CCA before TX option */
    init->opt = NETCONF_OPT_CCA_BEFORE_TX;
    res = dev->driver->set(dev, init->opt, init->data,(size_t)init->data_len);
    if (res != sizeof(opt->data_len)) {
        DEBUG("csma_mac-error: set returned %i\n", res);
    }
    dev_options |= CSMA_MAC_DEV_SUPPORTS_AUTO_CCA;
    if(res == -ENOTSUP) {
        DEBUG("csma_mac-error: Radio device does not support automatic CCA before TX!\
                 MAC layer will perform send without CSMA instead.\n");
        dev_options &= ~(CSMA_MAC_DEV_SUPPORTS_AUTO_CCA);
    }

    /* Check for Auto Ack after RX option */
    init->opt = NETCONF_OPT_AUTOACK;
    res = dev->driver->set(dev, init->opt, init->data,(size_t)init->data_len);
    if (res != sizeof(opt->data_len)) {
        DEBUG("csma_mac-error: set returned %i\n", res);
    }
    dev_options |= CSMA_MAC_DEV_SUPPORTS_AUTO_ACK;
    if(res == -ENOTSUP) {
        DEBUG("csma_mac-error: Radio device does not support automatic CCA before TX!\
                 MAC layer will perform send without CSMA instead.\n");
        dev_options &= ~(CSMA_MAC_DEV_SUPPORTS_AUTO_ACK);
    }

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, CSMA_MAC_MSG_QUEUE_SIZE);
    /* save the PID to the device descriptor and register the device */
    dev->mac_pid = thread_getpid();
    ng_netif_add(dev->mac_pid);
    /* register the event callback with the device driver */
    dev->driver->add_event_callback(dev, _event_cb);

    /* start the event loop */
    while (1) {
        //DEBUG("csma_mac: waiting for incoming messages\n");
        msg_receive(&msg);
        opt = (ng_netapi_opt_t * )msg.content.ptr;
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NG_NETDEV_MSG_TYPE_EVENT:
                DEBUG("csma_mac: NG_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr_event(dev, msg.content.value);
                break;

            case NG_NETAPI_MSG_TYPE_SND:
                DEBUG("csma_mac: NG_NETAPI_MSG_TYPE_SND received\n");
                if (dev_options & CSMA_MAC_DEV_SUPPORTS_AUTO_CCA) {
                    res = _mac_send_statemachine(dev, (ng_pktsnip_t *)msg.content.ptr);
                    if(res < 0) {
                        DEBUG("csma_mac-error: send_statemachine returned error code: %i\n", res);
                    }
                }
                else {          /* Send data without CSMA algorithm if device
                                    is not capable of doing CCA before TX */
                    dev->driver->send_data(dev, (ng_pktsnip_t *)msg.content.ptr);
                    /* TODO: Check for the correct number of bytes, send function returnes */
                    if (res < 0) {
                        DEBUG("csma_mac-error: send_data returned %i\n", res);
                    }
                }
                //reply.type = ?NG_NETAPI_MSG_TYPE_ACK?;
                //reply.content.value = (uint32_t)res;
                //msg_reply(&msg, &reply);
                /* TODO: Inform upper layer that channel is busy */
                DEBUG("csma_mac: Channel busy\n");
                break;
            case NG_NETAPI_MSG_TYPE_SET:
                /* TODO: filter out MAC layer options -> for now forward
                         everything to the device driver */
                DEBUG("nomac: NG_NETAPI_MSG_TYPE_SET received\n");
                /* read incoming options */
                opt = (ng_netapi_opt_t *)msg.content.ptr;
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                if (res != sizeof(opt->data_len)) {
                    DEBUG("csma_mac-error: set returned %i\n", res);
                }
                /* send reply to calling thread */
                reply.type = NG_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case NG_NETAPI_MSG_TYPE_GET:
                /* TODO: filter out MAC layer options -> for now forward
                         everything to the device driver */
                DEBUG("nomac: NG_NETAPI_MSG_TYPE_GET received\n");
                /* read incoming options */
                opt = (ng_netapi_opt_t *)msg.content.ptr;
                /* get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                /* send reply to calling thread */
                reply.type = NG_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            default:
                DEBUG("nomac: Unknown command %" PRIu16 "\n", msg.type);
                break;
        }
    }
    /* never reached */
    return NULL;
}

kernel_pid_t csma_mac_init(char *stack, int stacksize, char priority,
                               const char *name, ng_netdev_t *dev)
{
    unsigned int us_per_tick = 1;
    kernel_pid_t res;

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
    /* If timer event occures, _mac_send_statemachine is called in ISR. */
    timer_init(mac_tmr, us_per_tick, task_release);
    timer_start(mac_tmr);
    DEBUG("csma_mac-info: Timer and RNG successfull initialized.\n");
    return res;
}
