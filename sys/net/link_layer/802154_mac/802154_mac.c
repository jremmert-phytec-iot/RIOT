/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_nomac
 * @file
 * @brief       Implementation of the NOMAC MAC protocol
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @}
 */

#include <errno.h>

#include "kernel.h"
#include "msg.h"
#include "thread.h"
#include "net/ng_nomac.h"
#include "net/ng_netbase.h"
#include "net/ng_netif.h"
#include "net/ieee802154_frame.h"

#define ENABLE_DEBUG    (0)
#define INTERNAL_MSG_QUEUE_SIZE 100
#include "debug.h"
typedef enum {
        IDLE,
        WAIT,
        ASSOCIATE,
    } mac_states_t;

static mac_states_t mac_state = IDLE;
static msg_t internal_msg_queue[INTERNAL_MSG_QUEUE_SIZE];
static int internal_msg_queue_index;

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event         type of event
 * @param[in] data          optional parameter
 */
static void _event_cb(ng_netdev_event_t event, void *data)
{
    DEBUG("nomac: event triggered -> %i\n", event);
    /* NOMAC only understands the RX_COMPLETE event... */
    if (event == NETDEV_EVENT_RX_COMPLETE) {
        ng_pktsnip_t *pkt;
        ng_netreg_entry_t *sendto;

        /* get pointer to the received packet */
        pkt = (ng_pktsnip_t *)data;
        /* find out, who to send the packet to */
        sendto = ng_netreg_lookup(pkt->type, NG_NETREG_DEMUX_CTX_ALL);
        /* throw away packet if no one is interested */
        if (sendto == NULL) {
            DEBUG("nomac: unable to forward packet of type %i\n", pkt->type);
            ng_pktbuf_release(pkt);
            return;
        }
        /* send the packet to everyone interested in it's type */
        ng_pktbuf_hold(pkt, ng_netreg_num(pkt->type, NG_NETREG_DEMUX_CTX_ALL) - 1);
        while (sendto != NULL) {
            DEBUG("nomac: sending pkt %p to PID %u\n", (void*)pkt, sendto->pid);
            ng_netapi_receive(sendto->pid, pkt);
            sendto = ng_netreg_getnext(sendto);
        }
    }
    /* RAW-data-frame, MAC-layer has to process it*/
    if (event == NETDEV_EVENT_RX_COMPLETE) {
        DEBUG("802154 MAC: Raw Pkt received \n");
        ng_pktsnip_t *pkt;
        pkt = (ng_pktsnip_t *)data;
    /* What to do with packet now? */
    }
}

int _mlme_scan(ng_netdev_t *dev, ng_netconf_mlme_attributes_t *param)
{
    int res = 0;

    DEBUG("802154_mac: mlme scan\n");

    /* Iterate through all requested channels */
    for(int i = 0; i<MAX_CHANNELS; i++) {
        if(param->scan_channels[i]) {
            res = dev->driver->set(dev, NETCONF_OPT_CHANNEL, &(param->scan_channels[i]), 1);
            DEBUG("802154_mac: Channel %i scanned\n", param->scan_channels[i]);
            if(res < 0) {
                DEBUG("802154_mac: Error when scanning channel, returned %i\n", res);
            }
            }
    }
    return 0;
}

/* The association to a pan coordinator is described in 5.1.3.1 IEEE802.15.4 Standard */
int _mlme_associate(ng_netdev_t *dev, ng_netconf_mlme_attributes_t *param)
{
    int res = 0;
    ieee802154_frame_type_t type;
    uint8_t frame_buf[127];
    ng_pktsnip_t *pkt;

    DEBUG("802154_mac: mlme associate\n");

    /* set channel */
    res = dev->driver->set(dev, NETCONF_OPT_CHANNEL,
        &(param->channel_number), sizeof(param->channel_number));

    /* set NID */
    res = dev->driver->set(dev, NETCONF_OPT_NID, &(param->coord_pan_id), 2);

    ieee802154_frame_t ng_802154_hdr;

    /* Basic initialization for testing */
    type = IEEE_802154_MAC_CMD_FRAME;
    ng_802154_hdr.fcf.frame_type = type;
    ng_802154_hdr.fcf.sec_enb = 0;
    ng_802154_hdr.fcf.frame_pend = 0;
    ng_802154_hdr.fcf.ack_req = 0;
    ng_802154_hdr.fcf.panid_comp = 0;
    ng_802154_hdr.fcf.dest_addr_m = 2;  /* short addr mode */
    ng_802154_hdr.fcf.frame_ver = 1;    /* 802154 2006 frame, 0 for 2003 version */
    ng_802154_hdr.fcf.src_addr_m = 2;   /* short addr mode */

    ng_802154_hdr.dest_pan_id = 0xffff;
    ng_802154_hdr.dest_addr[0] = 0xca;  /* Fill in here the address of the received beacon */
    ng_802154_hdr.dest_addr[1] = 0xfe;

    ieee802154_frame_print_fcf_frame(&ng_802154_hdr);
    uint8_t hdr_size = ieee802154_frame_get_hdr_len(&ng_802154_hdr);
    ieee802154_frame_init(&ng_802154_hdr, frame_buf);

    printf("size frame: %i\n", hdr_size);
    //pkt = ng_pktbuf_add(NULL, send_content, sizeof(send_content), NG_NETTYPE_UNDEF);
    pkt = ng_pktbuf_add(NULL, frame_buf, hdr_size, NG_NETTYPE_802154);
    //pkt = ng_pktbuf_add(NULL, nethdr, sizeof(ng_netif_hdr_t),
    //                        NG_NETTYPE_UNDEF);

    dev->driver->send_data(dev, pkt);
    return 0;
}

/**
 * @brief   Startup code and event loop of the NOMAC layer
 *
 * @param[in] args          expects a pointer to the underlying netdev device
 *
 * @return                  never returns
 */
static void *_nomac_thread(void *args)
{
    ng_netdev_t *dev = (ng_netdev_t *)args;
    ng_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[NG_NOMAC_MSG_QUEUE_SIZE];

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, NG_NOMAC_MSG_QUEUE_SIZE);
    /* save the PID to the device descriptor and register the device */
    dev->mac_pid = thread_getpid();
    ng_netif_add(dev->mac_pid);
    /* register the event callback with the device driver */
    dev->driver->add_event_callback(dev, _event_cb);
    /* Activate RAW-Receive mode */
    bool tmp = NETCONF_ENABLE;
    res = dev->driver->set(dev, NETCONF_OPT_RAWMODE,
        &tmp, sizeof(ng_netconf_enable_t));

    /* start the event loop */
    while (1) {
        DEBUG("nomac: waiting for incoming messages\n");
        if((mac_state == IDLE) && (internal_msg_queue_index !=0)) {
            msg = internal_msg_queue[internal_msg_queue_index];
            internal_msg_queue_index--;
            }
        else {
            msg_receive(&msg);
            }
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NG_NETDEV_MSG_TYPE_EVENT:
                DEBUG("nomac: NG_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr_event(dev, msg.content.value);
                break;
            case NG_NETAPI_MSG_TYPE_SND:
                DEBUG("nomac: NG_NETAPI_MSG_TYPE_SND received\n");
                if(mac_state == IDLE) {
                    dev->driver->send_data(dev, (ng_pktsnip_t *)msg.content.ptr);
                }
                /* MAC layer is currently busy, or waits for events, queue incoming requests */
                if(internal_msg_queue_index < INTERNAL_MSG_QUEUE_SIZE) {
                    internal_msg_queue_index++;
                    internal_msg_queue[internal_msg_queue_index] = msg;
                }
                else {
                    DEBUG("802154_mac: error internal queue full\n");
                }
                break;
            case NG_NETAPI_MSG_TYPE_SET:
                /* TODO: filter out MAC layer options -> for now forward
                         everything to the device driver */
                DEBUG("nomac: NG_NETAPI_MSG_TYPE_SET received\n");
                /* read incoming options */
                opt = (ng_netapi_opt_t *)msg.content.ptr;
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("nomac: response of netdev->set: %i\n", res);
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
                if(opt->opt == NETCONF_OPT_MLME_SCAN) {
                    res = _mlme_scan(dev, (ng_netconf_mlme_attributes_t *)opt->data);
                    reply.type = NG_NETAPI_MSG_TYPE_ACK;
                    reply.content.value = (uint32_t)res;
                }
                if(opt->opt == NETCONF_OPT_MLME_ASSOCIATE) {
                    res = _mlme_associate(dev, (ng_netconf_mlme_attributes_t *)opt->data);
                    reply.type = NG_NETAPI_MSG_TYPE_ACK;
                    reply.content.value = (uint32_t)res;
                }
                else {
                    res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                    DEBUG("nomac: response of netdev->get: %i\n", res);
                    /* send reply to calling thread */
                    reply.type = NG_NETAPI_MSG_TYPE_ACK;
                    reply.content.value = (uint32_t)res;
                }
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

kernel_pid_t ng_nomac_init(char *stack, int stacksize, char priority,
                        const char *name, ng_netdev_t *dev)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (dev == NULL || dev->driver == NULL) {
        return -ENODEV;
    }
    /* create new NOMAC thread */
    res = thread_create(stack, stacksize, priority, CREATE_STACKTEST,
                         _nomac_thread, (void *)dev, name);
    if (res <= 0) {
        return -EINVAL;
    }
    return res;
}
