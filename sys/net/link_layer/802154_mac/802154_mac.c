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

#define ENABLE_DEBUG    (1)
#include "debug.h"

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
}

int _mlme_scan(ng_netdev_t *dev, ng_netconf_mlme_attributes_t *param)
{
    //msg_t msg;
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
    uint8_t send_content[] = "test";
    ieee802154_frame_type_t type;

    DEBUG("802154_mac: mlme associate\n");

    /* set channel */
    res = dev->driver->set(dev, NETCONF_OPT_CHANNEL,
        &(param->channel_number), sizeof(param->channel_number));

    /* set channel */
    res = dev->driver->set(dev, NETCONF_OPT_NID, &(param->coord_pan_id), 2);

    /* set addr */
    //ng_netif_hdr_t ng_netif_hdr;
    //ng_netif_hdr_t *nethdr = &ng_netif_hdr;
    //ng_netif_hdr_init(nethdr, 2, 2);
    //ng_netif_hdr_set_dst_addr(nethdr, &(param->coord_address[0]), 2);


typedef struct __attribute__((packed)) {
    ieee802154_frame_type_t frame_type;
    uint8_t sec_enb;
    uint8_t frame_pend;
    uint8_t ack_req;
    uint8_t panid_comp;
    uint8_t dest_addr_m;
    uint8_t frame_ver;
    uint8_t src_addr_m;
} ieee802154_frame_fcf_frame_t;

typedef struct __attribute__((packed)) {
    ieee802154_frame_fcf_frame_t fcf;
    uint8_t seq_nr;
    uint16_t dest_pan_id;
    uint8_t dest_addr[8];
    uint16_t src_pan_id;
    uint8_t src_addr[8];
    uint8_t *payload;
    uint8_t payload_len;
} ieee802154_frame_t;
    ieee802154_frame_t ng_802154_hdr;

    /* Basic initialization for testing */
    type = IEEE_802154_DATA_FRAME;
    ng_802154_hdr.fcf.frame_type = type;
    ng_802154_hdr.fcf.sec_enb = 0;
    ng_802154_hdr.fcf.frame_pend = 0;
    ng_802154_hdr.fcf.ack_req = 0;
    ng_802154_hdr.fcf.panid_comp = 0;
    ng_802154_hdr.fcf.dest_addr_m = 2; /* short addr mode */
    ng_802154_hdr.fcf.frame_ver = 1; /* 802154 frame */
    ng_802154_hdr.fcf.src_addr_m = 2; /* short addr mode */

    ng_802154_hdr.dest_pan_id = 0x0001;
    ng_802154_hdr.dest_addr[0] = 0x01;
    ng_802154_hdr.dest_addr[1] = 0x00;

    ieee802154_frame_t *hdr_802154 = &ng_802154_hdr;

    ng_pktsnip_t *pkt;

    //pkt = ng_pktbuf_add(NULL, send_content, sizeof(send_content), NG_NETTYPE_UNDEF);
    pkt = ng_pktbuf_add(NULL, hdr_802154, sizeof(ieee802154_frame_t *),
                            NG_NETTYPE_802154);
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

    /* start the event loop */
    while (1) {
        DEBUG("nomac: waiting for incoming messages\n");
        msg_receive(&msg);
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NG_NETDEV_MSG_TYPE_EVENT:
                DEBUG("nomac: NG_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr_event(dev, msg.content.value);
                break;
            case NG_NETAPI_MSG_TYPE_SND:
                DEBUG("nomac: NG_NETAPI_MSG_TYPE_SND received\n");
                dev->driver->send_data(dev, (ng_pktsnip_t *)msg.content.ptr);
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
