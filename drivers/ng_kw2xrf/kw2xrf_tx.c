/*
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_kw2xrf
 * @{
 * @file        kw2xrf_tx.c
 * @brief       Transmit functionality of kw2xrf driver
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 */
#include <stdio.h>
#include <string.h>

#include "ng_kw2xrf.h"
#include "kw2xrf_reg.h"
#include "kw2xrf_spi.h"
#include "kw2xrf_internal.h"
#include "ieee802154_frame.h"

#include "irq.h"
#include "hwtimer.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"
#define MKW2XDRF_ACK_WAIT_DELAY_uS   1000
#define ACK_LENGTH  5

static void kw2xrf_gen_pkt(uint8_t *buf, kw2xrf_packet_t *packet);
static uint8_t sequence_nr;

int16_t kw2xrf_send(kw2xrf_packet_t *packet)
{

    /* - FCS + one octet for FRAME_LEN */
    uint8_t pkt[MKW2XDRF_MAX_PKT_LENGTH - 1];

    /* Set missing frame information */
    packet->frame.fcf.frame_ver = 0;

    if (packet->frame.src_pan_id == packet->frame.dest_pan_id) {
        packet->frame.fcf.panid_comp = 1;
    }
    else {
        packet->frame.fcf.panid_comp = 0;
    }

    if (packet->frame.fcf.src_addr_m == 2) {
	uint16_t src_addr = kw2xrf_get_address();
        packet->frame.src_addr[0] = (uint8_t)(src_addr >> 8);
        packet->frame.src_addr[1] = (uint8_t)(src_addr & 0xFF);
    }
    else if (packet->frame.fcf.src_addr_m == 3) {
	uint64_t src_addr = kw2xrf_get_address_long();
        packet->frame.src_addr[0] = (uint8_t)(src_addr >> 56);
        packet->frame.src_addr[1] = (uint8_t)(src_addr >> 48);
        packet->frame.src_addr[2] = (uint8_t)(src_addr >> 40);
        packet->frame.src_addr[3] = (uint8_t)(src_addr >> 32);
        packet->frame.src_addr[4] = (uint8_t)(src_addr >> 24);
        packet->frame.src_addr[5] = (uint8_t)(src_addr >> 16);
        packet->frame.src_addr[6] = (uint8_t)(src_addr >> 8);
        packet->frame.src_addr[7] = (uint8_t)(src_addr & 0xFF);
    }

    packet->frame.src_pan_id = kw2xrf_get_pan();
    packet->frame.seq_nr = sequence_nr;

    sequence_nr += 1;

    /* calculate size of the package (header + payload + fcs) */
    packet->length = ieee802154_frame_get_hdr_len(&packet->frame) +
                     packet->frame.payload_len + 2;

    if (packet->length > MKW2XDRF_MAX_PKT_LENGTH) {
        return -1;
    }

    if (kw2xrf_read_dreg(MKW2XDM_SEQ_STATE)) {
        /* abort any ongoing sequence */
        DEBUG("tx: abort SEQ_STATE: %x\n", kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));
        kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, MKW2XDM_PHY_CTRL1_XCVSEQ(0));
    	while (kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));
    }

    /* generate pkt */
    pkt[0] = packet->length;
    kw2xrf_gen_pkt((pkt + 1), packet);
    kw2xrf_write_fifo(pkt, packet->length - 1);

    kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_TXIRQ);
    /* programm TR sequence */
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, MKW2XDM_PHY_CTRL1_XCVSEQ(2));
    while (!(kw2xrf_read_dreg(MKW2XDM_IRQSTS1) & MKW2XDM_IRQSTS1_TXIRQ));
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, MKW2XDM_PHY_CTRL1_XCVSEQ(0));

    kw2xrf_switch_to_rx();

    return packet->length;
}

static void kw2xrf_gen_pkt(uint8_t *buf, kw2xrf_packet_t *packet)
{
    uint8_t index, offset;
    index = ieee802154_frame_init(&packet->frame, buf);
    offset = index;

    while (index < packet->length - 2) {
        buf[index] = packet->frame.payload[index - offset];
        index += 1;
    }
}
/** @} */
