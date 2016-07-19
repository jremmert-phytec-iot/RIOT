/*
 * Copyright (C) 2016 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_kw2xrf
 * @{
 * @file
 * @brief       Basic functionality of kw2xrf driver
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @}
 */
#include <stdint.h>
#include <string.h>

#include "mutex.h"
#include "msg.h"
#include "periph/gpio.h"
#include "periph/cpuid.h"
#include "net/gnrc.h"
#include "net/ieee802154.h"

#include "kw2xrf.h"
#include "kw2xrf_spi.h"
#include "kw2xrf_reg.h"
#include "kw2xrf_netdev.h"
#include "kw2xrf_getset.h"
#include "kw2xrf_intern.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#ifdef MODULE_GNRC_SIXLOWPAN
#define KW2XRF_PROTO    GNRC_NETTYPE_SIXLOWPAN
#elif MODULE_GNRC
#define KW2XRF_PROTO    GNRC_NETTYPE_UNDEF
#endif

#ifndef CPUID_LEN
#define CPUID_LEN       0
#endif

#if CPUID_LEN
/* make sure that the buffer is always big enough to store a 64bit value */
#if CPUID_LEN < IEEE802154_LONG_ADDRESS_LEN
#define KW2XRF_ID_LEN   IEEE802154_LONG_ADDRESS_LEN
#else
#define KW2XRF_ID_LEN   CPUID_LEN
#endif
#endif

static void kw2xrf_set_address(kw2xrf_t *dev)
{
    uint8_t cpuid[KW2XRF_ID_LEN];
    eui64_t addr_long;
    addr_long.uint64.u64 = KW2XRF_DEFAULT_ADDR_LONG;
    uint16_t addr_short = KW2XRF_DEFAULT_SHORT_ADDR;

    if (CPUID_LEN) {
        /* in case CPUID_LEN < 8, fill missing bytes with zeros */
        memset(cpuid, 0, CPUID_LEN);

        cpuid_get(cpuid);

	/* generate short hardware address if CPUID_LEN > 0 */
	for (int i = 0; i < CPUID_LEN; i++) {
	    /* XOR each even byte of the CPUID with LSB of short address
	       and each odd byte with MSB */
	    addr_short ^= (uint16_t)(cpuid[i] << ((i & 0x01) * 8));
	}

        for (int i = IEEE802154_LONG_ADDRESS_LEN; i < CPUID_LEN; i++) {
            cpuid[i & 0x07] ^= cpuid[i];
        }

        /* make sure we mark the address as non-multicast and not globally unique */
        cpuid[0] &= ~(0x01);
        cpuid[0] |= 0x02;
        /* copy and set long address */
        memcpy(&addr_long, cpuid, IEEE802154_LONG_ADDRESS_LEN);
    }
    kw2xrf_set_addr_long(dev, addr_long.uint64.u64);
    kw2xrf_set_addr_short(dev, addr_short);
}

void kw2xrf_setup(kw2xrf_t *dev, const kw2xrf_params_t *params)
{
    netdev2_t *netdev = (netdev2_t *)dev;

    netdev->driver = &kw2xrf_driver;
    /* initialize device descriptor */
    memcpy(&dev->params, params, sizeof(kw2xrf_params_t));
    dev->idle_state = XCVSEQ_RECEIVE;
    dev->state = 0;
    dev->pending_tx = 0;

    kw2xrf_spi_init(dev);
    kw2xrf_set_power_mode(dev, KW2XRF_IDLE);
    DEBUG("[kw2xrf]: setup finished\n");
}

int kw2xrf_init(kw2xrf_t *dev, gpio_cb_t cb)
{
    if (dev == NULL) {
        return -ENODEV;
    }

    kw2xrf_set_out_clk(dev);
    kw2xrf_disable_interrupts(dev);
    /* set up GPIO-pin used for IRQ */
    gpio_init_int(dev->params.int_pin, GPIO_IN, GPIO_FALLING, cb, dev);

    kw2xrf_abort_sequence(dev);
    kw2xrf_update_overwrites(dev);
    kw2xrf_timer_init(dev, KW2XRF_TIMEBASE_62500HZ);
    DEBUG("[kw2xrf]: init finished\n");

    return 0;
}

void kw2xrf_reset_phy(kw2xrf_t *dev)
{
    /* reset options and sequence number */
    dev->netdev.seq = 0;
    dev->netdev.flags = 0;
    /* set default protocol */
    dev->netdev.proto = KW2XRF_PROTO;

    dev->tx_power = KW2XRF_DEFAULT_TX_POWER;
    kw2xrf_set_tx_power(dev, dev->tx_power);

    kw2xrf_set_channel(dev, KW2XRF_DEFAULT_CHANNEL);

    kw2xrf_set_pan(dev, KW2XRF_DEFAULT_PANID);
    kw2xrf_set_address(dev);

    kw2xrf_set_cca_mode(dev, 1);

    kw2xrf_set_rx_watermark(dev, 1);

    kw2xrf_set_option(dev, KW2XRF_OPT_AUTOACK, true);
    kw2xrf_set_option(dev, KW2XRF_OPT_ACK_REQ, true);
    kw2xrf_set_option(dev, KW2XRF_OPT_AUTOCCA, true);

    kw2xrf_set_power_mode(dev, KW2XRF_AUTODOZE);
    kw2xrf_set_sequence(dev, dev->idle_state);

    kw2xrf_set_option(dev, KW2XRF_OPT_TELL_RX_START, true);
    kw2xrf_set_option(dev, KW2XRF_OPT_TELL_RX_END, true);
    kw2xrf_set_option(dev, KW2XRF_OPT_TELL_TX_END, true);
    kw2xrf_clear_dreg_bit(dev, MKW2XDM_PHY_CTRL2, MKW2XDM_PHY_CTRL2_SEQMSK);

    kw2xrf_enable_irq_b(dev);

    DEBUG("[kw2xrf]: Initialized and set to channel %d and pan %d.\n",
          KW2XRF_DEFAULT_CHANNEL, KW2XRF_DEFAULT_PANID);
}
