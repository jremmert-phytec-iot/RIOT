/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2016 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/*
 * @ingroup auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief   Auto initialization for kw2xrf network interfaces
 *
 * @author  Kaspar Schleiser <kaspar@schleiser.de>
 * @author  Jonas Remmert <j.remmert@phytec.de>
 */

#ifdef MODULE_KW2XRF

#include "board.h"
#include "net/gnrc/netdev2.h"
#include "net/gnrc/netdev2/ieee802154.h"
#include "net/gnrc.h"

#include "kw2xrf.h"
#include "kw2xrf_params.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @brief   Define stack parameters for the MAC layer thread
 * @{
 */
#define KW2XRF_MAC_STACKSIZE     (THREAD_STACKSIZE_DEFAULT)
#define KW2XRF_MAC_PRIO          (THREAD_PRIORITY_MAIN - 4)

#define KW2XRF_NUM (sizeof(kw2xrf_params)/sizeof(kw2xrf_params[0]))

static kw2xrf_t kw2xrf_devs[KW2XRF_NUM];
static gnrc_netdev2_t gnrc_adpt[KW2XRF_NUM];
static char _kw2xrf_stacks[KW2XRF_MAC_STACKSIZE][KW2XRF_NUM];

void auto_init_kw2xrf(void)
{
    for (int i = 0; i < KW2XRF_NUM; i++) {
        const kw2xrf_params_t *p = &kw2xrf_params[i];
        int res;

        DEBUG("Initializing KW2xrf radio at SPI_%i\n", p->spi);
        kw2xrf_setup(&kw2xrf_devs[i], (kw2xrf_params_t*) p);
        res = gnrc_netdev2_ieee802154_init(&gnrc_adpt[i],
                                           (netdev2_ieee802154_t *)&kw2xrf_devs[i]);
        if (res < 0) {
            DEBUG("Error initializing KW2xrf radio device!");
        }
        else {
            gnrc_netdev2_init(_kw2xrf_stacks[i],
                              KW2XRF_MAC_STACKSIZE,
                              KW2XRF_MAC_PRIO,
                              "kw2xrf",
                              &gnrc_adpt[i]);
        }
    }
}
#else
typedef int dont_be_pedantic;
#endif /* MODULE_GNRC_KW2XRF */

/** @} */
