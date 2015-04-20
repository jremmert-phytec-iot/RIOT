/*
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     tests
 * @file
 * @brief       Test application for KW2x network device driver
 *
 * @author      Jonas Remmert <j.remmert@phytec.de>
 * @}
 */

#include <stdio.h>
#include "board.h"
#include "kernel.h"
#include "shell.h"
#include "shell_commands.h"
#include "kw2xrf.h"
#include "net/ng_netbase.h"
#include "net/ng_pktdump.h"
#include "net/802154_mac.h"
#include "net/ng_netif.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define SHELL_BUFSIZE   (UART0_BUFSIZE)

/**
 * @brief   Stack for the nomac thread
 */
static char nomac_stack[KERNEL_CONF_STACKSIZE_DEFAULT];

static kw2xrf_t dev;

int shell_read(void)
{
    return (int)getchar();
}

void shell_put(int c)
{
    putchar((char)c);
}

int main(void)
{
    kernel_pid_t iface;
    int res;
    shell_t shell;
    ng_netreg_entry_t dump;
    ng_netconf_mlme_attributes_t mlme;

    puts("kw2xrf device driver test");

    /* initialize network module(s) */
    ng_netif_init();

    dump.pid = ng_pktdump_init();

    if (dump.pid <= KERNEL_PID_UNDEF) {
        puts("Error starting pktdump thread");
        return -1;
    }

    dump.demux_ctx = NG_NETREG_DEMUX_CTX_ALL;
    ng_netreg_register(NG_NETTYPE_UNDEF, &dump);

    /* setup KW2x device */
    res = kw2xrf_init(&dev);

    if (res < 0) {
        puts("Error initializing kw2xrf device driver");
        return -1;
    }

    /* start MAC layer */
    iface = ng_nomac_init(nomac_stack, sizeof(nomac_stack), PRIORITY_MAIN - 3,
                          "kw2xrf", (ng_netdev_t *)&dev);

    if (iface <= KERNEL_PID_UNDEF) {
        puts("Error initializing MAC layer");
        return -1;
    }

    /* register network device */
    res = ng_netif_add(iface);

    if (res < 0) {
        puts("Error registering network device with netif");
        return -1;
    }

    for(int i = 0; i< MAX_CHANNELS; i++) {
        mlme.scan_channels[i] = 0;
    }
    mlme.scan_channels[1] = 13;
    mlme.coord_pan_id[0] = 0x01;
    mlme.coord_pan_id[1] = 0x00;
    mlme.coord_address[0] = 0x01;
    mlme.coord_address[1] = 0x00;
    mlme.channel_number = 19;

    //res = ng_netapi_get(iface, NETCONF_OPT_MLME_SCAN, 0, &mlme, sizeof(ng_netconf_mlme_attributes_t));
    //DEBUG("802154_mac: scan returned %i\n", res);

    res = ng_netapi_get(iface, NETCONF_OPT_MLME_ASSOCIATE, 0, &mlme, sizeof(ng_netconf_mlme_attributes_t));
    DEBUG("802154_mac: associate returned %i\n", res);

    
    /* start the shell */
    shell_init(&shell, NULL, SHELL_BUFSIZE, shell_read, shell_put);
    shell_run(&shell);

    return 0;
}
