/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for Xbee S1 network device driver
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "board.h"
#include "kernel.h"
#include "shell.h"
#include "shell_commands.h"
#include "ng_at86rf2xx.h"
#include "net/ng_netbase.h"
#include "net/ng_nomac.h"
#include "net/ng_pktdump.h"

/**
 * @brief   Buffer size used by the shell
 */
#define SHELL_BUFSIZE           (64U)

/**
 * @brief   The Xbee device descriptor
 */
static ng_at86rf2xx_t dev;

/**
 * @brief   Stack for the nomac thread
 */
static char nomac_stack[KERNEL_CONF_STACKSIZE_DEFAULT];

/**
 * @brief   Read chars from STDIO
 */
int shell_read(void)
{
    return (int)getchar();
}

/**
 * @brief   Write chars to STDIO
 */
void shell_put(int c)
{
    putchar((char)c);
}

/**
 * @brief   Maybe you are a golfer?!
 */
int main(void)
{
    kernel_pid_t iface;
    shell_t shell;
    ng_netreg_entry_t dump;

    puts("AT86rf2xx device driver test");
    printf("Initializing the at86rf2xx device\n");

    /* initialize network module(s) */
    ng_netif_init();

    /* initialize and register pktdump */
    dump.pid = ng_pktdump_getpid();
    if (dump.pid <= KERNEL_PID_UNDEF) {
        puts("Error starting pktdump thread");
        return -1;
    }
    dump.demux_ctx = NG_NETREG_DEMUX_CTX_ALL;
    ng_netreg_register(NG_NETTYPE_UNDEF, &dump);

    /* setup  at86rf2xx device */
    ng_at86rf2xx_initialise(&dev);

    /* start MAC layer */
    iface = ng_nomac_init(nomac_stack, sizeof(nomac_stack), PRIORITY_MAIN - 3,
                          "xbee_l2", (ng_netdev_t *)&dev);
    if (iface <= KERNEL_PID_UNDEF) {
        puts("Error initializing MAC layer");
        return -1;
    }

    /* start the shell */
    shell_init(&shell, NULL, SHELL_BUFSIZE, shell_read, shell_put);
    shell_run(&shell);

    printf("Done Initializing...");
    return 0;
}
