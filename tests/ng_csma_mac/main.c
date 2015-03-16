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
 * @brief       Tests for CSMA_MAC layer
 *
 * @author      Jonas Remmert <j.remmert@phytec.de>
 * @}
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>

#include "board.h"
#include "kernel.h"
#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "net/ng_csma_mac.h"
#include "net/ng_netapi.h"
#include "net/ng_netif.h"
#include "net/ng_netconf.h"
#include "net/ng_pkt.h"
#include "net/ng_pktbuf.h"
#include "net/ng_netif/hdr.h"
#include "ng_kw2xrf.h"

#include "net/ng_nomac.h"
#include "net/ng_netif.h"

#define UNITTESTS_CSMA_MAC_STACKSIZE (KERNEL_CONF_STACKSIZE_DEFAULT)
#define UNITTESTS_NOMAC_NAME        "unittests_csma_mac"

#define SHELL_TEST      (0)
#define ENABLE_DEBUG    (0)
#include "debug.h"

static char csma_mac_stack[KERNEL_CONF_STACKSIZE_DEFAULT];
static kernel_pid_t ng_csma_mac_pid = KERNEL_PID_UNDEF;

static kw2xrf_t kw2xrf_dev;

#define SHELL_BUFSIZE   (UART0_BUFSIZE)
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
    kw2xrf_dev.driver = &kw2xrf_driver;
#if(SHELL_TEST == 0)
    msg_t msg;

    int ret = 0;
    int msg_counter = 0;
    uint8_t send_content[] = "test";
    uint8_t addr[8];
    addr[1] = 0;
    addr[0] = 1;

    ng_netif_hdr_t ng_netif_hdr;
    ng_netif_hdr_t *nethdr = &ng_netif_hdr;
    ng_netif_hdr_init(nethdr, 2, 2);
    ng_netif_hdr_set_dst_addr(nethdr, addr, sizeof(addr));

    ng_pktsnip_t *pkt;

    pkt = ng_pktbuf_add(NULL, send_content, sizeof(send_content), NG_NETTYPE_UNDEF);
    pkt = ng_pktbuf_add(pkt, nethdr, sizeof(ng_netif_hdr_t),
                            NG_NETTYPE_UNDEF);

    msg.content.ptr = (char *)pkt;
    puts("\nRIOT netdev test");
    puts("Starting csma_mac thread\n");

    /* TODO: Initialize driver completely, with cb-functions */
    /* There is an error that cb-function is called before csma_mac thread started.*/
    kw2xrf_init((ng_netdev_t *)&kw2xrf_dev);
    msg.type = NG_NETAPI_MSG_TYPE_SND;

    ng_csma_mac_pid = csma_mac_init(csma_mac_stack,
                           KERNEL_CONF_STACKSIZE_DEFAULT,
                           PRIORITY_MAIN - 1, UNITTESTS_NOMAC_NAME,
                           (ng_netdev_t *)&kw2xrf_dev);


    while(1){
        msg.content.ptr = (char*)pkt;
        ret = msg_send(&msg, ng_csma_mac_pid);
        if(ret){
            msg_counter++;
        }
    }
#else
    kernel_pid_t iface;
    int res;
    shell_t shell;

    puts("kw2xrf device driver test");
    /* initialize network modules */
    ng_netif_init();
    /* setup kw2xrf device */
    res = kw2xrf_init((ng_netdev_t *)&kw2xrf_dev);
    if (res < 0) {
        puts("Error initializing kw2xrf device driver");
        return -1;
    }
    /* start MAC layer */
    iface = ng_nomac_init(csma_mac_stack, sizeof(csma_mac_stack), PRIORITY_MAIN - 3,
                          "kw2xrf_l2", (ng_netdev_t *)&kw2xrf_dev);
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
    /* start the shell */
    shell_init(&shell, NULL, STDIO_RX_BUFSIZE, shell_read, shell_put);
    shell_run(&shell);

#endif
    return 0;
}
