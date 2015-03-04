/*
 * Copyright (C) 2014 Martine Lenders <mlenders@inf.fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Tests for general network device interface
 *
 *
 * @author      Martine Lenders <mlenders@inf.fu-berlin.de>
 *
 * @}
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>

#ifdef MODULE_NATIVENET
#include "nativenet.h"
#endif

#include "net_help.h"
#include "netdev/base.h"
#include "netdev/default.h"
#include "net/ng_csma_mac.h"
#include "net/ng_netapi.h"
#include "ng_kw2xrf.h"

#define UNITTESTS_CSMA_MAC_STACKSIZE (KERNEL_CONF_STACKSIZE_DEFAULT)
#define UNITTESTS_NOMAC_NAME        "unittests_csma_mac"

static char unittests_csma_mac_stack[UNITTESTS_CSMA_MAC_STACKSIZE];
static kernel_pid_t ng_csma_mac_pid = KERNEL_PID_UNDEF;

static ng_netdev_t index_dev;
static ng_netdev_t *dev = &index_dev;

#define SHELL_BUFSIZE   (UART0_BUFSIZE)

int main(void)
{
    msg_t msg;
    
    ng_netapi_opt_t index_opt;
    ng_netapi_opt_t *opt = &index_opt;

    kw2xrf_init();
    dev->driver = &kw2xrf_driver;
    
    int ret = 0;
    int msg_counter = 0;

    puts("\nRIOT netdev test");
    puts("Starting csma_mac thread\n");
    
    msg.type = NG_NETAPI_MSG_TYPE_SND;
    
    ng_csma_mac_pid = csma_mac_init(unittests_csma_mac_stack,
                           UNITTESTS_CSMA_MAC_STACKSIZE,
                           PRIORITY_MAIN - 1, UNITTESTS_NOMAC_NAME,
                           dev); 
    
    while(1){
        msg.content.value = msg_counter;
        msg.content.ptr = (char*)opt;
        ret = msg_send(&msg, ng_csma_mac_pid);
        if(ret){
            msg_counter++;
        }
    }
    return 0;
}
