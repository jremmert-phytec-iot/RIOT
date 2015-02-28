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

#define UNITTESTS_CSMA_MAC_STACKSIZE (KERNEL_CONF_STACKSIZE_DEFAULT)
#define UNITTESTS_NOMAC_NAME        "unittests_csma_mac"

static char unittests_csma_mac_stack[UNITTESTS_CSMA_MAC_STACKSIZE];
static kernel_pid_t ng_csma_mac_pid = KERNEL_PID_UNDEF;
static netdev_t *dev = NULL;

#ifdef NETDEV_DEFAULT
#define SHELL_BUFSIZE   (UART0_BUFSIZE)


int main(void)
{

    puts("\nRIOT netdev test");

    /*if (dev == NULL) {
        puts("Default device was NULL");
        return 1;
    }*/

    printf("Initialized dev ");

    switch (dev->type) {
        case NETDEV_TYPE_UNKNOWN:
            printf("of unknown type\n");
            break;

        case NETDEV_TYPE_BASE:
            printf("as basic device\n");
            break;

        default:
            printf("of undefined type\n");
            break;
    }

    if (dev->driver == NULL) {
        puts("Default driver is defined as NULL!");
        return 1;
    }
    return 0;
}

#else

int main(void)
{
    msg_t msg;

    puts("\nRIOT netdev test");
    puts("Starting csma_mac thread\n");
    
    msg.type = NG_NETAPI_MSG_TYPE_SND;
    //msg.content.
    
    ng_csma_mac_pid = csma_mac_init(unittests_csma_mac_stack,
                           UNITTESTS_CSMA_MAC_STACKSIZE,
                           PRIORITY_MAIN - 1, UNITTESTS_NOMAC_NAME,
                           dev); 

    msg_send(&msg, ng_csma_mac_pid);
    return 0;
}

#endif /* !NETDEV_DEFAULT */
