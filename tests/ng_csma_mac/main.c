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
#include "net/ng_simplemac.h"


#ifdef NETDEV_DEFAULT
#define SHELL_BUFSIZE   (UART0_BUFSIZE)

static netdev_t *dev = NULL;

int main(void)
{

    puts("\nRIOT netdev test");

    if (dev == NULL) {
        puts("Default device was NULL");
        return 1;
    }

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
    puts("\nRIOT netdev test");
    puts("Default netdev type and driver unknown!");

    return 0;
}

#endif /* !NETDEV_DEFAULT */
