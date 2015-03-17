/*
 * Copyright (C) 2015 Phytec Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_kw2xrf kw2x radio-driver
 * @ingroup     drivers
 * @brief       Device driver for the Freescale MKW2xD radio
 * @{
 *
 * @file
 * @brief       Interface definition for the MKW2xD device driver
 *
 * @author     Johann Fischer <j.fischer@phytec.de>
 * @author     Jonas Remmert <j.remmert@phytec.de>
 */
/*
 * @brief packet buffer
 * <pre>
     buffer for tx            buffer for rx
     -----------------         -----------------
   0 | Frame Length  |       0 | PSDU byte 0   |
     -----------------         -----------------
   1 | PSDU byte 0   |       1 | PSDU byte 1   |
     -----------------         -----------------
   2 | PSDU byte 1   |       2 | PSDU byte 2   |
     -----------------         -----------------
 ... |               |     ... |               |
     -----------------         -----------------
 125 | PSDU byte 125 |     125 | FCS byte 0    |
     -----------------         -----------------
 126 | res. (FCS)    |     126 | FCS byte 1    |
     -----------------         -----------------
 127 | res. (FCS)    |     127 | LQI           |
     -----------------         -----------------
  </pre>
*/

#ifndef MKW2XDRF_H_
#define MKW2XDRF_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "kernel_types.h"
#include "ieee802154_frame.h"
#include "netdev/802154.h"
#include "net/ng_netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MKW2XDRF_MAX_PKT_LENGTH     127      /**< Max packet length for kw2xrf device. */
#define MKW2XDRF_DEFAULT_CHANNR     13       /**< Default radio channel. */
#define MKW2XDRF_DEFAULT_RADIO_PAN  0x0001   /**< Default radio pan ID */

/**
 *  Structure to represent a kw2xrf packet.
 */
typedef struct __attribute__((packed))
{
    uint8_t length;             /**< The length of the frame of the frame including fcs. */
    ieee802154_frame_t frame;   /**< The ieee802154 frame. */
    uint8_t lqi;                /**< The link quality indicator. */
    int8_t rssi;                /**< The rssi value. */
    bool crc;                   /**< 1 if crc was successfull, 0 otherwise. */
}
kw2xrf_packet_t;

/* Dummy definition for successfull build */
typedef struct {
    int tmp;
} kw2xrf_t;

int kw2xrf_init(ng_netdev_t *dev);

/**
 * Netdev representation of this driver.
 */
extern ng_netdev_t *kw2xrf_netdev;

extern const ng_netdev_driver_t kw2xrf_driver;

#ifdef __cplusplus
}
#endif

#endif
