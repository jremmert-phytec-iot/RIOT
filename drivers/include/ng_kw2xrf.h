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

#include <stdint.h>

//#include "kernel_types.h"
#include "kernel.h"
#include "periph/gpio.h"
#include "net/ng_netbase.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Maximum packet length, including XBee API frame overhead
 */
#define KW2XRF_MAX_PKT_LENGTH         (127U)

/**
 * @brief   Maximum length of a command response
 */
//TODO: neccessary?
//#define KW2XRF_MAX_RESP_LENGTH        (16U)

/**
 * @brief   Default protocol for data that is coming in
 */
#ifdef MODULE_NG_SIXLOWPAN
#define KW2XRF_DEFAULT_PROTOCOL       NG_NETTYPE_SIXLOWPAN
#else
#define KW2XRF_DEFAULT_PROTOCOL       NG_NETTYPE_UNDEF
#endif

/**
 * @brief   Default short address used after initialization
 */
#define KW2XRF_DEFAULT_SHORT_ADDR     (0x0240)

/**
 * @brief   Default PAN ID used after initialization
 */
#define KW2XRF_DEFAULT_PANID          (0x0001)

/**
 * @brief   Default channel used after initialization
 */
#define KW2XRF_DEFAULT_CHANNEL        (13U)

/* Dummy definition for successfull build */
typedef struct {
    /* netdev fields */
    ng_netdev_driver_t *driver;     /**< pointer to the devices interface */
    ng_netdev_event_cb_t event_cb;  /**< netdev event callback */
    kernel_pid_t mac_pid;           /**< the driver's thread's PID */
    /* Devide driver specific fields */
    uint8_t buf[KW2XRF_MAX_PKT_LENGTH];
    ng_netconf_state_t state;       /**< Variable to keep radio driver's state */
    uint8_t seq_nr;                 /**< Next packets sequence number */
    uint16_t radio_pan;             /**< The PAN the radio device is using */
    uint8_t radio_channel;          /**< The channel the radio device is using */
    uint8_t addr_short[2];          /**< The short address the radio device is using */
    uint8_t addr_long[8];           /**< The long address the radio device is using */
    uint8_t options;                /**< Bit field to save enable/disable options */
    ng_nettype_t proto;             /**< Protocol the interface speaks */
} kw2xrf_t;

/**
 * @brief   Initialize the given KW2XRF device
 *
 * @param[out] dev          KW2XRF device to initialize
 *
 * @return                  0 on success
 * @return                  -ENODEV on invalid device descriptor
 */
int kw2xrf_init(kw2xrf_t *dev);

/**
 * Reference to the KW2XRF driver interface.
 */
extern const ng_netdev_driver_t kw2xrf_driver;

#ifdef __cplusplus
}
#endif

#endif
/** @} */
