/*
 * Copyright (C) 2016 Phytec Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_kw2xrf kw2x radio-driver
 * @ingroup     drivers_netdev_netdev2
 * @brief       Device driver for the NXP CR20A and KW2xD radios
 * @{
 *
 * @file
 * @brief       Interface definition for the kw2xrf driver
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 */

#ifndef MKW2XDRF_H_
#define MKW2XDRF_H_

#include <stdint.h>

#include "board.h"
#include "periph/spi.h"
#include "periph/gpio.h"
#include "net/netdev2.h"
#include "net/netdev2/ieee802154.h"
#include "net/gnrc/nettype.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Maximum packet length
 */
#define KW2XRF_MAX_PKT_LENGTH           (127U)

/**
 * @brief   Default short address used after initialization
 */
#define KW2XRF_DEFAULT_SHORT_ADDR       (0x0042)

/**
 * @brief   Default short address used after initialization
 */
#define KW2XRF_DEFAULT_ADDR_LONG        (0x0000000DEADCAB1E)

/**
 * @brief   Default PAN ID used after initialization
 */
#define KW2XRF_DEFAULT_PANID            (0x0023)

/**
 * @brief   Default channel used after initialization
 */
#ifdef DEFAULT_CHANNEL
#define KW2XRF_DEFAULT_CHANNEL (DEFAULT_CHANNEL)
#endif
#ifndef KW2XRF_DEFAULT_CHANNEL
#define KW2XRF_DEFAULT_CHANNEL          (26U)
#endif
#define KW2XRF_MIN_CHANNEL              (11U)
#define KW2XRF_MAX_CHANNEL              (26U)

/**
 * @brief   Default TX_POWER in dbm used after initialization
 */
#define KW2XRF_DEFAULT_TX_POWER         (0)

/**
 * @brief   Maximum output power of the kw2x device in dBm
 */
#define MKW2XDRF_OUTPUT_POWER_MAX       (8)

/**
 * @brief   Minimum output power of the kw2x device in dBm
 */
#define MKW2XDRF_OUTPUT_POWER_MIN       (-35)

/**
 * @brief   Internal device option flags
 *
 * `0x00ff` is reserved for general IEEE 802.15.4 flags
 * (see @ref netdev2_ieee802154_t)
 *
 * @{
 */
#define KW2XRF_OPT_SRC_ADDR_LONG    (NETDEV2_IEEE802154_SRC_MODE_LONG) /**< legacy define */
#define KW2XRF_OPT_RAWDUMP          (NETDEV2_IEEE802154_RAW)           /**< legacy define */
#define KW2XRF_OPT_AUTOACK          (NETDEV2_IEEE802154_ACK_REQ)       /**< legacy define */

#define KW2XRF_OPT_AUTOCCA          (0x0100)    /**< CCA befor TX active */
#define KW2XRF_OPT_PROMISCUOUS      (0x0200)    /**< promiscuous mode
                                                  *   active */
#define KW2XRF_OPT_PRELOADING       (0x0400)    /**< preloading enabled */
#define KW2XRF_OPT_TELL_TX_START    (0x0800)    /**< notify MAC layer on TX
                                                  *   start */
#define KW2XRF_OPT_TELL_TX_END      (0x1000)    /**< notify MAC layer on TX
                                                  *   finished */
#define KW2XRF_OPT_TELL_RX_START    (0x2000)    /**< notify MAC layer on RX
                                                  *   start */
#define KW2XRF_OPT_TELL_RX_END      (0x4000)    /**< notify MAC layer on RX
                                                     *   finished */
/** @} */

/**
 * @brief struct holding all params needed for device initialization
 */
typedef struct kw2xrf_params {
    spi_t spi;                          /**< SPI bus the device is connected to */
    spi_speed_t spi_speed;              /**< SPI speed to use */
    gpio_t cs_pin;                      /**< GPIO pin connected to chip select */
    gpio_t int_pin;                     /**< GPIO pin connected to the interrupt pin */
} kw2xrf_params_t;

/**
 * @brief   Device descriptor for KW2XRF radio devices
 *
 * @extends netdev2_ieee802154_t
 */
typedef struct {
    netdev2_ieee802154_t netdev;         /**< netdev2 parent struct */
    /**
     * @brief   device specific fields
     * @{
     */
    kw2xrf_params_t params;             /**< parameters for initialization */
    uint8_t buf[KW2XRF_MAX_PKT_LENGTH]; /**< Buffer for incoming or outgoing packets */
    uint8_t state;                      /**< current state of the radio */
    uint8_t tx_frame_len;               /**< length of the current TX frame */
    uint8_t idle_state;                 /**< state to return to after sending */
    uint8_t pending_tx;                 /**< keep track of pending TX calls
                                             this is required to know when to
                                             return to @ref kw2xrf_t::idle_state */
    int16_t tx_power;                   /**< The current tx-power setting of the device */
    /** @} */
} kw2xrf_t;

/**
 * @brief   Setup an KW2XRF based device state
 *
 * @param[out] dev          device descriptor
 * @param[in]  params       parameters for device initialization
 */
void kw2xrf_setup(kw2xrf_t *dev, const kw2xrf_params_t *params);

/**
 * @brief   Initialize the given KW2XRF device
 * @param[out] dev          device descriptor
 * @param[in] dev           irq callback 
 *
 * @return                  0 on success
 * @return                  <0 on error
 */
int kw2xrf_init(kw2xrf_t *dev, gpio_cb_t cb);

/**
 * @brief   Configure radio with default values
 *
 * @param[in] dev           device to reset
 */
void kw2xrf_reset_phy(kw2xrf_t *dev);


#ifdef __cplusplus
}
#endif

#endif
/** @} */
