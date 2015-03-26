/*
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_kw2xrf
 * @{
 * @file        kw2xrf_internal.h
 * @brief       Definition of KW2XRF internal used functions
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 */
#ifndef KW2XRF_INTERNAL_H
#define KW2XRF_INTERNAL_H

#include <stdio.h>
#include "board.h"
#include "cpu.h"
#include "periph/spi.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Turn KW2XRF on.
 *
 * @return 0 if the radio was correctly turned on.
 */
int kw2xrf_on(kw2xrf_t *dev);

/**
 * @brief Turn KW2XRF off.
 */
void kw2xrf_off(void);

/**
 * @brief Indicate if the KW2XRF is on.
 *
 * @return 1 if the radio transceiver is on (active); 0 otherwise.
 */
int kw2xrf_is_on(void);

/**
 * @brief Sets the output (TX) power of the KW2XRF.
 *
 * @param[in] pow The desired TX (output) power in dBm,
 *                 valid values are -35 to 8; other values
 *                 will be "saturated" into this range.
 *
 * @return The set TX (output) power after calling.
 */
int kw2xrf_set_tx_power(int pow);

/**
 * @brief Gets the current output (TX) power of the KW2XRF.
 *
 * @return The current TX (output) power.
 */
int kw2xrf_get_tx_power(void);

/**
 * @brief Checks if the radio medium is available/clear to send
 *         ("Clear Channel Assessment" a.k.a. CCA).
 *
 * @param[in] dev   An IEEE 802.15.4 network device.
 *
 * @return a 0 value if radio medium is clear (available),
 *         a error-value otherwise.
 *
 */
//int kw2xrf_channel_clear(netdev_t *dev);

/**
 * @brief Interrupt handler, gets fired when a RX overflow happens.
 *
 */
void kw2xrf_rxoverflow_irq(void);

/**
 * @brief Interrupt handler, gets fired when bytes in the RX FIFO are present.
 *
 */
void kw2xrf_rx_irq(void *args);

/**
 * @brief Sets the function called back when a packet is received.
 *        (Low-level mechanism, parallel to the `transceiver` module).
 *
 * @param[in] dev       An IEEE 802.15.4 network device.
 * @param[in] recv_cb   callback function for 802.15.4 packet arrival
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
//int kw2xrf_add_raw_recv_callback(netdev_t *dev, netdev_802154_raw_packet_cb_t recv_cb);

/**
 * @brief Unsets the function called back when a packet is received.
 *        (Low-level mechanism, parallel to the `transceiver` module).
 *
 * @param[in] dev       An IEEE 802.15.4 network device.
 * @param[in] recv_cb   callback function to unset
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 */
//int kw2xrf_rem_raw_recv_callback(netdev_t *dev, netdev_802154_raw_packet_cb_t recv_cb);

/**
 * @brief Sets a function called back when a data packet is received.
 *
 * @param[in] dev       An IEEE 802.15.4 network device.
 * @param[in] recv_cb   callback function for 802.15.4 data packet arrival
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
//int kw2xrf_add_data_recv_callback(netdev_t *dev, netdev_rcv_data_cb_t recv_cb);

/**
 * @brief Unsets a function called back when a data packet is received.
 *
 * @param[in] dev       An IEEE 802.15.4 network device.
 * @param[in] recv_cb   callback function to unset
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 */
//int kw2xrf_rem_data_recv_callback(netdev_t *dev, netdev_rcv_data_cb_t recv_cb);

/**
 * @brief RX handler, process data from the RX FIFO.
 *
 */

/**
 * @brief Switches the KW2XRF into receive mode.
 */
void kw2xrf_switch_to_rx(void);

/**
 * @brief Turns monitor (promiscuous) mode on or off.
 *
 * @param[in] mode The desired mode:
 *                 1 for monitor (promiscuous) mode;
 *                 0 for normal (auto address-decoding) mode.
 */
void kw2xrf_set_monitor(int mode);

/**
 * @brief Indicate if the KW2XRF is in monitor (promiscuous) mode.
 *
 * @return 1 if the transceiver is in monitor (promiscuous) mode;
 *         0 if it is in normal (auto address-decoding) mode.
 */
int kw2xrf_get_monitor(void);

/**
 * @brief Set the channel of the KW2XRF.
 *
 * @param[in] chan The desired channel, valid channels are from 11 to 26.
 *
 * @return The tuned channel after calling, or -1 on error.
 */
int kw2xrf_set_channel(unsigned int chan);

/**
 * @brief Get the channel of the KW2XRF.
 *
 * @return The tuned channel.
 */
unsigned int kw2xrf_get_channel(void);

/**
 * @brief Sets the short address of the KW2XRF.
 *
 * @param[in] addr The desired address.
 *
 * @return The set address after calling.
 */
uint16_t kw2xrf_set_address(uint16_t addr);

/**
 * @brief Gets the current short address of the KW2XRF.
 *
 * @return The current short address.
 */
uint16_t kw2xrf_get_address(void);

/**
 * @brief Sets the IEEE long address of the KW2XRF.
 *
 * @param[in] addr The desired address.
 *
 * @return The set address after calling.
 */
uint64_t kw2xrf_set_address_long(uint64_t addr);

/**
 * @brief Gets the current IEEE long address of the KW2XRF.
 *
 * @return The current IEEE long address.
 */
uint64_t kw2xrf_get_address_long(void);

/**
 * @brief Sets the pan ID of the KW2XRF.
 *
 * @param[in] pan The desired pan ID.
 *
 * @return The set pan ID after calling.
 */
uint16_t kw2xrf_set_pan(uint16_t pan);

/**
 * @brief Gets the current IEEE long address of the KW2XRF.
 *
 * @return The current pan ID.
 */

uint16_t kw2xrf_get_pan(void);
/**
 * @brief   Reference to the kw2xrf driver interface
 */

void kw2xrf_rx_handler(void);
#ifdef __cplusplus
}
#endif

#endif
/** @} */
