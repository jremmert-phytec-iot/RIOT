/*
 * Copyright (C) 2015 PHYTEC Messtechnik GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     kw2xrf
 * @{
 * @file        kw2xrf.c
 * @brief       Basic functionality of kw2xrf driver
 *
 * @author      Johann Fischer <j.fischer@phytec.de>
 * @author      Jonas Remmert <j.remmert@phytec.de>
 * @}
 */
#include "crash.h"
#include "ng_kw2xrf.h"
#include "kw2xrf_spi.h"
#include "kw2xrf_reg.h"
#include "kw2xrf_internal.h"
//#include "periph_conf.h"
#include "mutex.h"
//#include "hwtimer.h"
#include "msg.h"
#include "periph/uart.h"
#include "periph/gpio.h"
#include "net/ng_netbase.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"
/* Enables integrated testing functions, such as LED-toggling
 * on state-change and idle-wait functions to test arbitration
 * using multiple boards.
 */
#define TESTING_FUNCTIONS   (1)

/**
 * @brief   Internal driver event type in case of an unexpected interrupt
 */
#define ISR_EVENT_UNKNOWN      (0x0020)

#define MKW2XDRF_OUTPUT_POWER_MAX  8        /**< Maximum output power of the kw2x-rf radio in dBm */
#define MKW2XDRF_OUTPUT_POWER_MIN  (-35)    /**< Minimum output power of the kw2x-rf radio in dBm */

/* Modem_PA_PWR Register (PA Power Control) has a valid range from 3-31 */
#define MKW2XDRF_PA_RANGE_MAX      31       /**< Maximum value of PA Power Control Register */
#define MKW2XDRF_PA_RANGE_MIN      3        /**< Minimum value of PA Power Control Register */

/* PLL integer lookup table */
static const uint8_t pll_int_lt[16] = {
    11, 11, 11, 11,
    11, 11, 12, 12,
    12, 12, 12, 12,
    13, 13, 13, 13
};

/* PLL frequency fractional lookup table */
static const uint16_t pll_frac_lt[16] = {
    10240, 20480, 30720, 40960,
    51200, 61440, 6144, 16384,
    26624, 36864, 47104, 57344,
    2048, 12288, 22528, 32768
};

static const uint8_t pow_lt[44] = {
    3, 4, 4, 5,
    6, 6, 7, 7,
    8, 9, 9, 10,
    11, 11, 12, 13,
    13, 14, 14, 15,
    16, 16, 17, 18,
    18, 19, 20, 20,
    21, 21, 22, 23,
    23, 24, 25, 25,
    26, 27, 27, 28,
    28, 29, 30, 31
};

static const int level_lt[29] = {
    -35, -34, -32, -31,
    -29, -28, -26, -25,
    -23, -22, -20, -19,
    -17, -16, -14, -13,
    -11, -10, -8, -7,
    -5, -4, -2, -1,
    1, 2, 4, 5,
    7
};
/**************Internal Functions****************************/

int kw2xrf_set_tx_power(int pow)
{
    if (pow > MKW2XDRF_OUTPUT_POWER_MAX) {
        pow = MKW2XDRF_OUTPUT_POWER_MAX;
    }

    if (pow < MKW2XDRF_OUTPUT_POWER_MIN) {
        pow = MKW2XDRF_OUTPUT_POWER_MIN;
    }

    uint8_t level = pow_lt[pow - MKW2XDRF_OUTPUT_POWER_MIN];
    kw2xrf_write_dreg(MKW2XDM_PA_PWR, MKW2XDM_PA_PWR(level));
    return pow;
}

int _set_channel(uint8_t *val, size_t len)
{
    if (val[0] < 11 || val[0] > 26) {
        DEBUG("Invalid channel %i set. Valid channels are 11 through 26\n", val[0]);
        return -EINVAL;
    }

    if (len != 2 || val[1] != 0) {
        DEBUG("kw2xrf: set channel failed, len: %u, val[0]:%u\n", len, val[0]);
        return -EINVAL;
    }

    /*
     * Fc = 2405 + 5(k - 11) , k = 11,12,...,26
     *
     * Equation for PLL frequency, MKW2xD Reference Manual, p.255 :
     * F = ((PLL_INT0 + 64) + (PLL_FRAC0/65536))32MHz
     *
     */
    val[0] -= 11;
    kw2xrf_write_dreg(MKW2XDM_PLL_INT0, MKW2XDM_PLL_INT0_VAL(pll_int_lt[val[0]]));
    kw2xrf_write_dreg(MKW2XDM_PLL_FRAC0_LSB, (uint8_t)pll_frac_lt[val[0]]);
    kw2xrf_write_dreg(MKW2XDM_PLL_FRAC0_MSB, (uint8_t)(pll_frac_lt[val[0]] >> 8));
    return 2;
}

int _get_channel(uint8_t *val, size_t max)
{
    if (max < 2) {
        return -EOVERFLOW;
    }
    uint8_t pll_int = kw2xrf_read_dreg(MKW2XDM_PLL_INT0);
    uint16_t pll_frac = kw2xrf_read_dreg(MKW2XDM_PLL_FRAC0_LSB);
    pll_frac |= ((uint16_t)kw2xrf_read_dreg(MKW2XDM_PLL_FRAC0_MSB) << 8);

    for (int i = 0; i < 16; i++) {
        if ((pll_frac_lt[i] == pll_frac) && (pll_int_lt[i] == pll_int)) {
            val[0] = i + 11;
            val[1] = 0;
            return 2;
        }
    }
    return -EINVAL;
}

void _set_sequence(kw2xrf_physeq_t seq)
{
    
    uint8_t reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
    reg &= ~MKW2XDM_PHY_CTRL1_XCVSEQ_MASK; /* set last three bit to 0 */

   	if (kw2xrf_read_dreg(MKW2XDM_SEQ_STATE)) {
        /* abort any ongoing sequence */
        DEBUG("tx: abort SEQ_STATE: %x\n", kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));
        kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
    	while (kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));

        /* Clear all pending interrupts */
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, 0x7f);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS2, 0x03);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS3, 0xff);

        /* Mask all possible interrupts */
        reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL2);
        reg |= ~(MKW2XDM_PHY_CTRL2_CRC_MSK);
        kw2xrf_write_dreg(MKW2XDM_PHY_CTRL2, reg);

        reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL3);
        reg |= (MKW2XDM_PHY_CTRL3_PB_ERR_MSK | MKW2XDM_PHY_CTRL3_WAKE_MSK);
        reg &= (MKW2XDM_PHY_CTRL3_PB_ERR_MSK | MKW2XDM_PHY_CTRL3_WAKE_MSK);
        kw2xrf_write_dreg(MKW2XDM_PHY_CTRL2, reg);
    }
    /* For all sequences only enable SEQ-irq, that is set when sequence was completed */
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL2);
    reg &= ~(MKW2XDM_PHY_CTRL2_SEQMSK);
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL2, reg);

    /* Progrmm new sequence */
    reg |= MKW2XDM_PHY_CTRL1_XCVSEQ(seq);
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
    DEBUG("ng_kw2xrf: Set sequence to %i\n", seq);

    /* TODO: For testing, ignores CRCVALID in receive mode */
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL2);
    reg &= ~(MKW2XDM_PHY_CTRL4_CRC_MSK);
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL2, reg);

    /* TODO: For testing, set up promiscous mode */
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL4);
    reg |= MKW2XDM_PHY_CTRL4_PROMISCUOUS;
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL4, reg);
}

int _set_tx(void)
{
    _set_sequence(XCVSEQ_TRANSMIT);
    return 0;
}

int _set_rx(void)
{
    DEBUG("ng_kw2xrf: info: switch_to_rx\n");
    _set_sequence(XCVSEQ_RECEIVE);
    return 0;
}

void kw2xrf_irq_handler(void *args)
{
    msg_t msg;
    kw2xrf_t *dev = (kw2xrf_t *)args;
    uint8_t irqst1 = kw2xrf_read_dreg(MKW2XDM_IRQSTS1);
    
    if ((irqst1 & MKW2XDM_IRQSTS1_RXIRQ) && (irqst1 & MKW2XDM_IRQSTS1_SEQIRQ)) {
        msg.content.value = NETDEV_EVENT_RX_COMPLETE;
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_RXIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
    }
    else if ((irqst1 & MKW2XDM_IRQSTS1_TXIRQ) && (irqst1 & MKW2XDM_IRQSTS1_SEQIRQ)) {
        msg.content.value = NETDEV_EVENT_TX_COMPLETE;
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_TXIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
    }
    else if ((irqst1 & MKW2XDM_IRQSTS1_CCAIRQ) && (irqst1 & MKW2XDM_IRQSTS1_SEQIRQ)) {
        msg.content.value = NETDEV_EVENT_TX_STARTED;
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, MKW2XDM_IRQSTS1_CCAIRQ | MKW2XDM_IRQSTS1_SEQIRQ);
    }
    else {
        msg.content.value = ISR_EVENT_UNKNOWN;
        /* Clear all interrupts to prevent ISR-loop */
        kw2xrf_write_dreg(MKW2XDM_IRQSTS1, 0x7f);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS2, 0x03);
        kw2xrf_write_dreg(MKW2XDM_IRQSTS3, 0xff);
    }

    /* packet is complete */
    msg.type = NG_NETDEV_MSG_TYPE_EVENT;
    msg_send_int(&msg, dev->mac_pid);
}

/* Set up interrupt sources, triggered by the radio-module */
void kw2xrf_init_interrupts(kw2xrf_t *dev)
{
    /* Disable all interrups:
     * Selectively enable only one interrupt source selectively in sequence manager.
     * After reset state all interrupts are disabled, except WAKE_IRQ.
     */
    kw2xrf_write_dreg(MKW2XDM_IRQSTS1, 0x7f);
    kw2xrf_write_dreg(MKW2XDM_IRQSTS2, 0x03);
    kw2xrf_write_dreg(MKW2XDM_IRQSTS3, 0xff);

    /* Mask all possible interrupts */
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL2);
    reg |= ~(MKW2XDM_PHY_CTRL2_CRC_MSK);
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL2, reg);

    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL3);
    reg |= (MKW2XDM_PHY_CTRL3_PB_ERR_MSK | MKW2XDM_PHY_CTRL3_WAKE_MSK);
    reg &= (MKW2XDM_PHY_CTRL3_PB_ERR_MSK | MKW2XDM_PHY_CTRL3_WAKE_MSK);
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL2, reg);

    /* set up GPIO-pin used for IRQ */
    gpio_init_int(GPIO_KW2XDRF, GPIO_NOPULL, GPIO_FALLING, &kw2xrf_irq_handler, dev);
}

int _set_pan(uint8_t *val, size_t len)
{
    if (len != 2) {
        return -EINVAL;
    }
    kw2xrf_write_iregs(MKW2XDMI_MACPANID0_LSB, val, 2);
    return 2;
}

int _get_pan(uint8_t *val, size_t max)
{
    if (max < 2) {
        return -EOVERFLOW;
    }
    kw2xrf_read_iregs(MKW2XDMI_MACPANID0_LSB, val, 2);
    return 2;
}

int kw2xrf_on(void)
{
    uint8_t tmp;
    /* check modem's crystal oscillator, CLK_OUT shall be 4MHz */
    tmp = kw2xrf_read_dreg(MKW2XDM_CLK_OUT_CTRL);

    if (tmp != 0x8Bu) {
        return -1;
    }

    DEBUG("SEQ_STATE: %x\n", kw2xrf_read_dreg(MKW2XDM_SEQ_STATE));

    /* enable RFon mode */
    kw2xrf_write_dreg(MKW2XDM_PWR_MODES,
                      (MKW2XDM_PWR_MODES_XTALEN | MKW2XDM_PWR_MODES_PMC_MODE));

    /* abort any ongoing sequence */
    _set_sequence(XCVSEQ_IDLE);

    return 0;
}

uint8_t _channel_clear(void){
    _set_sequence(XCVSEQ_CCA);   /* start CCA, interrupt is triggered if ready */
    return 0;
}

int _get_addr(uint8_t *val, size_t len)
{
    if (len == 2) {
        kw2xrf_read_iregs(MKW2XDMI_MACSHORTADDRS0_LSB, val, 2);
        return 2;
    }
    else if (len == 8){
        kw2xrf_read_iregs(MKW2XDMI_MACLONGADDRS0_0, val, 8);
        return 8;
    }
    return -ENOTSUP;
}

int _set_addr(uint8_t *val, size_t len)
{
    if (len == 2) {
        kw2xrf_write_iregs(MKW2XDMI_MACSHORTADDRS0_LSB, val, 2);
        return 2;
    }
    else if (len == 8){
        kw2xrf_write_iregs(MKW2XDMI_MACLONGADDRS0_0, val, 8);
        return 8;
    }
    return -ENOTSUP;
}

int kw2xrf_init(kw2xrf_t *dev) {
    uint8_t reg = 0;
    uint8_t tmp[2];

    /* check device parameters */
    if (dev == NULL) {
        return -ENODEV;
    }

    if (!(kw2xrf_on() == 0)) {
        core_panic(0x42, "Could not start MKW2XD radio transceiver");
    }

    /* Gerneral initialization of interrupt sources.
     * sets radio to idle modewith all interrupt masked
     */
    kw2xrf_init_interrupts(dev);
    kw2xrf_spi_init();

    /* set device driver */
    dev->driver = &kw2xrf_driver;
    /* set default options */
    dev->proto = KW2XRF_DEFAULT_PROTOCOL;
    dev->options = 0;
    dev->addr_short[0] = (uint8_t)(KW2XRF_DEFAULT_SHORT_ADDR >> 8);
    dev->addr_short[1] = (uint8_t)(KW2XRF_DEFAULT_SHORT_ADDR);
    /* set default short address */
    _set_addr(dev, dev->addr_short, 2);
    /* load long address */
    _get_addr_long(dev, dev->addr_long, 8);

    kw2xrf_set_tx_power(0);

    /* set default channel */
    tmp[1] = 0;
    tmp[0] = KW2XRF_DEFAULT_CHANNEL;
    _set_channel(dev, tmp, 2);
    /* set default PAN ID */
    tmp[1] = (uint8_t)(KW2XRF_DEFAULT_PANID >> 8);
    tmp[0] = (uint8_t)(KW2XRF_DEFAULT_PANID & 0xff);
    _set_panid(dev, tmp, 2);

    //kw2xrf_read_iregs(MKW2XDMI_MACSHORTADDRS0_LSB, (uint8_t *)kw2xrf_t->radio_address, 2);
    //kw2xrf_read_iregs(MKW2XDMI_MACLONGADDRS0_0, (uint8_t *)kw2xrf_t->radio_address_long, 8);

    /* CCA Setup */
    reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL4);
    reg |= MKW2XDM_PHY_CTRL4_CCATYPE(1); /* Set up CCA mode 1 (RSSI threshold) */
    kw2xrf_write_dreg(MKW2XDM_PHY_CTRL4, reg);
    DEBUG("kw2xrf: Initialized and set to channel %i and pan %i.\n",
    MKW2XDRF_DEFAULT_CHANNR, MKW2XRF_DEFAULT_PANID);
    return 0;
}

int _add_cb(ng_netdev_t *dev, ng_netdev_event_cb_t cb)
{
    if (dev == NULL) {
        return -ENODEV;
    }
    if (dev->event_cb != NULL) {
        return -ENOBUFS;
    }
    dev->event_cb = cb;
    return 0;
}

int _rem_cb(ng_netdev_t *dev, ng_netdev_event_cb_t cb)
{
    if (dev == NULL) {
        return -ENODEV;
    }
    if (dev->event_cb != cb) {
        return -ENOENT;
    }
    dev->event_cb = NULL;
    return 0;
}

int _get(ng_netdev_t *netdev, ng_netconf_opt_t opt, void *value, size_t max_len)
{
    int res = 0;
    switch (opt) {
        case NETCONF_OPT_ADDRESS:
            res = _get_addr((uint8_t *)value, max_len);
            return res;
        case NETCONF_OPT_CHANNEL:
            res = _get_channel((uint8_t *)value, max_len);
            return res;
        case NETCONF_OPT_NID:
            res = _get_pan((uint8_t *)value, max_len);
            return res;
        case NETCONF_OPT_IS_CHANNEL_CLR:
            return _channel_clear();
        case NETCONF_OPT_STATE:
            return 0;
        default:
            return -ENOTSUP;
    }
}

int _set(ng_netdev_t *netdev, ng_netconf_opt_t opt, void *value, size_t value_len)
{
    uint8_t reg;
    switch (opt) {
        case NETCONF_OPT_CHANNEL:
            return _set_channel((uint8_t *)value, value_len);
        case NETCONF_OPT_ADDRESS:
            return _set_addr((uint8_t *)value, value_len);
        case NETCONF_OPT_NID:
            return _set_pan((uint8_t *)value, value_len);
        case NETCONF_OPT_IS_CHANNEL_CLR:
            return _channel_clear();
        case NETCONF_OPT_CCA_BEFORE_TX:
            reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
            reg |= MKW2XDM_PHY_CTRL1_CCABFRTX;          /* Set up CCA before TX */
            kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
            return 0;
        case NETCONF_OPT_AUTOACK:
            reg = kw2xrf_read_dreg(MKW2XDM_PHY_CTRL1);
            /* Set up HW generated automatic ACK after Receive */
            reg |= MKW2XDM_PHY_CTRL1_AUTOACK;
            kw2xrf_write_dreg(MKW2XDM_PHY_CTRL1, reg);
            return 0;
        case NETCONF_OPT_STATE:
            switch (*((int *)value))
                case NETCONF_STATE_TX:
                    DEBUG("KW2xrf: Function _set_tx entered");
                    _set_tx();
                    return 2;
                case NETCONF_STATE_RX:
                    _set_rx();
                    return 2;
        default:
            return -ENOTSUP;
    }
}

void _isr_event(ng_netdev_t *netdev, uint32_t event_type)
{
    kw2xrf_t *dev = (xbee_t *)netdev;
    ng_pktsnip_t *pkt_head;
    ng_pktsnip_t *pkt;
    ng_netif_hdr_t *hdr;
    size_t pos;
    size_t addr_len;
    uint8_t cksum = 0;
    
    /* check device */
    if (dev == NULL) {
        return;
    }
    
    if (event_type == NETDEV_EVENT_RX_COMPLETE) {
         /* TODO: Not handled right now. */
    }
    
    if (event_type == NETDEV_EVENT_TX_STARTED) {
        uint8_t irqst2 = kw2xrf_read_dreg(MKW2XDM_IRQSTS2);
        if (irqst2 & MKW2XDM_IRQSTS2_CCA) { /* Channel busy */
        }
    }
    
    if (event_type == NETDEV_EVENT_TX_COMPLETE) {
        //return success of send process
    }
}

int _send(ng_netdev_t *netdev, ng_pktsnip_t *pkt)
{
    if (netdev == NULL) {
        return -ENODEV;
    }
    if (pkt == NULL) {
        return -ENOMSG;
    }

    uint8_t mhr[24];
    uint8_t index;
    uint8_t *data = pkt->data;
    kw2xrf_t *dev = (kw2xrf_t*) netdev;
    ng_netif_hdr_t *hdr = (ng_netif_hdr_t *) data;

    /* default to data kind TODO: provide means to specify */
    mhr[0] = 0x01;
    /* default to wants_ack TODO: provide means to specify */
    mhr[0] |= 0x20;
    if (hdr->dst_l2addr_len == 2) {
        /* default to compress pan TODO: provide means to specify dst PAN*/
        mhr[0] |= 0x40;
        /* default to 2 byte addresses for src and dst */
        mhr[1] |= 0x88;
        index = 3;
        mhr[index++] = (uint8_t)((dev->radio_pan)&0xff);
        mhr[index++] = (uint8_t)((dev->radio_pan)>>8);
        /* set destination address located directly after ng_ifhrd_t in memory */
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+2);
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+1);
        /* set source address */
        mhr[index++] = (uint8_t)((dev->radio_address)&0xff);
        mhr[index++] = (uint8_t)((dev->radio_address)>>8);
    }
    else if (hdr->dst_l2addr_len == 8) {
        /* default to use long address mode for src and dst */
        mhr[1] = 0xcc;
        /* set destination address located directly after ng_ifhrd_t in memory */
        index = 3;
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+8);
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+7);
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+6);
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+5);
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+4);
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+3);
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+2);
        mhr[index++] = (uint8_t)*(data+sizeof(ng_netif_hdr_t)+1);
        /* set source address */
        mhr[index++] = (uint8_t)((dev->radio_address_long)&0xff);
        mhr[index++] = (uint8_t)((dev->radio_address_long)>>8);
        mhr[index++] = (uint8_t)((dev->radio_address_long)>>16);
        mhr[index++] = (uint8_t)((dev->radio_address_long)>>24);
        mhr[index++] = (uint8_t)((dev->radio_address_long)>>32);
        mhr[index++] = (uint8_t)((dev->radio_address_long)>>40);
        mhr[index++] = (uint8_t)((dev->radio_address_long)>>48);
        mhr[index++] = (uint8_t)((dev->radio_address_long)>>56);
    }
    /* unsupported address length */
    else {
        return -ENOMSG;
    }
    /* set sequence number */
    mhr[2] = dev->seq_nr++;

    /* index counts sent bytes from now on */
    index += 1;
    for (int i=1; i < index; i++) {
        /* buf addressing is shifted by one because of len field */
        dev->buf[i+1] = mhr[i];
    }

    while (pkt) {
        /* check we don't exceed FIFO size */
        if (index+pkt->size > MKW2XDRF_MAX_PKT_LENGTH) {
            ng_pktbuf_release(pkt);
            DEBUG("Packet exceeded FIFO size.\n");
            return -ENOBUFS;
        }
        for (int i=0; i < pkt->size; i++) {
            uint8_t *tmp = pkt->data;
            dev->buf[index+i+1] = tmp[i];
        }
        /* count bytes */
        index += pkt->size;

        /* next snip */
        pkt = pkt->next;
    }
    dev->buf[0] = index; /* set packet size */

    DEBUG("ng_kw2xrf: send packet with size %i\n", dev->buf[0]);
    kw2xrf_write_fifo(dev->buf, dev->buf[0]);

    if ((dev->options&(1<<NETCONF_OPT_PRELOADING)) == NETCONF_DISABLE) {
        DEBUG("Sending now.\n");
        _set_sequence(XCVSEQ_TRANSMIT);
    }

    return index;
}

/* implementation of the netdev interface */
const ng_netdev_driver_t kw2xrf_driver = {
    .send_data = _send,
    .add_event_callback = _add_cb,
    .rem_event_callback = _rem_cb,
    .get = _get,
    .set = _set,
    .isr_event = _isr_event,
};
